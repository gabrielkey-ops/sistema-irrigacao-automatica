/************************************************************
  Projeto: Sistema de Irrigacao Automatizado com ESP32
Este programa foi corrigido após a apresentação final.
************************************************************/

#define BLYNK_TEMPLATE_ID   "---"
#define BLYNK_TEMPLATE_NAME "---"
#define BLYNK_AUTH_TOKEN    "---"

#include <Arduino.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ==========================================================
// Wi-Fi
// ==========================================================
char ssid[] = "NOME_DA_REDE";
char pass[] = "SENHA_DA_REDE";

// ==========================================================
// Pinos fisicos do ESP32
// ==========================================================
const int PINO_UMIDADE_AO = 34;
const int PINO_UMIDADE_DO = 32;
const int PINO_NIVEL      = 35;
const int PINO_RELE       = 26;

const int PINO_SDA = 21;
const int PINO_SCL = 22;

// LCD I2C
// Caso nao funcione, testar 0x3F.
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ==========================================================
// Datastreams Blynk
// ==========================================================
#define VPIN_UMIDADE          V0
#define VPIN_RESERVATORIO     V1
#define VPIN_BOMBA            V2
#define VPIN_STATUS           V3
#define VPIN_LIMITE_UMIDADE   V4
#define VPIN_TEMPO_IRRIGACAO  V5

// ==========================================================
// Calibracao dos sensores
// Ajustar conforme os valores reais medidos no Serial Monitor
// ==========================================================
int ADC_SOLO_SECO  = 3200;
int ADC_SOLO_UMIDO = 1300;

int ADC_NIVEL_VAZIO = 100;
int ADC_NIVEL_CHEIO = 2600;

int LIMIAR_NIVEL_COM_AGUA = 700;

// ==========================================================
// Rele
// Pelo teste realizado:
// HIGH -> rele acionado
// LOW  -> rele desligado
// ==========================================================
const bool RELE_ATIVO_EM_LOW = false;

// ==========================================================
// Parametros configuraveis
// ==========================================================
int limiteUmidade = 40;
unsigned long tempoIrrigacao_s = 10;

const unsigned long INTERVALO_MINIMO_ENTRE_IRRIGACOES_MS = 30000;

// ==========================================================
// Estados do sistema
// ==========================================================
enum EstadoSistema {
  ST_INICIALIZANDO = 0,
  ST_MONITORANDO,
  ST_VERIFICANDO_RESERVATORIO,
  ST_IRRIGANDO,
  ST_BLOQUEADO_FALTA_AGUA,
  ST_AGUARDANDO_INTERVALO,
  ST_COUNT
};

// ==========================================================
// Eventos do sistema
// ==========================================================
enum EventoSistema {
  EV_INIT_OK = 0,
  EV_UMIDADE_OK,
  EV_SOLO_SECO,
  EV_SOLO_SECO_INTERVALO,
  EV_RESERVATORIO_COM_AGUA,
  EV_RESERVATORIO_VAZIO,
  EV_TEMPO_IRRIGACAO_FIM,
  EV_INTERVALO_FIM,
  EV_COUNT
};

// ==========================================================
// Acoes do sistema
// ==========================================================
enum AcaoSistema {
  AC_NENHUMA = 0,
  AC_ENTRAR_MONITORAMENTO,
  AC_VERIFICAR_RESERVATORIO,
  AC_INICIAR_IRRIGACAO,
  AC_FINALIZAR_IRRIGACAO,
  AC_BLOQUEAR_FALTA_AGUA,
  AC_AGUARDAR_INTERVALO
};

// ==========================================================
// Estrutura da matriz de transicao
// ==========================================================
struct Transicao {
  EstadoSistema proximoEstado;
  AcaoSistema acao;
};

Transicao matrizTransicao[ST_COUNT][EV_COUNT];

// ==========================================================
// Estrutura dos eventos na fila
// ==========================================================
struct EventoMsg {
  EventoSistema evento;
  unsigned long instante_ms;
};

// ==========================================================
// Variaveis globais protegidas por mutex
// ==========================================================
EstadoSistema estadoAtual = ST_INICIALIZANDO;

int adcUmidade = 0;
int adcNivel = 0;

int umidadePercentual = 0;
int nivelPercentual = 0;

bool soloSecoDigital = false;
bool reservatorioComAgua = false;
bool bombaLigada = false;

unsigned long instanteInicioIrrigacao = 0;
unsigned long instanteUltimaIrrigacao = 0;

char statusAtual[24] = "Iniciando";

// ==========================================================
// Objetos FreeRTOS
// ==========================================================
QueueHandle_t filaEventos;
SemaphoreHandle_t mutexDados;
SemaphoreHandle_t mutexBlynk;

// ==========================================================
// Prototipos
// ==========================================================
void taskSensores(void *pvParameters);
void taskFSM(void *pvParameters);
void taskInterface(void *pvParameters);
void taskBlynk(void *pvParameters);

void inicializarMatrizTransicao();
void publicarEvento(EventoSistema evento);
void executarAcao(AcaoSistema acao);
void atualizarSensores();
void atualizarLCD();
void atualizarBlynk();

int lerMediaADC(int pino);
int converterUmidadeParaPercentual(int adc);
int converterNivelParaPercentual(int adc);

void ligarBomba();
void desligarBomba();

const char* nomeEstado(EstadoSistema estado);
const char* nomeEvento(EventoSistema evento);
const char* nomeAcao(AcaoSistema acao);

void setStatus(const char *texto);

// ==========================================================
// Funcoes Blynk
// ==========================================================
BLYNK_WRITE(VPIN_LIMITE_UMIDADE) {
  int valor = param.asInt();
  valor = constrain(valor, 0, 100);

  if (mutexDados != NULL && xSemaphoreTake(mutexDados, pdMS_TO_TICKS(50)) == pdTRUE) {
    limiteUmidade = valor;
    xSemaphoreGive(mutexDados);
  }

  Serial.print("[BLYNK] Novo limite de umidade: ");
  Serial.print(valor);
  Serial.println("%");
}

BLYNK_WRITE(VPIN_TEMPO_IRRIGACAO) {
  int valor = param.asInt();
  valor = constrain(valor, 1, 60);

  if (mutexDados != NULL && xSemaphoreTake(mutexDados, pdMS_TO_TICKS(50)) == pdTRUE) {
    tempoIrrigacao_s = valor;
    xSemaphoreGive(mutexDados);
  }

  Serial.print("[BLYNK] Novo tempo de irrigacao: ");
  Serial.print(valor);
  Serial.println(" s");
}

BLYNK_CONNECTED() {
  Blynk.syncVirtual(VPIN_LIMITE_UMIDADE);
  Blynk.syncVirtual(VPIN_TEMPO_IRRIGACAO);
}

// ==========================================================
// Setup
// ==========================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  // Relé desligado imediatamente
  digitalWrite(PINO_RELE, LOW);
  pinMode(PINO_RELE, OUTPUT);
  desligarBomba();

  pinMode(PINO_UMIDADE_AO, INPUT);
  pinMode(PINO_UMIDADE_DO, INPUT);
  pinMode(PINO_NIVEL, INPUT);

  analogReadResolution(12);
  analogSetPinAttenuation(PINO_UMIDADE_AO, ADC_11db);
  analogSetPinAttenuation(PINO_NIVEL, ADC_11db);

  Wire.begin(PINO_SDA, PINO_SCL);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Inicializando");
  lcd.setCursor(0, 1);
  lcd.print("FreeRTOS");

  // Objetos RTOS
  filaEventos = xQueueCreate(15, sizeof(EventoMsg));
  mutexDados = xSemaphoreCreateMutex();
  mutexBlynk = xSemaphoreCreateMutex();

  if (filaEventos == NULL || mutexDados == NULL || mutexBlynk == NULL) {
    Serial.println("[ERRO] Falha ao criar objetos FreeRTOS.");
    while (true) {
      delay(1000);
    }
  }

  inicializarMatrizTransicao();

  // Wi-Fi e Blynk
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  Blynk.config(BLYNK_AUTH_TOKEN);

  // Tarefas FreeRTOS
  xTaskCreatePinnedToCore(
    taskFSM,
    "Task_FSM",
    4096,
    NULL,
    3,
    NULL,
    1
  );

  xTaskCreatePinnedToCore(
    taskSensores,
    "Task_Sensores",
    4096,
    NULL,
    2,
    NULL,
    1
  );

  xTaskCreatePinnedToCore(
    taskInterface,
    "Task_Interface",
    4096,
    NULL,
    1,
    NULL,
    1
  );

  xTaskCreatePinnedToCore(
    taskBlynk,
    "Task_Blynk",
    8192,
    NULL,
    2,
    NULL,
    0
  );

  publicarEvento(EV_INIT_OK);
}

// ==========================================================
// Loop principal
// Com FreeRTOS, o loop fica vazio.
// ==========================================================
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}

// ==========================================================
// Inicializacao da matriz de transicao
// ==========================================================
void inicializarMatrizTransicao() {
  for (int s = 0; s < ST_COUNT; s++) {
    for (int e = 0; e < EV_COUNT; e++) {
      matrizTransicao[s][e].proximoEstado = (EstadoSistema)s;
      matrizTransicao[s][e].acao = AC_NENHUMA;
    }
  }

  // Inicializacao
  matrizTransicao[ST_INICIALIZANDO][EV_INIT_OK] = {
    ST_MONITORANDO,
    AC_ENTRAR_MONITORAMENTO
  };

  // Monitorando
  matrizTransicao[ST_MONITORANDO][EV_UMIDADE_OK] = {
    ST_MONITORANDO,
    AC_ENTRAR_MONITORAMENTO
  };

  matrizTransicao[ST_MONITORANDO][EV_SOLO_SECO] = {
    ST_VERIFICANDO_RESERVATORIO,
    AC_VERIFICAR_RESERVATORIO
  };

  matrizTransicao[ST_MONITORANDO][EV_SOLO_SECO_INTERVALO] = {
    ST_AGUARDANDO_INTERVALO,
    AC_AGUARDAR_INTERVALO
  };

  // Verificando reservatorio
  matrizTransicao[ST_VERIFICANDO_RESERVATORIO][EV_RESERVATORIO_COM_AGUA] = {
    ST_IRRIGANDO,
    AC_INICIAR_IRRIGACAO
  };

  matrizTransicao[ST_VERIFICANDO_RESERVATORIO][EV_RESERVATORIO_VAZIO] = {
    ST_BLOQUEADO_FALTA_AGUA,
    AC_BLOQUEAR_FALTA_AGUA
  };

  // Irrigando
  matrizTransicao[ST_IRRIGANDO][EV_TEMPO_IRRIGACAO_FIM] = {
    ST_MONITORANDO,
    AC_FINALIZAR_IRRIGACAO
  };

  matrizTransicao[ST_IRRIGANDO][EV_RESERVATORIO_VAZIO] = {
    ST_BLOQUEADO_FALTA_AGUA,
    AC_BLOQUEAR_FALTA_AGUA
  };

  // Bloqueado por falta de agua
  matrizTransicao[ST_BLOQUEADO_FALTA_AGUA][EV_RESERVATORIO_COM_AGUA] = {
    ST_MONITORANDO,
    AC_ENTRAR_MONITORAMENTO
  };

  matrizTransicao[ST_BLOQUEADO_FALTA_AGUA][EV_RESERVATORIO_VAZIO] = {
    ST_BLOQUEADO_FALTA_AGUA,
    AC_BLOQUEAR_FALTA_AGUA
  };

  // Aguardando intervalo minimo
  matrizTransicao[ST_AGUARDANDO_INTERVALO][EV_INTERVALO_FIM] = {
    ST_VERIFICANDO_RESERVATORIO,
    AC_VERIFICAR_RESERVATORIO
  };

  matrizTransicao[ST_AGUARDANDO_INTERVALO][EV_UMIDADE_OK] = {
    ST_MONITORANDO,
    AC_ENTRAR_MONITORAMENTO
  };
}

// ==========================================================
// Publicacao de eventos
// ==========================================================
void publicarEvento(EventoSistema evento) {
  if (filaEventos == NULL) return;

  EventoMsg msg;
  msg.evento = evento;
  msg.instante_ms = millis();

  xQueueSend(filaEventos, &msg, 0);
}

// ==========================================================
// Task da FSM
// Consome eventos da fila e aplica a matriz de transicao.
// ==========================================================
void taskFSM(void *pvParameters) {
  EventoMsg msg;

  while (true) {
    if (xQueueReceive(filaEventos, &msg, portMAX_DELAY) == pdTRUE) {
      EstadoSistema estadoAnterior;
      EstadoSistema proximoEstado;
      AcaoSistema acao;

      if (xSemaphoreTake(mutexDados, portMAX_DELAY) == pdTRUE) {
        estadoAnterior = estadoAtual;

        Transicao t = matrizTransicao[estadoAtual][msg.evento];
        proximoEstado = t.proximoEstado;
        acao = t.acao;

        estadoAtual = proximoEstado;

        xSemaphoreGive(mutexDados);
      }

      if (acao != AC_NENHUMA) {
        Serial.print("[FSM] ");
        Serial.print(nomeEstado(estadoAnterior));
        Serial.print(" + ");
        Serial.print(nomeEvento(msg.evento));
        Serial.print(" -> ");
        Serial.print(nomeEstado(proximoEstado));
        Serial.print(" | Acao: ");
        Serial.println(nomeAcao(acao));

        executarAcao(acao);
      }
    }
  }
}

// ==========================================================
// Task dos sensores
// Le sensores e gera eventos para a FSM.
// ==========================================================
void taskSensores(void *pvParameters) {
  while (true) {
    atualizarSensores();

    EstadoSistema estado;
    int umidade;
    int limite;
    bool agua;
    unsigned long ultimaIrrigacao;
    unsigned long inicioIrrigacao;
    unsigned long tempoIrrigacao;

    if (xSemaphoreTake(mutexDados, portMAX_DELAY) == pdTRUE) {
      estado = estadoAtual;
      umidade = umidadePercentual;
      limite = limiteUmidade;
      agua = reservatorioComAgua;
      ultimaIrrigacao = instanteUltimaIrrigacao;
      inicioIrrigacao = instanteInicioIrrigacao;
      tempoIrrigacao = tempoIrrigacao_s;
      xSemaphoreGive(mutexDados);
    }

    unsigned long agora = millis();

    switch (estado) {
      case ST_MONITORANDO:
        if (umidade < limite) {
          if (agora - ultimaIrrigacao >= INTERVALO_MINIMO_ENTRE_IRRIGACOES_MS) {
            publicarEvento(EV_SOLO_SECO);
          } else {
            publicarEvento(EV_SOLO_SECO_INTERVALO);
          }
        } else {
          publicarEvento(EV_UMIDADE_OK);
        }
        break;

      case ST_VERIFICANDO_RESERVATORIO:
        if (agua) {
          publicarEvento(EV_RESERVATORIO_COM_AGUA);
        } else {
          publicarEvento(EV_RESERVATORIO_VAZIO);
        }
        break;

      case ST_IRRIGANDO:
        if (!agua) {
          publicarEvento(EV_RESERVATORIO_VAZIO);
        } else if (agora - inicioIrrigacao >= tempoIrrigacao * 1000UL) {
          publicarEvento(EV_TEMPO_IRRIGACAO_FIM);
        }
        break;

      case ST_BLOQUEADO_FALTA_AGUA:
        if (agua) {
          publicarEvento(EV_RESERVATORIO_COM_AGUA);
        } else {
          publicarEvento(EV_RESERVATORIO_VAZIO);
        }
        break;

      case ST_AGUARDANDO_INTERVALO:
        if (umidade >= limite) {
          publicarEvento(EV_UMIDADE_OK);
        } else if (agora - ultimaIrrigacao >= INTERVALO_MINIMO_ENTRE_IRRIGACOES_MS) {
          publicarEvento(EV_INTERVALO_FIM);
        }
        break;

      case ST_INICIALIZANDO:
      default:
        break;
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// ==========================================================
// Task de interface local e remota
// Atualiza LCD e envia dados para o Blynk.
// ==========================================================
void taskInterface(void *pvParameters) {
  while (true) {
    atualizarLCD();
    atualizarBlynk();

    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

// ==========================================================
// Task de conexao Blynk/Wi-Fi
// Mantem a comunicacao sem bloquear a FSM.
// ==========================================================
void taskBlynk(void *pvParameters) {
  unsigned long ultimaTentativaWiFi = 0;
  unsigned long ultimaTentativaBlynk = 0;

  while (true) {
    unsigned long agora = millis();

    if (WiFi.status() != WL_CONNECTED) {
      if (agora - ultimaTentativaWiFi >= 10000) {
        ultimaTentativaWiFi = agora;
        Serial.println("[WiFi] Tentando reconectar...");
        WiFi.disconnect();
        WiFi.begin(ssid, pass);
      }
    } else {
      if (!Blynk.connected()) {
        if (agora - ultimaTentativaBlynk >= 10000) {
          ultimaTentativaBlynk = agora;
          Serial.println("[Blynk] Tentando conectar...");

          if (xSemaphoreTake(mutexBlynk, pdMS_TO_TICKS(1000)) == pdTRUE) {
            Blynk.connect(1000);
            xSemaphoreGive(mutexBlynk);
          }
        }
      } else {
        if (xSemaphoreTake(mutexBlynk, pdMS_TO_TICKS(50)) == pdTRUE) {
          Blynk.run();
          xSemaphoreGive(mutexBlynk);
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// ==========================================================
// Leitura dos sensores
// ==========================================================
void atualizarSensores() {
  int adcU = lerMediaADC(PINO_UMIDADE_AO);
  int adcN = lerMediaADC(PINO_NIVEL);

  bool d0Seco = digitalRead(PINO_UMIDADE_DO) == HIGH;

  int umidade = converterUmidadeParaPercentual(adcU);
  int nivel = converterNivelParaPercentual(adcN);

  bool agua = adcN >= LIMIAR_NIVEL_COM_AGUA;

  EstadoSistema estadoCopia;
  bool bombaCopia;

  if (xSemaphoreTake(mutexDados, portMAX_DELAY) == pdTRUE) {
    adcUmidade = adcU;
    adcNivel = adcN;
    soloSecoDigital = d0Seco;
    umidadePercentual = umidade;
    nivelPercentual = nivel;
    reservatorioComAgua = agua;

    estadoCopia = estadoAtual;
    bombaCopia = bombaLigada;

    xSemaphoreGive(mutexDados);
  }

  Serial.print("[SENSORES] ADC Umidade: ");
  Serial.print(adcU);
  Serial.print(" | Umidade: ");
  Serial.print(umidade);
  Serial.print("%");

  Serial.print(" || D0: ");
  Serial.print(d0Seco ? "SECO" : "UMIDO");

  Serial.print(" || ADC Nivel: ");
  Serial.print(adcN);
  Serial.print(" | Nivel: ");
  Serial.print(nivel);
  Serial.print("%");

  Serial.print(" || Reservatorio: ");
  Serial.print(agua ? "COM AGUA" : "VAZIO");

  Serial.print(" || Bomba: ");
  Serial.print(bombaCopia ? "LIGADA" : "DESLIGADA");

  Serial.print(" || Estado: ");
  Serial.print(nomeEstado(estadoCopia));

  Serial.print(" || WiFi: ");
  Serial.print(WiFi.status() == WL_CONNECTED ? "OK" : "DESCONECTADO");

  Serial.print(" || Blynk: ");
  Serial.println(Blynk.connected() ? "OK" : "DESCONECTADO");
}

// ==========================================================
// Atualizacao do LCD
// ==========================================================
void atualizarLCD() {
  int umidade;
  int nivel;
  EstadoSistema estado;

  if (xSemaphoreTake(mutexDados, portMAX_DELAY) == pdTRUE) {
    umidade = umidadePercentual;
    nivel = nivelPercentual;
    estado = estadoAtual;
    xSemaphoreGive(mutexDados);
  }

  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("U:");
  lcd.print(umidade);
  lcd.print("% ");

  lcd.print("N:");
  lcd.print(nivel);
  lcd.print("%");

  lcd.setCursor(0, 1);

  switch (estado) {
    case ST_INICIALIZANDO:
      lcd.print("Iniciando");
      break;

    case ST_MONITORANDO:
      lcd.print("Monitorando");
      break;

    case ST_VERIFICANDO_RESERVATORIO:
      lcd.print("Verif. agua");
      break;

    case ST_IRRIGANDO:
      lcd.print("Irrigando");
      break;

    case ST_BLOQUEADO_FALTA_AGUA:
      lcd.print("Reserv vazio");
      break;

    case ST_AGUARDANDO_INTERVALO:
      lcd.print("Aguardando");
      break;

    default:
      lcd.print("Erro");
      break;
  }
}

// ==========================================================
// Atualizacao do Blynk
// ==========================================================
void atualizarBlynk() {
  if (!Blynk.connected()) return;

  int umidade;
  bool agua;
  bool bomba;
  char status[24];

  if (xSemaphoreTake(mutexDados, portMAX_DELAY) == pdTRUE) {
    umidade = umidadePercentual;
    agua = reservatorioComAgua;
    bomba = bombaLigada;
    strncpy(status, statusAtual, sizeof(status));
    status[sizeof(status) - 1] = '\0';
    xSemaphoreGive(mutexDados);
  }

  if (xSemaphoreTake(mutexBlynk, pdMS_TO_TICKS(100)) == pdTRUE) {
    Blynk.virtualWrite(VPIN_UMIDADE, umidade);
    Blynk.virtualWrite(VPIN_RESERVATORIO, agua ? 1 : 0);
    Blynk.virtualWrite(VPIN_BOMBA, bomba ? 1 : 0);
    Blynk.virtualWrite(VPIN_STATUS, status);

    xSemaphoreGive(mutexBlynk);
  }
}

// ==========================================================
// Execucao das acoes
// ==========================================================
void executarAcao(AcaoSistema acao) {
  switch (acao) {
    case AC_ENTRAR_MONITORAMENTO:
      desligarBomba();
      setStatus("Monitorando");
      break;

    case AC_VERIFICAR_RESERVATORIO:
      desligarBomba();
      setStatus("Verif. agua");
      break;

    case AC_INICIAR_IRRIGACAO:
      ligarBomba();

      if (xSemaphoreTake(mutexDados, portMAX_DELAY) == pdTRUE) {
        instanteInicioIrrigacao = millis();
        xSemaphoreGive(mutexDados);
      }

      setStatus("Irrigando");
      break;

    case AC_FINALIZAR_IRRIGACAO:
      desligarBomba();

      if (xSemaphoreTake(mutexDados, portMAX_DELAY) == pdTRUE) {
        instanteUltimaIrrigacao = millis();
        xSemaphoreGive(mutexDados);
      }

      setStatus("Monitorando");
      break;

    case AC_BLOQUEAR_FALTA_AGUA:
      desligarBomba();

      if (xSemaphoreTake(mutexDados, portMAX_DELAY) == pdTRUE) {
        instanteUltimaIrrigacao = millis();
        xSemaphoreGive(mutexDados);
      }

      setStatus("Sem agua");
      break;

    case AC_AGUARDAR_INTERVALO:
      desligarBomba();
      setStatus("Aguardando");
      break;

    case AC_NENHUMA:
    default:
      break;
  }
}

// ==========================================================
// Controle da bomba
// ==========================================================
void ligarBomba() {
  if (RELE_ATIVO_EM_LOW) {
    digitalWrite(PINO_RELE, LOW);
  } else {
    digitalWrite(PINO_RELE, HIGH);
  }

  if (xSemaphoreTake(mutexDados, portMAX_DELAY) == pdTRUE) {
    bombaLigada = true;
    xSemaphoreGive(mutexDados);
  }
}

void desligarBomba() {
  if (RELE_ATIVO_EM_LOW) {
    digitalWrite(PINO_RELE, HIGH);
  } else {
    digitalWrite(PINO_RELE, LOW);
  }

  if (mutexDados != NULL && xSemaphoreTake(mutexDados, pdMS_TO_TICKS(50)) == pdTRUE) {
    bombaLigada = false;
    xSemaphoreGive(mutexDados);
  }
}

// ==========================================================
// Funcoes auxiliares
// ==========================================================
int lerMediaADC(int pino) {
  long soma = 0;
  const int n = 10;

  for (int i = 0; i < n; i++) {
    soma += analogRead(pino);
    delay(2);
  }

  return soma / n;
}

int converterUmidadeParaPercentual(int adc) {
  int percentual = map(adc, ADC_SOLO_SECO, ADC_SOLO_UMIDO, 0, 100);
  return constrain(percentual, 0, 100);
}

int converterNivelParaPercentual(int adc) {
  int percentual = map(adc, ADC_NIVEL_VAZIO, ADC_NIVEL_CHEIO, 0, 100);
  return constrain(percentual, 0, 100);
}

void setStatus(const char *texto) {
  if (xSemaphoreTake(mutexDados, portMAX_DELAY) == pdTRUE) {
    strncpy(statusAtual, texto, sizeof(statusAtual) - 1);
    statusAtual[sizeof(statusAtual) - 1] = '\0';
    xSemaphoreGive(mutexDados);
  }
}

const char* nomeEstado(EstadoSistema estado) {
  switch (estado) {
    case ST_INICIALIZANDO:
      return "INICIALIZANDO";

    case ST_MONITORANDO:
      return "MONITORANDO";

    case ST_VERIFICANDO_RESERVATORIO:
      return "VERIFICANDO_RESERVATORIO";

    case ST_IRRIGANDO:
      return "IRRIGANDO";

    case ST_BLOQUEADO_FALTA_AGUA:
      return "BLOQUEADO_FALTA_AGUA";

    case ST_AGUARDANDO_INTERVALO:
      return "AGUARDANDO_INTERVALO";

    default:
      return "DESCONHECIDO";
  }
}

const char* nomeEvento(EventoSistema evento) {
  switch (evento) {
    case EV_INIT_OK:
      return "EV_INIT_OK";

    case EV_UMIDADE_OK:
      return "EV_UMIDADE_OK";

    case EV_SOLO_SECO:
      return "EV_SOLO_SECO";

    case EV_SOLO_SECO_INTERVALO:
      return "EV_SOLO_SECO_INTERVALO";

    case EV_RESERVATORIO_COM_AGUA:
      return "EV_RESERVATORIO_COM_AGUA";

    case EV_RESERVATORIO_VAZIO:
      return "EV_RESERVATORIO_VAZIO";

    case EV_TEMPO_IRRIGACAO_FIM:
      return "EV_TEMPO_IRRIGACAO_FIM";

    case EV_INTERVALO_FIM:
      return "EV_INTERVALO_FIM";

    default:
      return "EV_DESCONHECIDO";
  }
}

const char* nomeAcao(AcaoSistema acao) {
  switch (acao) {
    case AC_NENHUMA:
      return "AC_NENHUMA";

    case AC_ENTRAR_MONITORAMENTO:
      return "AC_ENTRAR_MONITORAMENTO";

    case AC_VERIFICAR_RESERVATORIO:
      return "AC_VERIFICAR_RESERVATORIO";

    case AC_INICIAR_IRRIGACAO:
      return "AC_INICIAR_IRRIGACAO";

    case AC_FINALIZAR_IRRIGACAO:
      return "AC_FINALIZAR_IRRIGACAO";

    case AC_BLOQUEAR_FALTA_AGUA:
      return "AC_BLOQUEAR_FALTA_AGUA";

    case AC_AGUARDAR_INTERVALO:
      return "AC_AGUARDAR_INTERVALO";

    default:
      return "AC_DESCONHECIDA";
  }
}
