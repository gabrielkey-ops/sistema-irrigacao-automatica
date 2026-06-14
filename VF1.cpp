/************************************************************
  Projeto: Irrigacao Automatica com ESP32 + Blynk
************************************************************/

#define BLYNK_TEMPLATE_ID   "TMPL2OefXAqXz"
#define BLYNK_TEMPLATE_NAME "Irrigação Automática"
#define BLYNK_AUTH_TOKEN    "_bH5JS-FwVtGwzE4M6LeV2TTgAC6NvPq"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ==========================================================
// Wi-Fi
// ==========================================================
char ssid[] = "Matsuda";
char pass[] = "keyyuzo2610!";

// ==========================================================
// Pinagem ESP32
// ==========================================================
const int PINO_UMIDADE_AO = 34;   // A0 do sensor de umidade
const int PINO_UMIDADE_DO = 32;   // D0 do sensor de umidade
const int PINO_NIVEL      = 35;   // A0/SIG do sensor de nivel
const int PINO_RELE       = 26;   // IN do rele

const int PINO_SDA = 21;
const int PINO_SCL = 22;

// Se o LCD nao funcionar, trocar 0x27 por 0x3F
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ==========================================================
// Blynk Virtual Pins
// ==========================================================
#define VPIN_UMIDADE          V0
#define VPIN_RESERVATORIO     V1
#define VPIN_BOMBA            V2
#define VPIN_STATUS           V3
#define VPIN_LIMITE_UMIDADE   V4
#define VPIN_TEMPO_IRRIGACAO  V5

BlynkTimer timer;

// ==========================================================
// Calibracao inicial
// Ajuste depois pelo Serial Monitor
// ==========================================================
int ADC_SOLO_SECO  = 4095;
int ADC_SOLO_UMIDO = 1350;

int ADC_NIVEL_VAZIO = 100;
int ADC_NIVEL_CHEIO = 2600;

int LIMIAR_NIVEL_COM_AGUA = 700;

// ==========================================================
// Rele
// ==========================================================
// Pelo seu teste, o rele esta ativo em HIGH.
// HIGH -> rele liga
// LOW  -> rele desliga
const bool RELE_ATIVO_EM_LOW = false;

// ==========================================================
// Parametros configuraveis pelo Blynk
// ==========================================================
int limiteUmidade = 40;              // %
unsigned long tempoIrrigacao_s = 5;  // segundos

const unsigned long INTERVALO_MINIMO_ENTRE_IRRIGACOES_MS = 30000;

// ==========================================================
// Variaveis do sistema
// ==========================================================
int adcUmidade = 0;
int adcNivel = 0;

int umidadePercentual = 0;
int nivelPercentual = 0;

bool soloSecoDigital = false;
bool reservatorioComAgua = false;
bool bombaLigada = false;

String mensagemAtual = "Inicializando";

unsigned long instanteInicioIrrigacao = 0;
unsigned long instanteUltimaIrrigacao = 0;
unsigned long ultimaTentativaBlynk = 0;

// ==========================================================
// Maquina de estados
// ==========================================================
enum EstadoSistema {
  INICIALIZANDO,
  MONITORANDO,
  VERIFICANDO_RESERVATORIO,
  IRRIGANDO,
  BLOQUEADO_FALTA_AGUA
};

EstadoSistema estadoAtual = INICIALIZANDO;

// ==========================================================
// Controle da bomba
// ==========================================================
void desligarBomba() {
  if (RELE_ATIVO_EM_LOW) {
    digitalWrite(PINO_RELE, HIGH);
  } else {
    digitalWrite(PINO_RELE, LOW);
  }

  bombaLigada = false;
}

void ligarBomba() {
  if (RELE_ATIVO_EM_LOW) {
    digitalWrite(PINO_RELE, LOW);
  } else {
    digitalWrite(PINO_RELE, HIGH);
  }

  bombaLigada = true;
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

const char* nomeEstado() {
  switch (estadoAtual) {
    case INICIALIZANDO:
      return "Inicializando";
    case MONITORANDO:
      return "Monitorando";
    case VERIFICANDO_RESERVATORIO:
      return "Verificando reservatorio";
    case IRRIGANDO:
      return "Irrigando";
    case BLOQUEADO_FALTA_AGUA:
      return "Bloqueado falta agua";
    default:
      return "Desconhecido";
  }
}

void definirMensagem(String novaMensagem) {
  if (novaMensagem != mensagemAtual) {
    mensagemAtual = novaMensagem;
    Serial.print("Mensagem: ");
    Serial.println(mensagemAtual);
  }
}

// ==========================================================
// Leitura dos sensores
// ==========================================================
void atualizarSensores() {
  adcUmidade = lerMediaADC(PINO_UMIDADE_AO);
  adcNivel = lerMediaADC(PINO_NIVEL);

  // Sensor C113:
  // D0 = HIGH -> solo seco
  // D0 = LOW  -> solo umido
  soloSecoDigital = digitalRead(PINO_UMIDADE_DO) == HIGH;

  umidadePercentual = converterUmidadeParaPercentual(adcUmidade);
  nivelPercentual = converterNivelParaPercentual(adcNivel);

  reservatorioComAgua = adcNivel >= LIMIAR_NIVEL_COM_AGUA;

  Serial.print("ADC Umidade: ");
  Serial.print(adcUmidade);
  Serial.print(" | Umidade: ");
  Serial.print(umidadePercentual);
  Serial.print("%");

  Serial.print(" || D0: ");
  Serial.print(soloSecoDigital ? "SECO" : "UMIDO");

  Serial.print(" || ADC Nivel: ");
  Serial.print(adcNivel);
  Serial.print(" | Nivel: ");
  Serial.print(nivelPercentual);
  Serial.print("%");

  Serial.print(" || Reservatorio: ");
  Serial.print(reservatorioComAgua ? "COM AGUA" : "VAZIO");

  Serial.print(" || Bomba: ");
  Serial.print(bombaLigada ? "LIGADA" : "DESLIGADA");

  Serial.print(" || Estado: ");
  Serial.print(nomeEstado());

  Serial.print(" || WiFi: ");
  Serial.print(WiFi.status() == WL_CONNECTED ? "OK" : "DESCONECTADO");

  Serial.print(" || Blynk: ");
  Serial.println(Blynk.connected() ? "OK" : "DESCONECTADO");
}

// ==========================================================
// Blynk nao bloqueante
// ==========================================================
void conectarBlynkSePossivel() {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  if (Blynk.connected()) {
    return;
  }

  unsigned long agora = millis();

  if (agora - ultimaTentativaBlynk >= 10000) {
    ultimaTentativaBlynk = agora;

    Serial.println("Tentando conectar ao Blynk...");
    Blynk.connect(3000);
  }
}

String statusCurtoBlynk() {
  switch (estadoAtual) {
    case INICIALIZANDO:
      return "Iniciando";

    case MONITORANDO:
      return "Monitorando";

    case VERIFICANDO_RESERVATORIO:
      return "Verif. agua";

    case IRRIGANDO:
      return "Irrigando";

    case BLOQUEADO_FALTA_AGUA:
      return "Sem agua";

    default:
      return "Erro";
  }
}

void atualizarBlynk() {
  if (!Blynk.connected()) {
    return;
  }

  // V0: Umidade em %
  Blynk.virtualWrite(VPIN_UMIDADE, umidadePercentual);

  // V1: Reservatorio
  // 1 = com agua, 0 = vazio
  Blynk.virtualWrite(VPIN_RESERVATORIO, reservatorioComAgua ? 1 : 0);

  // V2: Bomba
  // 1 = ligada, 0 = desligada
  Blynk.virtualWrite(VPIN_BOMBA, bombaLigada ? 1 : 0);

  // V3: Status curto para o LCD/Status do Blynk
  Blynk.virtualWrite(VPIN_STATUS, statusCurtoBlynk());
}
// ==========================================================
// LCD
// ==========================================================
void atualizarLCD() {
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("U:");
  lcd.print(umidadePercentual);
  lcd.print("% ");

  lcd.print("N:");
  lcd.print(nivelPercentual);
  lcd.print("%");

  lcd.setCursor(0, 1);

  if (estadoAtual == MONITORANDO) {
    lcd.print("Monitorando");
  } else if (estadoAtual == VERIFICANDO_RESERVATORIO) {
    lcd.print("Verif reserv.");
  } else if (estadoAtual == IRRIGANDO) {
    lcd.print("Irrigando");
  } else if (estadoAtual == BLOQUEADO_FALTA_AGUA) {
    lcd.print("Reserv vazio");
  } else {
    lcd.print("Inicializando");
  }
}

// ==========================================================
// Maquina de estados
// ==========================================================
void executarMaquinaDeEstados() {
  unsigned long agora = millis();

  switch (estadoAtual) {

    case INICIALIZANDO:
      desligarBomba();
      definirMensagem("Sistema em monitoramento");
      estadoAtual = MONITORANDO;
      break;

    case MONITORANDO:
      desligarBomba();

      if (umidadePercentual < limiteUmidade) {
        bool intervaloOk =
          agora - instanteUltimaIrrigacao >= INTERVALO_MINIMO_ENTRE_IRRIGACOES_MS;

        if (intervaloOk) {
          estadoAtual = VERIFICANDO_RESERVATORIO;
        } else {
          definirMensagem("Aguardando nova irrigacao");
        }
      } else {
        definirMensagem("Sistema em monitoramento");
      }
      break;

    case VERIFICANDO_RESERVATORIO:
      if (reservatorioComAgua) {
        ligarBomba();
        instanteInicioIrrigacao = agora;
        definirMensagem("Solo seco: iniciando irrigacao");
        estadoAtual = IRRIGANDO;
      } else {
        desligarBomba();
        definirMensagem("Reservatorio vazio");
        estadoAtual = BLOQUEADO_FALTA_AGUA;
      }
      break;

    case IRRIGANDO:
      if (!reservatorioComAgua) {
        desligarBomba();
        instanteUltimaIrrigacao = agora;
        definirMensagem("Reservatorio vazio: bomba desligada");
        estadoAtual = BLOQUEADO_FALTA_AGUA;
        break;
      }

      if (agora - instanteInicioIrrigacao >= tempoIrrigacao_s * 1000UL) {
        desligarBomba();
        instanteUltimaIrrigacao = agora;
        definirMensagem("Irrigacao finalizada");
        estadoAtual = MONITORANDO;
      }
      break;

    case BLOQUEADO_FALTA_AGUA:
      desligarBomba();

      if (reservatorioComAgua) {
        definirMensagem("Reservatorio reabastecido");
        estadoAtual = MONITORANDO;
      } else {
        definirMensagem("Reservatorio vazio");
      }
      break;
  }
}

// ==========================================================
// Tarefas periodicas
// ==========================================================
void tarefaControle() {
  atualizarSensores();
  executarMaquinaDeEstados();
}

void tarefaInterface() {
  atualizarLCD();
  atualizarBlynk();
}

// ==========================================================
// Recebimento de parametros pelo Blynk
// ==========================================================
BLYNK_WRITE(VPIN_LIMITE_UMIDADE) {
  int valor = param.asInt();

  if (valor >= 0 && valor <= 100) {
    limiteUmidade = valor;

    Serial.print("Novo limite de umidade: ");
    Serial.print(limiteUmidade);
    Serial.println("%");
  }
}

BLYNK_WRITE(VPIN_TEMPO_IRRIGACAO) {
  int valor = param.asInt();

  if (valor >= 1 && valor <= 60) {
    tempoIrrigacao_s = valor;

    Serial.print("Novo tempo de irrigacao: ");
    Serial.print(tempoIrrigacao_s);
    Serial.println(" s");
  }
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

  // Desliga o rele imediatamente no boot
  digitalWrite(PINO_RELE, LOW);
  pinMode(PINO_RELE, OUTPUT);
  desligarBomba();

  delay(500);

  pinMode(PINO_UMIDADE_AO, INPUT);
  pinMode(PINO_UMIDADE_DO, INPUT);
  pinMode(PINO_NIVEL, INPUT);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  Wire.begin(PINO_SDA, PINO_SCL);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Inicializando");
  lcd.setCursor(0, 1);
  lcd.print("ESP32 + Blynk");

  Serial.println("Iniciando Wi-Fi sem bloquear...");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);

  Blynk.config(BLYNK_AUTH_TOKEN);

  timer.setInterval(1000L, tarefaControle);
  timer.setInterval(2000L, tarefaInterface);
  timer.setInterval(5000L, conectarBlynkSePossivel);

  estadoAtual = INICIALIZANDO;
}

// ==========================================================
// Loop principal
// ==========================================================
void loop() {
  if (Blynk.connected()) {
    Blynk.run();
  }

  timer.run();
}
