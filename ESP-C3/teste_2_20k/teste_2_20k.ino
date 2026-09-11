// CONFIGURAÇÃO ESPECIAL PARA 20kHz COM SEU CIRCUITO
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Definições do BLE (mantidas iguais)
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// Configurações de PWM para 20kHz
const int pwmFreq = 20000;      // 20kHz
const int pwmResolution = 10;    // REDUZIDO para 10 bits (0-1023) por causa do slew rate
const int pwmChannel = 0;

// Ajuste de ganho para compensar perdas em alta frequência
const float highFreqGain = 1.8;  // Compensação para 20kHz (empírico)

// PID ajustado para 20kHz
float Kp = 0.6;          // Reduzido drasticamente
float Ki = 0.15;         // Reduzido
float Kd = 0.05;         // Adicionado termo derivativo

// Filtro anti-aliasing para leituras
class ExponentialFilter {
private:
  float alpha;
  float filteredValue;
  
public:
  ExponentialFilter(float alphaValue = 0.3) {
    alpha = alphaValue;
    filteredValue = 0;
  }
  
  float filter(float input) {
    filteredValue = (alpha * input) + ((1 - alpha) * filteredValue);
    return filteredValue;
  }
  
  void reset() {
    filteredValue = 0;
  }
};

ExponentialFilter currentFilter(0.2);  // Filtro mais suave para 20kHz
ExponentialFilter voltageFilter(0.15);

// Variáveis do sistema
BLEServer* pServer = NULL;
BLECharacteristic* pTxCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// Pinagem (mesma do seu circuito)
#define pwm_pin 8
#define saida_pulsada_pin 10
#define power_on_pin 26
#define power_pin 2
#define work_pin 7
#define lowbat_pin 3
#define corrente_pin 4
#define tensao_pin 1
#define bateria_pin 0

// Variáveis de controle
float corrente_mA = 0;
long int tempoAplicacao = 0;
bool ligado = false;
int modoOperacao = 0;  // 0=contínua, 1=pulsada, etc.
float correnteReferencia = 0;
float correnteMedida = 0;
float tensaoMedida = 0;
int pwmValue = 0;

// Timer para 20kHz
hw_timer_t* timer = NULL;
volatile bool timerFlag = false;
long int contadorLoop = 0;

// Interrupção do timer
void IRAM_ATTR onTimer() {
  timerFlag = true;
}

// Função para aplicar PWM com compensação de frequência
void aplicarPWM(int valor, bool pulsado) {
  // Aplicar ganho de compensação para 20kHz
  int compensatedValue = (int)(valor * highFreqGain);
  
  // Limitar ao máximo
  if(compensatedValue > 1023) compensatedValue = 1023;
  if(compensatedValue < 0) compensatedValue = 0;
  
  if(pulsado) {
    ledcWrite(1, compensatedValue);  // Canal para saída pulsada
    ledcWrite(0, 0);                  // Desliga constante
  } else {
    ledcWrite(0, compensatedValue);  // Canal para saída constante
    ledcWrite(1, 0);                  // Desliga pulsada
  }
}

void zerarPWM() {
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  digitalWrite(pwm_pin, HIGH);
}

// Controle PID para 20kHz
float controlePID(float referencia, float medida) {
  static float erroAnterior = 0;
  static float integral = 0;
  static float saidaAnterior = 0;
  
  float erro = referencia - medida;
  
  // Anti-windup com limite
  integral += erro * 0.01;  // Tempo de amostragem = 10ms
  if(integral > 1.0) integral = 1.0;
  if(integral < -1.0) integral = -1.0;
  
  float derivativo = (erro - erroAnterior) / 0.01;
  
  float saida = (Kp * erro) + (Ki * integral) + (Kd * derivativo);
  
  // Limitar saída
  if(saida > 1.0) saida = 1.0;
  if(saida < 0) saida = 0;
  
  // Suavizar mudanças bruscas
  saida = (saida * 0.7) + (saidaAnterior * 0.3);
  
  erroAnterior = erro;
  saidaAnterior = saida;
  
  return saida;
}

// Leitura suavizada das grandezas
void lerSensores() {
  // Múltiplas leituras para média
  float somaCorrente = 0;
  float somaTensao = 0;
  float somaBateria = 0;
  
  for(int i = 0; i < 5; i++) {
    somaCorrente += analogRead(corrente_pin);
    somaTensao += analogRead(tensao_pin);
    somaBateria += analogRead(bateria_pin);
    delayMicroseconds(100);  // Pequeno delay entre leituras
  }
  
  float correnteRaw = somaCorrente / 5.0;
  float tensaoRaw = somaTensao / 5.0;
  float bateriaRaw = somaBateria / 5.0;
  
  // Aplicar filtros exponenciais
  correnteMedida = currentFilter.filter(correnteRaw);
  tensaoMedida = voltageFilter.filter(tensaoRaw);
  
  // Converter para valores reais
  correnteMedida = (correnteMedida / 4095.0) * 1000.0;  // Convertendo para mA
  tensaoMedida = (tensaoMedida / 4095.0) * 3.3;        // Convertendo para Volts
}

// Verificação de erros adaptada para 20kHz
int verificarErros() {
  if(!ligado) return 0;
  
  // Limiares ajustados para 20kHz
  if(correnteMedida > correnteReferencia * 2.5 && pwmValue < 100) {
    return 3;  // Curto-circuito
  }
  
  if(correnteMedida < 0.05 && pwmValue > 500) {
    return 1;  // Circuito aberto
  }
  
  if(correnteMedida < correnteReferencia * 0.6 && pwmValue > 900) {
    return 2;  // Saturação
  }
  
  return 0;  // Normal
}

// Função principal de controle (chamada pelo timer)
void controlarSaida() {
  contadorLoop++;
  
  // Ler sensores a cada ciclo (20kHz = 50µs entre chamadas)
  lerSensores();
  
  if(ligado) {
    // Controlar a cada 100 ciclos (5ms) devido ao slew rate limitado
    if(contadorLoop % 100 == 0) {
      float controle = controlePID(correnteReferencia, correnteMedida);
      pwmValue = (int)(controle * 1023);
      
      // Determinar modo de saída baseado no tempo
      long tempoDecorrido = millis() - tempoInicio;
      bool usarPulsado = false;
      
      switch(modoOperacao) {
        case 0: usarPulsado = false; break;
        case 1: usarPulsado = true; break;
        case 2: usarPulsado = (tempoDecorrido > tempoAplicacao / 2); break;
        case 3: usarPulsado = (tempoDecorrido <= tempoAplicacao / 2); break;
        // ... outros modos
        default: usarPulsado = false;
      }
      
      aplicarPWM(pwmValue, usarPulsado);
      
      // Verificar erros
      int erro = verificarErros();
      
      // Enviar dados via BLE a cada 500 ciclos (25ms)
      if(contadorLoop % 500 == 0 && deviceConnected) {
        String dados = String(correnteMedida, 2) + " " + 
                      String(tensaoMedida, 2) + " " + 
                      String(erro);
        pTxCharacteristic->setValue(dados.c_str());
        pTxCharacteristic->notify();
      }
    }
  }
  
  // Reset contador para evitar overflow
  if(contadorLoop >= 10000) contadorLoop = 0;
}

void setup() {
  Serial.begin(115200);
  
  // Configurar pinos
  pinMode(pwm_pin, OUTPUT);
  pinMode(saida_pulsada_pin, OUTPUT);
  pinMode(work_pin, OUTPUT);
  pinMode(lowbat_pin, OUTPUT);
  pinMode(power_pin, OUTPUT);
  pinMode(power_on_pin, OUTPUT);
  
  // Configurar PWM para 20kHz com resolução reduzida
  ledcSetup(0, pwmFreq, pwmResolution);  // Canal para constante
  ledcAttachPin(pwm_pin, 0);
  
  ledcSetup(1, pwmFreq, pwmResolution);  // Canal para pulsada
  ledcAttachPin(saida_pulsada_pin, 1);
  
  // Inicializar timer para 20kHz
  timer = timerBegin(0, 80, true);  // Prescaler 80 (80MHz/80 = 1MHz)
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 50, true); // 50µs = 20kHz
  timerAlarmEnable(timer);
  
  // Inicializar BLE
  BLEDevice::init("TechDevice_20kHz");
  pServer = BLEDevice::createServer();
  // ... (código BLE igual ao seu original)
  
  Serial.println("Sistema 20kHz inicializado");
  Serial.print("Frequência PWM: "); Serial.print(pwmFreq); Serial.println(" Hz");
  Serial.print("Resolução PWM: "); Serial.print(pwmResolution); Serial.println(" bits");
}

long tempoInicio = 0;

void loop() {
  if(timerFlag) {
    timerFlag = false;
    controlarSaida();
  }
  
  // Gerenciar tempo total
  if(ligado && millis() - tempoInicio >= tempoAplicacao) {
    ligado = false;
    zerarPWM();
    digitalWrite(work_pin, LOW);
    Serial.println("Tempo esgotado");
  }
}