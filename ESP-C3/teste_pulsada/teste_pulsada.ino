#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Definições do BLE
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer* pServer = NULL;
BLECharacteristic* pTxCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// --- PINAGEM ---
#define pwm 8              // LED Integrado
#define Saida_Pulsada 10   // Saída PWM
#define power_on 26        // Pino alterado (Correto, evite o 20)
#define power 2
#define work 7
#define lowbat 3
#define corrente 4
#define tensao  1
#define bateria 0

#define CANAL_Saida_Constante      0
#define CANAL_Saida_Pulsada        1

// Variáveis de configuração
int controle = 0;
float corrente_mA = 0.0;
long int TempoDeAplicacao = 0;
bool ligado = false;
int Valor_Opcao = 0;

// Variáveis de controle
float correnteADC1 = 0;
float tensaosaida = 0;
float TensaoBateria = 0;
long int TempoPassado = 0;
long int TempoRestante = 0;

// Variáveis do sistema
volatile bool timer_disparou = false;
long int contador = 0;
long int contador2 = 0;
int erro = 0;
int erro1 = 0;
bool toogle = false;
bool google = false;
int fraca = 3470;
int critica = 2900;
float correnteMedida = 0;
int Controle_Saida = 0;

// Médias
float soma = 0;      
float soma1 = 0;
float soma2 = 0;
int j = 0, k = 0, l = 0;

// PID - Ganhos separados para modo contínuo e pulsado
double Kp_continuo = 2.0;         
double Ki_continuo = 1.5;
double Kp_pulsado = 3.5;           // Aumentado para compensar a perda
double Ki_pulsado = 2.5;           // Aumentado para resposta mais rápida

double Kp_atual;
double Ki_atual;

float Erro_Anterior = 0;
float Controle_PWM = 0;
float Controle_Anterior = 0;
float Referencia = 0;    
float tensaoAtual = 0;

// Timer
hw_timer_t * timer = NULL;
int freq = 1000;   
int resolucao = 12;

// Fator de compensação para modo pulsado (ajuste fino)
float fatorCompensacaoPulsado = 1.45;  // Ajuste este valor conforme necessidade

// --- FUNÇÕES PWM ---
void aplicarPWM(int valor) {
  if (valor > 4095) valor = 4095;
  if (valor < 0) valor = 0; 
  ledcWrite(CANAL_Saida_Pulsada, valor);
  ledcWrite(CANAL_Saida_Constante, 0);
}

void aplicarPWM_Constante(int valor) {
  if (valor > 4095) valor = 4095;
  if (valor < 0) valor = 0;
  ledcWrite(CANAL_Saida_Constante, valor);
  ledcWrite(CANAL_Saida_Pulsada, 0);
}

void zerarPWM() {
  ledcWrite(CANAL_Saida_Pulsada, 0);
  ledcWrite(CANAL_Saida_Constante, 0);
  digitalWrite(pwm, HIGH);
}

// --- INTERRUPÇÃO ---
void IRAM_ATTR interrupt() {
  timer_disparou = true;
}

// --- CÁLCULO DE CORRENTE CORRIGIDO PARA MODO PULSADO ---
float calcularCorrenteMedia(float leituraADC, int modo) {
  float correnteInstantanea = leituraADC / 3847.0;
  
  if (modo == 0) {
    // Modo contínuo: leitura direta
    return correnteInstantanea;
  } else {
    // Modo pulsado: precisa compensar pelo duty cycle
    // Quanto menor o PWM, maior a compensação necessária
    float dutyCycle = (float)Controle_Saida / 4095.0;
    if (dutyCycle < 0.05) dutyCycle = 0.05;  // Evitar divisão por zero
    
    // Fórmula de compensação para modo pulsado
    // A corrente média eficaz é menor que o pico, precisa multiplicar pelo fator
    float fatorCompensacao = fatorCompensacaoPulsado / dutyCycle;
    if (fatorCompensacao > 3.0) fatorCompensacao = 3.0;  // Limitar compensação
    
    return correnteInstantanea * fatorCompensacao;
  }
}

// --- ROTINA DE CONTROLE ---
void rodarControle() {
  contador++;

  // 1. Leituras Analógicas
  float leitura_bat  = analogRead(bateria);
  float leitura_corr = analogRead(corrente);
  float leitura_tens = analogRead(tensao);

  // 2. Cálculo de Médias
  soma += leitura_bat; j++;
  if (j >= 10) { 
    TensaoBateria = soma / j; soma = 0; j = 0;
    if (TensaoBateria > fraca) { digitalWrite(lowbat, LOW); toogle = false; } 
    else if (TensaoBateria > critica) { digitalWrite(lowbat, HIGH); toogle = false; }
    else { toogle = true; }
  }
  
  if (toogle && contador % 50 == 0) digitalWrite(lowbat, !digitalRead(lowbat));

  soma1 += leitura_tens; k++;
  if (k >= 20) { tensaosaida = soma1 / k; soma1 = 0; k = 0; }

  soma2 += leitura_corr; l++;
  if (l >= 5) { 
    correnteADC1 = soma2 / l; 
    soma2 = 0; 
    l = 0; 
    
    // Determinar modo atual para cálculo da corrente
    int modoAtual = (Valor_Opcao == 0) ? 0 : 1;
    
    // Calcular corrente com compensação se necessário
    if (ligado) {
      correnteMedida = calcularCorrenteMedia(correnteADC1, modoAtual);
    } else {
      correnteMedida = correnteADC1 / 3847.0;
    }
  }

  // 3. Lógica PID
  if (ligado) {
    if(contador % 10 == 0){
      
      float Erro_Controle = Referencia - correnteMedida;
      
      // Usar ganhos conforme o modo atual
      if (Valor_Opcao == 0) {
        Kp_atual = Kp_continuo;
        Ki_atual = Ki_continuo;
      } else {
        Kp_atual = Kp_pulsado;
        Ki_atual = Ki_pulsado;
      }
      
      // PID com ganhos ajustados
      float Comp_Kp = Kp_atual * (Erro_Controle - Erro_Anterior);
      float Comp_Ki = 0.05 * Ki_atual * (Erro_Controle + Erro_Anterior);
      Controle_PWM = Comp_Kp + Comp_Ki + Controle_Anterior;

      // Anti-Windup
      if (Controle_PWM > 1.0) Controle_PWM = 1.0;
      if (Controle_PWM < 0.0) Controle_PWM = 0.0;

      Erro_Anterior = Erro_Controle;
      Controle_Anterior = Controle_PWM;

      Controle_Saida = (int)(4095.0 * Controle_PWM);
      
      // DEBUG
      if (contador % 100 == 0){
        Serial.print("\nModo: " + String(Valor_Opcao == 0 ? "Contínuo" : "Pulsado"));
        Serial.print(" | Ref: " + String(Referencia * 1000, 2) + " mA");
        Serial.print(" | Medido: " + String(correnteMedida * 1000, 2) + " mA");
        Serial.print(" | PWM: " + String(Controle_Saida));
        Serial.print(" | Erro: " + String(Erro_Controle * 1000, 2) + " mA");
        Serial.println();
      }

      // 4. Aplicação dos Modos de Saída
      float TempoDeAplicacaoSeg = TempoDeAplicacao * 0.001;
      float t = TempoDeAplicacaoSeg - TempoRestante;

      switch (Valor_Opcao) {
        case 0: 
          aplicarPWM_Constante(Controle_Saida); 
          break;
        case 1: 
          aplicarPWM(Controle_Saida); 
          break;
        case 2: // Metade Contínua -> Pulsada
          if (t < 0.5 * TempoDeAplicacaoSeg) {
            aplicarPWM_Constante(Controle_Saida);
          } else {
            aplicarPWM(Controle_Saida);
          }
          break;
        case 3: // Metade Pulsada -> Contínua
          if (t < 0.5 * TempoDeAplicacaoSeg) {
            aplicarPWM(Controle_Saida);
          } else {
            aplicarPWM_Constante(Controle_Saida);
          }
          break;
        case 4: // Misto A
          if ((t < 0.25 * TempoDeAplicacaoSeg) || (t >= 0.5 * TempoDeAplicacaoSeg && t < 0.75 * TempoDeAplicacaoSeg))
            aplicarPWM_Constante(Controle_Saida);
          else 
            aplicarPWM(Controle_Saida);
          break;
        case 5: // Misto B
          if ((t < 0.25 * TempoDeAplicacaoSeg) || (t >= 0.5 * TempoDeAplicacaoSeg && t < 0.75 * TempoDeAplicacaoSeg))
            aplicarPWM(Controle_Saida);
          else 
            aplicarPWM_Constante(Controle_Saida);
          break;
        default: 
          aplicarPWM_Constante(Controle_Saida); 
          break;
      }

      // 5. Verificação de Erros
      if (contador % 30 == 0) {
         tensaoAtual = 0.00247 * 1.02545 * (tensaosaida);     
         
         if (Valor_Opcao == 0) {
          // Modo contínuo
          if (tensaoAtual <= 0.1) { erro = 3; google = true; }
          else if (correnteMedida == 0 && Controle_Saida > 500) { erro = 1; google = true; }
          else if (correnteMedida < Referencia && correnteMedida > 0.1 && Controle_Saida == 4095) { erro = 2; google = true; }
          else { erro = 0; google = false; digitalWrite(power, HIGH); }
         } else {
          // Modo pulsado - critérios ajustados
          bool curtoReal = (correnteMedida > (Referencia * 2.0) && Controle_Saida < 300);
          bool semAlimentacao = (tensaoAtual <= 0.1 && Controle_Saida > 2500);

          if (curtoReal || semAlimentacao) { erro = 3; google = true; }
          else if (correnteMedida == 0 && Controle_Saida > 500) { erro = 1; google = true; }
          else if (correnteMedida < (Referencia * 0.7) && correnteMedida > 0.05 && Controle_Saida == 4095) { erro = 2; google = true; }
          else { erro = 0; google = false; digitalWrite(power, HIGH); }
         } 
            
         if (erro != erro1) { digitalWrite(work, LOW); contador2 = 0;}
         erro1 = erro;

         if (erro == 0) {digitalWrite(work, HIGH);}
         if (erro == 3 && contador % 20 == 0) {digitalWrite(work, !digitalRead(work));}

         if (google && erro != 3 && contador % 50 == 0) {
          if (contador2 < 2) {if (digitalRead(work)) digitalWrite(work, LOW);}
          if (!digitalRead(work)) { contador2++;}
          if (contador2 > 2 && contador2 <= (erro + 2)) {digitalWrite(work, !digitalRead(work));}
          if (contador2 > (erro + 2)) { digitalWrite(work, LOW); contador2 = 0;}
         }
         
         // ENVIO BLE
         if (deviceConnected && pTxCharacteristic != NULL) {
            String dados = String(correnteMedida * 1000, 2) + " " + String(tensaoAtual, 2) + " " + String(erro);
            pTxCharacteristic->setValue(dados.c_str());
            pTxCharacteristic->notify();
         }
      }
    }
  }
}

void startTimer() {
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &interrupt, true);
  timerAlarmWrite(timer, 10000, true); // 10ms
  timerAlarmEnable(timer);
}

// --- CALLBACKS BLE ---
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("✅ Cliente conectado!");
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("⚠️ Cliente desconectado!");
      pServer->getAdvertising()->start();
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();
      String cmd = String(rxValue.c_str());
      
      if (rxValue.length() > 0) {
        cmd.trim();
        Serial.print("RX: "); Serial.println(cmd);
        
        if (cmd.startsWith("#T#K1")) { // CONFIG
          int posC = cmd.indexOf('C');
          int posT = cmd.indexOf('T', posC + 1);
          int posI = cmd.indexOf('I', 2);
          
          if (posI != -1) {
            Valor_Opcao = (cmd.substring(posI + 2, posI + 3)).toInt();
            Serial.println("Modo selecionado: " + String(Valor_Opcao));
          }
          
          if (posC != -1 && posT != -1) {
             String correnteStr = cmd.substring(posC + 1, posT);
             corrente_mA = correnteStr.toInt() / 1.0; 
             Referencia = corrente_mA / 1000.0;
             controle = 0;
             
             // Resetar PID quando mudar a referência
             Erro_Anterior = 0;
             Controle_Anterior = 0;
             Controle_PWM = 0;
             
             int posDoisPontos = cmd.indexOf(':', posT);
             if (posDoisPontos != -1) {
               String horasStr = cmd.substring(posT + 1, posDoisPontos);
               String minutosStr = cmd.substring(posDoisPontos + 1, cmd.indexOf('#', posDoisPontos));
               TempoDeAplicacao = (horasStr.toInt() * 60000L) + (minutosStr.toInt() * 1000L);
               Serial.println("Tempo configurado: " + String(TempoDeAplicacao) + " ms");
             }
          }
        }
        else if (cmd.startsWith("#T#K2")) { // START
           Serial.println("START");
           if (TempoDeAplicacao == 0) return;
           TempoPassado = millis();
           Erro_Anterior = 0; 
           Controle_Anterior = 0; 
           Controle_PWM = 0;
           ligado = true; 
           google = false; 
           erro = 0;
           digitalWrite(work, HIGH); 
           digitalWrite(power, HIGH);
        }
        else if (cmd.startsWith("#T#K4")) { // STOP
           Serial.println("STOP");
           ligado = false;
           zerarPWM();
           digitalWrite(work, LOW); 
           digitalWrite(power, HIGH);
           if (deviceConnected) {
             pTxCharacteristic->setValue("0.00 0.00 5");
             pTxCharacteristic->notify();
           }
        }
      }
    }
};

void setup() {
  Serial.begin(115200);
  
  pinMode(work, OUTPUT); 
  pinMode(lowbat, OUTPUT); 
  pinMode(power, OUTPUT);
  pinMode(pwm, OUTPUT); 
  pinMode(Saida_Pulsada, OUTPUT); 
  pinMode(power_on, OUTPUT);
  pinMode(tensao, INPUT); 
  pinMode(bateria, INPUT); 
  pinMode(corrente, INPUT);

  // Inicializa Estados
  digitalWrite(Saida_Pulsada, LOW); 
  digitalWrite(work, LOW);
  digitalWrite(power, LOW); 
  digitalWrite(lowbat, LOW);
  
  // Config PWM
  ledcSetup(CANAL_Saida_Pulsada, freq, resolucao);
  ledcAttachPin(Saida_Pulsada, CANAL_Saida_Pulsada);
  ledcWrite(CANAL_Saida_Pulsada, 0);

  ledcSetup(CANAL_Saida_Constante, freq, resolucao);
  ledcAttachPin(pwm, CANAL_Saida_Constante); 

  // BLE
  BLEDevice::init("TechDevice");
  BLEDevice::setPower(ESP_PWR_LVL_P9);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());
  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
  pRxCharacteristic->setCallbacks(new MyCallbacks());
  pService->start();
  
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMaxPreferred(0x12);
  BLEDevice::setMTU(517);
  pAdvertising->start();
  
  Serial.println("Sistema Iniciado.");
  startTimer();
  
  digitalWrite(power, HIGH);
}

void loop() {
  if (timer_disparou) {
    timer_disparou = false;
    rodarControle();
  }

  // Gerenciamento de Conexão BLE
  if (!deviceConnected && oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }
  
  // Gestão de Tempo Total
  if (ligado) {
    long int Agora = millis();
    TempoRestante = (TempoDeAplicacao - (Agora - TempoPassado)) / 1000;

    if ((Agora - TempoPassado) >= TempoDeAplicacao) {
      Serial.println("=== TEMPO ESGOTADO ===");
      ligado = false;
      zerarPWM();
      digitalWrite(work, LOW); 
      digitalWrite(power, HIGH);
      if (deviceConnected) {
        pTxCharacteristic->setValue("0.00 0.00 4");
        pTxCharacteristic->notify();
      }
    }
  }
}