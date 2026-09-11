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

// --- PINAGEM (Mantida a sua configuração) ---
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
volatile bool timer_disparou = false; // FLAG IMPORTANTE
long int contador = 0;
long int contador2= 0;
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

// PID
double Kp = 2.0;         
double Ki = 1.5;         
float Erro_Anterior = 0;
float Controle_PWM = 0;
float Controle_Anterior = 0;
float Referencia = 0;    
float tensaoAtual = 0;
// Timer
hw_timer_t * timer = NULL;
int freq = 20000;   
int resolucao = 10;
// --- FUNÇÕES PWM ---
void aplicarPWM(int valor) {
  if (valor > 1023) valor = 1023;
  if (valor < 0) valor = 0; 
  ledcWrite(CANAL_Saida_Pulsada, valor);
  ledcWrite(CANAL_Saida_Constante, 0);
}

void aplicarPWM_Constante(int valor) {
  if (valor > 1023) valor = 1023;
  if (valor < 0) valor = 0;
  ledcWrite(CANAL_Saida_Constante, valor);
  ledcWrite(CANAL_Saida_Pulsada, 0);
}

void zerarPWM() {
  ledcWrite(CANAL_Saida_Pulsada, 0);
  ledcWrite(CANAL_Saida_Constante, 0);
  digitalWrite(pwm, HIGH); // Apaga LED invertido
}

// --- INTERRUPÇÃO LIMPA (SÓ LEVANTA A FLAG) ---
void IRAM_ATTR interrupt() {
  timer_disparou = true; // Avisa o loop que deu 10ms
}

// --- ROTINA DE CONTROLE (RODA NO LOOP QUANDO O TIMER MANDA) ---
void rodarControle() {
  contador++;

  // 1. Leituras Analógicas (Agora seguro, pois roda no loop context)
  float leitura_bat  = analogRead(bateria);
  float leitura_corr = analogRead(corrente);
  float leitura_tens = analogRead(tensao);

  // 2. Cálculo de Médias
  // Média Bateria (A cada 10 ciclos = 100ms)
  soma += leitura_bat; j++;
  if (j >= 10) { 
    TensaoBateria = soma / j; soma = 0; j = 0;
    
    // Gestão LED Bateria
    if (TensaoBateria > fraca) { digitalWrite(lowbat, LOW); toogle = false; } 
    else if (TensaoBateria > critica) { digitalWrite(lowbat, HIGH); toogle = false; }
    else { toogle = true; }
  }
  
  // Pisca LED Bateria Critica
  if (toogle && contador % 50 == 0) digitalWrite(lowbat, !digitalRead(lowbat));

  // Média Tensão Saída
  soma1 += leitura_tens; k++;
  if (k >= 20) { tensaosaida = soma1 / k; soma1 = 0; k = 0; }

  // Média Corrente
  soma2 += leitura_corr; l++;
  if (l >= 5) { correnteADC1 = soma2 / l; soma2 = 0; l = 0; }

  // 3. Lógica PID (Só roda se estiver ligado)
  if (ligado) {
    // PID roda a cada 100ms (contador % 10)
    if(contador % 10 == 0){
      
      correnteMedida = (correnteADC1) / 3847.0; 
      float Erro_Controle = Referencia - correnteMedida;

      // PID
      float Comp_Kp = Kp * (Erro_Controle - Erro_Anterior);
      float Comp_Ki = 0.05 * Ki * (Erro_Controle + Erro_Anterior);
      Controle_PWM = Comp_Kp + Comp_Ki + Controle_Anterior;

      // Anti-Windup
      if (Controle_PWM > 1.0) Controle_PWM = 1.0;
      if (Controle_PWM < 0.0) Controle_PWM = 0.0;

      Erro_Anterior = Erro_Controle;
      Controle_Anterior = Controle_PWM;

      Controle_Saida = (int)(1023.0 * Controle_PWM);
      
      // DEBUG SERIAL (Agora pode! Estamos fora da ISR)
      if (contador % 100 == 0){
        Serial.print("\nTensão: " + String(tensaoAtual) + " V | Medido: " + String(correnteMedida) + " mA | Referencia: " + String(Referencia) + " mA | Diferenca: " + String(Erro_Controle) +" A | PWM: " + String(Controle_Saida)+ "  Erro:  " + String(erro)); }

      // 4. Aplicação dos Modos de Saída
      float TempoDeAplicacaoSeg = TempoDeAplicacao * 0.001;
      float t = TempoDeAplicacaoSeg - TempoRestante; // Tempo decorrido em segundos

      switch (Valor_Opcao) {
        case 0: aplicarPWM_Constante(Controle_Saida); break;
        case 1: aplicarPWM(Controle_Saida); break;
        
        case 2: // Metade Contínua -> Pulsada
          if (t < 0.5 * TempoDeAplicacaoSeg) aplicarPWM_Constante(Controle_Saida);
          else aplicarPWM(Controle_Saida);
          break;

        case 3: // Metade Pulsada -> Contínua
          if (t < 0.5 * TempoDeAplicacaoSeg) aplicarPWM(Controle_Saida);
          else aplicarPWM_Constante(Controle_Saida);
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
           
        default: aplicarPWM_Constante(Controle_Saida); break;
      }

      // 5. Verificação de Erros e BLE
      if (contador % 30 == 0) { // ~300ms
         tensaoAtual = 0.00247*1.02545*(tensaosaida);     
         
         /*if (tensaoAtual <= 0.1) { erro = 3; google = true; } // Curto
         else if (correnteMedida == 0) { erro = 1; google = true; } // Aberto        
         else if (correnteMedida < Referencia && correnteMedida > 0.1 && Controle_Saida == 4095){erro = 2; google = true;}
         else { erro = 0; google = false; digitalWrite(power, HIGH); }
         */
          if (Valor_Opcao == 0) {
          // Modo contínuo: critério rígido de tensão
          if (tensaoAtual <= 0.1) { erro = 3; google = true; }
          else if (correnteMedida == 0 && Controle_Saida > 500) { erro = 1; google = true; }
          else if (correnteMedida < Referencia && correnteMedida > 0.1 && Controle_Saida == 1023) { erro = 2; google = true; }
          else { erro = 0; google = false; digitalWrite(power, HIGH); }
          } else {
          // Modos pulsados: detecta curto por corrente x PWM + fonte desligada
          bool curtoReal = (correnteMedida > (Referencia * 2.0) && Controle_Saida < 300);
          bool semAlimentacao = (tensaoAtual <= 0.1 && Controle_Saida > 600);

          if (curtoReal || semAlimentacao) { erro = 3; google = true; }
          else if (correnteMedida == 0 && Controle_Saida > 500) { erro = 1; google = true; }
          else if (correnteMedida < Referencia && correnteMedida > 0.05 && Controle_Saida == 1023) { erro = 2; google = true; }
          else { erro = 0; google = false; digitalWrite(power, HIGH); }
          } 
            
         if (erro != erro1) { digitalWrite(work, LOW); contador2 = 0;}
         erro1 = erro;

         if (erro ==0) {digitalWrite(work, HIGH);}
         if (erro == 3 && contador % 20 == 0) {digitalWrite (work, !digitalRead(work));}

         if (google && erro !=3 && contador % 50 == 0) {
          if (contador2< 2){if (digitalRead(work)) digitalWrite (work,LOW);}
          if (!digitalRead(work)){ contador2++;}
          if (contador2 > 2 && contador2 <= (erro + 2)){digitalWrite(work, !digitalRead(work));}
          if (contador2 > (erro +2)){ digitalWrite(work,LOW);contador2 = 0;}}
         
         // ENVIO BLE (Agora seguro!)
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
          
          if (posI != -1) Valor_Opcao = (cmd.substring(posI+2, posI+3)).toInt();
          
          if (posC != -1 && posT != -1) {
             String correnteStr = cmd.substring(posC + 1, posT);
             corrente_mA = correnteStr.toInt() / 1.0; 
             Referencia = corrente_mA / 1000.0;
             controle = 0;
             
             int posDoisPontos = cmd.indexOf(':', posT);
             if (posDoisPontos != -1) {
               String horasStr = cmd.substring(posT + 1, posDoisPontos);
               String minutosStr = cmd.substring(posDoisPontos + 1, cmd.indexOf('#', posDoisPontos));
               // Conversão correta para milissegundos
               TempoDeAplicacao = (horasStr.toInt() * 60000L) + (minutosStr.toInt() * 1000L); // trocar depois 3600000L e 60000L
               Serial.println("Configurado: " + String(TempoDeAplicacao));
             }
          }
        }
        else if (cmd.startsWith("#T#K2")) { // START
           Serial.println("START");
           if (TempoDeAplicacao == 0) return;
           TempoPassado = millis();
           Erro_Anterior = 0; Controle_Anterior = 0; Controle_PWM = 0;
           ligado = true; google = false; erro = 0;
           digitalWrite(work, HIGH); digitalWrite(power, HIGH);
        }
        else if (cmd.startsWith("#T#K4")) { // STOP
           Serial.println("STOP");
           ligado = false;
           zerarPWM();
           digitalWrite(work, LOW); digitalWrite(power, HIGH);
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
  
  pinMode(work, OUTPUT); pinMode(lowbat, OUTPUT); pinMode(power, OUTPUT);
  pinMode(pwm, OUTPUT); pinMode(Saida_Pulsada, OUTPUT); pinMode(power_on, OUTPUT);
  pinMode(tensao, INPUT); pinMode(bateria, INPUT); pinMode(corrente, INPUT);

  // Inicializa Estados
  digitalWrite(Saida_Pulsada, LOW); 
  digitalWrite(work, LOW);
  digitalWrite(power, LOW); 
  digitalWrite(lowbat, LOW);
//  digitalWrite(power_on, HIGH);
//  digitalWrite(pwm, HIGH); // Apagado
  digitalWrite(lowbat, HIGH); // Apagado
  delay(200);
  digitalWrite(lowbat, LOW); // Apagado
  digitalWrite(work, HIGH); // Apagado
  delay(200);
  digitalWrite(work, LOW); // Apagado
  digitalWrite(power, HIGH); // Apagado
  delay(200);
  digitalWrite(power, LOW); // Apagado
  digitalWrite(work, HIGH); // Apagado
  delay(200);
  digitalWrite(work, LOW); // Apagado
  digitalWrite(lowbat, HIGH); // Apagado
  delay(200);
  digitalWrite(lowbat, LOW); // Apagado
  digitalWrite(work, HIGH); // Apagado
  delay(200);
  digitalWrite(work, LOW); // Apagado
  digitalWrite(power, HIGH); // Apagado
  delay(200);
  digitalWrite(power, LOW); // Apagado
  digitalWrite(work, HIGH); // Apagado
  delay(200);
  digitalWrite(work, LOW); // Apagado
  digitalWrite(lowbat, HIGH); // Apagado
  delay(200);
  digitalWrite(lowbat, LOW); // Apagado
  digitalWrite(power, HIGH); // Apagado
//  digitalWrite(power, HIGH);    // Led que indica dispositivo ligado
//  digitalWrite(work, HIGH);      // Led que indica dispositivo aplicando medicamento
//  digitalWrite(lowbat, HIGH);    // Led que indica bateria fraca
//  delay(500);
//  digitalWrite(power, LOW);    // Led que indica dispositivo ligado
//  digitalWrite(work, LOW);      // Led que indica dispositivo aplicando medicamento
//  digitalWrite(lowbat, LOW);    // Led que indica bateria fraca
//  delay(500);
  // Config PWM
  ledcSetup(CANAL_Saida_Pulsada, freq, resolucao);
  ledcAttachPin(Saida_Pulsada, CANAL_Saida_Pulsada);
  ledcWrite(CANAL_Saida_Pulsada, 0);

  ledcSetup(CANAL_Saida_Constante, freq, resolucao);
  // Nota: Você precisa de um pino físico para o canal constante se for usar
  // Se for o mesmo pino, a lógica precisa mudar. Assumi que está configurado como no original.
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
  
  // Pisca Power indicando pronto
  digitalWrite(power, HIGH);
}

void loop() {
  // --- AQUI ESTÁ A MÁGICA ---
  // Se o Timer (interrupt) avisou que passou 10ms, rodamos o controle AQUI.
  if (timer_disparou) {
    timer_disparou = false; // Reseta o aviso
    rodarControle();        // Executa PID, BLE, Serial com segurança
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
      digitalWrite(work, LOW); digitalWrite(power, HIGH);
      if (deviceConnected) {
        pTxCharacteristic->setValue("0.00 0.00 4");
        pTxCharacteristic->notify();
      }
    }
  }
}