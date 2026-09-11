#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

// Definições BLE
#define SERVICE_UUID           "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_RX "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_TX "beb5483e-36e1-4688-b7f5-ea07361b26a9"

// Definições dos pinos
#define pwm 18
#define power_on 3
#define button_power 25
#define power 16
#define corrente 4
#define tensao 32
#define bateria 33
#define work 15
#define lowbat 2

// Variáveis BLE
BLECharacteristic *pCharacteristicTX;
BLECharacteristic *pCharacteristicRX;
bool deviceConnected = false;

// Variáveis de controle - MALHA ABERTA
String teste = "#T#K";
bool valido = false;
char horas[3];
char minutos[3];
int minuto;
int hora;
int controle;
char setpoint[4];
bool ligado;
float Controle_Saida = 0;  // Valor PWM fixo (0-4095)

// Variáveis de erro (adicionadas para monitoramento)
int erro = 0;           // 0=normal, 1=corrente zero, 2=alta impedância, 3=curto
int erro1 = 0;
bool google = false;
long int contador2 = 0;

// Constantes de calibração
const float ganho_cal = 1.028;
const float offset_cal = 0.033;

// Variáveis de temporização
long int contador = 0;
long int contador1 = 0;
int cont = 0;
long int TempoDeAplicacao = 0;
long int TempoAtual = 0;
long int TempoPassado = 0;
int freq = 1000;
int canal = 0;
int resolucao = 12;

// Variáveis de leitura de sensores
float correnteADC = 0;
float correnteADC1 = 0;
float tensaosaida = 0;
float tensaosaida0 = 0;  // Adicionado para detecção de erros
float TensaoBateria = 0;
int ler = 1;
int ler1 = 1;

// Médias para filtragem
int soma = 0;
int soma1 = 0;
int soma2 = 0;
int j = 0;
int k = 0;
int l = 0;

// Bateria
bool toogle = false;
int fraca = 2800;
int critica = 2700;
volatile bool enviarDados = false;

// Timer
hw_timer_t *timer = NULL;

// -------------------- DECLARAÇÃO ANTECIPADA DAS FUNÇÕES --------------------
void processReceivedData(String value);
void sendData(String data);
// -------------------------------------------------------------------------

// Classe para callback de conexão BLE
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("\n>>> BLE: dispositivo CONECTADO <<<");
    };

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("\n>>> BLE: dispositivo DESCONECTADO <<<");
        pServer->startAdvertising();
    }
};

// Classe para callback de recebimento de dados
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        
        if (value.length() > 0) {
            Serial.print("\nTexto recebido: ");
            Serial.println(value.c_str());
            
            processReceivedData(String(value.c_str()));
        }
    }
};

// Função para processar os dados recebidos
void processReceivedData(String value) {
    cont = 0;
    for (int i = 0; i <= 3; i++) {
        if (value[i] == teste[i]) {
            cont++;
        }
        if (cont == 4) {
            valido = true;
        }
    }

    if (valido) {
        Serial.print("\nMensagem valida");
        
        // Comando 1: Configurar tempo e corrente (setpoint)
        if (value[5] == '1') {
            horas[0] = value[14];
            horas[1] = value[15];
            minutos[0] = value[17];
            minutos[1] = value[18];

            setpoint[0] = value[8];
            setpoint[1] = value[10];
            setpoint[2] = value[11];

            hora = atoi(horas);
            minuto = atoi(minutos);
            TempoDeAplicacao = ((60 * hora) + minuto) * 60000;

            // MALHA ABERTA: Converte setpoint diretamente para PWM
            controle = atoi(setpoint);
            Controle_Saida = (controle * 4095.0) / 1000.0;
            
            if (Controle_Saida > 4095) Controle_Saida = 4095;
            if (Controle_Saida < 0) Controle_Saida = 0;
            
            Serial.print("\nTempo configurado: ");
            Serial.print(hora);
            Serial.print("h ");
            Serial.print(minuto);
            Serial.println("min");
            Serial.print("Setpoint: ");
            Serial.print(controle);
            Serial.print(" -> PWM FIXO: ");
            Serial.println((int)Controle_Saida);
        }
        
        // Comando 2: Ligar dispositivo
        else if (value[5] == '2') {
            TempoPassado = millis();
            ligado = true;
            Serial.println("\nDispositivo LIGADO - Malha Aberta");
            Serial.print("PWM aplicado: ");
            Serial.println((int)Controle_Saida);
        }
        
        // Comando 3: Ajuste de ganhos (IGNORADO em malha aberta)
        else if (value[5] == '3') {
            Serial.println("\nAVISO: Modo MALHA ABERTA - Ganhos PID ignorados!");
            sendData("Modo: MA | Corrente: 0.00 | PWM: 0 | Bateria: 0.00 | Erro: 0");
        }
        
        // Comando 4: Desligar dispositivo
        else if (value[5] == '4') {
            ligado = false;
            ledcWrite(canal, 0);
            digitalWrite(work, LOW);
            Serial.println("\nTechDevice desligado por comando do usuario");
            sendData("0.00 0 0.00 5");
        }
    } else {
        Serial.println("Mensagem invalida - formato incorreto");
    }
}

// Função para enviar dados via BLE
void sendData(String data) {
    if (deviceConnected) {
        pCharacteristicTX->setValue(data.c_str());
        pCharacteristicTX->notify();
    }
}

void interrupt() {
    ler = 1;
    ler1 = 1;
    contador += 1;
    contador1 += 1;
    
    // Leitura da bateria (média de 10 amostras)
    soma += TensaoBateria;
    j += 1;
    if (j == 10) {
        TensaoBateria = soma * 0.1;
        soma = 0;
        j = 0;
    }
    
    // Verificação da bateria a cada 1.5 segundos
    if (contador1 % 150 == 0) {
        if (TensaoBateria > fraca) {
            digitalWrite(lowbat, LOW);
            toogle = false;
        } else if (TensaoBateria <= fraca & TensaoBateria > critica) {
            digitalWrite(lowbat, HIGH);
            toogle = false;
        }
        if (TensaoBateria <= critica) {
            toogle = true;
        }
        if (toogle) {
            digitalWrite(lowbat, !digitalRead(lowbat));
        }
    }
    
    // Leitura da tensão de saída (média de 10 amostras)
    soma1 += tensaosaida;
    k += 1;
    if (k == 10) {
        tensaosaida = soma1 * 0.1;
        soma1 = 0;
        k = 0;
    }
    
    // Leitura da corrente (média de 10 amostras)
    soma2 += correnteADC;
    l += 1;
    if (l == 10) {
        correnteADC1 = soma2 * 0.1;
        soma2 = 0;
        l = 0;
    }
    
    // ==================== DETECÇÃO DE ERROS (ADICIONADO) ====================
    if (ligado) {
        // Converte leituras para valores reais
        correnteADC1 = (correnteADC1) * 2.599428125 * 0.0001;
        tensaosaida0 = tensaosaida;
        tensaosaida = (tensaosaida) * 2.66666 * 0.001;
        
        // Detecção de erros (apenas para monitoramento em malha aberta)
        if (tensaosaida == 0) {
            erro = 3;  // Curto circuito
            google = false;
        } else if ((correnteADC1 < 0.1) && (tensaosaida0 >= 4090)) {
            erro = 2;  // Alta impedância (circuito aberto)
            google = true;
        } else if (correnteADC1 < 0.01) {
            erro = 1;  // Corrente zero
            google = true;
        } else if ((correnteADC1 >= 0.1) && (tensaosaida0 != 0)) {
            erro = 0;  // Normal
            google = false;
            digitalWrite(work, HIGH);
        }
        
        // Atualiza erro1 para detecção de mudança
        if (erro != erro1) {
            digitalWrite(work, LOW);
        }
        erro1 = erro;
    }
    // =====================================================================
    
    // Envio de dados a cada 1 segundo
    if (contador1 % 100 == 0) {
        enviarDados = true;
    }
}

void startTimer() {
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &interrupt, true);
    timerAlarmWrite(timer, 10000, true);
    timerAlarmEnable(timer);
}

void setup() {
    Serial.begin(115200);
    Serial.println("\n\n=== INICIANDO DISPOSITIVO - MALHA ABERTA ===");

    // Configurar BLE
    Serial.println("Inicializando BLE...");
    BLEDevice::init("Dispositivo 1");
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Característica para RX (receber dados do app)
    pCharacteristicRX = pService->createCharacteristic(
        CHARACTERISTIC_UUID_RX,
        BLECharacteristic::PROPERTY_WRITE
    );
    pCharacteristicRX->setCallbacks(new MyCallbacks());

    // Característica para TX (enviar dados para o app)
    pCharacteristicTX = pService->createCharacteristic(
        CHARACTERISTIC_UUID_TX,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pCharacteristicTX->addDescriptor(new BLE2902());

    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();

    Serial.println("BLE iniciado - Aguardando conexao...");
    Serial.println("Modo: MALHA ABERTA (Open Loop)\n");

    // Configuração dos pinos
    pinMode(15, OUTPUT);
    pinMode(tensao, INPUT);
    pinMode(lowbat, OUTPUT);
    pinMode(bateria, INPUT);
    pinMode(power, OUTPUT);
    pinMode(pwm, OUTPUT);
    pinMode(corrente, INPUT);

    // Teste dos LEDs
    digitalWrite(power, HIGH);
    digitalWrite(work, HIGH);
    digitalWrite(lowbat, HIGH);
    delay(500);
    digitalWrite(power, LOW);
    digitalWrite(work, LOW);
    digitalWrite(lowbat, LOW);
    delay(500);
    digitalWrite(power, HIGH);
    digitalWrite(work, HIGH);
    digitalWrite(lowbat, HIGH);
    delay(500);
    digitalWrite(power, LOW);
    digitalWrite(work, LOW);
    digitalWrite(lowbat, LOW);
    delay(500);

    digitalWrite(power, HIGH);
    digitalWrite(work, LOW);
    digitalWrite(lowbat, LOW);

    // Configuração do PWM
    ledcSetup(canal, freq, resolucao);
    ledcAttachPin(pwm, canal);

    digitalWrite(power_on, HIGH);
    digitalWrite(power, HIGH);
    dtostrf(0, 4, 2, setpoint);
    startTimer();
    
    Serial.println("\nComandos disponiveis:");
    Serial.println("  #T#K1XXXYYY - Configurar (XXX=corrente 0-1000, YY=tempo)");
    Serial.println("  #T#K2 - Ligar");
    Serial.println("  #T#K3 - Ignorado (malha aberta)");
    Serial.println("  #T#K4 - Desligar");
    Serial.println("\nTipos de erro (apenas monitoramento):");
    Serial.println("  0 = Normal");
    Serial.println("  1 = Corrente zero");
    Serial.println("  2 = Alta impedancia (circuito aberto)");
    Serial.println("  3 = Curto circuito");
    Serial.println();
}

void loop() {
    if (ligado) {
        TempoAtual = millis();
        analogReadResolution(12);
        
        // MALHA ABERTA: PWM FIXO
        ledcWrite(canal, (int)Controle_Saida);
        
        // Leitura dos sensores
        if (ler) {
            correnteADC = analogRead(corrente);
            tensaosaida = analogRead(tensao);
            ler = 0;
        }

        // Verifica se o tempo de aplicação acabou
        if (TempoAtual - TempoPassado > TempoDeAplicacao) {
            ligado = false;
            ledcWrite(canal, 0);
            digitalWrite(work, LOW);
            Serial.println("\n##########");
            Serial.println("TechDevice desligado pela rotina do dispositivo!");
            sendData("0.00 0 0.00 4");
        }

        // Envio de dados a cada 1 segundo - Com erro adicionado
        if (enviarDados) {
            enviarDados = false;

            // Calcula corrente real
            float correnteReal = ganho_cal * correnteADC1 + offset_cal;
            
            // Calcula tensão da bateria em Volts
            float tensaoBateriaVolts = TensaoBateria * 0.001;
            
            // Mensagem com erro incluído
            String msgDebug = "Corrente: " + String(correnteReal, 2) + 
                             " | PWM: " + String((int)Controle_Saida) + 
                             " | Bateria: " + String(tensaoBateriaVolts, 2) + "V" +
                             " | Modo: MA" +
                             " | Erro: " + String(erro);

            Serial.println(msgDebug);
            sendData(msgDebug);
        }
    } else {
        // Dispositivo desligado - PWM em 0
        ledcWrite(canal, 0);
    }
    
    // Leitura da bateria
    if (ler1) {
        TensaoBateria = analogRead(bateria);
    }
    
    delay(1);
}