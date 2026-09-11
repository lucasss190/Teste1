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

// Variáveis de controle
String teste = "#T#K";
bool valido = false;
char horas[3];
char minutos[3];
char kp[5];
char ki[5];
double Kp = 2;
double Ki = 1.5;
int minuto;
int hora;
int controle;
char setpoint[4];
bool ligado;
char Mults[1];
int Mult;
float Controle_PWM = 0;
float leitura;
float Erro_Controle = 0;
float Referencia = 0;
float Referencia_c = 0;
float Erro_Anterior = 0;
float Controle_Anterior = 0;
float Controle_Saida;
double Comp_Kp = 0;
double Comp_Ki = 0;

const float ganho_cal = 1.028;
const float offset_cal = 0.033;

// --- NOVAS VARIÁVEIS PARA CONTROLE ANTI-OSCILAÇÃO ---
const float BANDA_MORTA = 0.008; // Aumentado para 0.8%
const float MAX_VARIACAO_PWM = 0.01; // Máximo 1% de variação por ciclo
const float FILTRO_ALPHA = 0.3; // Filtro exponencial (0-1), quanto menor mais suave
float correnteFiltrada = 0;
float erroFiltrado = 0;
bool pwmTravado = false;
float ultimoPWMValido = 0;
int contadorEstavel = 0;
const int MIN_AMOSTRAS_TRAVADO = 3; // Número de amostras estáveis para travar
// ----------------------------------------

long int contador = 0;
long int contador1 = 0;
long int contador2 = 0;
int cont = 0;
long int TempoDeAplicacao = 0;
long int TempoAtual = 0;
long int TempoPassado = 0;
long int TempoRestante = 0;
int freq = 1000;
int canal = 0;
int resolucao = 12;
float correnteADC = 0;
float correnteADC1 = 0;
float tensaosaida = 0;
float tensaosaida0 = 0;
float tensaosaida1 = 0;
float TensaoBateria = 0;
int erro = 0;
int erro1 = 0;
int ler = 1;
int ler1 = 1;
float variacaocorrente = 0;
float variacaotensao = 0;
int soma = 0;
int soma1 = 0;
int soma2 = 0;
bool toogle = false;
bool google = false;
int fraca = 2800;
int critica = 2700;
int j = 0;
int k = 0;
int l = 0;
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

            controle = atoi(setpoint);
            controle = ((3723 * (controle)) * 0.001);
        }
        if (value[5] == '2') {
            TempoPassado = millis();
            ligado = true;
            // Reset das variáveis de controle
            pwmTravado = false;
            ultimoPWMValido = 0;
            contadorEstavel = 0;
            Erro_Anterior = 0;
            Controle_Anterior = 0;
            correnteFiltrada = 0;
            erroFiltrado = 0;
            Serial.println("\nSistema ligado - Controle iniciado");
        }
        if (value[5] == '3') {
            kp[0] = value[9];
            kp[1] = value[10];
            kp[2] = value[11];
            kp[3] = value[12];

            ki[0] = value[16];
            ki[1] = value[17];
            ki[2] = value[18];
            ki[3] = value[19];
            
            Ki = atoi(ki);
            Ki = Ki * 0.1;
            Kp = atoi(kp);
            Kp = Kp * 0.1;
            
            Serial.print("\nGanho proporcional: " + (String)Kp + " \nGanho Integral: " + (String)Ki);
        }
        if (value[5] == '4') {
            ligado = false;
            ledcWrite(canal, 0);
            Controle_Anterior = 0;
            digitalWrite(work, LOW);
            pwmTravado = false;
            ultimoPWMValido = 0;
            contadorEstavel = 0;
            Serial.println("\nTechDevice desligado por comando do usuario");
            sendData("0.00 0.00 5");
        }
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
    soma += TensaoBateria;
    j += 1;
    
    if (j == 10) {
        TensaoBateria = soma * 0.1;
        soma = 0;
        j = 0;
    }
    
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
    
    soma1 += tensaosaida;
    k += 1;
    if (k == 10) {
        tensaosaida = soma1 * 0.1;
        soma1 = 0;
        k = 0;
    }
    
    soma2 += correnteADC;
    l += 1;
    if (l == 10) {
        correnteADC1 = soma2 * 0.1;
        soma2 = 0;
        l = 0;
    }
    
    if (ligado) {
        if (google & contador1 % 50 == 0) {
            if (contador2 < 2) {
                if (digitalRead(work)) {
                    digitalWrite(work, LOW);
                }
            }
            if (!digitalRead(work)) {
                contador2++;
            }
            if (contador2 > 2 && contador2 <= (erro + 2)) {
                digitalWrite(work, !digitalRead(work));
            }
            if (contador2 > (erro + 2)) {
                digitalWrite(work, LOW);
                contador2 = 0;
            }
        }
        if (erro == 3 & contador1 % 50 == 0) {
            digitalWrite(work, !digitalRead(work));
        }
    }
    
    if (contador1 % 100 == 0) {
        enviarDados = true;
    }

    if (contador1 % 10 == 0) {
        if (ligado) {
            correnteADC1 = (correnteADC1) * 2.599428125 * 0.0001;
            tensaosaida0 = tensaosaida;
            tensaosaida = (tensaosaida) * 2.66666 * 0.001;
            leitura = (float)(controle * 2.442 * 0.0001);

            Referencia = atoi(setpoint);
            Referencia = Referencia * 0.001;

            {
                Referencia_c = (Referencia - offset_cal) / ganho_cal;
                if (Referencia_c < 0) {
                    Referencia_c = 0;
                }
            }

            // --- FILTRO DA CORRENTE ---
            if (correnteFiltrada == 0) {
                correnteFiltrada = correnteADC1;
            } else {
                correnteFiltrada = (FILTRO_ALPHA * correnteADC1) + ((1 - FILTRO_ALPHA) * correnteFiltrada);
            }

            Erro_Controle = Referencia_c - correnteFiltrada;
            
            // --- FILTRO DO ERRO ---
            if (erroFiltrado == 0) {
                erroFiltrado = Erro_Controle;
            } else {
                erroFiltrado = (FILTRO_ALPHA * Erro_Controle) + ((1 - FILTRO_ALPHA) * erroFiltrado);
            }
            
            // --- NOVA LÓGICA COM BANDA MORTA E HISTERESE ---
            float erroParaControle = erroFiltrado;
            
            // Verifica se o erro está dentro da banda morta
            if (abs(erroFiltrado) < BANDA_MORTA) {
                // Incrementa contador de estabilidade
                contadorEstavel++;
                
                // Só trava após MIN_AMOSTRAS_TRAVADO amostras consecutivas
                if (contadorEstavel >= MIN_AMOSTRAS_TRAVADO) {
                    if (!pwmTravado) {
                        pwmTravado = true;
                        ultimoPWMValido = Controle_PWM;
                        Serial.println(">>> PWM TRAVADO - Erro estável dentro da banda morta");
                    }
                    // Mantém o PWM travado
                    Controle_PWM = ultimoPWMValido;
                } else {
                    // Ainda não travou, calcula normalmente
                    pwmTravado = false;
                    erroParaControle = 0; // Zera o erro para evitar oscilações
                }
            } else {
                // Erro fora da banda morta - reseta contador
                contadorEstavel = 0;
                pwmTravado = false;
                
                // Aplica histerese para evitar micro-oscilações
                if (abs(erroFiltrado) < BANDA_MORTA * 1.5) {
                    if (erroFiltrado > 0 && Erro_Anterior > 0) {
                        erroParaControle = erroFiltrado - BANDA_MORTA * 0.5;
                    } else if (erroFiltrado < 0 && Erro_Anterior < 0) {
                        erroParaControle = erroFiltrado + BANDA_MORTA * 0.5;
                    }
                }
            }
            
            // Só calcula o PID se não estiver travado
            if (!pwmTravado) {
                Comp_Kp = Kp * (erroParaControle - Erro_Anterior);
                Comp_Ki = 0.05 * Ki * (erroParaControle + Erro_Anterior);
                Controle_PWM = Comp_Kp + Comp_Ki + Controle_Anterior;
                
                // Limita o PWM
                if (Controle_PWM > 1) {
                    Controle_PWM = 1;
                }
                if (Controle_PWM < 0) {
                    Controle_PWM = 0;
                }
                
                // Limita a variação do PWM
                float variacao = Controle_PWM - Controle_Anterior;
                if (abs(variacao) > MAX_VARIACAO_PWM) {
                    if (variacao > 0) {
                        Controle_PWM = Controle_Anterior + MAX_VARIACAO_PWM;
                    } else {
                        Controle_PWM = Controle_Anterior - MAX_VARIACAO_PWM;
                    }
                }
                
                // Guarda o valor calculado como último válido
                ultimoPWMValido = Controle_PWM;
            }
            // --- FIM DA NOVA LÓGICA ---
            
            Controle_Saida = (int)(4095 * Controle_PWM);
            Erro_Anterior = erroParaControle;
            Controle_Anterior = Controle_PWM;

            if (tensaosaida == 0) {
                erro = 3;
                google = false;
            } else if ((correnteFiltrada < leitura) && (tensaosaida0 >= 4090)) {
                erro = 2;
                google = true;
            } else if (correnteFiltrada == 0) {
                erro = 1;
                google = true;
            } else if ((correnteFiltrada >= leitura) && (tensaosaida0 != 0)) {
                erro = 0;
                google = false;
                digitalWrite(work, HIGH);
            }
            
            if (erro != erro1) {
                digitalWrite(work, LOW);
            }
            erro1 = erro;
        }
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

    // Configurar BLE
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

    Serial.println("BLE iniciado - Aguardando conexão...");

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
}

void loop() {
    if (ligado) {
        TempoAtual = millis();
        analogReadResolution(12);
        noInterrupts();
        ledcWrite(canal, Controle_Saida);
        interrupts();
        
        if (ler) {
            correnteADC = analogRead(corrente);
            tensaosaida = analogRead(tensao);
            ler = 0;
        }

        if (TempoAtual - TempoPassado > TempoDeAplicacao) {
            ligado = false;
            ledcWrite(canal, 0);
            Controle_Anterior = 0;
            digitalWrite(work, LOW);
            pwmTravado = false;
            ultimoPWMValido = 0;
            contadorEstavel = 0;
            Serial.println("\n##########");
            Serial.println("TechDevice desligado pela rotina do dispositivo!");
            sendData("0.00 0.00 4");
        }

        if (enviarDados) {
            enviarDados = false;

            float correnteReal = ganho_cal * correnteFiltrada + offset_cal;
            String msgDebug ="| Cor: " + String(correnteReal, 3) +
                            " | Ref: " + String(Referencia, 3) +
                            " | Err: " + String(erroFiltrado, 3) +
                            " | PWM: " + String((int)Controle_Saida) +
                            " | Erro: " + String(erro);

            Serial.println(msgDebug);
            sendData(msgDebug);
        }
    }
    
    if (ler1) {
        TensaoBateria = analogRead(bateria);
    }
}