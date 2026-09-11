#include <BluetoothSerial.h>
BluetoothSerial SerialBT;
# define pwm 18
# define power_on 3        // Pino correspondente à GPIOP3 - saída ligar dispositivo (gate mosfet)
# define button_power 25   // Pino correspondente à GPIOP25 - entrada de sinal botao ligar
# define power 16          // Saída correspondente à GPIOP0 - dispositivo ligado
# define corrente 4        // Pino correspondente à GPIOP4 - entrada analógica sensor corrente
# define tensao  32        // Pino correspondente à GPIOP32 - entrada analógica sensor tensão  
# define bateria 33        // Pino correspondente à GPIOP39 - entrada analógica tensão bateria
# define work 15           // Pino que indica se esta ocorrendo aplicação 
# define lowbat 2          // Pino correspondente à GPIOP6

//Variavel teste de Validade
String teste  = "#T#K";
bool   valido = false;
//Variaveis para salvar tempo e corrente
char   horas[3];
char   minutos[3];
//Variaveis para controlador
char   kp[5];
char   ki[5];
double  Kp = 2;
double  Ki = 1.5;
int    minuto;
int    hora;
int    controle;
char   setpoint[4];
bool   ligado;
char   Mults[1];
int    Mult;
float  Controle_PWM      = 0;
float  leitura;
float  Erro_Controle     = 0;
float  Referencia        = 0;
float  Referencia_c        = 0;
float  Erro_Anterior     = 0;
float  Controle_Anterior = 0;
float  Controle_Saida;
double  Comp_Kp           = 0; 
double  Comp_Ki           = 0;

// NOVO: constantes de calibração movidas para o escopo global para poderem
// ser usadas tanto no cálculo de Referencia_c quanto no print de debug
// (antes estavam declaradas dentro de um bloco local e não existiam fora dele).
const float ganho_cal  = 1.028;
const float offset_cal = 0.033;

long int contador = 0;
long int contador1 = 0;
long int contador2 = 0;
/*int minRes;
  int horRes;
  int segRes;*/
//Variaveis diversas
int      cont = 0;
long int TempoDeAplicacao = 0;    // variável para armazenar o tempo de aplicação
long int TempoAtual = 0;          // variável para armazenar o tempo atual
long int TempoPassado = 0;        // variável para armazenar o tempo passado
long int TempoRestante = 0;
int freq      = 1000; // Frequencia
int canal     = 0;    // Total de 16 canais (0 a 15)
int resolucao = 12;   // Pode ser de 1 a 16 bits (variável int => intervalo de -32,768 a 32,767 {Equivale a 16 bits})
float correnteADC = 0;  // variável para leitura da corrente de aplicação
float correnteADC1 = 0;  // variável para leitura da corrente de aplicação
float tensaosaida = 0;  // variavel para leitura da tensao de aplicação
float tensaosaida0 = 0;  // variavel para leitura da tensao de aplicação
float tensaosaida1 = 0;  // variavel para leitura da tensao de aplicação
float TensaoBateria = 0; // variavel para leitura da tensao da bateria
int erro = 0;
int erro1 = 0;
int ler = 1;
int ler1 = 1;
float variacaocorrente = 0;
float variacaotensao   = 0;
int soma = 0;
int soma1 = 0;
int soma2 = 0;
bool toogle = false;
bool google = false;
int fraca = 2800;   // Tensão de bateria 3,4 V
int critica = 2700; // Tensão de bateria 3,1 V
int j = 0;        // Auxiliar para média bateria
int k = 0;        // Auxiliar para média tensao
int l = 0;        // Auxiliar para média corrente

// Variáveis auxiliares para timer
int tempo = 0;

// ---------------------- NOVO: flag para envio de dados a cada 1s ----------------------
// Fica marcada dentro da interrupção (só marca, não imprime!) e é lida/zerada no loop().
volatile bool enviarDados = false;

// ---------------------- NOVO: callback de conexão/desconexão Bluetooth ----------------------
// Esse callback é chamado automaticamente pela biblioteca BluetoothSerial sempre que
// o app conecta ou desconecta da porta serial Bluetooth do ESP32.
void callbackBT(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    // Disparado quando o app conecta no ESP32
    Serial.println("\n>>> Bluetooth: aplicativo CONECTADO <<<");
  }
  else if (event == ESP_SPP_CLOSE_EVT) {
    // Disparado quando o app desconecta (ou perde conexão) do ESP32
    Serial.println("\n>>> Bluetooth: aplicativo DESCONECTADO <<<");
  }
}

// Criando um ponteiro para o timer por hardware
hw_timer_t * timer = NULL;
void interrupt()//Estoura a cada 1 segundo segundos
{
  ler = 1;
  ler1 = 1;
  contador += 1;
  contador1 += 1;
  soma += TensaoBateria;
  j += 1;
  if (j == 10) // média de 10 amostras com tempo de 0.1 s
  {
    TensaoBateria = soma *0.1;
    soma = 0;
    j = 0;
  }
 if (contador1%150 == 0){
  if (TensaoBateria > fraca)
  {
    digitalWrite(lowbat, LOW);
    toogle = false;
  }
  else if (TensaoBateria <= fraca & TensaoBateria > critica)
  {
    digitalWrite(lowbat, HIGH);
    toogle = false;
  }
  if (TensaoBateria <= critica)
  {
    toogle = true;
  }
  if (toogle) // para piscar o led de bateria quando estiver no nível crítico
  {
    digitalWrite(lowbat, !digitalRead(lowbat));
  }
 }
  soma1 += tensaosaida;
  k += 1;
  if (k == 10) // média de 10 amostras com tempo de 0.1 s
  {
    tensaosaida = soma1 *0.1;
    soma1 = 0;
    k = 0;
  }
  soma2 += correnteADC;
  l += 1;
  if (l == 10) // média de 10 amostras com tempo de 1 s
  {
    correnteADC1 = soma2 *0.1;
    soma2 = 0;
    l = 0;
  }
  
  if (ligado){
        if (google & contador1%50 == 0) 
          {
          if (contador2 < 2) {   
              if (digitalRead(work)){
               digitalWrite(work,LOW);
               }
          }
           if (!digitalRead(work)){
           contador2++;
            }
           if(contador2 > 2 && contador2 <= (erro+2))
           {
            digitalWrite(work,!digitalRead(work));
           }
           if(contador2 > (erro+2))
           {
            digitalWrite(work,LOW);
            contador2 = 0;
           }
        } 
        if (erro == 3 & contador1%50 == 0){
          digitalWrite(work,!digitalRead(work));   
        }
     
  } 
  if (contador1 % 100 == 0)
  {
    enviarDados = true;
  }

  
    
  
  if (contador1 % 10 == 0)
  {    
      if (ligado)
      {
        
      
        //correnteADC = 0.84*1.1*1.08*1.111*(correnteADC) / 3847;
        
        correnteADC1 = (correnteADC1)*2.599428125*0.0001;
        
        tensaosaida0 = tensaosaida;
        tensaosaida = (tensaosaida)*2.66666*0.001;
        leitura = (float)(controle*2.442*0.0001);
        
        
//CONTROLADOR-##-------------------##----------------------------------##-------------------------------------------##-----------------

        
        Referencia = atoi(setpoint);
        Referencia = Referencia*0.001;
//CALIBRAGEM DE SENSORES PARA CADA DISPOSITIVO-##----------------------##----------------------------------##--------------------------
        
        // NOVO: calibração linear obtida com multímetro em série com a carga.
        // Ajuste medido = 1.028 * referencia + 0.033 (mA), então invertemos
        // a fórmula para que a corrente REAL bata com o valor desejado.
        {
          Referencia_c = (Referencia - offset_cal) / ganho_cal;
          if (Referencia_c < 0) {
            Referencia_c = 0; // proteção contra valor negativo em referências muito baixas
          }
        }         
        //Referencia_c = Referencia;
        Erro_Controle = Referencia_c - correnteADC1;
        Comp_Kp = Kp*(Erro_Controle - Erro_Anterior);
        Comp_Ki = 0.05*Ki*(Erro_Controle + Erro_Anterior);
        Controle_PWM = Comp_Kp + Comp_Ki + Controle_Anterior;
        if (Controle_PWM > 1)  
        {
          Controle_PWM = 1;
        }
        if (Controle_PWM < 0)
        {
          Controle_PWM  = 0;
        }
        Controle_Saida = (int)(4095*Controle_PWM);
        Erro_Anterior = Erro_Controle;
        Controle_Anterior = Controle_PWM;
   
  //---Sistema de Erros --------------------------##---------------------##-----------------------##---------------------##---------------      
        if (tensaosaida == 0) {
          erro = 3; //Curto circuito: resistência ~0, tensão na carga não sobe mesmo com corrente
          google = false;
        }
        else if ((correnteADC1 < leitura) && (tensaosaida0 >= 4090)) {
          erro = 2;
          google = true;
        }
        else if (correnteADC1 == 0) {
          erro = 1;
          google = true;
        }
        else if ((correnteADC1 >= leitura) && (tensaosaida0 != 0)) {
          erro = 0;
          google = false;
          digitalWrite(work,HIGH);
        }
        if (erro != erro1){
          digitalWrite(work,LOW);
        }
        erro1 = erro;
        
      }
    
  }

}
// ---------------------- Configuraçao dos timers -------------------
void startTimer()
{
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &interrupt, true);
  timerAlarmWrite(timer, 10000, true);  // estouro em 0.01 s

  //ativa o alarme
  timerAlarmEnable(timer); // bateria

} // Fim startTimer

void setup() {
  Serial.begin(115200);
  SerialBT.begin("Dispositivo 1");
  SerialBT.register_callback(callbackBT); // NOVO: ativa aviso de conectar/desconectar
  /* Serial.println("ESP32 Bluetooth Serial iniciado"); */

  //Definição dos pinos
  pinMode(15, OUTPUT);
  pinMode(tensao, INPUT);
  pinMode (lowbat, OUTPUT);       // Saída Power (led)
  pinMode(bateria, INPUT);
  pinMode (power, OUTPUT);        // Saída Power (led)
  pinMode (pwm, OUTPUT);          // Saída PWM
  pinMode(corrente, INPUT);       // sensor de corrente
  // ---------- Rotina Testar leds ------------------
  digitalWrite(power, HIGH);    // Led que indica dispositivo ligado
  digitalWrite(work, HIGH);      // Led que indica dispositivo aplicando medicamento
  digitalWrite(lowbat, HIGH);    // Led que indica bateria fraca
  delay(500);
  digitalWrite(power, LOW);    // Led que indica dispositivo ligado
  digitalWrite(work, LOW);      // Led que indica dispositivo aplicando medicamento
  digitalWrite(lowbat, LOW);    // Led que indica bateria fraca
  delay(500);
  digitalWrite(power, HIGH);    // Led que indica dispositivo ligado
  digitalWrite(work, HIGH);      // Led que indica dispositivo aplicando medicamento
  digitalWrite(lowbat, HIGH);    // Led que indica bateria fraca
  delay(500);
  digitalWrite(power, LOW);    // Led que indica dispositivo ligado
  digitalWrite(work, LOW);      // Led que indica dispositivo aplicando medicamento
  digitalWrite(lowbat, LOW);    // Led que indica bateria fraca
  delay(500);

  // ---------- Ligar ou desligar leds ------------------
  digitalWrite(power, HIGH);    // Led que indica dispositivo ligado
  digitalWrite(work, LOW);      // Led que indica dispositivo aplicando medicamento
  digitalWrite(lowbat,LOW);    // Led que indica bateria fraca

  // Configuraçao do PWM
  ledcSetup(canal, freq, resolucao);

  // Conecta o canal ao GPIO a ser controlado
  ledcAttachPin(pwm, canal);
  // Rotina para atrasar serviço bluetooth e ESP ligar
  digitalWrite(power_on, HIGH); // Satura mosfet tipo N
  digitalWrite(power, HIGH);    // Led que indica dispositivo ligado
  dtostrf(0, 4, 2, setpoint); // zerando o setpoint
  startTimer();
}

void loop() {

  if (SerialBT.available()) {
    String value = SerialBT.readString();     //Parte para testar se recebeu algo;
      Serial.print("\nTexto recebido: ");
      Serial.println(value);
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
        //código deve salvar os valores de corrente e tempo
        //Atribução de tempo e corrente
        horas[0] = {value[14]};
        horas[1] = {value[15]};
        minutos[0] = {value[17]};
        minutos[1] = {value[18]};

        setpoint[0] = {value[8]};
        setpoint[1] = {value[10]};
        setpoint[2] = {value[11]};
        // Conversão tempo
        hora = atoi(horas);
        minuto = atoi(minutos);
        TempoDeAplicacao = ((60 * hora) + minuto) * 60000; // passando o tempo para milissegundos mudei de 60000 para 6000 para teste, para o tempo passar mais rapido

        // Conversão corrente
        controle = atoi(setpoint); // corrente máxima de 1 mA que equivale a 1000 (era pra ser 100, mas sempo fica multiplicado por 10)
        controle = ((3723 * (controle))*0.001); // passando valor de corrente para binário de 12 bits (para ajuste do PWM)
      }
      if (value[5] == '2') {

        //digitalWrite(work,HIGH);
        TempoPassado = millis();

        //Serial.print(TempoPassado);
        ligado = true;
      }
      if (value[5] == '3') {
 //proporcional
        kp[0] =  {value[9]};
        kp[1] =  {value[10]};
        kp[2] =  {value[11]};
        kp[3] =  {value[12]};
// integral        
        ki[0] =  {value[16]};
        ki[1] =  {value[17]};
        ki[2] =  {value[18]};
        ki[3] =  {value[19]};
        Ki = atoi(ki);
        Ki = Ki*0.1;
        Kp = atoi(kp);
        Kp = Kp*0.1;
        Serial.print("\nGanho proporcional: "+ (String)Kp +" \nGanho Integral: "+(String)Ki);     
      }
      if (value[5] ==  '4') {
        ligado = false;
        ledcWrite(canal, 0);
        Controle_Anterior = 0;
        digitalWrite(work, LOW);
        Serial.println("\nTechDevice desligado por comando do usuario");
        SerialBT.print("0.00 0.00 5");
      }
    }

  }
  if(ler1){
  TensaoBateria = analogRead(bateria);
  }

  if (ligado) {
    TempoAtual = millis();
    analogReadResolution(12);
    noInterrupts();
    ledcWrite(canal, Controle_Saida);
    interrupts();
    if(ler){
    correnteADC = analogRead(corrente);
    tensaosaida = analogRead(tensao);
    ler = 0;
    }

    if (TempoAtual - TempoPassado > TempoDeAplicacao)
    {
      ligado = false;
      ledcWrite(canal, 0);
      Controle_Anterior = 0;
      digitalWrite(work, LOW);
      Serial.println("\n##########");
      Serial.println("TechDevice desligado pela rotina do dispositivo!");
      SerialBT.print("0.00 0.00 4");
    }

    // ---------------- NOVO: envio de dados de debug a cada 1 segundo ----------------
    if (enviarDados) {
      enviarDados = false; // limpa a flag até o próximo segundo

      float correnteReal = ganho_cal * correnteADC1 + offset_cal;
      String msgDebug = "Corrente: " + String(correnteReal, 4) +
                         " | Referencia: " + String(Referencia, 4) +
                         " | DiferencaCorrente: " + String(Erro_Controle, 4) +
                         " | PWM: " + String((int)Controle_Saida) +
                         " | TipoErro: " + String(erro);

      Serial.println(msgDebug);       // aparece no monitor serial
      SerialBT.println(msgDebug);     // aparece no app do celular
    }
    // ----------------------------------------------------------------------------
  }
}
