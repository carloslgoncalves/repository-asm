//Pins dos Sensores
const int pinLDR = A0;
const int pinLM35 = A1;

//Pins dos LEDs
const int pinLedVermelho = 3;
const int pinLedAmarelo = 4;
const int pinLedVerde = 5;

//Inicializar variáveis globais
///Sensores
float valorMedidoLDR = 0.0; // valor do LDR medido pelo Arduino (ADC de 10 bits)
float luminosidade = 0.0; // Luminosidade medida (em percentagem c/ 1 casa decimal)
float valorMedidoLM35 = 0.0; // valor do LM35 medido pelo Arduino (ADC de 10 bits)
float temperaturaMedida = 0.0; // Temperatura medida (em graus Celsius c/ 1 casa decimal)
float temperaturaSetpoint = 20.0; // Setpoint de temperatura selecionado no LabVIEW (em graus Celsius c/ 1 casa decimal)
///comunicação
bool sistemaOperacional = false;
unsigned long previousMillis1 = 0;
unsigned long previousMillis2 = 0;
unsigned long previousMillis3 = 0;
const unsigned long serialInterval = 900; // Intervalo de envio de dados (de 1 em 1 segundo neste caso)
///LEDs
bool estadoLedVermelho = LOW;
const unsigned long tOff = 1000; // Tempo off LED vermelho (1 segundo)


void setup() {
  Serial.begin(9600); //Iniciar Comunicação em Série

  pinMode(pinLedVermelho,OUTPUT);
  pinMode(pinLedAmarelo,OUTPUT);
  pinMode(pinLedVerde,OUTPUT);
  /* 
  pinMode(lighSensor,INPUT);
  pinMode(temperatureSensor,INPUT)
  */
}


void loop() {
  
  leituraSensores(); // Leitura dos sensores
  
  estadoSistema(); //Estado do sistema (indicado pelos LEDs)
  
  leituraDadosSerie(); // Leitura de dados porta série
}


void  leituraSensores(){    
  // 3 leituras do valor de luminosidade
  int valorMedidoLDR_1 = analogRead(pinLDR);
  delay(10);
  int valorMedidoLDR_2 = analogRead(pinLDR);
  delay(10); 
  int valorMedidoLDR_3 = analogRead(pinLDR); 
  // média ponderada das 3 medições
  valorMedidoLDR = (valorMedidoLDR_1 + valorMedidoLDR_2 + valorMedidoLDR_3)/3;
  // registo(em %) na variável luminosidae
  luminosidade = valorMedidoLDR * 100.0 / 1023.0;

  // 3 leituras do valor de temperatura
  int valorMedidoLM35_1 = analogRead(pinLM35);
  delay(10);
  int valorMedidoLM35_2 = analogRead(pinLM35);
  delay(10); 
  int valorMedidoLM35_3 = analogRead(pinLM35);
  // média ponderada das 3 medições
  valorMedidoLM35 = (valorMedidoLM35_1 + valorMedidoLM35_2 + valorMedidoLM35_3)/3; 
  // registo(em ºC) na variável temperaturaMedida
  temperaturaMedida = valorMedidoLM35 * (5.0 /1023.0) * 100; // 10mV por ºC
}


void estadoSistema(){
  //Caso de sistema não operacional, ou erro em algum dos sensores
  if (!sistemaOperacional || valorMedidoLM35 == 1023 || valorMedidoLM35 == 0 || valorMedidoLDR == 1023 || valorMedidoLDR == 0) {
    
    //LED amarelo liga
    digitalWrite(pinLedVerde, LOW);
    digitalWrite(pinLedVermelho, LOW);
    digitalWrite(pinLedAmarelo, HIGH);

    //Mensagem que traduz o estado do sistema
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis1 >= serialInterval) {
      previousMillis1 = currentMillis;

      // problemas na montagem do LM35
      if (valorMedidoLM35 == 1023 || valorMedidoLM35 == 0) {
        Serial.println("ERRO1");
        sistemaOperacional = false;

        // problemas na montagem do LDR
      } else if (valorMedidoLDR == 1023 || valorMedidoLDR == 0) {
        Serial.println("ERRO2");
        sistemaOperacional = false;

      } else if (!sistemaOperacional) {
        Serial.println("Aguardar pela Comunicacao Serie...");
      } 
    }

  //Caso de sistema operacional e nenhum erro nos sensores
  } else {

    //LED Verde liga
    digitalWrite(pinLedVerde, HIGH);
    digitalWrite(pinLedAmarelo, LOW);
 
    // Controlo do LED vermelho
    float erro = temperaturaSetpoint - temperaturaMedida;
    unsigned long  tOn = erro * 1000; //LED vermelho "erro(ºC) segundos" ON
    if (temperaturaSetpoint < temperaturaMedida) {
      digitalWrite(pinLedVermelho, LOW);
    } else if (erro > 10) { //se a temperatura desejada é superior a 10ºC acima daquela que é a temperatura atual
      digitalWrite(pinLedVermelho, HIGH);
    } else if (erro <= 10 && erro >= 0) { //se a temperatura desejada está até os 10ºC acima daquela que é a temperatura atual
      piscarLEDVermelho(tOn);
    }

    // Há envio de dados
    envioDados();
  }
}


void piscarLEDVermelho(unsigned long tOn) {
  unsigned long currentMillis = millis();
  if (estadoLedVermelho == LOW && currentMillis - previousMillis2 >= tOff) {
    //tON
    estadoLedVermelho = HIGH;
    previousMillis2 = currentMillis;
    digitalWrite(pinLedVermelho, HIGH);
  } else if (estadoLedVermelho == HIGH && currentMillis - previousMillis2 >= tOn) {
    //tOFF
    estadoLedVermelho = LOW;
    previousMillis2 = currentMillis;
    digitalWrite(pinLedVermelho, LOW);
  }
}


void leituraDadosSerie(){
  if (Serial.available() > 0) { //se existir data para ser lida no buffer
    String input = Serial.readStringUntil('\n'); //ler os dados até encontrar o fim de linha
    input.trim();  // Remove espaços em branco antes e depois da string

    if (input.startsWith("SET")) { //Verifica se a string começa com o texto "SET"
      temperaturaSetpoint = input.substring(3).toFloat(); //extrai tudo após os três primeiros caracteres (neste caso, o valor do setpoint), e o valor extraído é convertido de string para float
    } else if (input == "DANGER") {
      sistemaOperacional = false;
    } else if (input == "OK") {
      sistemaOperacional = true;
    }
  }
}


void envioDados(){
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis3 >= serialInterval) {
    previousMillis3 = currentMillis;
    
      Serial.print("Temp: ");
      Serial.print(temperaturaMedida, 2);
      Serial.print(" | SP: ");
      Serial.print(temperaturaSetpoint, 1);

      Serial.print(" ; LDR Value: ");
      Serial.print(valorMedidoLDR);
      Serial.print(" | LL: ");
      Serial.print(luminosidade, 2);
      Serial.println("%");
  }
}

