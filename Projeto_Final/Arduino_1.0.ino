// Configurações iniciais
const int ldrPin = A0;
const int tempSensorPin = A1;
const int ledRED = 3;
const int ledYELLOW = 4;
const int ledGREEN = 5;

float temperatura;
float setpoint = 25.0;

bool sistemaOperacional = false;

unsigned long previousMillis = 0;
unsigned long blinkInterval = 0;
unsigned long serialPreviousMillis = 0;
const unsigned int serialInterval = 1000;
bool redLedState = LOW;

void setup() {
  Serial.begin(9600);
  pinMode(ledRED, OUTPUT);
  pinMode(ledYELLOW, OUTPUT);
  pinMode(ledGREEN, OUTPUT);
  pinMode(ldrPin, INPUT);
  pinMode(tempSensorPin, INPUT);
}

void loop() {
  analogRead(tempSensorPin);
  delay(10);
  int sensorValue = analogRead(tempSensorPin);
  temperatura = (sensorValue * 5.0 * 100.0) / 1023.0;

  analogRead(ldrPin);
  delay(10);
  int ldrValue = analogRead(ldrPin);
  float lightLevel = ldrValue * 100.0 / 1023.0;

  if (!sistemaOperacional || temperatura >=470.0 || temperatura<=0.0||ldrValue>=1023||ldrValue<=0) {
    digitalWrite(ledGREEN, LOW);
    digitalWrite(ledRED, LOW);
    digitalWrite(ledYELLOW, HIGH);
    unsigned long currentMillis = millis();
    if (currentMillis - serialPreviousMillis >= serialInterval) {
      serialPreviousMillis = currentMillis;
      
      if (temperatura >=470.0 || temperatura<=0.0){
        Serial.println("ERROR1");
        sistemaOperacional=false;
      }else if(ldrValue>=1023||ldrValue<=0){
        Serial.println("ERROR2");
        sistemaOperacional=false;
      }else if(!sistemaOperacional){
        Serial.println("Waiting for connection...");
      } 
    }
  } else {
    digitalWrite(ledGREEN, HIGH);
    digitalWrite(ledYELLOW, LOW);



    float erro = setpoint - temperatura;
    if (erro < 0) {

      digitalWrite(ledRED, LOW);
    } else if (erro > 10) {

      digitalWrite(ledRED, HIGH);
    } else {
      blinkInterval = erro * 1000;
      piscarLed();
    }
    
    unsigned long currentMillis = millis();
    if (currentMillis - serialPreviousMillis >= serialInterval) {
      serialPreviousMillis = currentMillis;
      
      Serial.print("Temp: ");
      Serial.print(temperatura, 2);
      Serial.print(" | SP: ");
      Serial.print(setpoint, 1);

      Serial.print(" ; LDR Value: ");
      Serial.print(ldrValue);
      Serial.print(" | Light Level: ");
      Serial.print(lightLevel, 2);
      Serial.println("%");
    }
  }
  
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();  // Remove espaços em branco

    if (input.startsWith("SET")) {
      setpoint = input.substring(3).toFloat();

    } else if (input == "DANGER") {
      sistemaOperacional = false;

    } else if (input == "OK") {
      sistemaOperacional = true;
    }
  }
  
  //delay(1000);
}


void piscarLed() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= blinkInterval && redLedState) {
    previousMillis = currentMillis;

    redLedState = !redLedState;
    digitalWrite(ledRED, redLedState);
  } else if (!redLedState && currentMillis - previousMillis >= 1000) {
    previousMillis = currentMillis;

    redLedState = !redLedState;
    digitalWrite(ledRED, redLedState);
    
  }
}
