// Configurações iniciais
const int ldrPin = A0;
const int tempSensorPin = A1;
const int ledRED = 3;
const int ledYELLOW = 4;
const int ledGREEN = 5;

float temperatura;
int setpoint = 25;

int ldrValue = 0;
bool sistemaOperacional = true;

unsigned long previousMillis = 0;
unsigned long blinkInterval = 0;
bool redLedState = LOW;

void setup() {
  Serial.begin(9600);
  pinMode(ledRED, OUTPUT);
  pinMode(ledYELLOW, OUTPUT);
  pinMode(ledGREEN, OUTPUT);
}

void loop() {

  int sensorValue = analogRead(tempSensorPin);
  temperatura = (sensorValue * 5.0 / 1023.0) * 100.0;

  if (!sistemaOperacional) {
    digitalWrite(ledGREEN, LOW);
    digitalWrite(ledRED, LOW);
    digitalWrite(ledYELLOW, HIGH);

  } else {
    digitalWrite(ledGREEN, HIGH);
    digitalWrite(ledYELLOW, LOW);


    Serial.print("Temp: ");
    Serial.print(temperatura);
    Serial.print(" | SP: ");
    Serial.println(setpoint);

    int erro = setpoint - temperatura;
    if (erro < 0) {

      digitalWrite(ledRED, LOW);
    } else if (erro > 10) {

      digitalWrite(ledRED, HIGH);
    } else {
      blinkInterval = erro * 500;
      piscarLed();
    }
    
    ldrValue = analogRead(ldrPin);
    float lightLevel = map(ldrValue, 0, 1023, 100, 0);
    lightLevel = constrain(lightLevel, 0, 100);

    Serial.print("LDR Value: ");
    Serial.print(ldrValue);
    Serial.print(" | Light Level: ");
    Serial.print(lightLevel);
    Serial.println("%");

    if (Serial.available() > 0) {
      String input = Serial.readStringUntil('\n');
      input.trim();  // Remove espaços em branco

      if (input.startsWith("SET")) {
        setpoint = input.substring(3).toInt();
        // } else if (input == "LED ON") {
        //   digitalWrite(ledPin, HIGH);
        // } else if (input == "LED OFF") {
        //   digitalWrite(ledPin, LOW);
        // }
      }
    }
    delay(1000);
  }
}

void piscarLed() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= blinkInterval) {
    previousMillis = currentMillis;

    redLedState = !redLedState;
    digitalWrite(ledRED, redLedState);
  }
}
