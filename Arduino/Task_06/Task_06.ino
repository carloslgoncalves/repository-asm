// Task 6 (04_Out)

/* a.
Intervalo de Leitura de 1 Segundo --> delay(1000);
*/

/* b.
Sensibilidade sensor = 10mv/ºC
Resolução do ADC do Arduino = 10 bits --> 2^10-1 = 1023 
usar numeros decimais --> float data type
float voltage = sensorValue * (5.0 / 1023.0);
float temperature = voltage / 0.01;
*/

/* e.
analogReference(INTERNAL); --> Vref passa a ser 1.1 --> melhor precisao para medições de menor voltagem
*/

const int sensorPin = A0;
const int ledPin = 13;

int sensorValue = 0;

float voltageValue = 0.00;
float temperatureValue = 0.00;

void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  analogReference(INTERNAL);
}

void loop() {
  sensorValue = analogRead(sensorPin);

  voltageValue = sensorValue * (5.0 / 1023.0);
  temperatureValue = voltageValue / 0.01;

  if (temperatureValue > 25.00) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }

  Serial.print("sensorValue = ");
  Serial.print(sensorValue);
  Serial.println("voltageValue = ");
  Serial.print(voltageValue, 2);
  Serial.println("temperatureValue = ");
  Serial.print(temperatureValue, 2);

  delay(1000);
}
