#include <Servo.h>

const int soilMoisturePin = A0;
const int potentiometerPin = A1;
const int pumpPin = 9;
const int buzzerPin = 8;
const int ledPin = 7;
const int buttonPin = 2;

Servo servoMotor;
int wateringThreshold = 300;
bool systemActive = true;

void setup() {
  pinMode(soilMoisturePin, INPUT);
  pinMode(potentiometerPin, INPUT);
  pinMode(pumpPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  
  servoMotor.attach(10);
  
  Serial.begin(9600);
}

void loop() {
  if (systemActive) {
    int soilMoisture = analogRead(soilMoisturePin);
    int threshold = map(analogRead(potentiometerPin), 0, 1023, 200, 600);
    
    if (soilMoisture < threshold) {
      startWatering();
    } else {
      stopWatering();
    }
  } else {
    // Enter sleep mode
    servoMotor.write(0);
    digitalWrite(pumpPin, LOW);
    digitalWrite(ledPin, LOW);
    tone(buzzerPin, 0);
    
    // Wait for button press to wake up
    while (digitalRead(buttonPin) == HIGH) {
      delay(100); // debounce delay
    }
    delay(50); // additional debounce delay
    while (digitalRead(buttonPin) == LOW) {
      delay(100); // debounce delay
    }
    delay(50); // additional debounce delay
    
    systemActive = true;
    Serial.println("System awakened.");
  }
  
  delay(1000); // Check soil moisture every second
}

void startWatering() {
  digitalWrite(pumpPin, HIGH);
  servoMotor.write(90);
  digitalWrite(ledPin, HIGH);
  tone(buzzerPin, 1000);
  delay(200); // Short beep duration
  digitalWrite(ledPin, LOW);
  tone(buzzerPin, 0);
  delay(4800); // Watering duration
  digitalWrite(pumpPin, LOW);
  servoMotor.write(0);
  delay(100); // Wait a bit before checking moisture again
}

void stopWatering() {
  digitalWrite(pumpPin, LOW);
  servoMotor.write(0);
  digitalWrite(ledPin, LOW);
  tone(buzzerPin, 2000);
  delay(200); // Short beep duration
  tone(buzzerPin, 0);
  delay(100); // Wait a bit before checking moisture again
  digitalWrite(ledPin, LOW);
}

