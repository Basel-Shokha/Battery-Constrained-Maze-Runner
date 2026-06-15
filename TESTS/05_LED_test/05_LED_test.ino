#include <Arduino.h>

// This tells the ESP32 to control GPIO pin 4
const int ledPin = 4; 

void setup() {
  pinMode(ledPin, OUTPUT);
}

void loop() {
  digitalWrite(ledPin, HIGH);  // Turns the LED ON
  delay(1000);                 // Waits 1 second
  
  digitalWrite(ledPin, LOW);   // Turns the LED OFF
  delay(1000);                 // Waits 1 second
}