#include <Wire.h>
#include <VL53L1X.h>

#define XSHUT_1 13
#define XSHUT_2 27

VL53L1X sensor1;
VL53L1X sensor2;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // SDA=21, SCL=22

  pinMode(XSHUT_1, OUTPUT);
  pinMode(XSHUT_2, OUTPUT);
  
  // Keep both sensors in reset mode initially
  digitalWrite(XSHUT_1, LOW);
  digitalWrite(XSHUT_2, LOW);
  delay(10);

  // Initialize Sensor 1 (GPIO 13)
  digitalWrite(XSHUT_1, HIGH);
  delay(10);
  if (!sensor1.init()) { Serial.println("Sensor 1 (G13) failed!"); while (1); }
  sensor1.setAddress(0x30); // Move away from default 0x29

  // Initialize Sensor 2 (GPIO 27)
  digitalWrite(XSHUT_2, HIGH);
  delay(10);
  if (!sensor2.init()) { Serial.println("Sensor 2 (G27) failed!"); while (1); }
  sensor2.setAddress(0x31);

  // Start continuous measurements
  sensor1.startContinuous(50);
  sensor2.startContinuous(50);
}

void loop() {
  Serial.print("S1 (G13): "); Serial.print(sensor1.read()); Serial.print("mm | ");
  Serial.print("S2 (G27): "); Serial.print(sensor2.read()); Serial.println("mm");
  delay(100);
}