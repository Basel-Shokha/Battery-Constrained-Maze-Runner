#include <Wire.h>
#include <VL53L1X.h>

#define XSHUT_1 5
#define XSHUT_2 23
#define XSHUT_3 18

VL53L1X sensor1, sensor2, sensor3;
uint16_t dist1 = 0, dist2 = 0, dist3 = 0;
unsigned long lastPrintTime = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  // Reset all sensors by pulling XSHUT pins low
  pinMode(XSHUT_1, OUTPUT);
  pinMode(XSHUT_2, OUTPUT);
  pinMode(XSHUT_3, OUTPUT);
  digitalWrite(XSHUT_1, LOW);
  digitalWrite(XSHUT_2, LOW);
  digitalWrite(XSHUT_3, LOW);
  delay(10);

  // Initialize Sensor 1
  digitalWrite(XSHUT_1, HIGH); delay(10);
  sensor1.init(); sensor1.setAddress(0x30);

  // Initialize Sensor 2
  digitalWrite(XSHUT_2, HIGH); delay(10);
  sensor2.init(); sensor2.setAddress(0x31);

  // Initialize Sensor 3
  digitalWrite(XSHUT_3, HIGH); delay(10);
  sensor3.init(); sensor3.setAddress(0x32);

  // Start continuous readings at 50ms intervals
  sensor1.startContinuous(50);
  sensor2.startContinuous(50);
  sensor3.startContinuous(50);

  Serial.println("Sensors Initialized. Printing readings...");
}

void loop() {
  // Read sensor data if ready
  if (sensor1.dataReady()) dist1 = sensor1.read(false);
  if (sensor2.dataReady()) dist2 = sensor2.read(false);
  if (sensor3.dataReady()) dist3 = sensor3.read(false);

  // Print readings every 100ms
  if (millis() - lastPrintTime >= 100) {
    lastPrintTime = millis();
    
    Serial.print("Left: ");
    Serial.print(dist1);
    Serial.print(" mm | Front: ");
    Serial.print(dist2);
    Serial.print(" mm | Right: ");
    Serial.print(dist3);
    Serial.println(" mm");
  }
}
