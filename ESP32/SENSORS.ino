// ============================================================
//  TAB 5: SENSORS.ino — Asynchronous ToF Laser Driver Array
// ============================================================
#include "PROTOCOL.h"
#include <Wire.h>
#include <VL53L1X.h>

// ── PHYSICAL SENSOR XSHUT PINS ────────────────────────────
#define XSHUT_1 5   // Left Sensor
#define XSHUT_2 23  // Front Sensor
#define XSHUT_3 18  // Right Sensor

VL53L1X sensorL, sensorF, sensorR;

// Initializes the I2C bus and sequences ToF sensors via their XSHUT pins
void initSensors() {
    Wire.begin(21, 22);
    Wire.setClock(400000); // Elevate core I2C speed boundaries to 400kHz Fast Mode
    
    pinMode(XSHUT_1, OUTPUT);
    pinMode(XSHUT_2, OUTPUT);
    pinMode(XSHUT_3, OUTPUT);
    
    // Shut down power rails to reset state machines
    digitalWrite(XSHUT_1, LOW);
    digitalWrite(XSHUT_2, LOW);
    digitalWrite(XSHUT_3, LOW);
    delay(10);
    
    // Incrementally wake up sensors and reassign unique dynamic I2C hardware addresses
    digitalWrite(XSHUT_1, HIGH); delay(10); sensorL.init(); sensorL.setAddress(0x30);
    digitalWrite(XSHUT_2, HIGH); delay(10); sensorF.init(); sensorF.setAddress(0x31);
    digitalWrite(XSHUT_3, HIGH); delay(10); sensorR.init(); sensorR.setAddress(0x32);
    
    // Start asynchronous data collection loop at 50ms intervals
    sensorL.startContinuous(50);
    sensorF.startContinuous(50);
    sensorR.startContinuous(50);
}

// Asynchronously updates global registers when new data frames arrive
void readSensors() {
    if (sensorL.dataReady()) distLeft  = sensorL.read(false);
    if (sensorF.dataReady()) distFront = sensorF.read(false);
    if (sensorR.dataReady()) distRight = sensorR.read(false);
}

