
#include "PROTOCOL.h"
#include <Wire.h>
#include <VL53L1X.h>


#define XSHUT_1 5   
#define XSHUT_2 23  
#define XSHUT_3 18  

VL53L1X sensorL, sensorF, sensorR;


void initSensors() {
    Wire.begin(21, 22);
    Wire.setClock(400000); 
    
    pinMode(XSHUT_1, OUTPUT);
    pinMode(XSHUT_2, OUTPUT);
    pinMode(XSHUT_3, OUTPUT);
    
    
    digitalWrite(XSHUT_1, LOW);
    digitalWrite(XSHUT_2, LOW);
    digitalWrite(XSHUT_3, LOW);
    delay(10);
    
    
    digitalWrite(XSHUT_1, HIGH); delay(10); sensorL.init(); sensorL.setAddress(0x30);
    digitalWrite(XSHUT_2, HIGH); delay(10); sensorF.init(); sensorF.setAddress(0x31);
    digitalWrite(XSHUT_3, HIGH); delay(10); sensorR.init(); sensorR.setAddress(0x32);
    
    
    sensorL.startContinuous(50);
    sensorF.startContinuous(50);
    sensorR.startContinuous(50);
}


void readSensors() {
    if (sensorL.dataReady()) distLeft  = sensorL.read(false);
    if (sensorF.dataReady()) distFront = sensorF.read(false);
    if (sensorR.dataReady()) distRight = sensorR.read(false);
}

