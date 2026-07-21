
#include <Arduino.h>

extern const int CMD_BATTERY_UPDATE;
extern void sendPeripheralCmd(int commandId, int param1, int param2);


uint8_t robotOperatingMode  = 0; 
uint8_t maxBatteryCapacity  = 8;
uint8_t currentBatteryLevel = 8;


static uint8_t unitsDroppedThisLeg = 0;
static int lastTrackedStepIndex    = -1;


void initBatteryEngine(uint8_t mode, uint8_t capacity) {
    robotOperatingMode  = mode;
    maxBatteryCapacity  = capacity;
    currentBatteryLevel = capacity;
    unitsDroppedThisLeg = 0;
    lastTrackedStepIndex = -1;
    
    if (robotOperatingMode == 1) {
        
        sendPeripheralCmd(CMD_BATTERY_UPDATE, 255, 255);
    } else {
        
        sendPeripheralCmd(CMD_BATTERY_UPDATE, currentBatteryLevel, maxBatteryCapacity);
    }
}


void refreshBatteryDisplay() {
    if (robotOperatingMode == 1) {
        sendPeripheralCmd(CMD_BATTERY_UPDATE, 255, 255);
    } else {
        sendPeripheralCmd(CMD_BATTERY_UPDATE, currentBatteryLevel, maxBatteryCapacity);
    }
}


void updateBatteryWalking(unsigned long now, unsigned long stepStartTime, int activeStepIndex) {
    if (robotOperatingMode != 0) return; 

    
    if (activeStepIndex != lastTrackedStepIndex) {
        lastTrackedStepIndex = activeStepIndex;
        unitsDroppedThisLeg = 0;
    }

    unsigned long currentStepDuration = now - stepStartTime;
    uint8_t unitsToDrop = currentStepDuration / 1500; 

    if (unitsToDrop > unitsDroppedThisLeg) {
        uint8_t diff = unitsToDrop - unitsDroppedThisLeg;
        if (currentBatteryLevel >= diff) {
            currentBatteryLevel -= diff;
        } else {
            currentBatteryLevel = 0;
        }
        unitsDroppedThisLeg = unitsToDrop;
        sendPeripheralCmd(CMD_BATTERY_UPDATE, currentBatteryLevel, maxBatteryCapacity);
    }
}


void refillBatteryOnCharge() {
    if (robotOperatingMode == 0) {
        currentBatteryLevel = maxBatteryCapacity;
        sendPeripheralCmd(CMD_BATTERY_UPDATE, currentBatteryLevel, maxBatteryCapacity);
    } else if (robotOperatingMode == 1) {
        sendPeripheralCmd(CMD_BATTERY_UPDATE, 255, 255);
    }
}