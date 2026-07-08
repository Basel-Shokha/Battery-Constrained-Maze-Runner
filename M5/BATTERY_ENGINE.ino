// ============================================================
//  TAB 6: BATTERY_ENGINE.ino — Isolated Battery Tracker
// ============================================================
#include <Arduino.h>

extern const int CMD_BATTERY_UPDATE;
extern void sendPeripheralCmd(int commandId, int param1, int param2);

// ── GLOBAL REGISTERS ──
uint8_t robotOperatingMode  = 0; // 0 = CONSTRAINED, 1 = CONTINUOUS, 2 = CUSTOM_PATH (Stub)
uint8_t maxBatteryCapacity  = 8;
uint8_t currentBatteryLevel = 8;

// Local tracking variables for continuous sub-step calculations
static uint8_t unitsDroppedThisLeg = 0;
static int lastTrackedStepIndex    = -1;

// ── INITIALIZE ENGINE MODE AND CAPACITIES ──
void initBatteryEngine(uint8_t mode, uint8_t capacity) {
    robotOperatingMode  = mode;
    maxBatteryCapacity  = capacity;
    currentBatteryLevel = capacity;
    unitsDroppedThisLeg = 0;
    lastTrackedStepIndex = -1;
    
    if (robotOperatingMode == 1) {
        // Continuous mode trigger code parameter feed
        sendPeripheralCmd(CMD_BATTERY_UPDATE, 255, 255);
    } else {
        // Constrained mode dynamic boundary feed
        sendPeripheralCmd(CMD_BATTERY_UPDATE, currentBatteryLevel, maxBatteryCapacity);
    }
}

// ── REFRESH PERIPHERAL LED HANDSHAKE FEED ──
void refreshBatteryDisplay() {
    if (robotOperatingMode == 1) {
        sendPeripheralCmd(CMD_BATTERY_UPDATE, 255, 255);
    } else {
        sendPeripheralCmd(CMD_BATTERY_UPDATE, currentBatteryLevel, maxBatteryCapacity);
    }
}

// ── DYNAMIC CORRIDOR WALKING STEP TRACKER ──
// Calculates elapsed time continuously to drop 1 unit every 1.5 seconds while moving forward
void updateBatteryWalking(unsigned long now, unsigned long stepStartTime, int activeStepIndex) {
    if (robotOperatingMode != 0) return; // Ignore tracking if in continuous mode

    // Reset leg tracker if we advanced onto a new path grid element
    if (activeStepIndex != lastTrackedStepIndex) {
        lastTrackedStepIndex = activeStepIndex;
        unitsDroppedThisLeg = 0;
    }

    unsigned long currentStepDuration = now - stepStartTime;
    uint8_t unitsToDrop = currentStepDuration / 1500; // Deduct 1 unit per 1.5s block elapsed

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

// ── STATION CHARGE REFILL ──
void refillBatteryOnCharge() {
    if (robotOperatingMode == 0) {
        currentBatteryLevel = maxBatteryCapacity;
        sendPeripheralCmd(CMD_BATTERY_UPDATE, currentBatteryLevel, maxBatteryCapacity);
    } else if (robotOperatingMode == 1) {
        sendPeripheralCmd(CMD_BATTERY_UPDATE, 255, 255);
    }
}