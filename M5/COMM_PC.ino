#include <M5StickCPlus.h>

// Mirror the structures we mapped out
enum Direction : uint8_t { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

struct MovementStep {
    Direction orientation;
    uint8_t gridsToCross;
    uint8_t expectedStartGrids;
    uint8_t stopLimitGrids;
};

// Global shared variables exposed to the main loop
extern Direction initialSpawnDirection;
extern MovementStep missionPipeline[50];
extern uint8_t totalMissionSteps;
extern int8_t activeStepIndex;

extern volatile uint16_t s_left, s_front, s_right;
extern float yaw;
extern int state; // Reference to state machine index

unsigned long lastPcStreamTime = 0;

// ── STREAM TELEMETRY BACK TO PC EVERY 0.2 SECONDS ─────────
void streamTelemetryToPC(unsigned long now, const char* stateLabel) {
    if (now - lastPcStreamTime >= 200) { // Strict 0.2-second boundary clock
        lastPcStreamTime = now;
        
        // Format: TELEMETRY:step_idx,yaw,left,front,right,state_label
        Serial.printf("TELEMETRY:%d,%05.1f,%d,%d,%d,%s\n", 
                      activeStepIndex, 
                      yaw, 
                      s_left, 
                      s_front, 
                      s_right, 
                      stateLabel);
    }
}

// ── PARSE SERIAL COMMAND PACKETS FROM THE PC DISPATCHER ──
void parseIncomingPcCommands() {
    if (Serial.available() > 0) {
        String line = Serial.readStringUntil('\n');
        line.trim();

        if (line.startsWith("INIT_MISSION:")) {
            // Expected: INIT_MISSION:spawn_dir,total_steps
            int spawnDir, steps;
            if (sscanf(line.c_str(), "INIT_MISSION:%d,%d", &spawnDir, &steps) == 2) {
                initialSpawnDirection = (Direction)spawnDir;
                totalMissionSteps = steps;
                activeStepIndex = -1; // Ready to receive steps
                Serial.println("STATUS:READY_FOR_STEPS");
            }
        }
        else if (line.startsWith("M_STEP:")) {
            // Expected: M_STEP:idx,orient,grids,expect,stop
            int idx, orient, grids, expect, stop;
            if (sscanf(line.c_str(), "M_STEP:%d,%d,%d,%d,%d", &idx, &orient, &grids, &expect, &stop) == 5) {
                if (idx < 50) {
                    missionPipeline[idx].orientation       = (Direction)orient;
                    missionPipeline[idx].gridsToCross      = grids;
                    missionPipeline[idx].expectedStartGrids = expect;
                    missionPipeline[idx].stopLimitGrids    = stop;
                    
                    // Acknowledge receipt of step
                    Serial.printf("STATUS:RCVD_%d\n", idx);
                    
                    // If we just received the final step, arm the engine
                    if (idx == totalMissionSteps - 1) {
                        Serial.println("STATUS:MISSION_ARMED");
                    }
                }
            }
        }
    }
}
