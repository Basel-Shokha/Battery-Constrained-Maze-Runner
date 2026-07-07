// ============================================================
//  TAB 2: COMM_PC.ino — Wi-Fi Network & Event Notification Hub
// ============================================================
#include <WiFi.h>
#include <HTTPClient.h>

extern Direction initialSpawnDirection;
extern MovementStep missionPipeline[50];
extern uint8_t totalMissionSteps;
extern int8_t activeStepIndex;
extern int state;
extern enum DrivePhase currentPhase;

extern volatile uint16_t s_left, s_front, s_right;
extern float yaw;
extern const int CMD_PLAY_AUDIO, AUDIO_MISSION_START, AUDIO_ROUTE_ERROR;
extern void resetSpeedHistory();
extern void resetRunState();
extern void sendPeripheralCmd(int commandId, int param1, int param2);

const char* WIFI_SSID   = "A";
const char* WIFI_PASS   = "00000000";
const char* SERVER_IP   = "192.168.137.212";
const char* SERVER_PORT = "8085";

unsigned long lastPcStreamTime       = 0;
unsigned long lastInstructionPollTime = 0;
unsigned long lastControlPollTime     = 0;

void connectWiFi() {
  M5.Lcd.setTextColor(CYAN); M5.Lcd.println("Connecting Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(250); M5.Lcd.print("."); }
  M5.Lcd.fillScreen(BLACK);
}

// ── UNACKNOWLEDGED HIGH-FREQUENCY TELEMETRY OUTFLOW ─────────
void streamTelemetryToPC(unsigned long now) {
    if (now - lastPcStreamTime >= 200) { 
        lastPcStreamTime = now;
        if (WiFi.status() != WL_CONNECTED) return;
        
        HTTPClient http;
        char url[128];
        sprintf(url, "http://%s:%s/update_telemetry", SERVER_IP, SERVER_PORT);
        http.begin(url);
        http.addHeader("Content-Type", "application/json");

        const char* stateLabel = (state == 1) ? "RUNNING" : "IDLE";
        char jsonBuf[256];
        sprintf(jsonBuf, "{\"stepIdx\":%d,\"yaw\":%.1f,\"left\":%d,\"front\":%d,\"right\":%d,\"state\":\"%s\"}",
                activeStepIndex, yaw, s_left, s_front, s_right, stateLabel);

        http.POST(jsonBuf);
        http.end();
    }
}

// ── RULE 5: SERVER HANDSHAKE FOR EXPANDED DISCRETE WORK EVENTS ──
void sendPCNotification(const char* eventType, int logValue) {
    if (WiFi.status() != WL_CONNECTED) return;
    
    HTTPClient http;
    char url[128];
    sprintf(url, "http://%s:%s/notify_event", SERVER_IP, SERVER_PORT);
    http.begin(url);
    http.addHeader("Content-Type", "application/json");
    
    char jsonBuf[128];
    sprintf(jsonBuf, "{\"event\":\"%s\",\"value\":%d}", eventType, logValue);
    
    int httpCode = http.POST(jsonBuf);
    if (httpCode == HTTP_CODE_OK) {
        Serial.printf("[PC-HANDSHAKE-ACK] Event processed successfully: %s\n", eventType);
    } else {
        Serial.printf("[PC-HANDSHAKE-FAIL] Server return error code: %d\n", httpCode);
    }
    http.end();
}

void pollInstructionsFromServer() {
    if (WiFi.status() != WL_CONNECTED) return;

    HTTPClient http;
    char url[128];
    sprintf(url, "http://%s:%s/get_instructions", SERVER_IP, SERVER_PORT);
    http.begin(url);

    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) {
        String payload = http.getString();
        payload.trim();

        if (payload.length() == 0 || payload.startsWith("START:1,0")) {
            http.end(); return;
        }

        static String lastPayload = "";
        if (payload == lastPayload) { http.end(); return; }
        lastPayload = payload;

        int startIdx = 0; totalMissionSteps = 0;
        while (startIdx < payload.length()) {
            int endIdx = payload.indexOf('\n', startIdx);
            if (endIdx == -1) endIdx = payload.length();
            String line = payload.substring(startIdx, endIdx);
            line.trim(); startIdx = endIdx + 1;

            // ── RULE 5: WIDER COMMAND CONTROLS (PC COMMANDS START JOURNEY) ──
            if (line.startsWith("CMD:START_JOURNEY")) {
                if (totalMissionSteps > 0 && state == 0) {
                    activeStepIndex = 0;
                    extern float targetHeading; targetHeading = yaw; 
                    resetSpeedHistory(); currentPhase = WALKING_FWD;
                    
                    sendPeripheralCmd(CMD_PLAY_AUDIO, AUDIO_MISSION_START, 0);
                    delay(5000); 
                    
                    state = 1; M5.Lcd.fillScreen(BLACK);
                    sendPCNotification("JOURNEY_STARTED_ACK", 1);
                    Serial.println("STATUS:LAUNCHED_VIA_SERVER_CMD");
                }
            }
            else if (line.startsWith("START:")) {
                int spawn, steps;
                if (sscanf(line.c_str(), "START:%d,%d", &spawn, &steps) == 2) {
                    initialSpawnDirection = (Direction)spawn;
                }
            }
            else if (line.startsWith("STEP:")) {
                int idx, orient, grids, expect, stop, act;
                if (sscanf(line.c_str(), "STEP:%d,%d,%d,%d,%d,%d", &idx, &orient, &grids, &expect, &stop, &act) == 6) {
                    if (idx < 50) {
                        missionPipeline[idx].orientation        = (Direction)orient;
                        missionPipeline[idx].gridsToCross       = grids;
                        missionPipeline[idx].expectedStartGrids = expect;
                        missionPipeline[idx].stopLimitGrids     = stop;
                        missionPipeline[idx].action             = act;
                        totalMissionSteps++;
                    }
                }
            }
        }
    }
    http.end();
}


void pollCalibrationRequest() {
    if (WiFi.status() != WL_CONNECTED) return;
    HTTPClient http;
    char url[128];
    sprintf(url, "http://%s:%s/get_calibrate", SERVER_IP, SERVER_PORT);
    http.begin(url);
    if (http.GET() == HTTP_CODE_OK) {
        String payload = http.getString();
        if (payload.indexOf("\"calibrate\":true") >= 0) {
            extern void calibrateGyro();
            M5.Lcd.fillScreen(BLACK);
            M5.Lcd.setTextColor(YELLOW);
            M5.Lcd.println("Calibrating Gyro...");
            calibrateGyro();
            M5.Lcd.fillScreen(BLACK);
            sendPCNotification("GYRO_CALIBRATED_ACK", 1);
        }
    }
    http.end();
}


void pollResetRequest() {
    if (WiFi.status() != WL_CONNECTED) return;
    HTTPClient http;
    char url[128];
    sprintf(url, "http://%s:%s/get_reset", SERVER_IP, SERVER_PORT);
    http.begin(url);
    if (http.GET() == HTTP_CODE_OK) {
        String payload = http.getString();
        if (payload.indexOf("\"reset\":true") >= 0) {
            resetRunState();
            sendPCNotification("RUN_RESET_ACK", 1);
            Serial.println("STATUS:RESET_VIA_SERVER_CMD");
        }
    }
    http.end();
}



void handlePCNetworking(unsigned long now) {
    if (now - lastControlPollTime >= 500) {
        lastControlPollTime = now;
        pollResetRequest();
    }

    if (state == 0 && (now - lastInstructionPollTime >= 1500)) {
        lastInstructionPollTime = now;
        pollInstructionsFromServer();
        pollCalibrationRequest();

    }
    streamTelemetryToPC(now);
}

void triggerNetworkAlert(int conditionId) {
    if (conditionId == 404) {
        sendPeripheralCmd(CMD_PLAY_AUDIO, AUDIO_ROUTE_ERROR, 0);
    }
}
