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

extern int AUDIO_MISSION_START_TRACK;
extern int AUDIO_GOING_STATION_TRACK;

///#GEMINI: References linked into the configuration parser
extern uint8_t robotOperatingMode;
extern uint8_t maxBatteryCapacity;
extern uint8_t currentBatteryLevel;
extern void initBatteryEngine(uint8_t mode, uint8_t capacity);

const char* WIFI_SSID   = "A";
const char* WIFI_PASS   = "00000000";
const char* SERVER_IP   = "192.168.137.1";
const char* SERVER_PORT = "8085";

unsigned long lastPcStreamTime       = 0;
unsigned long lastInstructionPollTime = 0;
unsigned long lastControlPollTime     = 0;

unsigned long TELEMETRY_INTERVAL_MS = 1000;

void connectWiFi() {
  M5.Lcd.setTextColor(CYAN); M5.Lcd.println("Connecting Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(250); M5.Lcd.print("."); }
  M5.Lcd.fillScreen(BLACK);
}

void streamTelemetryToPC(unsigned long now) {
    if (state == 1 && currentPhase != WALKING_FWD) return;
    if (now - lastPcStreamTime >= TELEMETRY_INTERVAL_MS) { 
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
            http.end();
            return;
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
            
            ///#GEMINI: Intercept infeasible route profiles immediately
            if (line.startsWith("CMD:ROUTE_ERROR")) {
                sendPeripheralCmd(CMD_PLAY_AUDIO, 105, 0); // Plays file 005 in Directory 01
                Serial.println("[ROUTE ERROR] Server signaled route is infeasible.");
            }
            else if (line.startsWith("CMD:START_JOURNEY")) {
                if (totalMissionSteps > 0 && state == 0) {
                    activeStepIndex = 0;
                    extern float targetHeading; targetHeading = yaw; 
                    resetSpeedHistory(); currentPhase = WALKING_FWD;
                    
                    ///#GEMINI: Refactored battery initialization engine call
                    initBatteryEngine(robotOperatingMode, maxBatteryCapacity);
                    
                    ///#GEMINI: Audio sequence fix for Step 0 charge node layout launches
                    sendPeripheralCmd(CMD_PLAY_AUDIO, AUDIO_MISSION_START_TRACK, 0);
                    delay(5000); // Allow track 1 to play through completely
                    
                    if (missionPipeline[0].action == 1) {
                        sendPeripheralCmd(CMD_PLAY_AUDIO, AUDIO_GOING_STATION_TRACK, 0);
                        delay(1500); // Secondary clear block window for track 3
                    }
                    
                    state = 1; M5.Lcd.fillScreen(BLACK);
                    sendPCNotification("JOURNEY_STARTED_ACK", 1);
                    Serial.println("STATUS:LAUNCHED_VIA_SERVER_CMD");
                }
            }
            else if (line.startsWith("START:")) {
                int spawn, steps, parsedMode = 0, parsedCap = 8;
                ///#GEMINI: Expanded parameter payload scan matching main.cpp
                if (sscanf(line.c_str(), "START:%d,%d,%d,%d", &spawn, &steps, &parsedMode, &parsedCap) >= 2) {
                    initialSpawnDirection = (Direction)spawn;
                    robotOperatingMode  = parsedMode;
                    maxBatteryCapacity  = parsedCap;
                    currentBatteryLevel = parsedCap;
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