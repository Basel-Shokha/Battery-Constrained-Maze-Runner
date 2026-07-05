// ============================================================
//  M5StickC Plus — Robot Master Controller (Pure Wireless Engine)
//  PC Hotspot Communication (192.168.137.1:8085)
// ============================================================
#include <M5StickCPlus.h>
#include <Wire.h>
#include <math.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <HTTPClient.h>

// ── NETWORK CONFIGURATION ────────────────────────────────
const char* WIFI_SSID = "A";
const char* WIFI_PASS = "00000000";
const char* SERVER_IP = "192.168.137.1";
const char* SERVER_PORT = "8085";

HardwareSerial ESP32Serial(2);

enum Direction : uint8_t { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };
struct MovementStep {
    Direction orientation;
    uint8_t gridsToCross;
    uint8_t expectedStartGrids;
    uint8_t stopLimitGrids;
};

Direction initialSpawnDirection = EAST;
MovementStep missionPipeline[50];
uint8_t totalMissionSteps       = 0;
int8_t activeStepIndex          = -1;
int state                       = 0;  // 0 = IDLE/STOPPED, 1 = RUNNING

volatile uint16_t s_left = 0, s_front = 0, s_right = 0;
unsigned long lastDrawTime   = 0;
unsigned long lastSampleTime = 0;
unsigned long lastPcStreamTime = 0;
unsigned long lastInstructionPollTime = 0;
int16_t baselineSpeed = 45;

extern float yaw, targetHeading;
extern void calibrateGyro();
extern void updateYaw();
extern void walkForward();
extern void resetSpeedHistory();
extern void processParallelAlignment(unsigned long now, unsigned long &lastSampleTime);

void sendI2C(uint8_t reg, int8_t speed) {
  Wire.beginTransmission(0x38); 
  Wire.write(reg); 
  Wire.write(speed); 
  Wire.endTransmission();
}

void setMotors(int8_t fl, int8_t fr, int8_t rl, int8_t rr) {
  sendI2C(0x00, fl); 
  sendI2C(0x01, fr); 
  sendI2C(0x02, rl); 
  sendI2C(0x03, rr);
}

void stopMotors() { 
  setMotors(0, 0, 0, 0); 
}

const char* getDirectionName(Direction dir) {
  if (dir == NORTH) return "NORTH";
  if (dir == EAST) return "EAST";
  if (dir == SOUTH) return "SOUTH"; 
  if (dir == WEST) return "WEST";
  return "UNKNOWN";
}

// ── STREAM TELEMETRY BACK TO PC EVERY 0.2 SECONDS ─────────
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

// ── POLL AND PARSE DYNAMIC TEXT MAZE INSTRUCTIONS OVER WI-FI ──
void pollInstructionsFromServer() {
    if (state != 0) return; // Only scan the network if we are sitting idle
    if (WiFi.status() != WL_CONNECTED) return;

    HTTPClient http;
    char url[128];
    sprintf(url, "http://%s:%s/get_instructions", SERVER_IP, SERVER_PORT);
    http.begin(url);

    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) {
        String payload = http.getString();
        payload.trim();

        // Avoid zero-element processing steps on baseline initialization templates
        if (payload.length() == 0 || payload.startsWith("START:1,0")) {
            http.end();
            return;
        }

        // Cache verification string check to ensure we don't spam parsing routines
        static String lastPayload = "";
        if (payload == lastPayload) {
            http.end();
            return;
        }
        lastPayload = payload;

        int startIdx = 0;
        totalMissionSteps = 0;

        // Parse custom line-separated matrix commands natively over the air
        while (startIdx < payload.length()) {
            int endIdx = payload.indexOf('\n', startIdx);
            if (endIdx == -1) endIdx = payload.length();
            String line = payload.substring(startIdx, endIdx);
            line.trim();
            startIdx = endIdx + 1;

            if (line.startsWith("START:")) {
                int spawn, steps;
                if (sscanf(line.c_str(), "START:%d,%d", &spawn, &steps) == 2) {
                    initialSpawnDirection = (Direction)spawn;
                }
            }
            else if (line.startsWith("STEP:")) {
                int idx, orient, grids, expect, stop;
                if (sscanf(line.c_str(), "STEP:%d,%d,%d,%d,%d", &idx, &orient, &grids, &expect, &stop) == 5) {
                    if (idx < 50) {
                        missionPipeline[idx].orientation        = (Direction)orient;
                        missionPipeline[idx].gridsToCross       = grids;
                        missionPipeline[idx].expectedStartGrids = expect;
                        missionPipeline[idx].stopLimitGrids     = stop;
                        totalMissionSteps++;
                    }
                }
            }
        }

        // AUTO-LAUNCH: Trigger mission execution state the millisecond instructions drop via Wi-Fi
        if (totalMissionSteps > 0) {
            activeStepIndex = 0;
            targetHeading = yaw; 
            resetSpeedHistory(); 
            lastSampleTime = millis();
            ESP32Serial.println("COLOR_GREEN"); 
            ESP32Serial.println("AUDIO_PLAY");
            state = 1; // Kick state directly to RUNNING
            M5.Lcd.fillScreen(BLACK);
            Serial.println("STATUS:MISSION_STARTED_VIA_WIFI");
        }
    }
    http.end();
}

// =============================================================
//  Setup
// =============================================================
void setup() {
  M5.begin(); 
  Serial.begin(115200); 
  Wire.begin(0, 26);
  ESP32Serial.begin(115200, SERIAL_8N1, 32, 33);
  
  M5.IMU.Init(); 
  M5.IMU.SetGyroFsr(MPU6886::GFS_250DPS);
  M5.Lcd.setRotation(3); 
  M5.Lcd.fillScreen(BLACK); 
  M5.Lcd.setTextSize(2);
  
  // Bind onto the unchangeable 2.4GHz laptop hotspot bubble
  M5.Lcd.setTextColor(CYAN); 
  M5.Lcd.println("Connecting Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) { 
    delay(250); 
    M5.Lcd.print("."); 
  }
  
  M5.Lcd.fillScreen(BLACK); 
  M5.Lcd.setTextColor(YELLOW);
  M5.Lcd.println("Calibrating Gyro...");
  calibrateGyro(); 
  M5.Lcd.fillScreen(BLACK);
}

// =============================================================
//  Loop
// =============================================================
void loop() {
  updateYaw(); 
  unsigned long now = millis();

  // ── READ SENSOR DATA FROM ESP32 ────────────────────────
  if (ESP32Serial.available() > 0) {
    String inLine = ESP32Serial.readStringUntil('\n');
    inLine.trim();
    if (inLine.startsWith("DIST:")) {
      int pL, pF, pR;
      if (sscanf(inLine.c_str(), "DIST:%d,%d,%d", &pL, &pF, &pR) == 3) {
        s_left = pL; s_front = pF; s_right = pR;
      }
    }
  }

  // ── POLL PC SERVER FOR NEW INSTRUCTIONS EVERY 1.5 SECONDS ──
  if (state == 0 && (now - lastInstructionPollTime >= 1500)) {
      lastInstructionPollTime = now;
      pollInstructionsFromServer();
  }

  // ── CLOSED-LOOP STEP ADVANCEMENT ──────────────────────────
  if (state == 1) {
    processParallelAlignment(now, lastSampleTime);
    if (activeStepIndex >= 0 && activeStepIndex < totalMissionSteps) {
      MovementStep currentLeg = missionPipeline[activeStepIndex];
      uint16_t stopThresholdMm = (currentLeg.stopLimitGrids * 300) + 150;

      if (s_front > 0 && s_front <= stopThresholdMm) {
        activeStepIndex++;
        resetSpeedHistory();
        if (activeStepIndex >= totalMissionSteps) { 
          state = 0; 
          stopMotors();
          ESP32Serial.println("AUDIO_STOP");
          Serial.println("STATUS:MISSION_COMPLETE");
        }
      } else {
        walkForward();
      }
    }
  } else { 
    stopMotors();
  }

  // ── STREAM TELEMETRY BACK TO PC EVERY 200ms ───────────
  streamTelemetryToPC(now);

  // ── LCD DISPLAY UPDATE ─────────────────────────────────
  if (now - lastDrawTime > 100) {
    lastDrawTime = now;
    M5.Lcd.setCursor(0, 0);
    if (state == 0) {
      M5.Lcd.setTextColor(CYAN, BLACK); 
      M5.Lcd.println("--- WIFI IDLE ---");
      M5.Lcd.setTextColor(WHITE, BLACK);
      M5.Lcd.printf("IP: %s\n", WiFi.localIP().toString().c_str());
      M5.Lcd.printf("Steps: %d\n\n", totalMissionSteps);
      M5.Lcd.setTextColor(YELLOW, BLACK); 
      M5.Lcd.println("Awaiting Chrome...");
    } else {
      M5.Lcd.setTextColor(GREEN, BLACK); 
      M5.Lcd.printf("STEP: %d/%d\n", activeStepIndex + 1, totalMissionSteps);
      M5.Lcd.setTextColor(WHITE, BLACK);
      if (activeStepIndex >= 0 && activeStepIndex < totalMissionSteps) {
        MovementStep activeLeg = missionPipeline[activeStepIndex];
        M5.Lcd.printf("DIR: %s\n", getDirectionName(activeLeg.orientation));
        M5.Lcd.printf("Stop: %d grids\n\n", activeLeg.stopLimitGrids);
      }
    }
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.printf("L:%04d F:%04d R:%04d\n", s_left, s_front, s_right);
  }
  delay(2);
}