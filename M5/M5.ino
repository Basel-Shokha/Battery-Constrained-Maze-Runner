// ============================================================
//  TAB 1: M5.ino — Maze Core State Orchestration Engine
// ============================================================
#include <M5StickCPlus.h>
#include <Wire.h>
#include <math.h>
#include <WiFi.h> 

HardwareSerial ESP32Serial(2);
const int CMD_PLAY_AUDIO     = 1;
const int CMD_STOP_AUDIO     = 2;
const int CMD_START_CHARGE   = 3;
const int CMD_BATTERY_UPDATE = 4;

const int AUDIO_MISSION_START = 101;
const int AUDIO_GOING_STATION = 102;
const int AUDIO_CHARGING_TUNE = 103;
const int AUDIO_ROUTE_ERROR   = 104;
const int AUDIO_VICTORY       = 105;
enum Direction : uint8_t { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };
struct MovementStep {
    Direction orientation;
    uint8_t gridsToCross;
    uint8_t expectedStartGrids;
    uint8_t stopLimitGrids;
    uint8_t action; 
};
Direction initialSpawnDirection = EAST;
Direction currentRobotDirection = EAST; 
MovementStep missionPipeline[50];
uint8_t totalMissionSteps       = 0;
int8_t activeStepIndex          = -1;
int state                       = 0;
enum DrivePhase { WALKING_FWD, CHARGE_HANDSHAKE, CHARGING_EXEC, POST_CHARGE_CHIRP, TURNING, VERIFYING_TURN };
DrivePhase currentPhase = WALKING_FWD;

bool chargeStartAckReceived = false;
bool esp32ChargeFinished     = false;
unsigned long handshakeResendTime = 0;
unsigned long postChargeTimer      = 0;

///#GEMINI
// ── GEMINI: ACTIVE BATTERY LEVEL MANAGEMENT REGISTERS ──
uint8_t currentBatteryLevel = 8;
bool batteryDecrementedThisStep = false;

// ── TUNABLE CONFIGURATION PARAMETERS (EDIT HERE) ──
unsigned long FIRST_GRID_BLIND_TIME = 1300; 
unsigned long REST_GRID_BLIND_TIME  = 1300; 
int8_t BASE_TURN_SPEED              = 23;   
uint8_t MP3_VOLUME                  = 25;   

///#GEMINI
int AUDIO_MISSION_START_TRACK       = 101;  // Track 1 (101 - 100 = 1 on ESP32 side)
///#GEMINI
int AUDIO_GOING_STATION_TRACK       = 103;  // Track 3 (103 - 100 = 3 on ESP32 side)
///#GEMINI
int AUDIO_MISSION_COMPLETE_TRACK    = 104;  // Track 4 (104 - 100 = 4 on ESP32 side)

// ── BLIND SPOT TIMERS & STRAFING FLAGS ──────────────────
int8_t lastStepIndex          = -1;
unsigned long stepStartTime   = 0;
bool isStrafingLeft           = false;
bool isStrafingRight          = false;
volatile uint16_t s_left = 0, s_front = 0, s_right = 0;
unsigned long lastDrawTime   = 0;
unsigned long lastSampleTime = 0;
unsigned long settleTimer    = 0;
int16_t baselineSpeed        = 30; 

unsigned long lastStuckCheckTime = 0;
float lastStuckCheckYaw          = 0.0f;
int16_t antiStuckSpeedBoost      = 0;

extern float yaw, targetHeading, turnTargetYaw, vLeft, vRight;
extern void calibrateGyro();
extern void updateYaw();
extern void walkForward();
extern void resetSpeedHistory();
extern void processParallelAlignment(unsigned long now, unsigned long &lastSampleTime);
extern float angleDiff(float target, float current);
extern float wrap360(float a); 

extern void connectWiFi();
extern void handlePCNetworking(unsigned long now);
extern void sendPCNotification(const char* eventType, int logValue);
extern void initPeripheralUART();
extern void receivePeripheralTelemetry();
extern void sendPeripheralCmd(int commandId, int param1, int param2);

void sendI2C(uint8_t reg, int8_t speed) {
  Wire.beginTransmission(0x38); Wire.write(reg); Wire.write(speed); Wire.endTransmission();
}
void setMotors(int8_t fl, int8_t fr, int8_t rl, int8_t rr) {
  sendI2C(0x00, fl); sendI2C(0x01, fr); sendI2C(0x02, rl); sendI2C(0x03, rr);
}
void stopMotors() { setMotors(0, 0, 0, 0); }

const char* getDirectionName(Direction dir) {
  if (dir == NORTH) return "NORTH";
  if (dir == EAST)  return "EAST";
  if (dir == SOUTH) return "SOUTH"; 
  if (dir == WEST)  return "WEST";
  return "UNKNOWN";
}

void resetRunState() {
    stopMotors();               
    state = 0;                  
    activeStepIndex = -1;       
    totalMissionSteps = 0;
    lastStepIndex = -1;         
    currentPhase = WALKING_FWD; 
    isStrafingLeft = false;
    isStrafingRight = false;
    antiStuckSpeedBoost = 0;

    chargeStartAckReceived = false;
    esp32ChargeFinished    = false;

    ///#GEMINI
    currentBatteryLevel        = 8;
    batteryDecrementedThisStep = false;

    resetSpeedHistory();
    Serial.println("[RESET INTERRUPT] Local run state fully purged. Return to IDLE.");
}

void setup() {
  M5.begin(); Serial.begin(115200); Wire.begin(0, 26); 
  initPeripheralUART(); 
  M5.IMU.Init();
  M5.IMU.SetGyroFsr(MPU6886::GFS_250DPS);
  M5.Lcd.setRotation(3); M5.Lcd.fillScreen(BLACK); M5.Lcd.setTextSize(2);
  connectWiFi(); 
  sendPeripheralCmd(CMD_BATTERY_UPDATE, MP3_VOLUME, MP3_VOLUME);
  M5.Lcd.fillScreen(BLACK); M5.Lcd.setTextColor(YELLOW);
  M5.Lcd.println("Calibrating Gyro...");
  calibrateGyro();
  sendPCNotification("GYRO_CALIBRATED_ACK", 1);
  M5.Lcd.fillScreen(BLACK);
}

void loop() {
  M5.update();
  updateYaw(); 
  unsigned long now = millis();

  if (M5.BtnB.wasPressed() || (Serial.available() > 0 && Serial.read() == 'r')) {
      resetRunState();
      return;
  }

  receivePeripheralTelemetry();

  if (state == 0) {
      handlePCNetworking(now);
  }

  if (state == 1) {
    if (!isStrafingLeft && !isStrafingRight) {
        if (s_left > 0 && s_left <= 50) { isStrafingRight = true; } 
        else if (s_right > 0 && s_right <= 50) { isStrafingLeft = true; }
    }
    if (isStrafingRight) {
        if (s_left >= 90) { isStrafingRight = false; } 
        else { setMotors(baselineSpeed, -baselineSpeed, -baselineSpeed, baselineSpeed); goto renderDisplayLink; }
    }
    if (isStrafingLeft) {
        if (s_right >= 90) { isStrafingLeft = false; } 
        else { setMotors(-baselineSpeed, baselineSpeed, baselineSpeed, -baselineSpeed); goto renderDisplayLink; }
    }

    switch (currentPhase) {
      case WALKING_FWD: {
        if (activeStepIndex != lastStepIndex) {
            stepStartTime = now;
            lastStepIndex = activeStepIndex;
            isStrafingLeft = false;
            isStrafingRight = false;
            ///#GEMINI
            batteryDecrementedThisStep = false; // Clear step tracking flag
        }

        processParallelAlignment(now, lastSampleTime);

        ///#GEMINI
        // ── STEP DECREMENT LATCH (Triggers 1.5s into walking a leg) ──
        if (!batteryDecrementedThisStep && (now - stepStartTime >= FIRST_GRID_BLIND_TIME)) {
            batteryDecrementedThisStep = true;
            if (currentBatteryLevel > 0) {
                currentBatteryLevel--;
            }
            sendPeripheralCmd(CMD_BATTERY_UPDATE, currentBatteryLevel, 8);
        }

        if (activeStepIndex >= 0 && activeStepIndex < totalMissionSteps) {
          MovementStep currentLeg = missionPipeline[activeStepIndex];
          uint16_t stopThresholdMm = (currentLeg.stopLimitGrids * 300) + 150;

          unsigned long blindSpotDuration = 0;
          if (currentLeg.gridsToCross > 0) {
              blindSpotDuration = FIRST_GRID_BLIND_TIME + (currentLeg.gridsToCross - 1) * REST_GRID_BLIND_TIME;
          }
          bool isBlindSpotActive = (now - stepStartTime < blindSpotDuration);
          if (!isBlindSpotActive && s_front > 0 && s_front <= stopThresholdMm) {
            stopMotors();
            if (currentLeg.action == 1) {
              sendPCNotification("ARRIVED_STATION", activeStepIndex);
              chargeStartAckReceived = false;
              esp32ChargeFinished    = false;
              currentPhase = CHARGE_HANDSHAKE;
            } else {
              activeStepIndex++;
              resetSpeedHistory();
              goto targetTurnCalculation;
            }
          } else {
            walkForward();
          }
        }
        break;
      }

      case CHARGE_HANDSHAKE: {
        if (!chargeStartAckReceived) {
          if (now - handshakeResendTime >= 150) { 
              handshakeResendTime = now;
              sendPeripheralCmd(CMD_PLAY_AUDIO, AUDIO_CHARGING_TUNE, 0);
              sendPeripheralCmd(CMD_START_CHARGE, 1, 0);
          }
        } else {
          sendPCNotification("CHARGING_START", activeStepIndex);

          ///#GEMINI
          // ── INSTANT REFILL ON VALID CHARGING HANDSHAKE ACK ──
          currentBatteryLevel = 8;
          sendPeripheralCmd(CMD_BATTERY_UPDATE, currentBatteryLevel, 8);

          currentPhase = CHARGING_EXEC;
        }
        break;
      }

      case CHARGING_EXEC: {
        if (esp32ChargeFinished) {
            sendPCNotification("CHARGING_END", activeStepIndex);
            sendPeripheralCmd(CMD_STOP_AUDIO, 0, 0);
            sendPeripheralCmd(CMD_BATTERY_UPDATE, 8, 8); 
            postChargeTimer = now;
            currentPhase = POST_CHARGE_CHIRP;
        }
        break;
      }

      case POST_CHARGE_CHIRP: {
        if (now - postChargeTimer >= 1500) {
            activeStepIndex++;
            resetSpeedHistory();
            goto targetTurnCalculation;
        }
        break;
      }

      targetTurnCalculation:
      case TURNING: {
        if (activeStepIndex >= totalMissionSteps) {
          state = 0;
          stopMotors();
          sendPeripheralCmd(CMD_PLAY_AUDIO, AUDIO_MISSION_COMPLETE_TRACK, 0);
          sendPCNotification("MISSION_COMPLETE", 200);
          break;
        }

        MovementStep nextLeg = missionPipeline[activeStepIndex];
        if (currentPhase != TURNING) {
            if (activeStepIndex == 0) currentRobotDirection = initialSpawnDirection;
            Direction nextDir = nextLeg.orientation;
            
            if (currentRobotDirection == nextDir) {
                targetHeading = yaw;
                resetSpeedHistory(); lastSampleTime = millis();
                currentPhase = WALKING_FWD; 
                if (missionPipeline[activeStepIndex].action == 1) {
                    sendPeripheralCmd(CMD_PLAY_AUDIO, AUDIO_GOING_STATION_TRACK, 0);
                }
                break;
            }

            float relativeAngleDelta = 0.0f;
            if ((currentRobotDirection + 1) % 4 == nextDir)      relativeAngleDelta = 90.0f;
            else if ((currentRobotDirection + 2) % 4 == nextDir) relativeAngleDelta = 180.0f;
            else if ((currentRobotDirection + 3) % 4 == nextDir) relativeAngleDelta = -90.0f; 

            turnTargetYaw = wrap360(yaw + relativeAngleDelta);
            sendPCNotification("TURNING_START", (int)turnTargetYaw);
            lastStuckCheckTime = now;
            lastStuckCheckYaw = yaw; antiStuckSpeedBoost = 0;
            currentPhase = TURNING;
        }

        float error  = angleDiff(turnTargetYaw, yaw);
        float absErr = fabsf(error);
        if (absErr <= 2.0f) {
          if (error > 0) setMotors(-25, 25, -25, 25);
          else           setMotors(25, -25, 25, -25);
          delay(15); stopMotors(); settleTimer = millis();
          currentPhase = VERIFYING_TURN;
          break;
        }
        
        if (now - lastStuckCheckTime >= 300) {
          float deltaYaw = fabsf(angleDiff(yaw, lastStuckCheckYaw));
          if (deltaYaw < 10.0f) antiStuckSpeedBoost = constrain(antiStuckSpeedBoost + 1, 0, 30);
          else                  antiStuckSpeedBoost = 0;
          lastStuckCheckYaw = yaw; lastStuckCheckTime = now;
        }

        int8_t finalizedSpeed = BASE_TURN_SPEED + antiStuckSpeedBoost;
        if (error > 0) setMotors(finalizedSpeed, -finalizedSpeed, finalizedSpeed, -finalizedSpeed);
        else           setMotors(-finalizedSpeed, finalizedSpeed, -finalizedSpeed, finalizedSpeed);
        break;
      }

      case VERIFYING_TURN: {
        if (millis() - settleTimer < 100) break;
        float error = angleDiff(turnTargetYaw, yaw);
        if (fabsf(error) <= 2.0f) {
          targetHeading = turnTargetYaw;
          sendPCNotification("TURN_CONFIRMED_ACK", (int)targetHeading);
          currentRobotDirection = missionPipeline[activeStepIndex].orientation;
          resetSpeedHistory(); lastSampleTime = millis();
          currentPhase = WALKING_FWD;
          if (missionPipeline[activeStepIndex].action == 1) {
              sendPeripheralCmd(CMD_PLAY_AUDIO, AUDIO_GOING_STATION_TRACK, 0);
          }
        } else {
          lastStuckCheckTime = now; lastStuckCheckYaw = yaw;
          currentPhase = TURNING;
        }
        break;
      }
    }
  } else { stopMotors(); }

  renderDisplayLink:
  if (now - lastDrawTime > 100) {
    lastDrawTime = now; M5.Lcd.setCursor(0, 0);
    if (state == 0) {
      M5.Lcd.setTextColor(CYAN, BLACK);   M5.Lcd.println("--- WIFI ACTIVE ---");
      M5.Lcd.setTextColor(WHITE, BLACK);
      M5.Lcd.printf("IP: %s\n", WiFi.localIP().toString().c_str());
      M5.Lcd.printf("Steps Loaded: %d\n\n", totalMissionSteps);
      M5.Lcd.setTextColor(YELLOW, BLACK); M5.Lcd.println("Awaiting Dispatch...");
    } else {
      M5.Lcd.setTextColor(GREEN, BLACK);  M5.Lcd.printf("RUNNING LEG: %d/%d\n", activeStepIndex + 1, totalMissionSteps);
    }
    M5.Lcd.setTextColor(WHITE, BLACK); M5.Lcd.printf("L:%04d F:%04d R:%04d\n", s_left, s_front, s_right);
  }
  delay(2);
}

