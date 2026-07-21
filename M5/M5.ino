
#include <M5StickCPlus.h>
#include <Wire.h>
#include <math.h>
#include <WiFi.h> 

HardwareSerial ESP32Serial(2);
const int CMD_PLAY_AUDIO     = 1;
const int CMD_STOP_AUDIO     = 2;
const int CMD_START_CHARGE   = 3;
const int CMD_BATTERY_UPDATE = 4;
const int CMD_RED_LED_RING   = 5; 

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

extern uint8_t robotOperatingMode;
extern uint8_t maxBatteryCapacity;
extern uint8_t currentBatteryLevel;
extern void initBatteryEngine(uint8_t mode, uint8_t capacity);
extern void refreshBatteryDisplay();
extern void updateBatteryWalking(unsigned long now, unsigned long stepStartTime, int activeStepIndex);
extern void refillBatteryOnCharge();

bool journeyJustStarted = true;


unsigned long FIRST_GRID_BLIND_TIME = 1500; 
unsigned long REST_GRID_BLIND_TIME  = 1300;
int8_t BASE_TURN_SPEED              = 23; 
uint8_t MP3_VOLUME                  = 25;

int AUDIO_MISSION_START_TRACK       = 101; 
int AUDIO_GOING_STATION_TRACK       = 103; 
int AUDIO_MISSION_COMPLETE_TRACK    = 104; 

int8_t lastStepIndex          = -1;
unsigned long stepStartTime   = 0;
bool isStrafingLeft           = false;
bool isStrafingRight          = false;
volatile uint16_t s_left = 0, s_front = 0, s_right = 0;
unsigned long lastDrawTime   = 0;
unsigned long lastSampleTime = 0;
unsigned long settleTimer    = 0;
int16_t baselineSpeed        = 25; 

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
extern bool pcConnected;
extern bool routeErrorFlag;

uint8_t batteryAtStartOfLeg = 8;

unsigned long journeyStartTime          = 0;
unsigned long journeyEndTime            = 0; 
uint32_t totalGridsWalked               = 0;
uint32_t totalChargingStationsCleared   = 0;
uint32_t totalSteeringCorrections       = 0;
uint32_t totalPivotTurns                = 0;
bool showSummaryScreen                  = false;
bool summaryScreenCleared               = false; 
extern bool isAligning;

void sendI2C(uint8_t reg, int8_t speed) {
  Wire.beginTransmission(0x38); Wire.write(reg); Wire.write(speed); Wire.endTransmission();
}
void setMotors(int8_t fl, int8_t fr, int8_t rl, int8_t rr) {
  sendI2C(0x00, fl); sendI2C(0x01, fr); sendI2C(0x02, rl); sendI2C(0x03, rr);
}
void stopMotors() { setMotors(0, 0, 0, 0); }

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
    journeyJustStarted     = true;
    showSummaryScreen      = false;
    summaryScreenCleared   = false;
    routeErrorFlag         = false;

    sendPeripheralCmd(CMD_BATTERY_UPDATE, 0, 0);
    resetSpeedHistory();
    M5.Lcd.fillScreen(BLACK);
    Serial.println("[RESET INTERRUPT] Local run state fully purged. Return to IDLE.");
}

void setup() {
  M5.begin(); Serial.begin(115200);
  Wire.begin(0, 26); 
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
  
  if (M5.BtnA.wasPressed() || M5.BtnB.wasPressed() || (Serial.available() > 0 && Serial.read() == 'r')) {
      resetRunState();
      return;
  }

  receivePeripheralTelemetry();
  handlePCNetworking(now);

  if (state == 1) {
    if (journeyJustStarted) {
        journeyJustStarted = false;
        currentPhase = WALKING_FWD; 
        
        journeyStartTime             = millis();
        journeyEndTime               = 0;
        totalGridsWalked             = 0;
        totalChargingStationsCleared = 0;
        totalSteeringCorrections     = 0;
        totalPivotTurns              = 0;
        showSummaryScreen            = false;
        summaryScreenCleared         = false;
        
        goto targetTurnCalculation;
    }

    if (!isStrafingLeft && !isStrafingRight) {
        if (s_left > 0 && s_left <= 75) { isStrafingRight = true; } 
        else if (s_right > 0 && s_right <= 75) { isStrafingLeft = true; }
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
            batteryAtStartOfLeg = currentBatteryLevel;
        }

        processParallelAlignment(now, lastSampleTime);
        
        static bool lastAlignState = false;
        if (isAligning && !lastAlignState) {
            totalSteeringCorrections++;
        }
        lastAlignState = isAligning;
        
        updateBatteryWalking(now, stepStartTime, activeStepIndex);

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
            
            if (batteryAtStartOfLeg >= currentLeg.gridsToCross) {
                currentBatteryLevel = batteryAtStartOfLeg - currentLeg.gridsToCross;
            } else {
                currentBatteryLevel = 0;
            }
            refreshBatteryDisplay();
            totalGridsWalked += currentLeg.gridsToCross;

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
          refillBatteryOnCharge();
          currentPhase = CHARGING_EXEC;
        }
        break;
      }

      case CHARGING_EXEC: {
        if (esp32ChargeFinished) {
            sendPCNotification("CHARGING_END", activeStepIndex);
            sendPeripheralCmd(CMD_STOP_AUDIO, 0, 0);
            refreshBatteryDisplay();
            totalChargingStationsCleared++; 
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
          journeyEndTime = millis(); 
          sendPeripheralCmd(CMD_PLAY_AUDIO, AUDIO_MISSION_COMPLETE_TRACK, 0);
          sendPCNotification("MISSION_COMPLETE", 200);
          showSummaryScreen = true; 
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
                if (activeStepIndex > 0 && missionPipeline[activeStepIndex].action == 1) {
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
            
            refreshBatteryDisplay();
            totalPivotTurns++; 
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
          lastStuckCheckTime = now;
          lastStuckCheckYaw = yaw;
          currentPhase = TURNING;
        }
        break;
    }
}
  } else { stopMotors(); }

  renderDisplayLink:
  if (now - lastDrawTime > 100) {
    lastDrawTime = now; 
    
    if (showSummaryScreen) {
      if (!summaryScreenCleared) {
          summaryScreenCleared = true;
          M5.Lcd.fillScreen(BLACK); 
      }
      M5.Lcd.setTextSize(2);
      M5.Lcd.setCursor(0, 0);
      M5.Lcd.setTextColor(GREEN, BLACK);
      M5.Lcd.println("=== JOURNEY SUMMARY ===");
      M5.Lcd.setTextColor(WHITE, BLACK);
      M5.Lcd.printf("Dist: %d Grids-%.1fm\n", totalGridsWalked, (float)totalGridsWalked * 0.3f);
      M5.Lcd.printf("Docks: %d Pitstops ⚡\n", totalChargingStationsCleared);
      M5.Lcd.printf("Turns: %d Pivot Turns\n", totalPivotTurns);
      
      float elapsedSec = (journeyEndTime - journeyStartTime) / 1000.0f; 
      M5.Lcd.printf("Time : %.1f seconds\n", elapsedSec);
      float avgSpd = 0.0f;
      if (elapsedSec > 0.0f) {
          avgSpd = (totalGridsWalked * 30.0f) / elapsedSec;
      }
      M5.Lcd.printf("Spd  : %.1f cm/s\n", avgSpd);
      M5.Lcd.printf("Steer: %d Adjustments\n", totalSteeringCorrections);
      M5.Lcd.setTextColor(YELLOW, BLACK);
      M5.Lcd.println("=======================");
      M5.Lcd.setTextSize(2);
    }
    else {
      M5.Lcd.setCursor(0, 0);
      if (routeErrorFlag) {
          M5.Lcd.fillScreen(RED); 
          M5.Lcd.setTextColor(WHITE, RED);
          M5.Lcd.println("===================");
          M5.Lcd.println("   ROUTE ERROR!    ");
          M5.Lcd.println(" MAZE INFEASIBLE   ");
          M5.Lcd.println("===================");
          routeErrorFlag = false; 
      }
      else if (state == 0) {
        M5.Lcd.setTextColor(CYAN, BLACK);   M5.Lcd.println("--- WIFI ACTIVE ---");
        M5.Lcd.setTextColor(WHITE, BLACK);
        M5.Lcd.printf("IP: %s\n", WiFi.localIP().toString().c_str());
        M5.Lcd.printf("Steps Loaded: %d\n\n", totalMissionSteps);
        
        if (WiFi.status() != WL_CONNECTED || !pcConnected) {
            M5.Lcd.setTextColor(RED, BLACK); M5.Lcd.println("PC: OFFLINE");
        } else {
            M5.Lcd.setTextColor(YELLOW, BLACK); M5.Lcd.println("Awaiting Dispatch...");
        }
      } else {
        M5.Lcd.setTextColor(GREEN, BLACK);  M5.Lcd.printf("RUNNING LEG: %d/%d\n", activeStepIndex + 1, totalMissionSteps);
        M5.Lcd.setTextColor(WHITE, BLACK); M5.Lcd.printf("L:%04d F:%04d R:%04d\n", s_left, s_front, s_right);
      }
    }
  }
  delay(2);
}

