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

// ── NEW: BLIND SPOT TIMERS & STRAFING FLAGS ──────────────────
int8_t lastStepIndex          = -1;
unsigned long stepStartTime   = 0;
bool isStrafingLeft           = false;
bool isStrafingRight          = false;

volatile uint16_t s_left = 0, s_front = 0, s_right = 0;
unsigned long lastDrawTime   = 0;
unsigned long lastSampleTime = 0;
unsigned long settleTimer    = 0;
int16_t baselineSpeed        = 45; 

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




void setup() {
  M5.begin(); Serial.begin(115200); Wire.begin(0, 26); 
  initPeripheralUART(); 
  M5.IMU.Init(); M5.IMU.SetGyroFsr(MPU6886::GFS_250DPS);
  M5.Lcd.setRotation(3); M5.Lcd.fillScreen(BLACK); M5.Lcd.setTextSize(2);
  connectWiFi(); 
  M5.Lcd.fillScreen(BLACK); M5.Lcd.setTextColor(YELLOW);
  M5.Lcd.println("Calibrating Gyro...");
  calibrateGyro();
  sendPCNotification("GYRO_CALIBRATED_ACK", 1);
  M5.Lcd.fillScreen(BLACK);
}

void loop() {
  updateYaw(); 
  unsigned long now = millis();

  receivePeripheralTelemetry();
  handlePCNetworking(now);



  if (state == 1) {
    // ── FOOLPROOF TIMING LATCH FOR LEG CHANGEOVERS ──────────
    if (activeStepIndex != lastStepIndex) {
        stepStartTime = now;
        lastStepIndex = activeStepIndex;
        isStrafingLeft = false;
        isStrafingRight = false;
    }

    // ── NEW: GLOBAL SIDE-WALL GUARD (BYPASSES GYRO ENTIRELY) ──
    if (!isStrafingLeft && !isStrafingRight) {
        if (s_left > 0 && s_left <= 50) { // 5cm or less on Left
            isStrafingRight = true;
        } else if (s_right > 0 && s_right <= 50) { // 5cm or less on Right
            isStrafingLeft = true;
        }
    }

    if (isStrafingRight) {
        if (s_left >= 90) { // Clear once distance reaches 9cm
            isStrafingRight = false;
        } else {
            // Wheels 1 & 4 forward (+45), Wheels 2 & 3 backward (-45) -> Strafe Right
            setMotors(45, -45, -45, 45);
            goto renderDisplayLink; // Bypass regular driving timeline updates
        }
    }

    if (isStrafingLeft) {
        if (s_right >= 90) { // Clear once distance reaches 9cm
            isStrafingLeft = false;
        } else {
            // Wheels 1 & 4 backward (-45), Wheels 2 & 3 forward (+45) -> Strafe Left
            setMotors(-45, 45, 45, -45);
            goto renderDisplayLink; // Bypass regular driving timeline updates
        }
    }

    // ── REGULAR MISSION DRIVE ENGINES ────────────────────────
    switch (currentPhase) {
      
      case WALKING_FWD: {
        processParallelAlignment(now, lastSampleTime);
        if (activeStepIndex >= 0 && activeStepIndex < totalMissionSteps) {
          MovementStep currentLeg = missionPipeline[activeStepIndex];
          uint16_t stopThresholdMm = (currentLeg.stopLimitGrids * 300) + 150;

          // ── NEW: NON-BLOCKING FRONT SENSOR BLIND SPOT TIMER ──
          unsigned long blindSpotDuration = currentLeg.gridsToCross * 1500;
          bool isBlindSpotActive = (now - stepStartTime < blindSpotDuration);

          if (!isBlindSpotActive && s_front > 0 && s_front <= stopThresholdMm) {
            stopMotors();
            
            if (currentLeg.action == 1) {
              sendPCNotification("ARRIVED_STATION", activeStepIndex);
              chargeStartAckReceived = false;
              esp32ChargeFinished    = false;
              handshakeResendTime    = 0;
              currentPhase           = CHARGE_HANDSHAKE;
            } else {
              activeStepIndex++;
              resetSpeedHistory();
              goto targetTurnCalculation;
            }
          } else {
            walkForward(); // Invokes Argentina parallel tracking loop
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
          currentPhase = CHARGING_EXEC;
        }
        break;
      }

      case CHARGING_EXEC: {
        if (esp32ChargeFinished) {
            sendPeripheralCmd(CMD_STOP_AUDIO, 0, 0);
            sendPeripheralCmd(CMD_BATTERY_UPDATE, 8, 8); 
            sendPCNotification("CHARGING_END", activeStepIndex);
            
            postChargeTimer = now;
            sendPeripheralCmd(CMD_PLAY_AUDIO, AUDIO_GOING_STATION, 0); 
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
          state = 0; stopMotors();
          sendPeripheralCmd(CMD_PLAY_AUDIO, AUDIO_VICTORY, 0);
          sendPCNotification("MISSION_COMPLETE", 200);
          break;
        }

        MovementStep nextLeg = missionPipeline[activeStepIndex];
        if      (nextLeg.orientation == NORTH) turnTargetYaw = 0.0f;
        else if (nextLeg.orientation == EAST)  turnTargetYaw = 90.0f;
        else if (nextLeg.orientation == SOUTH) turnTargetYaw = 180.0f;
        else if (nextLeg.orientation == WEST)  turnTargetYaw = 270.0f;

        if (currentPhase != TURNING) {
            sendPCNotification("TURNING_START", (int)turnTargetYaw);
            lastStuckCheckTime = now; lastStuckCheckYaw = yaw; antiStuckSpeedBoost = 0;
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
        
        if (now - lastStuckCheckTime > 300) {
          float deltaYaw = fabsf(angleDiff(yaw, lastStuckCheckYaw));
          if (deltaYaw < 10.0f) antiStuckSpeedBoost = constrain(antiStuckSpeedBoost + 5, 0, 40);
          else                  antiStuckSpeedBoost = 0;
          lastStuckCheckYaw = yaw; lastStuckCheckTime = now;
        }

        int8_t baseTurningSpeed = (int8_t)constrain(absErr * 0.5f + 18, 18, 20);
        int8_t finalizedSpeed   = baseTurningSpeed + antiStuckSpeedBoost;

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
          resetSpeedHistory(); lastSampleTime = millis();
          currentPhase = WALKING_FWD; 
        } else {
          lastStuckCheckTime = now; lastStuckCheckYaw = yaw; currentPhase = TURNING;
        }
        break;
      }
    }
  } else { 
    stopMotors();
  }

  renderDisplayLink:
  if (now - lastDrawTime > 100) {
    lastDrawTime = now; M5.Lcd.setCursor(0, 0);
    if (state == 0) {
      M5.Lcd.setTextColor(CYAN, BLACK);   M5.Lcd.println("--- WIFI ACTIVE ---");
      M5.Lcd.setTextColor(WHITE, BLACK);   M5.Lcd.printf("IP: %s\n", WiFi.localIP().toString().c_str());
      M5.Lcd.printf("Steps Loaded: %d\n\n", totalMissionSteps);
      M5.Lcd.setTextColor(YELLOW, BLACK); M5.Lcd.println("Awaiting Dispatch...");
    } else {
      M5.Lcd.setTextColor(GREEN, BLACK);  M5.Lcd.printf("RUNNING LEG: %d/%d\n", activeStepIndex + 1, totalMissionSteps);
      M5.Lcd.setTextColor(WHITE, BLACK);
      if (activeStepIndex >= 0 && activeStepIndex < totalMissionSteps) {
        MovementStep activeLeg = missionPipeline[activeStepIndex];
        M5.Lcd.printf("DIR: %s       \n", getDirectionName(activeLeg.orientation));
        if (isStrafingLeft)       M5.Lcd.println("SAFETY: GUARD LEFT ");
        else if (isStrafingRight) M5.Lcd.println("SAFETY: GUARD RIGHT");
        else                      M5.Lcd.printf("Phase: %d     \n", currentPhase);
      }
    }
    M5.Lcd.setTextColor(WHITE, BLACK); M5.Lcd.printf("L:%04d F:%04d R:%04d\n", s_left, s_front, s_right);
  }
  delay(2);
}