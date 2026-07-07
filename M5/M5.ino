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

// ── BLIND SPOT TIMERS & STRAFING FLAGS ──────────────────
int8_t lastStepIndex          = -1;
unsigned long stepStartTime   = 0;
bool isStrafingLeft           = false;
bool isStrafingRight          = false;

// ── RELATIVE FRONT-STOP TUNING ─────────────────────────────
const int GRID_MM             = 300;   // physical length of one grid cell (mm)
const int FRONT_STOP_MARGIN   = 100;   // R0 must clear grids*GRID_MM by this much to be trusted (mm)
const int FRONT_STOP_MAX_MM   = 3000;  // readings beyond this = "no wall in range" -> use fallback
const int FRONT_STOP_DEBOUNCE = 3;     // consecutive qualifying frames required before stopping

// ── RELATIVE FRONT-STOP STATE (latched fresh each leg) ─────
uint16_t legStartFrontMm   = 0;        // R0: front reading captured when this leg's drive begins
uint16_t frontStopTargetMm = 0;        // target = R0 - gridsToCross*GRID_MM
bool     relativeStopArmed = false;    // true when R0 was a valid wall with room to move
uint8_t  frontStopHits     = 0;        // debounce counter for the stop condition

volatile uint16_t s_left = 0, s_front = 0, s_right = 0;
unsigned long lastDrawTime   = 0;
unsigned long lastSampleTime = 0;
unsigned long settleTimer    = 0;
int16_t baselineSpeed        = 20; 

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
  sendPeripheralCmd(CMD_STOP_AUDIO, 0, 0);

  state                  = 0;
  activeStepIndex        = -1;
  currentRobotDirection  = initialSpawnDirection;
  currentPhase           = WALKING_FWD;

  chargeStartAckReceived = false;
  esp32ChargeFinished    = false;
  handshakeResendTime    = 0;
  postChargeTimer        = 0;

  lastStepIndex          = -1;
  stepStartTime          = 0;
  isStrafingLeft         = false;
  isStrafingRight        = false;

  legStartFrontMm        = 0;
  frontStopTargetMm      = 0;
  relativeStopArmed      = false;
  frontStopHits          = 0;

  settleTimer            = 0;
  lastStuckCheckTime     = 0;
  lastStuckCheckYaw      = yaw;
  antiStuckSpeedBoost    = 0;

  targetHeading          = yaw;
  turnTargetYaw          = yaw;
  resetSpeedHistory();
  lastSampleTime         = millis();

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(CYAN, BLACK);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.println("Run Reset");
  M5.Lcd.println("Ready");
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
    // ── GLOBAL SIDE-WALL GUARD (BYPASSES GYRO ENTIRELY) ──
    if (!isStrafingLeft && !isStrafingRight) {
        if (s_left > 0 && s_left <= 50) { 
            isStrafingRight = true;
        } else if (s_right > 0 && s_right <= 50) { 
            isStrafingLeft = true;
        }
    }

    if (isStrafingRight) {
        if (s_left >= 90) { 
            isStrafingRight = false;
        } else {
            setMotors(45, -45, -45, 45);
            goto renderDisplayLink; 
        }
    }

    if (isStrafingLeft) {
        if (s_right >= 90) { 
            isStrafingLeft = false;
        } else {
            setMotors(-45, 45, 45, -45);
            goto renderDisplayLink; 
        }
    }

    // ── REGULAR MISSION DRIVE ENGINES ────────────────────────
    switch (currentPhase) {
      
      case WALKING_FWD: {
        // ── FIXED LATCH: STARTS THE TIMER ONLY WHEN WE ACTUALLY BEGIN DRIVING ──
        if (activeStepIndex != lastStepIndex) {
          stepStartTime = now;
          lastStepIndex = activeStepIndex;
          isStrafingLeft = false;
          isStrafingRight = false;

          // ── LATCH R0 FOR THIS LEG (relative front-stop) ──
          int gridsMm       = missionPipeline[activeStepIndex].gridsToCross * GRID_MM;
          legStartFrontMm   = s_front;
          frontStopHits     = 0;
          // Trust the relative method only if we latched a real wall with room to move.
          if (legStartFrontMm > 0 &&
              legStartFrontMm <= FRONT_STOP_MAX_MM &&
              legStartFrontMm >= gridsMm + FRONT_STOP_MARGIN) {
              frontStopTargetMm = (uint16_t)(legStartFrontMm - gridsMm);
              relativeStopArmed = true;
          } else {
              relativeStopArmed = false;   // fall back to absolute threshold this leg
          }
        }

        processParallelAlignment(now, lastSampleTime);
        if (activeStepIndex >= 0 && activeStepIndex < totalMissionSteps) {
          MovementStep currentLeg = missionPipeline[activeStepIndex];

          // ── ARRIVAL TEST ────────────────────────────────────────
          bool arrived = false;
          if (relativeStopArmed) {
            // RELATIVE: stop once the front wall has closed in by gridsToCross*300 mm
            // from the reading we latched at the start of this leg.
            if (s_front > 0 && s_front <= frontStopTargetMm) {
              if (++frontStopHits >= FRONT_STOP_DEBOUNCE) arrived = true;
            } else {
              frontStopHits = 0;
            }
          } else {
            // FALLBACK: no trustworthy wall at leg start -> original absolute rule.
            uint16_t stopThresholdMm   = (currentLeg.stopLimitGrids * 300) + 150;
            unsigned long blindSpotDur = currentLeg.gridsToCross * 1500;
            bool isBlindSpotActive     = (now - stepStartTime < blindSpotDur);
            if (!isBlindSpotActive && s_front > 0 && s_front <= stopThresholdMm) arrived = true;
          }

          if (arrived) {
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

        if (currentPhase != TURNING) {
            if (activeStepIndex == 0) {
                currentRobotDirection = initialSpawnDirection;
            }

            Direction nextDir = nextLeg.orientation;
            float relativeAngleDelta = 0.0f;

            if ((currentRobotDirection + 1) % 4 == nextDir) {
                relativeAngleDelta = 90.0f;  
            } 
            else if ((currentRobotDirection + 2) % 4 == nextDir) {
                relativeAngleDelta = 180.0f; 
            } 
            else if ((currentRobotDirection + 3) % 4 == nextDir) {
                relativeAngleDelta = -90.0f; 
            }

            turnTargetYaw = wrap360(yaw + relativeAngleDelta);

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

        int8_t baseTurningSpeed = (int8_t)constrain(absErr * 0.5f + 10, 10, 10);
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
          
          currentRobotDirection = missionPipeline[activeStepIndex].orientation;

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
