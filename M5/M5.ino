// ============================================================
//  TAB 1: M5.ino — Core State Orchestration Engine
// ============================================================
#include <M5StickCPlus.h>
#include <Wire.h>
#include <math.h>
#include <WiFi.h> 

// FIXED: Added the missing hardware serial instance declaration for the linker to reference
HardwareSerial ESP32Serial(2);

// ── INTEGRAL COMMAND CONSTANTS FOR PERIPHERAL ESP32 ────────
const int CMD_PLAY_AUDIO     = 1;
const int CMD_STOP_AUDIO     = 2;
const int CMD_START_CHARGE   = 3;
const int CMD_BATTERY_UPDATE = 4;

// ── INTEGRAL AUDIO TRACK CODES MAPPED TO EXACT TIME DURATIONS ──
const int AUDIO_MISSION_START = 101; // Duration: 5.0 Seconds (5000 ms)
const int AUDIO_GOING_STATION = 102; // Duration: 1.5 Seconds (1500 ms)
const int AUDIO_CHARGING_TUNE = 103; // Duration: 8.5 Seconds (8500 ms)
const int AUDIO_ROUTE_ERROR   = 104; // Duration: 4.5 Seconds (4500 ms)
const int AUDIO_VICTORY       = 105; // Duration: 3.5 Seconds (3500 ms)

// ── SHARED LAYOUT STRUCTURE PROTOCOLS ───────────────────────
enum Direction : uint8_t { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };
struct MovementStep {
    Direction orientation;
    uint8_t gridsToCross;
    uint8_t expectedStartGrids;
    uint8_t stopLimitGrids;
    uint8_t action; // 0 = Normal Move, 1 = Stop & Charge Station
};

// ── GLOBAL REGISTERS ────────────────────────────────────────
Direction initialSpawnDirection = EAST;
MovementStep missionPipeline[50];
uint8_t totalMissionSteps       = 0;
int8_t activeStepIndex          = -1;
int state                       = 0; // 0 = IDLE, 1 = RUNNING

enum DrivePhase { WALKING_FWD, TURNING, VERIFYING_TURN };
DrivePhase currentPhase = WALKING_FWD;

// Telemetry Registers
volatile uint16_t s_left = 0, s_front = 0, s_right = 0;
unsigned long lastDrawTime   = 0;
unsigned long lastSampleTime = 0;
unsigned long settleTimer    = 0;
int16_t baselineSpeed        = 45; 

// Anti-Stuck Trackers
unsigned long lastStuckCheckTime = 0;
float lastStuckCheckYaw          = 0.0f;
int16_t antiStuckSpeedBoost      = 0;

// ── EXTERN SIGNATURES (Linked from Gyroscope.ino) ───────────
extern float yaw, targetHeading, turnTargetYaw, vLeft, vRight;
extern bool isAligning;
extern void calibrateGyro();
extern void updateYaw();
extern void walkForward();
extern void resetSpeedHistory();
extern void processParallelAlignment(unsigned long now, unsigned long &lastSampleTime);
extern float angleDiff(float target, float current);
extern float wrap360(float a);

// ── EXTERN DRIVERS (Linked from tab 2 and tab 3 modules) ─────
extern void connectWiFi();
extern void handlePCNetworking(unsigned long now);
extern void initPeripheralUART();
extern void receivePeripheralTelemetry();
extern void sendPeripheralCmd(int commandId, int param1, int param2);

// ── MOTOR CONTROL OVER I2C (RoverC Pro at 0x38) ─────────────
void sendI2C(uint8_t reg, int8_t speed) {
  Wire.beginTransmission(0x38); 
  Wire.write(reg); 
  Wire.write(speed);
  Wire.endTransmission();
}
void setMotors(int8_t fl, int8_t fr, int8_t rl, int8_t rr) {
  sendI2C(0x00, fl); sendI2C(0x01, fr); sendI2C(0x02, rl); sendI2C(0x03, rr);
}
void stopMotors() { setMotors(0, 0, 0, 0); }

const char* getDirectionName(Direction dir) {
  if (dir == NORTH) return "NORTH";
  if (dir == EAST) return "EAST";
  if (dir == SOUTH) return "SOUTH"; if (dir == WEST) return "WEST";
  return "UNKNOWN";
}

void setup() {
  M5.begin(); 
  Serial.begin(115200); 
  Wire.begin(0, 26); 
  
  initPeripheralUART(); 
  M5.IMU.Init(); 
  M5.IMU.SetGyroFsr(MPU6886::GFS_250DPS);
  M5.Lcd.setRotation(3); 
  M5.Lcd.fillScreen(BLACK); 
  M5.Lcd.setTextSize(2);
  
  connectWiFi(); 
  
  M5.Lcd.fillScreen(BLACK); 
  M5.Lcd.setTextColor(YELLOW);
  M5.Lcd.println("Calibrating Gyro...");
  calibrateGyro();
  M5.Lcd.fillScreen(BLACK);
}

void loop() {
  updateYaw(); 
  unsigned long now = millis();

  receivePeripheralTelemetry();
  handlePCNetworking(now);

  if (state == 1) {
    switch (currentPhase) {
      
      case WALKING_FWD: {
        processParallelAlignment(now, lastSampleTime);
        if (activeStepIndex >= 0 && activeStepIndex < totalMissionSteps) {
          MovementStep currentLeg = missionPipeline[activeStepIndex];
          uint16_t stopThresholdMm = (currentLeg.stopLimitGrids * 300) + 150;

          if (s_front > 0 && s_front <= stopThresholdMm) {
            
            // ⚡ HANDLE INTEGRAL CHARGE STATION WITH 8.5s AUDIO & BLINK SYNC
            if (currentLeg.action == 1) {
              stopMotors();

              // Command ESP32 to trigger track 103 and enter dark blue blinking loop
              sendPeripheralCmd(CMD_PLAY_AUDIO, AUDIO_CHARGING_TUNE, 0);
              sendPeripheralCmd(CMD_START_CHARGE, 1, 0); 
              
              // Freeze master loop execution during the 8.5-second runtime duration
              delay(8500); 
              
              sendPeripheralCmd(CMD_STOP_AUDIO, 0, 0);

              // Update battery to full fraction capacity (8/8) so ESP32 renders all 16 green leds
              sendPeripheralCmd(CMD_BATTERY_UPDATE, 8, 8); 
              delay(1000);

              sendPeripheralCmd(CMD_PLAY_AUDIO, AUDIO_GOING_STATION, 0); // Brief milestone chirp
              delay(1500);
            }

            activeStepIndex++;
            resetSpeedHistory();
            if (activeStepIndex >= totalMissionSteps) { 
              state = 0;
              stopMotors(); 
              sendPeripheralCmd(CMD_PLAY_AUDIO, AUDIO_VICTORY, 0); // 3.5s victory audio
              delay(3500);
              Serial.println("STATUS:MISSION_COMPLETE");
            } else {
              MovementStep nextLeg = missionPipeline[activeStepIndex];
              if      (nextLeg.orientation == NORTH) turnTargetYaw = 0.0f;
              else if (nextLeg.orientation == EAST)  turnTargetYaw = 90.0f;
              else if (nextLeg.orientation == SOUTH) turnTargetYaw = 180.0f;
              else if (nextLeg.orientation == WEST)  turnTargetYaw = 270.0f;
              
              lastStuckCheckTime = now; 
              lastStuckCheckYaw = yaw; 
              antiStuckSpeedBoost = 0;
              currentPhase = TURNING;
            }
          } else {
            walkForward();
          }
        }
        break;
      }

      case TURNING: {
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
          if (deltaYaw < 10.0f) {
            antiStuckSpeedBoost += 5;
            if (antiStuckSpeedBoost > 40) antiStuckSpeedBoost = 40; 
          } else {
            antiStuckSpeedBoost = 0;
          }
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
          resetSpeedHistory(); lastSampleTime = millis();
          currentPhase = WALKING_FWD; 
        } else {
          lastStuckCheckTime = now;
          lastStuckCheckYaw = yaw; currentPhase = TURNING;
        }
        break;
      }
    }
  } else { 
    stopMotors();
  }

  // Screen Telemetry Update Line
  if (now - lastDrawTime > 100) {
    lastDrawTime = now;
    M5.Lcd.setCursor(0, 0);
    if (state == 0) {
      M5.Lcd.setTextColor(CYAN, BLACK);   M5.Lcd.println("--- WIFI ACTIVE ---");
      M5.Lcd.setTextColor(WHITE, BLACK);
      M5.Lcd.printf("IP: %s\n", WiFi.localIP().toString().c_str());
      M5.Lcd.printf("Steps Loaded: %d\n\n", totalMissionSteps);
      M5.Lcd.setTextColor(YELLOW, BLACK); M5.Lcd.println("Awaiting Dispatch...");
    } else {
      M5.Lcd.setTextColor(GREEN, BLACK);  M5.Lcd.printf("RUNNING LEG: %d/%d\n", activeStepIndex + 1, totalMissionSteps);
      M5.Lcd.setTextColor(WHITE, BLACK);
      if (activeStepIndex >= 0 && activeStepIndex < totalMissionSteps) {
        MovementStep activeLeg = missionPipeline[activeStepIndex];
        M5.Lcd.printf("DIR: %s       \n", getDirectionName(activeLeg.orientation));
        M5.Lcd.printf("Stop Target: %d   \n\n", activeLeg.stopLimitGrids);
      }
    }
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.printf("L:%04d F:%04d R:%04d\n", s_left, s_front, s_right);
  }
  delay(2);
}

