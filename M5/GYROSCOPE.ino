#include <M5StickCPlus.h>
#include <Wire.h>
#include <math.h>

// Link directly to the main loop's external speed register variable
extern int16_t baselineSpeed; 

// ── GYRO & ORIENTATION GLOBALS ────────────────────────────
float gyroX, gyroY, gyroZ;
float gyroZoffset   = 0.0f;
float yaw           = 0.0f;
float targetHeading = 0.0f;
float turnTargetYaw = 0.0f;
unsigned long lastTime = 0;

// ── ROLLING AVERAGE FILTERS FOR WALL ALIGNMENT ────────────
float vRightHistory[10] = {0};
float vLeftHistory[10]  = {0};
int historyIdx          = 0;
float lastDistR         = 0.0f;
float lastDistL         = 0.0f;

// Exposing these so your main loop screen display can still read them
float vRight     = 0.0f;
float vLeft      = 0.0f; 
bool isAligning  = false;

// FIXED: Added 'volatile' to match the main M5.ino file strictly!
extern volatile uint16_t s_left, s_front, s_right;
extern void setMotors(int8_t fl, int8_t fr, int8_t rl, int8_t rr);

// ── MATHEMATICAL UTILITIES ────────────────────────────────
float wrap360(float a) {
    a = fmodf(a, 360.0f);
    if (a < 0.0f) a += 360.0f;
    return a;
}

float angleDiff(float target, float current) {
    float diff = target - current;
    if (diff >  180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;
    return diff;
}

// ── CORE GYRO DRIVER FUNCTIONS ────────────────────────────
void calibrateGyro() {
    stopMotors();
    float sum = 0.0f;
    for (int i = 0; i < 300; i++) {
        M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
        sum += gyroZ;
        delay(6);
    }
    gyroZoffset   = sum / 300.0f;
    yaw           = 0.0f; 
    targetHeading = 0.0f;
}

void updateYaw() {
    M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
    float gz = -(gyroZ - gyroZoffset);
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0f;
    lastTime = now;
    if (dt < 0.0f) dt = 0.0f;
    if (dt > 0.1f) dt = 0.1f;
    
    yaw += gz * dt;
    yaw = wrap360(yaw);
}

// ── KINEMATICS & CHASSIS CONTROL ──────────────────────────
void resetSpeedHistory() {
    for(int i = 0; i < 10; i++) {
        vRightHistory[i] = 0.0f;
        vLeftHistory[i]  = 0.0f;
    }
    historyIdx = 0;
    lastDistR  = s_right;
    lastDistL  = s_left;
    vRight     = 0.0f;
    vLeft      = 0.0f;
    isAligning = false;
}

void walkForward() {
    float error      = angleDiff(targetHeading, yaw);
    float correction = constrain(0.4f * error, -30.0f, 30.0f);
    
    // baselineSpeed is dynamically injected from the Xbox controller menu
    int8_t L = constrain((int)(baselineSpeed + correction), -127, 127);
    int8_t R = constrain((int)(baselineSpeed - correction), -127, 127);
    
    setMotors(L, R, L, R);
}

void processParallelAlignment(unsigned long now, unsigned long &lastSampleTime) {
    float dtSample = (now - lastSampleTime) / 1000.0f;
    if (dtSample < 0.05f) return; 
    
    float rawVRight = (lastDistR - s_right) / dtSample;
    float rawVLeft  = (lastDistL - s_left) / dtSample;
    
    vRightHistory[historyIdx] = rawVRight;
    vLeftHistory[historyIdx]  = rawVLeft;
    historyIdx = (historyIdx + 1) % 10;
    
    float sumR = 0.0f, sumL = 0.0f;
    for(int i = 0; i < 10; i++) {
        sumR += vRightHistory[i];
        sumL += vLeftHistory[i];
    }
    vRight = sumR / 10.0f; 
    vLeft  = sumL / 10.0f;
    lastDistR = s_right;
    lastDistL = s_left;
    lastSampleTime = now;
}

