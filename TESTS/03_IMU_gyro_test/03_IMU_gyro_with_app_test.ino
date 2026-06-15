#include <M5StickCPlus.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

// ── Wi-Fi Configuration ─────────────────────────────────────
const char* ssid     = "Rover_Control";
const char* password = "password123"; // Must be at least 8 characters
WebServer server(80);

// ── Motors ───────────────────────────────────────────────
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
void stopMotors() { setMotors(0, 0, 0, 0); }

// ── Gyro — UNTOUCHED STABLE LOGIC ────────────────────────
float gyroX, gyroY, gyroZ;
float gyroZoffset = 0.0f;
float yaw = 0.0f;
unsigned long lastTime = 0;

static float wrap360(float a) {
  a = fmodf(a, 360.0f);
  if (a < 0.0f) a += 360.0f;
  return a;
}
static float angleDiff(float target, float current) {
  float diff = target - current;
  if (diff >  180.0f) diff -= 360.0f;
  if (diff < -180.0f) diff += 360.0f;
  return diff;
}
void calibrateGyro() {
  float sum = 0.0f;
  for (int i = 0; i < 300; i++) {
    M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
    sum += gyroZ;
    delay(6);
  }
  gyroZoffset = sum / 300.0f;
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

// ── Config ────────────────────────────────────────────────
const float   Kp             = 0.8f;
const int8_t  BASE_SPEED     = 45;
const float   MAX_CORRECTION = 30.0f;
const int8_t  MAX_TURN_SPEED = 20;
const int8_t  MIN_TURN_SPEED = 18;
const float   TURN_TOLERANCE = 2.0f;
const int     TURN_BRAKE_MS  = 15;

// ── State ─────────────────────────────────────────────────
enum State { STOPPED, WALKING_FWD, WALKING_BWD, TURNING, VERIFYING_TURN, WALKING_RAW_FWD };
State state = STOPPED;
float targetHeading  = 0.0f;
float turnTargetYaw  = 0.0f;
unsigned long settleTimer  = 0;
unsigned long lastDrawTime = 0;

// ── Walk functions ────────────────────────────────────────
void walkForward() {
  float error      = angleDiff(targetHeading, yaw);
  float correction = constrain(Kp * error, -MAX_CORRECTION, MAX_CORRECTION);
  int8_t L = constrain((int)(BASE_SPEED + correction), -100, 100);
  int8_t R = constrain((int)(BASE_SPEED - correction), -100, 100);
  setMotors(L, R, L, R);
}

void walkBackward() {
  float error      = angleDiff(targetHeading, yaw);
  float correction = constrain(Kp * error, -MAX_CORRECTION, MAX_CORRECTION);
  int8_t L = constrain((int)(-BASE_SPEED + correction), -100, 100);
  int8_t R = constrain((int)(-BASE_SPEED - correction), -100, 100);
  setMotors(L, R, L, R);
}

// ── Web Dashboard Interface & Key Press Listener ─────────
void handleRoot() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<title>Mac Rover Controller</title>";
  html += "<style>";
  html += "body { font-family: -apple-system, sans-serif; text-align: center; background: #1e1e24; color: #f5f5f7; padding-top: 40px; }";
  html += "h1 { color: #ff9f0a; font-size: 28px; }";
  html += ".container { max-width: 500px; margin: 0 auto; background: #2c2c35; padding: 20px; border-radius: 12px; box-shadow: 0 4px 15px rgba(0,0,0,0.3); }";
  html += ".key-list { text-align: left; display: inline-block; font-size: 18px; line-height: 2.0; }";
  html += "kbd { background: #444; border-radius: 4px; padding: 2px 8px; border: 1px solid #666; font-weight: bold; color: #ff9f0a; }";
  html += "#status { margin-top: 20px; font-weight: bold; font-size: 20px; color: #30d158; }";
  html += "</style>";
  html += "<script>";
  
  // JavaScript Event Listener to track keys down on your Mac
  html += "document.addEventListener('keydown', function(event) {";
  html += "  let key = event.key.toLowerCase();";
  html += "  let validKeys = ['a', 'x', 'b', 'y', 'l', 'r', 'm'];";
  html += "  if (validKeys.includes(key)) {";
  html += "    document.getElementById('status').innerText = 'Sending command: ' + key.toUpperCase();";
  html += "    fetch('/action?cmd=' + key)";
  html += "      .then(response => response.text())";
  html += "      .then(data => { document.getElementById('status').innerText = 'Current Mode: ' + data; });";
  html += "  }";
  html += "});";
  
  html += "</script></head><body>";
  html += "<div class='container'>";
  html += "<h1>Rover Terminal Interface</h1>";
  html += "<p>Click anywhere on this page, then use your keyboard:</p>";
  html += "<div class='key-list'>";
  html += "  <kbd>A</kbd> &rarr; Walk Forward (Gyro-stabilized)<br>";
  html += "  <kbd>X</kbd> &rarr; Walk Backward (Gyro-stabilized)<br>";
  html += "  <kbd>M</kbd> &rarr; Raw Forward (No Gyro)<br>";
  html += "  <kbd>L</kbd> &rarr; Turn Left 90&deg;<br>";
  html += "  <kbd>R</kbd> &rarr; Turn Right 90&deg;<br>";
  html += "  <kbd>B</kbd> &rarr; Emergency STOP<br>";
  html += "  <kbd>Y</kbd> &rarr; Zero Yaw & Re-Calibrate Gyro";
  html += "</div>";
  html += "<div id='status'>Current Mode: STOPPED</div>";
  html += "</div></body></html>";
  
  server.send(200, "text/html", html);
}

void handleAction() {
  String cmd = server.arg("cmd");
  String responseState = "UNKNOWN";
  bool isTurning = (state == TURNING || state == VERIFYING_TURN);

  if (!isTurning) {
    if (cmd == "a") { targetHeading = yaw; state = WALKING_FWD; responseState = "FORWARD"; }
    if (cmd == "x") { targetHeading = yaw; state = WALKING_BWD; responseState = "BACKWARD"; }
    if (cmd == "m") { state = WALKING_RAW_FWD; responseState = "RAW FORWARD"; }
    if (cmd == "r") { turnTargetYaw = wrap360(yaw + 90.0f); state = TURNING; responseState = "TURNING RIGHT"; }
    if (cmd == "l") { turnTargetYaw = wrap360(yaw - 90.0f); state = TURNING; responseState = "TURNING LEFT"; }
    if (cmd == "y") {
      state = STOPPED;
      stopMotors();
      yaw = 0.0f;
      targetHeading = 0.0f;
      M5.Lcd.fillScreen(BLACK);
      M5.Lcd.setTextColor(YELLOW);
      M5.Lcd.setCursor(5, 40);
      M5.Lcd.println("Keep still...");
      calibrateGyro();
      M5.Lcd.fillScreen(BLACK);
      responseState = "RE-CALIBRATED & STOPPED";
    }
  } else {
    responseState = "BUSY TURNING";
  }

  if (cmd == "b") { state = STOPPED; stopMotors(); responseState = "STOPPED"; }

  server.send(200, "text/plain", responseState);
}

// ── Setup ─────────────────────────────────────────────────
void setup() {
  M5.begin();
  Wire.begin(0, 26);
  M5.IMU.Init();
  M5.IMU.SetGyroFsr(MPU6886::GFS_250DPS);

  M5.Lcd.setRotation(3);
  M5.Lcd.setTextSize(2);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(YELLOW);
  M5.Lcd.setCursor(5, 40);
  M5.Lcd.println("Keep still...");

  calibrateGyro();

  // Set up the local Access Point network
  WiFi.softAP(ssid, password);
  IPAddress myIP = WiFi.softAPIP();

  // Route requests to handler loops
  server.on("/", handleRoot);
  server.on("/action", handleAction);
  server.begin();

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setCursor(5, 5);
  M5.Lcd.printf("WiFi: %s\n", ssid);
  M5.Lcd.printf("IP:   %s\n", myIP.toString().c_str());
  M5.Lcd.println("-----------------");

  lastTime = micros();
}

// ── Loop ──────────────────────────────────────────────────
void loop() {
  server.handleClient(); // Process incoming server commands from your browser
  updateYaw();

  switch (state) {
    case STOPPED:      stopMotors();    break;
    case WALKING_FWD:  walkForward();   break;
    case WALKING_BWD:  walkBackward();  break;
    
    case WALKING_RAW_FWD: 
      setMotors(100, 100, 100, 100); 
      break;

    case TURNING: {
      float error  = angleDiff(turnTargetYaw, yaw);
      float absErr = fabsf(error);

      if (absErr <= TURN_TOLERANCE) {
        if (error > 0) setMotors(-25,  25, -25,  25);
        else           setMotors( 25, -25,  25, -25);
        delay(TURN_BRAKE_MS);
        stopMotors();
        settleTimer = millis();
        state = VERIFYING_TURN;
        break;
      }
      int8_t spd = (int8_t)constrain(absErr * 0.5f + MIN_TURN_SPEED, MIN_TURN_SPEED, MAX_TURN_SPEED);
      if (error > 0) setMotors( spd, -spd,  spd, -spd);
      else           setMotors(-spd,  spd, -spd,  spd);
      break;
    }

    case VERIFYING_TURN: {
      if (millis() - settleTimer < 100) break;
      float error = angleDiff(turnTargetYaw, yaw);
      if (fabsf(error) <= TURN_TOLERANCE) {
        targetHeading = turnTargetYaw;
        state = WALKING_FWD;
      } else {
        state = TURNING;
      }
      break;
    }
  }

  // ── Display every 100ms ───────────────────────────────────
  if (millis() - lastDrawTime > 100) {
    lastDrawTime = millis();
    M5.Lcd.setCursor(0, 50);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.printf("Yaw: %06.1f  \n", yaw);

    M5.Lcd.setTextColor(
      state == WALKING_FWD     ? GREEN   :
      state == WALKING_RAW_FWD ? MAGENTA : 
      state == WALKING_BWD     ? CYAN    :
      state == TURNING         ? YELLOW  :
      state == VERIFYING_TURN  ? BLUE    : RED, BLACK);
    
    M5.Lcd.printf("Mode: %s      \n",
      state == STOPPED         ? "STOPPED" :
      state == WALKING_FWD     ? "FORWARD" :
      state == WALKING_RAW_FWD ? "RAW FWD" :
      state == WALKING_BWD     ? "BACKWARD" :
      state == TURNING         ? "TURNING" : "VERIFYING");
  }

  delay(2);
}