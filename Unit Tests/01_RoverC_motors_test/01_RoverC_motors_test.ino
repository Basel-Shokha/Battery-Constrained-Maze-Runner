/*
 * ============================================================================
 *  UNIT TEST 1 - RoverC base (4x mecanum motors) + M5StickC Plus
 * ============================================================================
 *  Goal of this unit test:
 *    Verify that the M5StickC Plus can talk to the RoverC base over I2C and
 *    drive all four mecanum wheels: forward / backward / strafe / rotate.
 *
 *  Hardware:
 *    - M5StickC Plus inserted on top of the RoverC base
 *    - RoverC main controller is on I2C address 0x38
 *    - The base I2C is wired to the HAT pins: SDA = GPIO0, SCL = GPIO26
 *      (roverc.begin() configures these pins for you, do NOT call Wire.begin)
 *
 *  Library (install via Arduino IDE -> Library Manager):
 *    - "M5StickCPlus"  by M5Stack
 *    - "M5-RoverC"     by M5Stack   (provides class M5_RoverC, setSpeed, setServoAngle)
 *
 *  How to run the test:
 *    1. Put the rover on a stand so the wheels spin freely (or on open floor).
 *    2. Upload, then press the big front button (Button A) to start one cycle.
 *    3. Watch the screen labels and confirm each motion happens.
 *
 *  setSpeed(x, y, z):
 *    x = forward/back   (+ forward, - back)
 *    y = strafe         (+ right,   - left)   <-- mecanum sideways motion
 *    z = rotation       (+ CW,      - CCW)
 *    Each value is -100..100.
 * ============================================================================
 */

#include <M5StickCPlus.h>
#include "M5_RoverC.h"

M5_RoverC roverc;

const int8_t SPEED = 50;   // 0..100 ; keep moderate for a bench test

void banner(const char *line1, const char *line2) {
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(GREEN, BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(6, 18);
  M5.Lcd.print(line1);
  M5.Lcd.setCursor(6, 48);
  M5.Lcd.print(line2);
}

void step(const char *name, int8_t x, int8_t y, int8_t z, uint16_t ms) {
  banner("RoverC", name);
  Serial.printf("Drive %-10s  x=%d y=%d z=%d\n", name, x, y, z);
  roverc.setSpeed(x, y, z);
  delay(ms);
  roverc.setSpeed(0, 0, 0);   // stop between moves
  delay(700);
}

void setup() {
  M5.begin();
  Serial.begin(115200);
  M5.Lcd.setRotation(1);
  roverc.begin();             // I2C @ 0x38 on GPIO0/GPIO26
  roverc.setSpeed(0, 0, 0);   // make sure we start stopped
  banner("RoverC test", "Press BtnA");
  Serial.println("RoverC unit test ready. Press Button A to run a cycle.");
}

void loop() {
  M5.update();
  if (M5.BtnA.wasPressed()) {
    step("FORWARD",  SPEED,  0,     0,     1500);
    step("BACKWARD", -SPEED, 0,     0,     1500);
    step("STRAFE R", 0,      SPEED, 0,     1500);
    step("STRAFE L", 0,     -SPEED, 0,     1500);
    step("ROTATE CW",0,      0,     SPEED, 1500);
    step("ROTATE CCW",0,     0,    -SPEED, 1500);
    banner("RoverC", "DONE - BtnA");
    Serial.println("Cycle complete.\n");

    /* RoverC-PRO only (has servos for the gripper) - uncomment if you have PRO:
       roverc.setServoAngle(0, 30);
       roverc.setServoAngle(1, 30);
       delay(800);
       roverc.setServoAngle(0, 90);
       roverc.setServoAngle(1, 90);
    */
  }
}
