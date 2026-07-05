#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>

// ── CONFIGURATION MODULE TOGGLES ──────────────────────────
#define DEACTIVATE_SENSORS   0   // SET TO 1 TO BYPASS MISSING HARDWARE DURING BENCH TESTING
#define USE_ARGENTINA_FLAG   1   // Set to 1 for Argentina, 0 for Brazil Layout
#define USE_BRAZIL_FLAG      0   // Set to 1 for Brazil, 0 for Argentina Layout

// ── PHYSICAL SENSOR XSHUT PINS ────────────────────────────
#define XSHUT_1 5   // Left Sensor Pin
#define XSHUT_2 23  // Front Sensor Pin
#define XSHUT_3 18  // Right Sensor Pin

// Hardware Serial 2 maps to internal pins RX=16, TX=17
HardwareSerial M5Serial(2);

VL53L1X sensorLeft, sensorFront, sensorRight;
uint16_t distLeft = 0, distFront = 0, distRight = 0;
unsigned long lastSendTime = 0;

// Extern signatures pulled natively from separate compilation units (LED.ino, MP3.ino)
extern void initLED(); extern void clearRing(); extern void setSinglePixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
extern void initMP3(); extern void setVolume(uint8_t volume); extern void playTrack(uint8_t directory, uint8_t trackNumber); extern bool resetMP3();

// ── PERMANENT HARDWARE VISUAL STATE INITIALIZER ───────────
void turnOnMediaEngineAllTheTime() {
    // Crank volume instantly and map flag layout EXACTLY ONCE here
    setVolume(30); delay(50); playTrack(1, 1); delay(50);
    clearRing();
    
    #if USE_ARGENTINA_FLAG
        for (int i = 0; i < 16; i++) {
            if (i <= 3)       setSinglePixel(i, 0, 30, 90);     // Pixels 0-3: Deep Royal Blue
            else if (i <= 6)  setSinglePixel(i, 90, 90, 90);    // Pixels 4-6: White
            else if (i <= 8)  setSinglePixel(i, 110, 80, 0);    // Pixels 7-8: Golden Sun
            else if (i <= 11) setSinglePixel(i, 90, 90, 90);    // Pixels 9-11: White
            else              setSinglePixel(i, 0, 30, 90);     // Pixels 12-15: Deep Royal Blue
        }
    #endif

    #if USE_BRAZIL_FLAG
        for (int i = 0; i < 16; i++) {
            if (i <= 3)       setSinglePixel(i, 0, 110, 0);     // Pixels 0-3: Green
            else if (i <= 6)  setSinglePixel(i, 110, 80, 0);    // Pixels 4-6: Yellow
            else if (i <= 8)  setSinglePixel(i, 0, 0, 110);     // Pixels 7-8: Blue
            else if (i <= 11) setSinglePixel(i, 110, 80, 0);    // Pixels 9-11: Yellow
            else              setSinglePixel(i, 0, 110, 0);     // Pixels 12-15: Green
        }
    #endif
}

void setup() {
    Serial.begin(115200);
    initLED(); initMP3();
    turnOnMediaEngineAllTheTime(); // Fire visuals/audio and latch state immediately

#if !DEACTIVATE_SENSORS
    Wire.begin(21, 22); Wire.setClock(400000); 
    pinMode(XSHUT_1, OUTPUT); pinMode(XSHUT_2, OUTPUT); pinMode(XSHUT_3, OUTPUT);
    digitalWrite(XSHUT_1, LOW); digitalWrite(XSHUT_2, LOW); digitalWrite(XSHUT_3, LOW); delay(10);
    digitalWrite(XSHUT_1, HIGH); delay(10); sensorLeft.init();  sensorLeft.setAddress(0x30);
    digitalWrite(XSHUT_2, HIGH); delay(10); sensorFront.init(); sensorFront.setAddress(0x31);
    digitalWrite(XSHUT_3, HIGH); delay(10); sensorRight.init(); sensorRight.setAddress(0x32);
    sensorLeft.startContinuous(50); sensorFront.startContinuous(50); sensorRight.startContinuous(50);
#endif

    M5Serial.begin(115200, SERIAL_8N1, 16, 17);
    Serial.println("[ESP32] Core initialized. Media locked ON. ToF operational.");
}

void loop() {
    unsigned long now = millis();

#if !DEACTIVATE_SENSORS
    // ── ASYNCHRONOUS LASER READINGS ──
    if (sensorLeft.dataReady())  distLeft  = sensorLeft.read(false);
    if (sensorFront.dataReady()) distFront = sensorFront.read(false);
    if (sensorRight.dataReady()) distRight = sensorRight.read(false);

    // ── STREAM SENSORS TO M5 OVER UART (Every 60ms structured ASCII) ──
    if (now - lastSendTime >= 60) {
        lastSendTime = now;
        M5Serial.printf("DIST:%d,%d,%d\n", distLeft, distFront, distRight);
    }
#endif

    // ── PROCESS INCOMING CONTROL INPUT FROM M5 stick UART pipeline ──────
    if (M5Serial.available() > 0) {
        String command = M5Serial.readStringUntil('\n'); command.trim();
        if (command.length() == 0) return;
        if (command == "AUDIO_STOP") { resetMP3(); } 
        else if (command == "AUDIO_PLAY") { setVolume(30); playTrack(1, 1); }
        else if (command.startsWith("VOL_")) { setVolume(command.substring(4).toInt()); }
    }
}