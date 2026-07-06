// ============================================================
//  ESP32.ino — CLEAN REWRITE: Master Core Parser & Execution
// ============================================================
#include "PROTOCOL.h"
#include <Adafruit_NeoPixel.h>

#define TOTAL_NUM_PIXELS 16

HardwareSerial M5Serial(2);

// Global shared telemetry variables
uint16_t distLeft = 0, distFront = 0, distRight = 0;
unsigned long lastSendTime = 0;

// ── ORANGE BREATHING STATE ──────────────────────────────────
unsigned long lastOrangeUpdateTime = 0;
String inputBuffer = "";

// ── EXTERN SIGNATURES FROM OTHER TABS ───────────────────────
// From LED.ino
extern void initLED();
extern void clearRing();
extern void setRingColor(uint8_t r, uint8_t g, uint8_t b);
extern void setSinglePixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b);

// From MP3.ino
extern void initMP3();
extern void setVolume(uint8_t volume);
extern void playTrack(uint8_t directory, uint8_t trackNumber);
extern bool resetMP3();

// From SENSORS.ino
extern void initSensors();
extern void readSensors();

// From RECHARGING_LOGIC.ino
extern void runRechargingMode();

// Forward declarations for local functions
void parsePacket(String inLine);
void updateOrangeBreathing(unsigned long now);

void setup() {
    Serial.begin(115200);
    
    Serial.println("\n[ESP32] Initializing Master Core Architecture...");
    
    // Initialize external hardware tabs
    initLED(); 
    initMP3();     // Has its own delay(1200) internally for SD indexing - unavoidable
    initSensors(); 

    setVolume(30);
    
    // Light up orange IMMEDIATELY on boot, no extra delay
    setRingColor(200, 60, 0);
    lastOrangeUpdateTime = millis();
    
    M5Serial.begin(115200, SERIAL_8N1, 16, 17);
    Serial.println("[ESP32] Core operational. Ready for commands from M5.");
}

void loop() {
    unsigned long now = millis();

    // Run idle orange breathing loop when not charging
    updateOrangeBreathing(now);

    // ── Read ToF Laser Distance Sensors ─────────────────────────
    readSensors();
    if (now - lastSendTime >= 60) {
        lastSendTime = now;
        M5Serial.printf("DIST:%d,%d,%d\n", distLeft, distFront, distRight);
    }

    // ── Parse incoming command packets from M5 ───────────────────
    while (M5Serial.available() > 0) {
        char c = M5Serial.read();
        if (c == '\n') {
            if (inputBuffer.length() > 0) {
                parsePacket(inputBuffer);
            }
            inputBuffer = ""; 
        } else if (c != '\r') {
            inputBuffer += c; 
        }
    }
}

// ============================================================
//  ORANGE BREATHING: Smooth sinusoidal fade (5 second cycle)
// ============================================================
void updateOrangeBreathing(unsigned long now) {
    if (now - lastOrangeUpdateTime >= 20) {
        lastOrangeUpdateTime = now;
        
        float angle = (float)(now % 5000) * (2.0f * M_PI / 5000.0f);
        float factor = (sinf(angle) + 1.0f) / 2.0f;
        
        uint8_t redValue   = 50 + (uint8_t)(factor * 150.0f);
        uint8_t greenValue = 20 + (uint8_t)(factor * 60.0f);
        
        setRingColor(redValue, greenValue, 0);
    }
}

// ============================================================
//  COMMAND PARSER: Handle incoming PACKET strings from M5
// ============================================================
void parsePacket(String inLine) {
    inLine.trim();
    
    if (!inLine.startsWith("PACKET:")) {
        return;
    }
    
    int cmdId = 0, p1 = 0, p2 = 0;
    if (sscanf(inLine.c_str(), "PACKET:%d,%d,%d", &cmdId, &p1, &p2) != 3) {
        Serial.println("[PARSE] Invalid packet format");
        return;
    }
    
    Serial.printf("[PARSE] Command ID: %d, P1: %d, P2: %d\n", cmdId, p1, p2);
    
    // ── PLAY AUDIO ──────────────────────────────────────────────
    if (cmdId == CMD_PLAY_AUDIO) {
        setVolume(30);
        delay(10);
        int trackNumber = p1 - 100;  // 101 -> Track 1, 102 -> Track 2, etc.
        if (trackNumber >= 1 && trackNumber <= 5) {
            playTrack(1, trackNumber);
        }
    }
    
    // ── STOP AUDIO ──────────────────────────────────────────────
    else if (cmdId == CMD_STOP_AUDIO) {
        resetMP3();
    }
    
    // ── START CHARGING (Triggered by M5 Button A) ────────────────
    else if (cmdId == CMD_START_CHARGE) {
        Serial.println("[CHARGE] M5 Button A pressed!");
        
        // Runs the full animation + audio cycle from RECHARGING_LOGIC.ino
        // (audio is fired internally inside runRechargingMode - no need to call it here)
        runRechargingMode();
        
        // Reset breathing timer so idle animation resumes smoothly
        lastOrangeUpdateTime = millis();
    }
    
    // ── BATTERY UPDATE (Show percentage as green bar) ───────────
    else if (cmdId == CMD_BATTERY_UPDATE) {
        int greenLeds = (p1 * TOTAL_NUM_PIXELS) / p2;
        if (greenLeds > TOTAL_NUM_PIXELS) greenLeds = TOTAL_NUM_PIXELS;
        if (greenLeds < 0)                 greenLeds = 0;
        
        clearRing();
        for (int i = 0; i < TOTAL_NUM_PIXELS; i++) {
            if (i < greenLeds) {
                setSinglePixel(i, 0, 150, 0);
            } else {
                setSinglePixel(i, 0, 0, 0);
            }
        }
    }
}
