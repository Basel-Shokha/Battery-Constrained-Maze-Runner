// ============================================================
//  TAB 2: ESP32.ino — Master Core Parser & Execution Loop
// ============================================================
#include "PROTOCOL.h"
#include <Adafruit_NeoPixel.h>

#define TOTAL_NUM_PIXELS 16

HardwareSerial M5Serial(2);

uint16_t distLeft = 0, distFront = 0, distRight = 0;
unsigned long lastSendTime = 0;

unsigned long lastOrangeUpdateTime = 0;
unsigned long notifyResendTime      = 0;
bool isAwaitingChargeDoneAck        = false;

String inputBuffer = "";

extern void initLED();
extern void clearRing();
extern void setRingColor(uint8_t r, uint8_t g, uint8_t b);
extern void setSinglePixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b);

extern void initMP3();
extern void setVolume(uint8_t volume);
extern void playTrack(uint8_t directory, uint8_t trackNumber);
extern bool resetMP3();

extern void initSensors();
extern void readSensors();

// Linked directly to your untouched function in RECHARGING_LOGIC.ino
extern void runRechargingMode();

void parsePacket(String inLine);
void updateOrangeBreathing(unsigned long now);

void setup() {
    Serial.begin(115200);
    initLED(); 
    initMP3();
    initSensors(); 

    setVolume(30);
    setRingColor(200, 60, 0);
    lastOrangeUpdateTime = millis();
    M5Serial.begin(115200, SERIAL_8N1, 16, 17);
}

void loop() {
    unsigned long now = millis();

    // ── RULE 2: COMPLETION HANDSHAKE RESEND TRACKER ──
    if (isAwaitingChargeDoneAck) {
        if (now - notifyResendTime >= 100) { 
            notifyResendTime = now;
            M5Serial.println("NOTIFY:CHARGE_DONE");
        }
    } 
    else {
        updateOrangeBreathing(now);
    }

    readSensors();
    if (now - lastSendTime >= 60) {
        lastSendTime = now;
        M5Serial.printf("DIST:%d,%d,%d\n", distLeft, distFront, distRight);
    }

    while (M5Serial.available() > 0) {
        char c = M5Serial.read();
        if (c == '\n') {
            inputBuffer.trim();
            
            // Check for completion confirmation from M5
            if (inputBuffer == "ACK:CHARGE_DONE") {
                isAwaitingChargeDoneAck = false;
                lastOrangeUpdateTime = millis(); 
            } else if (inputBuffer.length() > 0) {
                parsePacket(inputBuffer);
            }
            inputBuffer = "";
        } else if (c != '\r') {
            inputBuffer += c;
        }
    }
}

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

void parsePacket(String inLine) {
    if (!inLine.startsWith("PACKET:")) return;
    
    int cmdId = 0, p1 = 0, p2 = 0;
    if (sscanf(inLine.c_str(), "PACKET:%d,%d,%d", &cmdId, &p1, &p2) != 3) return;
    
    if (cmdId == CMD_PLAY_AUDIO) {
        setVolume(30);
        delay(10);
        int trackNumber = p1 - 100;
        if (trackNumber >= 1 && trackNumber <= 5) {
            playTrack(1, trackNumber);
        }
    }
    else if (cmdId == CMD_STOP_AUDIO) {
        resetMP3();
    }
    // ── RULE 2: RECHARGING INTERCEPT PROMPTS ACK:3 BEFORE TRIGGERING YOUR FUNCTION ──
    else if (cmdId == CMD_START_CHARGE) {
        M5Serial.println("ACK:3"); 
        
        // Triggers your untouched function directly
        runRechargingMode();
        
        // Arm completion handshake once your function returns
        isAwaitingChargeDoneAck = true;
        notifyResendTime = 0;
    }
    // ── RULE 3: RENDER BATTERY ON LEDS FROM PACKET FEED ──
    else if (cmdId == CMD_BATTERY_UPDATE) {
        int greenLeds = (p1 * TOTAL_NUM_PIXELS) / p2;
        greenLeds = constrain(greenLeds, 0, TOTAL_NUM_PIXELS);
        clearRing();
        for (int i = 0; i < greenLeds; i++) {
            setSinglePixel(i, 0, 150, 0);
        }
    }
}

