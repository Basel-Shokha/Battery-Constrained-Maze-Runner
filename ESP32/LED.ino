#include <Adafruit_NeoPixel.h>

// Hardcoded for testing: Pin D13 controls a 16-pixel ring
#define TEST_LED_PIN 13
#define TEST_NUM_PIXELS 16

Adafruit_NeoPixel ring(TEST_NUM_PIXELS, TEST_LED_PIN, NEO_GRB + NEO_KHZ800);

void initLED() {
    ring.begin(); 
    ring.setBrightness(50); // Safe power limit for testing
    clearRing();
}

void clearRing() {
    ring.clear();
    ring.show();
}

void setRingColor(uint8_t r, uint8_t g, uint8_t b) {
    for(int i = 0; i < TEST_NUM_PIXELS; i++) {
        ring.setPixelColor(i, ring.Color(r, g, b));
    }
    ring.show(); 
}

void setSinglePixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
    if (index >= TEST_NUM_PIXELS) return; 
    ring.setPixelColor(index, ring.Color(r, g, b));
    ring.show();
}