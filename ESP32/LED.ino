
#include <Adafruit_NeoPixel.h>

#define LED_RING_PIN    13
#define TOTAL_NUM_PIXELS 16

Adafruit_NeoPixel ring(TOTAL_NUM_PIXELS, LED_RING_PIN, NEO_GRB + NEO_KHZ800);

void initLED() {
    ring.begin();
    ring.setBrightness(50); 
    clearRing();
}

void clearRing() {
    ring.clear();
    ring.show();
}

void setRingColor(uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < TOTAL_NUM_PIXELS; i++) {
        ring.setPixelColor(i, ring.Color(r, g, b));
    }
    ring.show();
}

void setSinglePixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
    if (index >= TOTAL_NUM_PIXELS) return;
    ring.setPixelColor(index, ring.Color(r, g, b));
    ring.show();
}


void applyArgentinaFlag() {
    ring.clear();
    for (int i = 0; i < TOTAL_NUM_PIXELS; i++) {
        if      (i <= 3)  ring.setPixelColor(i, ring.Color(0, 40, 120));   
        else if (i <= 6)  ring.setPixelColor(i, ring.Color(120, 120, 120));
        else if (i <= 8)  ring.setPixelColor(i, ring.Color(150, 110, 0));  
        else if (i <= 11) ring.setPixelColor(i, ring.Color(120, 120, 120));
        else              ring.setPixelColor(i, ring.Color(0, 40, 120));  
    }
    ring.show();
}

