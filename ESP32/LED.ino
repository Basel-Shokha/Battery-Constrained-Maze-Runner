#include <Adafruit_NeoPixel.h>

// PIN 13 is D13 on our ESP32 development board
#define LED_PIN       13 
#define NUM_PIXELS    16

// Standard setup for WS2812B GRB pixel rings running at 800 KHz
Adafruit_NeoPixel ring(NUM_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

/**
 * Initializes the hardware pin and enforces a safe default brightness limit.
 * ESP32 internal regulators can get unstable if 16 pixels draw full current at once.
 */
void initLED() {
    ring.begin(); 
    
    // Kept at 50/255. It's plenty bright and stops the ESP32 from dropping voltage.
    ring.setBrightness(50); 
    
    clearRing();
}

/**
 * Turns off all pixels on the ring immediately.
 */
void clearRing() {
    ring.clear();
    ring.show();
}

/**
 * Sets every pixel on the ring to a single solid RGB color.
 * Useful for solid status flags (e.g., solid green for safe, solid red for error).
 */
void setRingColor(uint8_t r, uint8_t g, uint8_t b) {
    for(int i = 0; i < NUM_PIXELS; i++) {
        // .Color() formats the individual bytes into a single 32-bit package the library expects
        ring.setPixelColor(i, ring.Color(r, g, b));
    }
    // Crucial: The hardware won't change physically until you push the buffer using .show()
    ring.show(); 
}

/**
 * Lights up a single specific pixel without wiping out what the rest of the ring is doing.
 * index range: 0 to 15
 */
void setSinglePixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
    if (index >= NUM_PIXELS) return; // Quick boundary check so we don't write out of memory
    
    ring.setPixelColor(index, ring.Color(r, g, b));
    ring.show();
}

/**
 * Runs a loading animation around the circle.
 * Ideal for indicating when the rover is processing or rotating into position.
 */
void runLoadingAnimation(uint8_t r, uint8_t g, uint8_t b, uint16_t delayMs) {
    for(int i = 0; i < NUM_PIXELS; i++) {
        ring.clear();
        ring.setPixelColor(i, ring.Color(r, g, b));
        ring.show();
        delay(delayMs);
    }
    ring.clear();
    ring.show();
}
