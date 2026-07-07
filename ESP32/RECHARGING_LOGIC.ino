// ============================================================
//  RECHARGING_LOGIC.ino — Callable Recharging Mode Function
//  Call runRechargingMode() anywhere and it blocks for exactly
//  8.5 seconds while playing the charge animation + audio.
// ============================================================
#include <math.h>

// Functions from LED.ino
extern void clearRing();
extern void setRingColor(uint8_t r, uint8_t g, uint8_t b);
extern void setSinglePixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b);

// Functions from MP3.ino
extern void setVolume(uint8_t volume);
extern void playTrack(uint8_t directory, uint8_t trackNumber);

// ── Sequential fill, used for the charging animation ──
void fillChargePixels(uint8_t filledCount, uint8_t pulse) {
    clearRing();
    for (int i = 0; i < 16; i++) {
        if (i < filledCount) {
            setSinglePixel(i, 0, 120 + pulse, 20);
        } else {
            setSinglePixel(i, 0, 0, 35 + pulse);
        }
    }
}

void showFullCharge() {
    clearRing();
    for (int i = 0; i < 16; i++) {
        setSinglePixel(i, 0, 180, 0);
    }
}

// ── Full 8.5 second recharging cycle ────────────────────────
// Audio plays in parallel at the start (fire-and-forget over UART)
void runRechargingMode() {
    Serial.println("[RECHARGE] Starting 8.5s cycle...");

    playTrack(1, 2);

    const unsigned long chargeDurationMs = 8500;
    const unsigned long startMs = millis();
    while (millis() - startMs < chargeDurationMs) {
        unsigned long elapsed = millis() - startMs;
        uint8_t filled = constrain((int)((elapsed * 16UL) / chargeDurationMs) + 1, 1, 16);
        uint8_t pulse = (uint8_t)((sinf(elapsed * 0.008f) + 1.0f) * 35.0f);
        fillChargePixels(filled, pulse);
        delay(120);
    }

    showFullCharge();
    delay(700);
    Serial.println("[RECHARGE] Cycle complete.");
}
