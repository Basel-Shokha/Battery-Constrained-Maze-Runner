// ============================================================
//  RECHARGING_LOGIC.ino — Callable Recharging Mode Function
//  Call runRechargingMode() anywhere and it blocks for exactly
//  8.5 seconds while playing the charge animation + audio.
// ============================================================

// Functions from LED.ino
extern void clearRing();
extern void setRingColor(uint8_t r, uint8_t g, uint8_t b);
extern void setSinglePixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b);

// Functions from MP3.ino
extern void setVolume(uint8_t volume);
extern void playTrack(uint8_t directory, uint8_t trackNumber);

// ── Sequential blue sweep (one full pass around the ring) ──
void blueSweep() {
    clearRing();
    for (int i = 0; i < 16; i++) {
        setSinglePixel(i, 0, 0, 255);
        delay(100);
    }
}

// ── Full 8.5 second recharging cycle ────────────────────────
// Stage 1: 2s solid blue
// Stage 2: sweep #1 (~1.5s)
// Stage 3: sweep #2 (~1.5s)
// Audio plays in parallel at the start (fire-and-forget over UART)
void runRechargingMode() {
    Serial.println("[RECHARGE] Starting 8.5s cycle...");

    // Kick off audio - non-blocking, module plays independently
    setVolume(30);
    
    playTrack(1, 2);  // Charging tune track

    // Stage 1: solid blue for 2 seconds
    setRingColor(0, 0, 255);
    delay(2000);

    // Stage 2: first sweep
    blueSweep();

    // Stage 3: second sweep
    blueSweep();

    // Cleanup
    clearRing();
    Serial.println("[RECHARGE] Cycle complete.");
}

