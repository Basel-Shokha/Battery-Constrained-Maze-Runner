
extern void clearRing();
extern void setRingColor(uint8_t r, uint8_t g, uint8_t b);
extern void setSinglePixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b);


extern void setVolume(uint8_t volume);
extern void playTrack(uint8_t directory, uint8_t trackNumber);


void blueSweep() {
    clearRing();
    for (int i = 0; i < 16; i++) {
        setSinglePixel(i, 0, 0, 255);
        delay(100);
    }
}


void runRechargingMode() {
    Serial.println("[RECHARGE] Starting 8.5s cycle...");
   
    setVolume(30);
    
    playTrack(1, 2);
    

   
    setRingColor(0, 0, 255);
    delay(2000);

    
    blueSweep();

    
    blueSweep();

    
    delay(3300);

    
    clearRing();
    Serial.println("[RECHARGE] Cycle complete.");
}