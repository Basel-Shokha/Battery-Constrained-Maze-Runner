
#include <HardwareSerial.h>

HardwareSerial MP3(1);

static const byte START_BYTE       = 0x7E;
static const byte END_BYTE         = 0xEF;
static const byte CMD_SET_VOLUME   = 0x31;
static const byte CMD_PLAY_FILE    = 0x42;

static const uint8_t RESET_CMD[]     = { 0x7E, 0x03, 0x35, 0x05, 0xEF };
static const uint8_t SELECT_SD_CMD[] = { 0x7E, 0x03, 0x35, 0x01, 0xEF };

void selectSDCard() {
    for (int i = 0; i < 5; i++) {
        MP3.write(SELECT_SD_CMD[i]);
    }
    delay(20);
}

void initMP3() {
    
    MP3.begin(9600, SERIAL_8N1, 27, 14);
    delay(100);
    
    resetMP3();
    selectSDCard();
    delay(1200); 
}

bool resetMP3() {
    MP3.flush();
    for (int i = 0; i < 5; i++) {
        MP3.write(RESET_CMD[i]);
    }
    delay(50);
    return MP3.available() > 0;
}

void setVolume(uint8_t volume) {
    if (volume > 30) volume = 30; 
    
    MP3.write(START_BYTE);
    MP3.write((byte)0x03);
    MP3.write(CMD_SET_VOLUME);
    MP3.write(volume);
    MP3.write(END_BYTE);
    delay(20);
}

void playTrack(uint8_t directory, uint8_t trackNumber) {
    MP3.write(START_BYTE);
    MP3.write((byte)0x04);
    MP3.write(CMD_PLAY_FILE);
    MP3.write(directory);
    MP3.write(trackNumber);
    MP3.write(END_BYTE);
    delay(20);
}

