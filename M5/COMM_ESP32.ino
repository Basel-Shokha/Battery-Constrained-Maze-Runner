// ============================================================
//  TAB 3: COMM_ESP32.ino — Hardware UART Peripheral Interface
// ============================================================
#include <HardwareSerial.h>

extern HardwareSerial ESP32Serial;
extern volatile uint16_t s_left, s_front, s_right;

void initPeripheralUART() {
    ESP32Serial.begin(115200, SERIAL_8N1, 32, 33);
}

void receivePeripheralTelemetry() {
    while (ESP32Serial.available() > 0) {
        String inLine = ESP32Serial.readStringUntil('\n');
        inLine.trim();
        
        if (inLine.startsWith("DIST:")) {
            int parsedLeft, parsedFront, parsedRight;
            if (sscanf(inLine.c_str(), "DIST:%d,%d,%d", &parsedLeft, &parsedFront, &parsedRight) == 3) {
                s_left  = parsedLeft;
                s_front = parsedFront;
                s_right = parsedRight;
            }
        }
    }
}

// ── RAW HARDWARE INTEGER SERIAL PACKET WRITER ────────────────
void sendPeripheralCmd(int commandId, int param1, int param2) {
    ESP32Serial.printf("PACKET:%d,%d,%d\n", commandId, param1, param2);
}

