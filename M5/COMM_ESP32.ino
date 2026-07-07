// ============================================================
//  TAB 3: COMM_ESP32.ino — Non-Blocking UART Handshake Receiver
// ============================================================
#include <HardwareSerial.h>

extern HardwareSerial ESP32Serial;
extern volatile uint16_t s_left, s_front, s_right;

extern bool chargeStartAckReceived;
extern bool esp32ChargeFinished;

String m5UartBuffer = "";
void initPeripheralUART() {
    ESP32Serial.begin(115200, SERIAL_8N1, 32, 33);
}

void receivePeripheralTelemetry() {
    // ── RULE 1 & 2: CHARACTER-STREAM TELEMETRY ACCUMULATION ──
    while (ESP32Serial.available() > 0) {
        char c = ESP32Serial.read();
        if (c == '\n') {
            m5UartBuffer.trim();
            // Unacknowledged continuous distance stream
            if (m5UartBuffer.startsWith("DIST:")) {
                int parsedLeft, parsedFront, parsedRight;
                if (sscanf(m5UartBuffer.c_str(), "DIST:%d,%d,%d", &parsedLeft, &parsedFront, &parsedRight) == 3) {
                    s_left  = parsedLeft;
                    s_front = parsedFront;
                    s_right = parsedRight;
                }
            }
            // Acknowledgment for Charging Initialization
            else if (m5UartBuffer == "ACK:3") {
                chargeStartAckReceived = true;
                Serial.println("[HANDSHAKE-UART] ESP32 confirmed charging cycle setup.");
            }
            // Acknowledgment loop intercept for Charging Completion
            else if (m5UartBuffer == "NOTIFY:CHARGE_DONE") {
                esp32ChargeFinished = true;
                // Instantly echo confirmation back so the co-processor releases its loop hold
                ESP32Serial.println("ACK:CHARGE_DONE");
            }
            
            m5UartBuffer = "";
        } 
        else if (c != '\r') {
            m5UartBuffer += c;
        }
    }
}

void sendPeripheralCmd(int commandId, int param1, int param2) {
    ESP32Serial.printf("PACKET:%d,%d,%d\n", commandId, param1, param2);
}