#include <HardwareSerial.h>

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200, SERIAL_8N1, 18, 17);
}

void loop() {
    if (Serial.available()) {
        Serial1.write(Serial.read());
    }
    if (Serial1.available()) {
        Serial.write(Serial1.read());
    }
}