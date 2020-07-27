#include "serial.h"

handler ser;

void setup() {
    Serial.begin(115200);
    pinMode(A3, OUTPUT);
    digitalWrite(A3, LOW);
    pinMode(A2, INPUT);

    ser = handler();
}

void loop() {
    ser.handle();
}
