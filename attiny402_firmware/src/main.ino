#include "serial.h"

void setup() {
    Serial.begin(115200);
    pinMode(A3, OUTPUT);
    digitalWrite(A3, LOW);
    pinMode(A2, INPUT);
}

void loop() {
    handler ser;

    while(1){
        ser.handle();
    }
}