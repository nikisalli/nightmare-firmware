#include <Arduino.h>

const uint8_t FT4232_RESET_PIN = 12;

void setup(){
    pinMode(FT4232_RESET_PIN, OUTPUT);      //bootstrap usb chip to make sure it is recognized by the kernel
    digitalWrite(FT4232_RESET_PIN, LOW);
    delay(5000);
    digitalWrite(FT4232_RESET_PIN, HIGH);
}

void loop(){

}