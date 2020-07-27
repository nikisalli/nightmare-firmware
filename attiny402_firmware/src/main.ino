#include "serial.h"

handler ser;

int main(){
    init();

    Serial.begin(115200);   //serial initialization

    pinMode(A3, OUTPUT);    
    digitalWrite(A3, LOW);
    pinMode(A2, INPUT);

    ser = handler();

    while(true){
        ser.handle();
    }
    return 0;
}
