#include "serial.h"
#include "led.h"

int main(){
    init();

    Serial.begin(115200);   //serial initialization
    
    pinMode(A3, OUTPUT);    
    digitalWrite(A3, LOW);
    pinMode(A2, INPUT);
    
    handler ser;    //initialize deserializer

    led_init();     //initialize led
    TCA0_init();    //initialize timer interrupt

    sei();          //enable global interrupts

    while(true){
        ser.handle();   //handle incoming data
    }
    return 0;
}
