#include "servos.h"
#include "definitions.h"
#include "utils.h"

TaskHandle_t _servo_writer_task;
servo servos[26];

bool servo::write_protection = false;

void setup(){
	servo::init(SERVO_PIN_TX_ENB, SERVO_PIN_RX_ENB);
	for(int i=0; i<26; i++){
		servos[i].id = i+1;
	}
	xTaskCreatePinnedToCore(servo_writer_task, "servo writer task", 10000, NULL, 1, &_servo_writer_task, 1);
}

void loop(){
    while(1){
		delay(100);
	}
}

void servo_writer_task( void * parameter) {
	while(1) {
		for (int i = 0; i < 26; i++) {
			if (servos[i+1].active && !servo::write_protection){
				servos[i+1].move();
			}
		}
	}
}