#include "servos.h"
#include "definitions.h"
#include "utils.h"

TaskHandle_t _servo_writer_task;
servo servos[26];

bool servo::write_protection = false;

void setup(){
	servo::init(SERVO_PIN_TX_ENB, SERVO_PIN_RX_ENB);

	int x = 0;
	for(auto& servo : servos){
		servo.id = ++x;
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
		for(auto& servo : servos){
			if (servo.active && !servo::write_protection){
				servo.move();
			}
		}
	}
}