#include "servos.h"
#include "definitions.h"
#include "utils.h"

TaskHandle_t _servo_writer_task;      //task handle for writer task
servo servos[26];                     //servo objects

bool servo::write_protection = false; //flag to stop task when reading servos

void setup(){
	/* initialize hardware */
	Serial.begin(460800);               //ROS serial port

	pinMode(PANDA_POWER_PIN,OUTPUT);    //lattepanda power on gpio
	pinMode(FAN1_PIN, OUTPUT);          //fans
  	pinMode(FAN2_PIN, OUTPUT);
  	pinMode(DEBUG_LED_PIN_1, OUTPUT);   //debug leds
  	pinMode(DEBUG_LED_PIN_2, OUTPUT);


  	digitalWrite(FAN1_PIN, HIGH);
  	digitalWrite(FAN2_PIN, LOW);

	servo::init(SERVO_PIN_TX_ENB, SERVO_PIN_RX_ENB); //init servo circuitry

	int x = 0;
	for(auto& servo : servos){  //assign ids 1..26; 1..24 for legs, 25 head, 26 tail
		servo.id = ++x;
	}

	xTaskCreatePinnedToCore(servo_writer_task,    //task function
							"servo writer task",  //task name
							10000,                //task stack depth in bytes
							NULL,                 //parameters
							1,                    //priority 0..32, smaller = less
							&_servo_writer_task,  //task handle
							1);                   //core (0 or 1)
}

void loop(){
	
}

void servo_writer_task( void * parameter) { //task to write to servos while not reading
	while(1) {
		for(auto& servo : servos){
			if (servo.active && !servo::write_protection){
				servo.move(); //move to pos set in servo::pos
			}
		}
	}
}