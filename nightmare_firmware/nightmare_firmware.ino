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

	/* initialize tasks */
	xTaskCreatePinnedToCore(servo_writer_task,    //task function
							"servo writer task",  //task name
							10000,                //task stack depth in bytes
							NULL,                 //parameters
							1,                    //priority 0..32, smaller = less
							&_servo_writer_task,  //task handle
							1);                   //core (0 or 1)
}

void loop(){
	/* esp32 bound packet (ROS -> esp32):
		>HEADER: (ten 0x55 bytes by default)
		>SERVO ANGLES: an array of 27 angles, one byte each going from 0 to 240 meaning -120 to 120 
					  the last angle (tilt of the head) is in range -90..90 because it is a standard servo
					  and not a serial one
		>SERVO ENABLES: 27 bytes containing 27 enable bits
		>FAN SPEED: two bytes going from 0 to 255 indicating the onboard fan speed (0 to 100%)
		>CHECKSUM: the first 8 bits of the sum of all the packet's bytes

		total bytes: 10 (if header not changed) + 27 + 27 + 2 + 1 = 67 bytes
		checksum bytes: 27 + 27 + 2 + 1 = 57 bytes
	*/

	/* search and wait for header */
	uint8_t iters = 0;
	while(iters < sizeof(HEADER)/sizeof(HEADER[0])){
		if(Serial.available() > 0){
			if(Serial.read() == HEADER[iters]){
				iters++;
			} else {
				iters = 0;
			}
		}
	}

	/* read payload and checksum */
	//Serial.setTimeout(100);
	uint8_t buf[66] = {};  // read everything 
	Serial.readBytes(buf, 66);
	int checksum = Serial.read();

	/* evaluate and compare checksum */
	int _checksum = 0;
	for(auto& x : buf){
		_checksum += x;
	}
	if(!((uint8_t)_checksum == checksum)){
		return; //start the loop again because the packet isn't valid
	}

	/* unpack the packet and fill angle buffers */
	for(int i=0; i<26; i++){
		servos[i].angle = buf[i] - 120;
		if(buf[i+27] == 1){
			servos[i].attach();
		} else {
			servos[i].detach();
		}
	}

	//TODO set fan speed
	//TODO send response packet
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