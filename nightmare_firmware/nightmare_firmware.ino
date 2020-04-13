#include "servos.h"
#include "definitions.h"
#include "utils.h"
#include <ESP32Servo.h>

TaskHandle_t _servo_writer_task;        //task handle for writer task
servo servos[26];                       //servo objects
Servo hservo;
lp_filter voltage_filter(1);		    //voltage filter object
lp_filter current_filter(1);		    //current filter object

bool servo::write_protection = false;   //flag to stop task when reading servos

void setup(){
	/* initialize hardware */
	Serial.begin(460800);               //ROS serial port

	pinMode(PANDA_POWER_PIN,OUTPUT);    //lattepanda power on gpio
	pinMode(FAN1_PIN, OUTPUT);          //fans
  	pinMode(FAN2_PIN, OUTPUT);
  	pinMode(DEBUG_LED_PIN_1, OUTPUT);   //debug leds
  	pinMode(DEBUG_LED_PIN_2, OUTPUT);


  	digitalWrite(FAN1_PIN, HIGH); //enable just one fan by default
  	digitalWrite(FAN2_PIN, LOW);

	analogReadResolution(10); //set adc precision to 10 bits
  	analogSetPinAttenuation(BATT_VOLTAGE_READ_PIN, ADC_0db); //set adc to full scale read

	servo::init(SERVO_PIN_TX_ENB, SERVO_PIN_RX_ENB); //init servo circuitry
	hservo.setPeriodHertz(SERVO_FREQ_HEAD_TILT);

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
	read_packet();
	write_packet();
}

/**
 * @brief function to send ROS bound packet
 * @note ROS bound packet (esp32 -> ROS):
 * 
		## BASIC DATA PACKET ##

			{HEADER + BATTERY VOLTAGE + BATTERY CURRENT + JOYSTICK POS + JOYSTICK STATE + CHECKSUM}

			total bytes: 10 (if header not changed) + 1 + 1 + 4 + 1 + 1 = 18 bytes
			checksum bytes: 1 + 1 + 4 + 1 + 1 = 8 bytes
			
		## FULL DATA PACKET ##

			{HEADER + BATTERY VOLTAGE + BATTERY CURRENT + JOYSTICK POS + JOYSTICK STATE + SERVO ANGLES + CHECKSUM}

			total bytes: 10 (if header not changed) + 1 + 1 + 4 + 1 + 54 + 1= 72 bytes
			checksum bytes: 1 + 1 + 4 + 1 + 54 + 1 = 62 bytes
			
		>HEADER -> (ten 0x55 bytes by default)
		>ID -> 0 for basic data, 1 for full data
		>SERVO ANGLES ->  an array of 27 angles, two bytes each going from 0 to 1500 meaning -180 to 180 
					  	the last angle (tilt of the head) is in range -90..90 because it is a standard servo
					  	and not a serial one but still sent as 2 bytes
		>BATTERY VOLTAGE -> one byte 0..255 meaning 0..9V
		>BATTERY CURRENT -> one byte 0..255 meaning 0..12A
		>JOYSTICK POS -> 4 bytes describing the two joystick positions in the following order: LX LY RX RY
		>JOYSTICK STATE -> 1 byte sent by the nightcontrol app through 433Mhz
		>CHECKSUM -> the first 8 bits of the sum of all the packet's bytes
 */
void write_packet(){
	/* check if all servos are attached */
	bool all_active = true;
	for(auto& servo : servos){
		if(servo.active == false){
			all_active = false;
			break;
		}
	}
	if(!hservo.attached()){
		all_active = false;
	}

	int header_size = sizeof(HEADER)/sizeof(HEADER[0]);
	uint8_t buf[67] = {};

	for(int i=0; i<header_size; i++){
		buf[i] = HEADER[i];
	}

	voltage_filter.input = fmap(analogRead(BATT_VOLTAGE_READ_PIN), 0, 1023, 0, 12.2); //scale the value read to 12.2V
	current_filter.input = ((analogRead(BATT_CURR_READ_PIN) * 0.0008625) - 2.49) * 10; //magic numbers to scale and offset the current data
	
	buf[10] = (byte)(fmap(voltage_filter.get_val(),0,9,0,255)); //battery voltage byte
	buf[11] = (byte)(fmap(current_filter.get_val(),0,12.0,0,255)); //battery current byte
	buf[12] = 0; //TODO joystick LX
	buf[13] = 0; //TODO joystick LY
	buf[14] = 0; //TODO joystick RX
	buf[15] = 0; //TODO joystick RY
	buf[16] = 0; //TODO joystick state

	if(all_active){
		// send basic data
		// calculate checksum
		int checksum = 0;
		for(int i=0; i<17; i++){ //packet length -1 because the last byte is the checksum
			checksum += buf[i];
		}
		buf[17] = (uint8_t)checksum;

		//write packet
		for(int i=0; i<18; i++){
			Serial.write(buf[i]);
		}
	} else {
		// send all data and poll servo positions
		for(int i=0; i<27; i++){
			int ang = servos[i].read();
			buf[17 + i*2] = (uint8_t)(ang >> 8);
			buf[18 + i*2 ] = (uint8_t)(ang & 0xFF);
		}
		int checksum = 0;
		for(int i=0; i<71; i++){ //packet length -1 because the last byte is the checksum
			checksum += buf_byte;
		}
		buf[71] = (uint8_t)checksum;

		//write packet
		for(int i=0; i<72; i++){
			Serial.write(buf[i]);
		}
	}
}

/**
 * @brief function to receive esp32 bound packets
 * @note  esp32 bound packet (ROS -> esp32):
		
		## PACKET ##

			{HEADER + SERVO ANGLES + SERVO ENABLES + FAN SPEED + CHECKSUM}

			total bytes -> 10 (if header not changed) + 27 + 27 + 2 + 1 = 67 bytes
			checksum bytes -> 27 + 27 + 2 + 1 = 57 bytes

		>HEADER -> (ten 0x55 bytes by default)
		>SERVO ANGLES -> an array of 27 angles, one byte each going from 0 to 240 meaning -120 to 120 
					  the last angle (tilt of the head) is in range -90..90 because it is a standard servo
					  and not a serial one
		>SERVO ENABLES -> 27 bytes containing 27 enable bits
		>FAN SPEED -> two bytes going from 0 to 255 indicating the onboard fan speed (0 to 100%)
		>CHECKSUM -> the first 8 bits of the sum of all the packet's bytes
 */
void read_packet(){
	/* 
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

	//analog servos use a different method for attach and detach so we do that here
	if(buf[53] == 1){
		hservo.attach(SERVO_PIN_HEAD_TILT, SERVO_MIN_WIDTH_HEAD_TILT, SERVO_MAX_WIDTH_HEAD_TILT);
	} else {
		hservo.detach();
	}

	//if the servo is attached write the position to it
	if(hservo.attached()){
		hservo.write(buf[26]);
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