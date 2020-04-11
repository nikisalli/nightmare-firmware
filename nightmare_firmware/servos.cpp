#include "servos.h"

#define SERVO_HEADER byte(0x55), byte(0x55)
#define SERVO_MOVE_TIME_WRITE       7,  1
#define SERVO_MOVE_TIME_READ        3,  2
#define SERVO_MOVE_TIME_WAIT_WRITE  7,  7
#define SERVO_MOVE_TIME_WAIT_READ   3,  8
#define SERVO_MOVE_START            3, 11
#define SERVO_MOVE_STOP             3, 12
#define SERVO_ID_WRITE              4, 13
#define SERVO_ID_READ               3, 14
#define SERVO_ANGLE_OFFSET_ADJUST   4, 17
#define SERVO_ANGLE_OFFSET_WRITE    3, 18
#define SERVO_ANGLE_OFFSET_READ     3, 19
#define SERVO_ANGLE_LIMIT_WRITE     7, 20
#define SERVO_ANGLE_LIMIT_READ      3, 21
#define SERVO_VIN_LIMIT_WRITE       7, 22
#define SERVO_VIN_LIMIT_READ        3, 23
#define SERVO_TEMP_MAX_LIMIT_WRITE  4, 24
#define SERVO_TEMP_MAX_LIMIT_READ   3, 25
#define SERVO_TEMP_READ             3, 26
#define SERVO_VIN_READ              3, 27
#define SERVO_POS_READ              3, 28
#define SERVO_OR_MOTOR_MODE_WRITE   7, 29
#define SERVO_OR_MOTOR_MODE_READ    3, 30
#define SERVO_LOAD_OR_UNLOAD_WRITE  4, 31
#define SERVO_LOAD_OR_UNLOAD_READ   3, 32
#define SERVO_LED_CTRL_WRITE        4, 33
#define SERVO_LED_CTRL_READ         3, 34
#define SERVO_LED_ERROR_WRITE       4, 35
#define SERVO_LED_ERROR_READ        3, 36

void servo::init(int _tx_enb, int _rx_enb){
	Serial2.begin(115200);          //initialize servo serial
	Serial2.setRxBufferSize(1024);  //make sure the buffer doesn't overflow
  
	pinMode(_tx_enb, OUTPUT);       //hardware tx enable pin
	pinMode(_rx_enb, OUTPUT);       //hardware rx enable pin
}

byte _write(byte b) {
	Serial2.write(b);
	return b;
}

template<typename... Args>
byte _write(byte b, Args... args) {
	return _write(b) + _write(args...);
}

template<typename... Args>
void servo_write(Args... args) {
	_write(SERVO_HEADER);
	byte checksum = ~_write(byte(args)...);
	_write(checksum);
}

void servo_tx_enb() {
	GPIO.out_w1ts |= (1 << SERVO_PIN_TX_ENB); //set tx 
	GPIO.out_w1tc |= (1 << SERVO_PIN_RX_ENB); //clear rx
}

void servo_rx_enb() {
	GPIO.out_w1tc |= (1 << SERVO_PIN_TX_ENB); //clear tx
	GPIO.out_w1ts |= (1 << SERVO_PIN_RX_ENB); //set rx
}

void servo::move(int ang){
	servo_tx_enb();
  
	ang = limit(map(ang, -120, 120, 0, 1000),0,1000); //angle comes in 2 bytes, 0..1000 means -120..120
	angle = ang;
	servo_write(
		id,
		SERVO_MOVE_TIME_WRITE,
		ang & 0xFF,
		ang >> 8,
		0x00, 0x00
	);
  
  	Serial2.flush(); //may cause slower write
  	servo_rx_enb();
}

void servo::move(){
	servo_tx_enb();
	int ang = angle;
	ang = limit(map(ang, -120, 120, 0, 1000),0,1000);
	servo_write(
		id,
		SERVO_MOVE_TIME_WRITE,
		ang & 0xFF,
		ang >> 8,
		0x00, 0x00
	);
  
  	Serial2.flush(); //may cause slower write
  	servo_rx_enb();
}

void servo::attach(){
  	servo_tx_enb();
  
  	servo_write(
		id,
		SERVO_LOAD_OR_UNLOAD_WRITE,
		0x01
  	);
	
	active = true;

  	Serial2.flush();
  	servo_rx_enb();
}

void servo::detach(){
  	servo_tx_enb();
  
  	servo_write(
		id,
		SERVO_LOAD_OR_UNLOAD_WRITE,
		0x00
  	);

	active = false;

  	Serial2.flush();
  	servo_rx_enb();
}

int servo::read(){
  	servo::write_protection = true;  //enable flag to make sure we can use the bus
  	Serial2.flush();                 //wait for bus to clear
  	servo_tx_enb();                  //enable write mode

  	servo_write(id, SERVO_POS_READ); //request angle

  	Serial2.flush();                 //wait the request for it being sent
  	servo_rx_enb();                  //enable listen mode

  	unsigned long time = micros();   //wait response with 5000ms timeout
  	while(Serial2.available() < 8){  //TODO add error state if timeout reached
		if((micros()-time)>5000){
	  		return INT_MAX;          //send INT_MAX flag if checksums don't match
		}	
  	}

  	while(Serial2.read() != 0x1C);   //wait for id byte

  	byte low = Serial2.read();
  	byte high = Serial2.read();
  	byte checksum = Serial2.read();

  	byte checksum_ = (~((byte)(id+0x21+low+high))); //eval and compare checksum
  	if(checksum == checksum_){
		int val = (low|(high<<8));   //convert the 2 bytes to an int
		if(val > 32767){             //fix overflow if 0 is exceded (angle < -120)
			val -= 65536;
		}
		angle = map(val,0,1000,-120,120); //hard set angle
		return angle;
  	} else {                         //send INT_MAX flag if checksums don't match
		return INT_MAX;
  	}
  	servo::write_protection = false; //disable protection flag
}