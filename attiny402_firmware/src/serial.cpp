#include "serial.h"

uint8_t led_mode = 0;

uint8_t _write(uint8_t b) {
	Serial.write(b);
	return b;
}

template<typename... Args>
uint8_t _write(uint8_t b, Args... args) {
	return _write(b) + _write(args...);
}

template<typename... Args>
void servo_write(Args... args) {
    sensor_tx_enb();
	_write(SENSOR_HEADER);
	uint8_t checksum = ~_write(uint8_t(args)...);
	_write(checksum);
    Serial.flush();
    sensor_rx_enb();
}

void sensor_tx_enb() {
    PORTA.OUT |= PIN3_bm;
}

void sensor_rx_enb() {
	PORTA.OUT &= ~PIN3_bm;
}

void handler::write_pressure(uint16_t pressure){
	servo_write(
        id,
        SENSOR_PRESSURE_READ_RES_LENGTH,
		SENSOR_PRESSURE_READ,
		pressure & 0xFF,
		pressure >> 8
	);
  
  	Serial.flush(); //may cause slower write
}

void handler::write_id(){
    servo_write(
        id,
        SENSOR_ID_READ_RES_LENGTH,
        SENSOR_ID_READ,
        DEFAULT_SENSOR_ID
	);
  
  	Serial.flush(); //may cause slower write
}

void handler::handle(){
    while(!Serial.available());
    if(Serial.read() != expected_headers[0][0]) return;
    while(!Serial.available());
    if(Serial.read() != expected_headers[0][1]) return;
    while(!Serial.available());
    uint8_t res = Serial.read();
    if(res != expected_headers[0][2] && res != expected_headers[1][2]) return;

    while(Serial.available() < 2);
    uint8_t length = Serial.read();         //read packet length and cmd id
    uint8_t cmd = Serial.read();
    uint8_t _id = res;

    uint8_t buf[7] = {};                    //array to store packet

    uint8_t sum = _id + length + cmd;

    while(Serial.available() < length - 2); //wait for bytes

    for(uint8_t i = 0; i < length - 2; i++){//iterate until the packet has been fully read
        buf[i] = Serial.read();
        if(i != length - 3) sum += buf[i];   //don't add checksum to calculated checksum
    }

    delayMicroseconds(100);

    if(((~sum) & 0xFF) != (buf[length - 3] & 0xFF)) return;     //match checksum
    
    if (_id == id) {
        switch (cmd) {
            case SENSOR_PRESSURE_READ:
                write_pressure((uint16_t)analogRead(A2));
                break;

            /*case SENSOR_ID_WRITE:         //not used since ids are hardcoded for now
                eeprom_write_byte((uint8_t*)EEPROM_ID_ADDR, buf[0]);
                id = buf[0];
                expected_headers[0][2] = buf[0];
                break;*/
                
            case SENSOR_LED_RGB_WRITE:
                if(led_mode != 0) return;   //only change color if the LED is in direct control mode (0)

                set_led_color(buf[0], buf[1], buf[2]);
                break;

            case SENSOR_LED_MODE_WRITE:
                led_mode = buf[0];
                if(led_mode == 0) set_led_color(0,0,0);
                break;
        }
    } else if(_id == 0xFE && cmd == SENSOR_ID_READ) {
        write_id();
    }   
}