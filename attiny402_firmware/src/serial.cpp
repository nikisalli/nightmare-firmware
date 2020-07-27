#include "serial.h"
#include <WS2812.h>

WS2812 LED(1);
cRGB val;

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

handler::handler(){
    LED.setOutput(A1); // Digital Pin PA1

    val.r = 0;
    val.g = 0;
    val.b = 0;

    LED.set_crgb_at(0, val);  // Set value at LED found at index 0
    LED.sync();  
}

void handler::write_pressure(uint16_t pressure){
	servo_write(
        id,
        SENSOR_PRESSURE_READ_LENGTH,
		SENSOR_PRESSURE_READ,
		pressure & 0xFF,
		pressure >> 8
	);
  
  	Serial.flush(); //may cause slower write
}

void handler::write_id(){
    servo_write(
        id,
        SENSOR_ID_READ_LENGTH,
        SENSOR_ID_READ,
        eeprom_read_byte((uint8_t*)EEPROM_ID_ADDR)
	);
  
  	Serial.flush(); //may cause slower write
}

void handler::handle(){
    for(auto b : expected_header){          //check if header and id are correct
        while(!Serial.available());         //wait for bytes 
        if(Serial.read() != b) return;
    }  
    
    while(Serial.available() < 2);
    uint8_t length = Serial.read();         //read packet length and cmd id
    uint8_t cmd = Serial.read();

    uint8_t buf[7] = {};                    //array to store packet

    uint8_t sum = id + length + cmd;
    
    while(Serial.available() < length - 2); //wait for bytes

    for(uint8_t i = 0; i < length - 2; i++){//iterate until the packet has been fully read
        buf[i] = Serial.read();
        if(i != length - 1) sum += buf[i];  //don't add checksum to calculated checksum
    }

    delayMicroseconds(500);

    //if(~sum != buf[length - 1]) return;     //match checksum
    
    switch(cmd){
        case SENSOR_PRESSURE_READ:
            write_pressure((uint16_t)analogRead(A2));
            break;

        case SENSOR_ID_WRITE:
            eeprom_write_byte((uint8_t*)EEPROM_ID_ADDR, buf[0]);
            id = buf[0];
            expected_header[2] = buf[0];
            break;

        case SENSOR_ID_READ:
            write_id();
            break;
            
        case SENSOR_LED_RGB_WRITE:
            if(led_mode != 0) return;   //only change color if the LED is in direct control mode (0)

            val.r = buf[0];
            val.g = buf[1];
            val.b = buf[2];

            LED.set_crgb_at(0, val);    // Set value at LED found at index 0
            LED.sync();  
            break;

        case SENSOR_LED_MODE_WRITE:
            led_mode = buf[0];
            break;
    }
}