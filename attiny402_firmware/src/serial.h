#ifndef SERIAL_H
#define SERIAL_H

#include <avr/io.h>
#include <avr/eeprom.h> 
#include <Arduino.h>

#define SENSOR_HEADER uint8_t(0x55), uint8_t(0x55)

#define SENSOR_PRESSURE_READ            37
#define SENSOR_PRESSURE_READ_LENGTH     3
#define SENSOR_ID_READ                  38
#define SENSOR_ID_READ_LENGTH           3
#define SENSOR_ID_WRITE                 39
#define SENSOR_ID_WRITE_LENGTH          4
#define SENSOR_LED_RGB_WRITE            40
#define SENSOR_LED_RGB_WRITE_LENGTH     6
#define SENSOR_LED_MODE_WRITE           41
#define SENSOR_LED_MODE_WRITE_LENGTH    4

#define EEPROM_ID_ADDR                  0

class handler{
    public:
        uint8_t id = eeprom_read_byte(EEPROM_ID_ADDR);
        uint8_t led_mode = 0;

        handler();
        void handle();
        void write_pressure(uint16_t pressure);
        void write_id();

    private:
        uint8_t expected_header[3] = {SENSOR_HEADER, id};

};

#endif