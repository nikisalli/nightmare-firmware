#ifndef SERIAL_H
#define SERIAL_H

#include "led.h"

#include <avr/io.h>
#include <avr/eeprom.h> 
#include <Arduino.h>

#define SENSOR_HEADER                   uint8_t(0x55), uint8_t(0x55)

#define SENSOR_BROADCAST_ID             254
#define DEFAULT_SENSOR_ID               1

#define SENSOR_PRESSURE_READ            37
#define SENSOR_PRESSURE_READ_LENGTH     3
#define SENSOR_PRESSURE_READ_RES_LENGTH 5
#define SENSOR_ID_READ                  14
#define SENSOR_ID_READ_LENGTH           3
#define SENSOR_ID_READ_RES_LENGTH       4
//#define SENSOR_ID_WRITE                 39    //not used since ids are hardcoded for now
//#define SENSOR_ID_WRITE_LENGTH          4
#define SENSOR_LED_RGB_WRITE            40
#define SENSOR_LED_RGB_WRITE_LENGTH     6
#define SENSOR_LED_MODE_WRITE           41
#define SENSOR_LED_MODE_WRITE_LENGTH    4

#define EEPROM_ID_ADDR                  0

class handler{
    public:
        uint8_t id = DEFAULT_SENSOR_ID;

        void handle();
        void write_pressure(uint16_t pressure);
        void write_id();

    private:
        uint8_t expected_headers[2][3] ={   
                                            {SENSOR_HEADER, id},
                                            {SENSOR_HEADER, SENSOR_BROADCAST_ID}
                                        };

};

#endif