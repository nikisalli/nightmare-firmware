#ifndef SENSORS_H
#define SENSORS_H

#include <stdio.h>
#include <string.h>
#include <pico/stdlib.h>
#include <hardware/gpio.h>
#include <hardware/adc.h>
#include <gy953.h>

typedef struct sensor_data {
    uint16_t battery_voltage_raw;
    float battery_voltage;
    uint16_t battery_current_raw;
    float battery_current;
    int32_t load_cells_raw[6];
    uint16_t rp2040_temperature_raw;
    float rp2040_temperature;
    float battery_power;
    uint16_t roll_raw;
    uint16_t pitch_raw;
    uint16_t yaw_raw;
    float roll;
    float pitch;
    float yaw;
} sensor_data;

void sensors_init();
void sensors_read(sensor_data* data);
bool sensors_update();

#endif