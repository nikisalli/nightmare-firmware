/*
Copyright 2016 .s.u.m.o.t.o.y.
Version 0.01, early aplha stage!
*/

#include <stdio.h>
#include <string.h>
#include <pico/stdlib.h>
#include <hardware/spi.h>
#include <hardware/gpio.h>

#ifndef gy953_h_
#define gy953_h_

#define _GY953_MAGACC		0x01
#define _GY953_MAGGYR		0x02
#define _GY953_MAGMAG		0x04
#define _GY953_MAGRPY		0x08
#define _GY953_MAG_Q4		0x10
#define _GY953_SETREG		0x41
#define _GY953_CALREG		0x42
#define _GY953_INTREG		0xC1
#define _GY953_UPFREQ_50	0x73
#define _GY953_UPFREQ_100	0x74
#define _GY953_UPFREQ_200	0x75
#define _GY953_CAL_ACC		0x04
#define _GY953_CAL_MAG		0x08
#define _GY953_CAL_CLR		0x80

typedef struct {
    uint8_t cs_pin;
    uint8_t int_pin;
    uint8_t sck_pin;
    uint8_t mosi_pin;
    uint8_t miso_pin;
    uint8_t	_currentMode;
	uint8_t	_refreshRate;
    int	_rpy[3];
	int	_raw[4];
    volatile uint8_t _stateReg;
} gy953;

void gy953_init(gy953* s);
bool gy953_update(gy953* s, uint8_t mode);
void gy953_getRPY(gy953* s, int *data);   //Roll,Pitch,Yaw
void gy953_getACC(gy953* s, int *data);   //x,y,z,0
void gy953_getGYR(gy953* s, int *data);   //x,y,z,0
void gy953_getMAG(gy953* s, int *data);   //x,y,z,0
void gy953_getQ(gy953* s, int *data);     //x,y,z
void gy953_setRefreshRate(gy953* s, uint8_t freq); //50,100,200
void gy953_calibration(gy953* s, uint8_t mode);
void gy953_readAccuracy(uint8_t *data);
void gy953_readRange(uint8_t *data);
void gy953_sensorEnabled(gy953* s, uint8_t mode);

void gy953_disableCS(gy953* s);
void gy953_enableCS(gy953* s);

void gy953_writeRegister(gy953* s, uint8_t reg, uint8_t *data, int len);
void gy953_readRegister(gy953* s, uint8_t reg, uint8_t *data, int len);
void gy953_enableInt(gy953* s);
static void gy953_interrupt_callback(uint gpio, uint32_t events);
void gy953_setMode(gy953* s, uint8_t mode);

#endif