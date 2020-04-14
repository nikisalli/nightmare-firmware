#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include "Arduino.h"

//
//                        /
//                      /    ______
//                    /     |      |                    |-|-----|-|
//                  FM     /| +--+ | ^               O               O
//                 /     /  |______| |               | \           / |
//               /     /    / |  |   |         LEG 0 |  \  (:::)  /  | LEG 7
//             /     /    /   |  |   |               |   \ __X__ /   |
//      <--CX-->   /    /     |  |   |          O\        X     X        /O
//       ___ ____/_   /       |  |   |    LEG 1 |   \     )     (     /   |
//------|   |      |/         |  |  TB          |      \X         X/      | LEG 6
//      | + | +--+ |          |  |   |          |O__     )       (     __O|
//------|___|______|          |  |   |           |   ''\X         X/''   |
//                            |__|   |           |        )     (        | 
//                            |  |   |    LEG 2  | O______X_____X______O |  LEG 5
//                            |  |   |             |        ]|[        |
//                            |__|   v             |       || ||       |
//                                          LEG 3  |       || ||       |  LEG 4
//                                                         |___|

/* software defines */
const uint8_t ROS_HEADER[] = {0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55};
const uint8_t HC12_HEADER[] = {0x55, 0x55};

/* hardware defines */
const int   FAN1_PIN = 21;
const int   FAN2_PIN = 22;
const int   SERVO_PIN_TX_ENB = GPIO_NUM_18;
const int   SERVO_PIN_RX_ENB = GPIO_NUM_19;
const int   SERVO_PIN_HEAD_TILT = 12;
const int   SERVO_MAX_WIDTH_HEAD_TILT = 2400; // microseconds
const int   SERVO_MIN_WIDTH_HEAD_TILT = 400; // microseconds
const int   SERVO_FREQ_HEAD_TILT = 50; //Hz
const int   HC12_PIN_TX = 2;
const int   HC12_PIN_RX = 15;
const int   HC12_PIN_SET = 13;
const int   DEBUG_LED_PIN_1 = 4;
const int   DEBUG_LED_PIN_2 = 5;
const int   BATT_VOLTAGE_READ_PIN = 35;
const int   BATT_VOLTAGE_READ_FILTER_CONSTANT = 2;
const int   BATT_CURR_READ_PIN = 32;
const int   BATT_CURR_READ_FILTER_CONSTANT = 1;
const int   HC12_BAUD_RATE = 9600;
const int   DEF_FAN_SPEED = 100;
const int   MAX_FAN_SPEED = 255;
const int   PANDA_POWER_PIN = 23;
const int   PANDA_POWER_TIME = 3000;
const float BATT_MIN_VOLTAGE = 7.0f;
const float BATT_MAX_VOLTAGE = 8.5f;

#endif