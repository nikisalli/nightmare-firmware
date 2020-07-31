#ifndef LED_H
#define LED_H

#include <WS2812.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define F_CPU           20000000UL

#define TIM0_PRESCALER  16
#define TIM0_MS         30
#define TIM0_MS_FACTOR  ( (F_CPU) / (1000 * TIM0_PRESCALER) )
#define TIM0_COMP_VAL   (TIM0_MS * TIM0_MS_FACTOR)

extern uint8_t led_mode;

void TCA0_init (void);
void set_led_color(uint8_t r, uint8_t g, uint8_t b);
void led_init();

#endif