#include "led.h"

WS2812 LED(1);                  //rgb led controller instance
cRGB val;                       //used for global color changes
uint8_t a_rgb[3] = {0,0,255};   //used for automatic color changes
uint8_t rgb_cycle_state = 0;    //used to keep track of the colors being changed in the rgb cycler

ISR(TCA0_CMP0_vect) {
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_CMP0_bm;          //clear interrupt flag

    switch(led_mode){
        case 0:
            return;                                     //in led_mode 0 just return so you can set the color externally
        case 1:
            a_rgb[rgb_cycle_state]+=2;                  //no questions. this cycles smoothly through rgb colors and it just works
            a_rgb[(rgb_cycle_state+1) % 3]-=2;
            if(a_rgb[rgb_cycle_state] > 253) rgb_cycle_state++;
            if(rgb_cycle_state > 2) rgb_cycle_state = 0;
            set_led_color(a_rgb[0], a_rgb[1], a_rgb[2]);
    }
}

void TCA0_init (void) {
    TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_FRQ_gc;       //set waveform generation mode
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV256_gc;    //set clock prescaler
    TCA0.SINGLE.CMP0 = TIM0_COMP_VAL;                   //set compare match value
    TCA0.SINGLE.INTCTRL = TCA_SINGLE_CMP0_bm;           //set interrupt on compare match
    TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm;          //enable timer
}

void set_led_color(uint8_t r, uint8_t g, uint8_t b){
    val.r = r;
    val.g = g;
    val.b = b;

    LED.set_crgb_at(0, val);  // Set value at LED found at index 0
    LED.sync(); 
}

void led_init(){
    LED.setOutput(A1); // Digital Pin PA1

    val.r = 0;
    val.g = 0;
    val.b = 0;

    LED.set_crgb_at(0, val);  // Set value at LED found at index 0
    LED.sync();  
}