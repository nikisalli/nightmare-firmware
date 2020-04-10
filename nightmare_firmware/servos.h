#ifndef N_SERVOS_H
#define N_SERVOS_H

#include "utils.h"
#include "Arduino.h"
#include "definitions.h"

class servo{
    public:
        /* static objects */
        static void init(int _tx_enb, int _rx_enb);
        static bool write_protection;

        /* dynamic objects */
        uint8_t id;             //servo id 1..26
        uint8_t angle;          //servo angle -180..180
        uint8_t active = false; //servo state; true on, false off
        void move(int pos);     //move to pos
        void move();            //move to angle (angle as buffer)
        void detach();          //deactivate servo
        void attach();          //activate servo
        int read();             //read servo angle -180..180
};

#endif
