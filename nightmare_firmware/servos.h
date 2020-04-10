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
        uint8_t id;
        uint8_t angle;
        uint8_t active = false;
        void move(int pos);
        void move();
        void detach();
        void attach();
        int read();
};

#endif
