#ifndef SERVOS_H
#define SERVOS_H

#include "utils.h"
#include "Arduino.h"
#include "definitions.h"

/**
 * @brief servo class to drive Lobot servos
 * @author nik
 */
class servo{
    public:
        /* static objects */
        static bool write_protection;
        

        /* dynamic objects */
        uint8_t id;             //servo id 1..26
        int angle;          //servo angle -180..180
        bool active = false; //servo state; true on, false off

        /**
         * @brief  function overload to write an angle directly to the servo
         * @param  pos: angle -120..120
         */
        void move(int pos);

        /**
         * @brief  function overload to write the angle stored in angle variable to the servo
         */
        void move();

        /**
         * @brief  function to power off the servo
         */
        void detach();

        /**
         * @brief  function to power on the servo
         */
        void attach();

        /**
         * @brief  function to read servo's position
         * @note   this function is blocking and will halt every write type communication to the servo
         *         until fineshed or timed out. The timeout is set to 5 seconds
         * @retval returns servo's angle in degrees -180..180 if the communication worked or INT_MAX if errors were found
         */
        int read();
};

#endif
