#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>

/**
 * @brief  linear interpolation in_min:in_max=out_min:out_max
 * @param  x: value relative to in_min..in_max range
 * @param  in_min: lower input reference for interpolation
 * @param  in_max: higher input reference for interpolation
 * @param  out_min: lower out reference for interpolation
 * @param  out_max: higher out reference for interpolation
 * @retval value relative to out_min..out_max 
 */
extern float fmap(float x, float in_min, float in_max, float out_min, float out_max);

/**
 * @brief  clamp value to range
 * @param  val: input value
 * @param  min: minimum value
 * @param  max: maximum value
 * @retval clamped value
 */
extern float clamp(float val, float min, float max);

/**
 * @brief class to create a simple low pass filter
 * @note this is based on micros() so it may create glitches every few hours (when micros overflows)
 * @author Nik
 */
class lp_filter{
    public:
        float input; //filter's input sample
        float constant; //filter's time constant

        /**
         * @brief  constructor that initializes both the filter constant and start value
         * @param  rc: filter's constant
         * @param  start_val: filter's initial input value
         */
        lp_filter(double rc, double start_val);

        /**
         * @brief  constructor overload that initializes only the filter constant
         * @param  rc: filter's constant
         */
        lp_filter(double rc);

        /**
         * @brief  get filtered value
         * @retval filtered value
         */
        double get_val(); //get filtered sample
    private:
        float prev_time;
        float prev_val;
};

#endif