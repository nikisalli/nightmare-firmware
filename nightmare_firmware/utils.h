#ifndef UTILS_H
#define UTILS_H

/**
 * @brief  linear interpolation in_min:in_max=out_min:out_max
 * @param  x: value relative to in_min..in_max range
 * @param  in_min: lower input reference for interpolation
 * @param  in_max: higher input reference for interpolation
 * @param  out_min: lower out reference for interpolation
 * @param  out_max: higher out reference for interpolation
 * @retval value relative to out_min..out_max 
 */
float fmap(float x, float in_min, float in_max, float out_min, float out_max);

/**
 * @brief  clamp value to range
 * @param  val: input value
 * @param  min: minimum value
 * @param  max: maximum value
 * @retval clamped value
 */
float limit(float val, float min, float max);

#endif