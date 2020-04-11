#include "utils.h"

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

float clamp(float val, float min, float max) {
  if (val > max) {
    val = max;
  }
  else if (val < min) {
    val = min;
  }
  return val;
}