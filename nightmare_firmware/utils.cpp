#include "utils.h"

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

float limit(float val, float downlimit, float uplimit) {
  if (val > uplimit) {
    val = uplimit;
  }
  else if (val < downlimit) {
    val = downlimit;
  }
  return val;
}