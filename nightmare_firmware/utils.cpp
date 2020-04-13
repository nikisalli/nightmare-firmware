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

/* lp filter methods */
lp_filter::lp_filter(double rc, double start_val){
  constant = rc;
  prev_val = start_val;
}

lp_filter::lp_filter(double rc){
  constant = rc;
}

double lp_filter::get_val(){
  double interv = (micros() - prev_time)/1000000.0;
  double a = interv / (constant + interv);
  double val = ((1.0 - a) * prev_val) + (a * input);
  prev_val = val;
  prev_time = micros();
  return val;
}