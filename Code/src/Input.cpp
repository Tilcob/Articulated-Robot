#include "Input.h"
#include "Config.h"
#include <Arduino.h>

static float mapf(int v, int inMin, int inMax, float outMin, float outMax) {
  float t = (float)(v - inMin) / (float)(inMax - inMin);
  if (t < 0) t = 0;
  if (t > 1) t = 1;
  return outMin + t * (outMax - outMin);
}

static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

InputState readInputs() {
  InputState s{};

  int ax = analogRead(PIN_POT_X);
  int ay = analogRead(PIN_POT_Y);
  int az = analogRead(PIN_POT_Z);
  int ag = analogRead(PIN_POT_G);

  s.target.x = mapf(ax, ADC_MIN, ADC_MAX, -R_MAX, +R_MAX);
  s.target.y = mapf(ay, ADC_MIN, ADC_MAX, -R_MAX, +R_MAX);
  s.target.z = mapf(az, ADC_MIN, ADC_MAX,  Z_MIN,  Z_MAX);

  float r = sqrt(s.target.x*s.target.x + s.target.y*s.target.y);
  
  if (r < R_MIN) {
    float scale = (r < 1e-6f) ? 0.0f : (R_MIN / r);
    s.target.x *= scale;
    s.target.y *= scale;
  } else if (r > R_MAX) {
    float scale = R_MAX / r;
    s.target.x *= scale;
    s.target.y *= scale;
  }

  s.gripper01 = mapf(ag, ADC_MIN, ADC_MAX, 0.0f, 1.0f);
  return s;
}