#include "Input.h"
#include "Config.h"
#include <Arduino.h>

static float mapf(const int v, int inMin, int inMax, const float outMin, const float outMax) {
  float t = static_cast<float>(v - inMin) / static_cast<float>(inMax - inMin);
  if (t < 0) t = 0;
  if (t > 1) t = 1;
  return outMin + t * (outMax - outMin);
}

void initInputs() {
  pinMode(PIN_BTN_COMMIT, INPUT_PULLUP);
}

InputState readInputs() {
  InputState inputState{};

  const int ax = analogRead(PIN_POT_X);
  const int ay = analogRead(PIN_POT_Y);
  const int az = analogRead(PIN_POT_Z);
  const int ag = analogRead(PIN_POT_G);

  inputState.target.x = mapf(ax, ADC_MIN, ADC_MAX, -R_MAX, +R_MAX);
  inputState.target.y = mapf(ay, ADC_MIN, ADC_MAX, -R_MAX, +R_MAX);
  inputState.target.z = mapf(az, ADC_MIN, ADC_MAX,  Z_MIN,  Z_MAX);

  const float r = sqrt(inputState.target.x*inputState.target.x + inputState.target.y*inputState.target.y);
  
  if (r < 1e-6f) {
    inputState.target.x = R_MIN;
    inputState.target.y = 0.0f;
  } else if (r < R_MIN) {
    const float scale = R_MIN / r;
    inputState.target.x *= scale;
    inputState.target.y *= scale;
  } else if (r > R_MAX) {
    const float scale = R_MAX / r;
    inputState.target.x *= scale;
    inputState.target.y *= scale;
  }

  inputState.gripper01 = mapf(ag, ADC_MIN, ADC_MAX, 0.0f, 1.0f);

  static bool lastRaw = false;
  static bool stable = false;
  static bool lastStable = false;
  static unsigned long lastChangeMs = 0;

  const bool raw = (digitalRead(PIN_BTN_COMMIT) == LOW);
  const unsigned long now = millis();

  if (raw != lastRaw) {
    lastRaw = raw;
    lastChangeMs = now;
  }

  if (now - lastChangeMs > 30) {
    stable = raw;
  }

  inputState.commitPressed = (!lastStable && stable);
  inputState.commitDown = stable;
  lastStable = stable;

  return inputState;
}