#include "Input.h"
#include "Config.h"
#include "Util.h"
#include <Arduino.h>

void initInputs() {
  pinMode(cfg::PIN_BTN_COMMIT, INPUT_PULLUP);
}

InputState readInputs() {
  InputState inputState{};

  const int ax = analogRead(cfg::PIN_POT_X);
  const int ay = analogRead(cfg::PIN_POT_Y);
  const int az = analogRead(cfg::PIN_POT_Z);
  const int ag = analogRead(cfg::PIN_POT_G);

  inputState.target.x = util::mapRangeClamped(ax, cfg::ADC_MIN, cfg::ADC_MAX, -cfg::R_MAX, +cfg::R_MAX);
  inputState.target.y = util::mapRangeClamped(ay, cfg::ADC_MIN, cfg::ADC_MAX, -cfg::R_MAX, +cfg::R_MAX);
  inputState.target.z = util::mapRangeClamped(az, cfg::ADC_MIN, cfg::ADC_MAX,  cfg::Z_MIN,  cfg::Z_MAX);

  const float r = sqrt(inputState.target.x*inputState.target.x + inputState.target.y*inputState.target.y);

  if (r < 1e-6f) {
    inputState.target.x = cfg::R_MIN;
    inputState.target.y = 0.0f;
  } else if (r < cfg::R_MIN) {
    const float scale = cfg::R_MIN / r;
    inputState.target.x *= scale;
    inputState.target.y *= scale;
  } else if (r > cfg::R_MAX) {
    const float scale = cfg::R_MAX / r;
    inputState.target.x *= scale;
    inputState.target.y *= scale;
  }

  inputState.gripper01 = util::mapRangeClamped(ag, cfg::ADC_MIN, cfg::ADC_MAX, 0.0f, 1.0f);

  static bool lastRaw = false;
  static bool stable = false;
  static bool lastStable = false;
  static unsigned long lastChangeMs = 0;

  const bool raw = (digitalRead(cfg::PIN_BTN_COMMIT) == LOW);
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