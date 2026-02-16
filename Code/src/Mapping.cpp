#include "Mapping.h"
#include "Config.h"
#include "Util.h"
#include <Arduino.h>

ServoAngles mapToServos(const Angles& q, float const gripper01) {
  ServoAngles inputState{};

  float const q1 = util::clampf(q.q1, cfg::Q1_MIN, cfg::Q1_MAX);
  float const q2 = util::clampf(q.q2, cfg::Q2_MIN, cfg::Q2_MAX);
  float const q3 = util::clampf(q.q3, cfg::Q3_MIN, cfg::Q3_MAX);

  const float baseRaw     = cfg::BASE_ZERO_DEG + cfg::BASE_SIGN * (q1 * cfg::RAD2DEG);
  const float shoulderRaw = cfg::SHOULDER_ZERO_DEG + cfg::SHOULDER_SIGN * (q2 * cfg::RAD2DEG);
  const float elbowRaw    = cfg::ELBOW_ZERO_DEG + cfg::ELBOW_SIGN * (q3 * cfg::RAD2DEG);

  inputState.gripperDeg = cfg::GRIPPER_MIN_DEG
                      + gripper01 * (cfg::GRIPPER_MAX_DEG - cfg::GRIPPER_MIN_DEG);

  inputState.baseDeg     = util::clampf(baseRaw, cfg::SERVO_MIN_DEG, cfg::SERVO_MAX_DEG);
  inputState.shoulderDeg = util::clampf(shoulderRaw, cfg::SERVO_MIN_DEG, cfg::SERVO_MAX_DEG);
  inputState.elbowDeg    = util::clampf(elbowRaw, cfg::SERVO_MIN_DEG, cfg::SERVO_MAX_DEG);
  inputState.gripperDeg  = util::clampf(inputState.gripperDeg, cfg::SERVO_MIN_DEG, cfg::SERVO_MAX_DEG);

  if (cfg::ENABLE_SERVO_SAT_WARN) {
    const bool sat = (baseRaw != inputState.baseDeg) ||
                     (shoulderRaw != inputState.shoulderDeg) ||
                     (elbowRaw != inputState.elbowDeg);
    if (sat) {
      static unsigned long lastWarnMs = 0;
      const unsigned long now = millis();
      if (now - lastWarnMs >= cfg::SERVO_SAT_WARN_PERIOD_MS) {
        lastWarnMs = now;
        Serial.print("WARN: servo saturation. raw base="); Serial.print(baseRaw, 1);
        Serial.print(" shoulder="); Serial.print(shoulderRaw, 1);
        Serial.print(" elbow="); Serial.print(elbowRaw, 1);
        Serial.println();
      }
    }
  }

  return inputState;
}