#include "Mapping.h"
#include "Config.h"
#include <Arduino.h>

static float rad2deg(float const r) { return r * 180.0f / PI; }

static float clampf(float const x, float const lo, float const hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

ServoAngles mapToServos(const Angles& q, float const gripper01) {
  ServoAngles inputState{};

  float const q1 = clampf(q.q1, Q1_MIN, Q1_MAX);
  float const q2 = clampf(q.q2, Q2_MIN, Q2_MAX);
  float const q3 = clampf(q.q3, Q3_MIN, Q3_MAX);

  float const qElbow = ELBOW_SERVO_USES_FOREARM_ABS ? (q2 + q3) : q3;

  const float baseRaw     = BASE_ZERO_DEG     + BASE_SIGN     * rad2deg(q1);
  const float shoulderRaw = SHOULDER_ZERO_DEG + SHOULDER_SIGN * rad2deg(q2);
  const float elbowRaw    = ELBOW_ZERO_DEG    + ELBOW_SIGN    * rad2deg(qElbow);

  constexpr float gripMin = 20.0f;
  constexpr float gripMax = 120.0f;
  inputState.gripperDeg = gripMin + gripper01 * (gripMax - gripMin);

  inputState.baseDeg     = clampf(baseRaw,     SERVO_MIN_DEG, SERVO_MAX_DEG);
  inputState.shoulderDeg = clampf(shoulderRaw, SERVO_MIN_DEG, SERVO_MAX_DEG);
  inputState.elbowDeg    = clampf(elbowRaw,    SERVO_MIN_DEG, SERVO_MAX_DEG);
  inputState.gripperDeg  = clampf(inputState.gripperDeg,  SERVO_MIN_DEG, SERVO_MAX_DEG);

  // If we had to clamp, it can look like "wrong kinematics".
  // Print a rate-limited warning to make this visible.
  if (ENABLE_SERVO_SAT_WARN) {
    const bool sat = (baseRaw != inputState.baseDeg) || (shoulderRaw != inputState.shoulderDeg) || (elbowRaw != inputState.elbowDeg);
    if (sat) {
      static unsigned long lastWarnMs = 0;
      const unsigned long now = millis();
      if (now - lastWarnMs >= SERVO_SAT_WARN_PERIOD_MS) {
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