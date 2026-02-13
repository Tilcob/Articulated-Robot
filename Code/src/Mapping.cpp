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

  inputState.baseDeg     = BASE_ZERO_DEG     + BASE_SIGN     * rad2deg(q1);
  inputState.shoulderDeg = SHOULDER_ZERO_DEG + SHOULDER_SIGN * rad2deg(q2);
  inputState.elbowDeg    = ELBOW_ZERO_DEG    + ELBOW_SIGN    * rad2deg(q3);

  constexpr float gripMin = 20.0f;
  constexpr float gripMax = 120.0f;
  inputState.gripperDeg = gripMin + gripper01 * (gripMax - gripMin);

  inputState.baseDeg     = clampf(inputState.baseDeg,     SERVO_MIN_DEG, SERVO_MAX_DEG);
  inputState.shoulderDeg = clampf(inputState.shoulderDeg, SERVO_MIN_DEG, SERVO_MAX_DEG);
  inputState.elbowDeg    = clampf(inputState.elbowDeg,    SERVO_MIN_DEG, SERVO_MAX_DEG);
  inputState.gripperDeg  = clampf(inputState.gripperDeg,  SERVO_MIN_DEG, SERVO_MAX_DEG);

  return inputState;
}