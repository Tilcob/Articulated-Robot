#include "Mapping.h"
#include "Config.h"
#include <Arduino.h>
#include <math.h>

static float rad2deg(float const r) { return r * 180.0f / PI; }

static float clampf(float const x, float const lo, float const hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

ServoAngles mapToServos(const Angles& q, float gripper01) {
  ServoAngles s{};

  // Limits (Kinematik-Winkel)
  float const q1 = clampf(q.q1, Q1_MIN, Q1_MAX);
  float const q2 = clampf(q.q2, Q2_MIN, Q2_MAX);
  float const q3 = clampf(q.q3, Q3_MIN, Q3_MAX);

  // Servo = zero + sign * deg(q)
  s.baseDeg     = BASE_ZERO_DEG     + BASE_SIGN     * rad2deg(q1);
  s.shoulderDeg = SHOULDER_ZERO_DEG + SHOULDER_SIGN * rad2deg(q2);
  s.elbowDeg    = ELBOW_ZERO_DEG    + ELBOW_SIGN    * rad2deg(q3);

  // Gripper: simple 0..1 auf Bereich (anpassen!)
  float const gripMin = 20.0f;
  float const gripMax = 120.0f;
  s.gripperDeg = gripMin + gripper01 * (gripMax - gripMin);

  // Servo Bereich clamp
  s.baseDeg     = clampf(s.baseDeg,     SERVO_MIN_DEG, SERVO_MAX_DEG);
  s.shoulderDeg = clampf(s.shoulderDeg, SERVO_MIN_DEG, SERVO_MAX_DEG);
  s.elbowDeg    = clampf(s.elbowDeg,    SERVO_MIN_DEG, SERVO_MAX_DEG);
  s.gripperDeg  = clampf(s.gripperDeg,  SERVO_MIN_DEG, SERVO_MAX_DEG);

  return s;
}