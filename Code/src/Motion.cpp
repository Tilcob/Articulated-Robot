#include "Motion.h"
#include <math.h>

static float clampf(const float x, const float lo, const float hi) {
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

void MotionController::setCurrent(const ServoAngles& angles) {
  cur = angles;
}

ServoAngles MotionController::stepToward(const ServoAngles& target,
  const float dtSeconds, const float maxDegPerS, const float maxAccDegPerS2) {

  auto step = [&](float& pos, float& v, const float& tgt) {
    const float velDestination = clampf((tgt - pos) / dtSeconds, -maxDegPerS, +maxDegPerS);

    const float dvMax = maxAccDegPerS2 * dtSeconds;
    float dv = velDestination - v;
    if (fabsf(dv) > dvMax) dv = dv > 0 ? dvMax : -dvMax;
    v += dv;
    pos += v * dtSeconds;
    if (fabsf(tgt - pos) < 1.0f) {
      pos = tgt;
      v = 0.0f;
    }
  };

  step(cur.baseDeg,     vel.baseDeg,     target.baseDeg);
  step(cur.shoulderDeg, vel.shoulderDeg, target.shoulderDeg);
  step(cur.elbowDeg,    vel.elbowDeg,    target.elbowDeg);
  step(cur.gripperDeg,  vel.gripperDeg,  target.gripperDeg);

  return cur;
}