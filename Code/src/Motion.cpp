#include "Motion.h"
#include <math.h>

static float moveToward(float cur, float tgt, float maxStep) {
  float d = tgt - cur;
  if (fabsf(d) <= maxStep) return tgt;
  return cur + (d > 0 ? maxStep : -maxStep);
}

void MotionController::setCurrent(const ServoAngles& a) {
  cur = a;
}

ServoAngles MotionController::stepToward(const ServoAngles& target, float dt, float maxDegPerS) {
  float maxStep = maxDegPerS * dt;

  cur.baseDeg     = moveToward(cur.baseDeg,     target.baseDeg,     maxStep);
  cur.shoulderDeg = moveToward(cur.shoulderDeg, target.shoulderDeg, maxStep);
  cur.elbowDeg    = moveToward(cur.elbowDeg,    target.elbowDeg,    maxStep);
  cur.gripperDeg  = moveToward(cur.gripperDeg,  target.gripperDeg,  maxStep);

  return cur;
}