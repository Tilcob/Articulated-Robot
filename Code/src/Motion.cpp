#include "Motion.h"
#include <math.h>

static float moveToward(const float cur, const float tgt, const float maxStep) {
  const float d = tgt - cur;
  if (fabsf(d) <= maxStep) return tgt;
  return cur + (d > 0 ? maxStep : -maxStep);
}

void MotionController::setCurrent(const ServoAngles& a) {
  cur = a;
}

ServoAngles MotionController::stepToward(const ServoAngles& target,
  const float dtSeconds, const float maxDegPerS) {

  const float maxStep = maxDegPerS * dtSeconds;

  cur.baseDeg     = moveToward(cur.baseDeg,     target.baseDeg,     maxStep);
  cur.shoulderDeg = moveToward(cur.shoulderDeg, target.shoulderDeg, maxStep);
  cur.elbowDeg    = moveToward(cur.elbowDeg,    target.elbowDeg,    maxStep);
  cur.gripperDeg  = moveToward(cur.gripperDeg,  target.gripperDeg,  maxStep);

  return cur;
}