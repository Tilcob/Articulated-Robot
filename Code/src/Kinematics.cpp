#include "Kinematics.h"
#include <Arduino.h>
#include <math.h>

static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

//   x =  cos(q1) * (L1 cos(q2) + L2 cos(q2+q3))
//   y = -sin(q1) * (L1 cos(q2) + L2 cos(q2+q3))
//   z =  h + L1 sin(q2) + L2 sin(q2+q3)
Vec3 forwardKinematics(const Angles& q, float L1, float L2, float h) {
  float r = L1 * cosf(q.q2) + L2 * cosf(q.q2 + q.q3);

  Vec3 p{};
  p.x = r * cosf(q.q1);
  p.y = - r * sinf(q.q1);
  p.z = h + L1 * sinf(q.q2) + L2 * sinf(q.q2 + q.q3);
  return p;
}

IKResult inverseKinematics(const Vec3& p, float L1, float L2, float h, bool elbowUp) {
  IKResult out{};
  out.ok = true;

  // Base: q1
  out.q.q1 = atan2f(-p.y, p.x);

  const float r = sqrtf(p.x * p.x + p.y * p.y);
  const float z = p.z - h;
  float d = sqrtf(r*r + z*z);

  const float dMin = fabsf(L1 - L2);
  const float dMax = L1 + L2;
  if (d < dMin) { d = dMin; out.ok = false; }
  if (d > dMax) { d = dMax; out.ok = false; }

  // cos(q3)
  float c3 = (d*d - L1*L1 - L2*L2) / (2.0f * L1 * L2);
  c3 = clampf(c3, -1.0f, 1.0f);
  // sin(q3)
  float s3 = sqrtf(fmaxf(0.0f, 1.0f - c3*c3));
  if (elbowUp) s3 = -s3;
  out.q.q3 = atan2f(s3, c3);

  // q2
  const float alpha = atan2f(z, r);
  const float beta  = atan2f(L2 * sinf(out.q.q3), L1 + L2 * cosf(out.q.q3));
  out.q.q2 = alpha - beta;

  return out;
}