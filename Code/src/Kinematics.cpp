#include "Kinematics.h"
#include <Arduino.h>
#include <math.h>

static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

Vec3 forwardKinematics(const Angles& q, float L1, float L2, float h, bool yIsNegative) {
  // r = L1 cos q2 + L2 cos(q2+q3)
  float r = L1 * cosf(q.q2) + L2 * cosf(q.q2 + q.q3);

  Vec3 p{};
  p.x = r * cosf(q.q1);
  float y = r * sinf(q.q1);
  p.y = yIsNegative ? -y : y;

  p.z = h + L1 * sinf(q.q2) + L2 * sinf(q.q2 + q.q3);
  return p;
}

IKResult inverseKinematics(const Vec3& p, float L1, float L2, float h,
                           bool yIsNegative, bool elbowUp) {
  IKResult out{};
  out.ok = true;

  // q1: base
  float py = yIsNegative ? -p.y : p.y; // invertiere zurück in "Standardform"
  out.q.q1 = atan2f(py, p.x);

  float r = sqrtf(p.x*p.x + py*py);
  float z = p.z - h;
  float d = sqrtf(r*r + z*z);

  // Erreichbarkeit
  float dMin = fabsf(L1 - L2);
  float dMax = (L1 + L2);

  // Wenn außerhalb: projizieren (statt hart failen)
  if (d < dMin) { d = dMin; out.ok = false; }
  if (d > dMax) { d = dMax; out.ok = false; }

  // cos(q3)
  float c3 = (d*d - L1*L1 - L2*L2) / (2.0f * L1 * L2);
  c3 = clampf(c3, -1.0f, 1.0f);

  float q3abs = acosf(c3);
  out.q.q3 = elbowUp ? -q3abs : +q3abs; // wähle Konfiguration

  // q2
  float alpha = atan2f(z, r);
  float beta  = atan2f(L2 * sinf(out.q.q3), L1 + L2 * cosf(out.q.q3));
  out.q.q2 = alpha - beta;

  return out;
}