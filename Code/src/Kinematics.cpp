#include "Kinematics.h"
#include "Util.h"
#include <math.h>

Vec3 forwardKinematics(const Angles& q, float const L1, float const L2, float const h) {
  float const r = L1 * cosf(q.q2) + L2 * cosf(q.q2 + q.q3);

  Vec3 p{};
  p.x = r * cosf(q.q1);
  p.y = - r * sinf(q.q1);
  p.z = h + L1 * sinf(q.q2) + L2 * sinf(q.q2 + q.q3);
  return p;
}

IKResult inverseKinematics(const Vec3& position, float const L1, float const L2,
  float const h, bool const elbowUp) {

  IKResult out{};
  out.ok = true;

  const float r = sqrtf(position.x * position.x + position.y * position.y);
  if (r < 1e-6f) out.q.q1 = 0.0f;
  else           out.q.q1 = atan2f(-position.y, position.x);

  const float z = position.z - h;
  float d = sqrtf(r*r + z*z);

  const float dMin = fabsf(L1 - L2);
  const float dMax = L1 + L2;
  constexpr float EPS = 1e-4f;

  if (d < dMin - EPS) { d = dMin; out.ok = false; }
  else if (d < dMin) d = dMin;

  if (d > dMax + EPS) { d = dMax; out.ok = false; }
  else if (d > dMax) d = dMax;

  float c3 = (d*d - L1*L1 - L2*L2) / (2.0f * L1 * L2);
  if (c3 < -1.0f || c3 > 1.0f) out.ok = false;
  c3 = util::clampf(c3, -1.0f, 1.0f);

  float s3 = sqrtf(fmaxf(0.0f, 1.0f - c3*c3));
  if (elbowUp) s3 = -s3;
  out.q.q3 = atan2f(s3, c3);

  const float alpha = atan2f(z, r);
  const float beta  = atan2f(L2 * sinf(out.q.q3), L1 + L2 * cosf(out.q.q3));
  out.q.q2 = alpha - beta;

  return out;
}

Mat3 tcpRotationBase(const Angles& q) {
  const float c1 = cosf(q.q1), s1 = sinf(q.q1);
  const float phi = q.q2 + q.q3;
  const float c = cosf(phi), s = sinf(phi);

  const Vec3 ex_r = { c1, -s1, 0.0f };
  const Vec3 ey   = { s1,  c1, 0.0f };
  constexpr Vec3 ez   = { 0.0f,0.0f,1.0f };

  const Vec3 x_tcp = { c*ex_r.x + s*ez.x, c*ex_r.y + s*ez.y, c*ex_r.z + s*ez.z };
  const Vec3 y_tcp = ey;
  const Vec3 z_tcp = math3::cross(x_tcp, y_tcp);

  Mat3 R{};
  R.m[0][0] = x_tcp.x; R.m[0][1] = y_tcp.x; R.m[0][2] = z_tcp.x;
  R.m[1][0] = x_tcp.y; R.m[1][1] = y_tcp.y; R.m[1][2] = z_tcp.y;
  R.m[2][0] = x_tcp.z; R.m[2][1] = y_tcp.z; R.m[2][2] = z_tcp.z;
  return R;
}