#pragma once
#include "Types.h"

struct IKResult {
  Angles q;
  bool ok;
};

IKResult inverseKinematics(const Vec3& p, float L1, float L2, float h,
                           bool yIsNegative, bool elbowUp);

Vec3 forwardKinematics(const Angles& q, float L1, float L2, float h,
                       bool yIsNegative);