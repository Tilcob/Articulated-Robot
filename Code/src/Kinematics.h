#pragma once
#include "Types.h"

struct IKResult {
  Angles q;
  bool ok;
};

IKResult inverseKinematics(const Vec3& position, float L1, float L2, float h, bool elbowUp);

Vec3 forwardKinematics(const Angles& q, float L1, float L2, float h);

Mat3 tcpRotationBase(const Angles& q);