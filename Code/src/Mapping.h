#pragma once
#include "Types.h"

struct ServoAngles {
  float baseDeg;
  float shoulderDeg;
  float elbowDeg;
  float gripperDeg;
};

ServoAngles mapToServos(const Angles& q, float gripper01);