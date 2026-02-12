#pragma once
#include "Types.h"

struct InputState {
  Vec3 target;
  float gripper01;
};

InputState readInputs();