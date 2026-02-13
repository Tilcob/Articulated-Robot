#pragma once
#include "Types.h"

struct InputState {
  Vec3 target;
  float gripper01;
  bool commitPressed;
};

void initInputs();
InputState readInputs();