#pragma once
#include "Types.h"

struct InputState {
  Vec3 target;
  float gripper01;
  bool commitPressed;
  bool commitDown;
};

void initInputs();
InputState readInputs();