#pragma once
#include "Mapping.h"

class MotionController {
public:
  void setCurrent(const ServoAngles& angles);
  ServoAngles stepToward(const ServoAngles& target, float dtSeconds, float maxDegPerS);

private:
  ServoAngles cur{};
};