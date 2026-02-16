#pragma once
#include "Mapping.h"

class MotionController {
public:
  void setCurrent(const ServoAngles& angles);
  ServoAngles stepToward(const ServoAngles& target, float dtSeconds, float maxDegPerS, float maxAccDegPerS2);


private:
  ServoAngles cur{};
  ServoAngles vel{};
};