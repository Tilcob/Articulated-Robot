#pragma once
#include <Servo.h>
#include "Mapping.h"

void attachServos(Servo& base, Servo& shoulder, Servo& elbow, Servo& gripper);
void writeServos(Servo& base, Servo& shoulder, Servo& elbow, Servo& gripper, const ServoAngles& a);
bool arrived(const ServoAngles& cur, const ServoAngles& tgt);