#include "ServoIO.h"
#include "Config.h"
#include <math.h>

static bool nearf(const float a, const float b, const float eps) {
	return fabsf(a - b) <= eps;
}

bool arrived(const ServoAngles& cur, const ServoAngles& tgt) {
	constexpr float tol = 1.0f;
	return nearf(cur.baseDeg,     tgt.baseDeg,     tol) &&
		   nearf(cur.shoulderDeg, tgt.shoulderDeg, tol) &&
		   nearf(cur.elbowDeg,    tgt.elbowDeg,    tol) &&
		   nearf(cur.gripperDeg,  tgt.gripperDeg,  tol);
}

void attachServos(Servo& base, Servo& shoulder, Servo& elbow, Servo& gripper) {
	base.attach(PIN_SERVO_BASE);
	shoulder.attach(PIN_SERVO_SHOULDER);
	elbow.attach(PIN_SERVO_ELBOW);
	gripper.attach(PIN_SERVO_GRIPPER);
}

void writeServos(Servo& base, Servo& shoulder, Servo& elbow, Servo& gripper, const ServoAngles& a) {
	base.write(static_cast<int>(lroundf(a.baseDeg)));
	shoulder.write(static_cast<int>(lroundf(a.shoulderDeg)));
	elbow.write(static_cast<int>(lroundf(a.elbowDeg)));
	gripper.write(static_cast<int>(lroundf(a.gripperDeg)));
}