#include "ServoIO.h"
#include "Config.h"
#include "Util.h"
#include <math.h>

bool arrived(const ServoAngles& cur, const ServoAngles& tgt) {
	constexpr float tol = 1.0f;
	return util::nearf(cur.baseDeg,     tgt.baseDeg,     tol) &&
		   util::nearf(cur.shoulderDeg, tgt.shoulderDeg, tol) &&
		   util::nearf(cur.elbowDeg,    tgt.elbowDeg,    tol) &&
		   util::nearf(cur.gripperDeg,  tgt.gripperDeg,  tol);
}

void attachServos(Servo& base, Servo& shoulder, Servo& elbow, Servo& gripper) {
	base.attach(cfg::PIN_SERVO_BASE);
	shoulder.attach(cfg::PIN_SERVO_SHOULDER);
	elbow.attach(cfg::PIN_SERVO_ELBOW);
	gripper.attach(cfg::PIN_SERVO_GRIPPER);
}

void writeServos(Servo& base, Servo& shoulder, Servo& elbow, Servo& gripper, const ServoAngles& a) {
	base.write(static_cast<int>(lroundf(a.baseDeg)));
	shoulder.write(static_cast<int>(lroundf(a.shoulderDeg)));
	elbow.write(static_cast<int>(lroundf(a.elbowDeg)));
	gripper.write(static_cast<int>(lroundf(a.gripperDeg)));
}