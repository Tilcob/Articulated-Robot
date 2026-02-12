#include <Servo.h>

#include "Config.h"
#include "Types.h"
#include "Input.h"
#include "Kinematics.h"
#include "Mapping.h"
#include "Motion.h"

Servo servoBase, servoShoulder, servoElbow, servoGripper;
MotionController motion;

static unsigned long lastMs = 0;

static void writeServos(const ServoAngles& a) {
  servoBase.write(static_cast<int>(a.baseDeg));
  servoShoulder.write(static_cast<int>(a.shoulderDeg));
  servoElbow.write(static_cast<int>(a.elbowDeg));
  servoGripper.write(static_cast<int>(a.gripperDeg));
}

void setup() {
  Serial.begin(115200);

  servoBase.attach(PIN_SERVO_BASE);
  servoShoulder.attach(PIN_SERVO_SHOULDER);
  servoElbow.attach(PIN_SERVO_ELBOW);
  servoGripper.attach(PIN_SERVO_GRIPPER);

  // Startpose = q=0 -> Servo-Nullwerte (aus Config)
  ServoAngles start{};
  start.baseDeg     = BASE_ZERO_DEG;
  start.shoulderDeg = SHOULDER_ZERO_DEG;
  start.elbowDeg    = ELBOW_ZERO_DEG;
  start.gripperDeg  = 60.0f;

  motion.setCurrent(start);
  writeServos(start);

  lastMs = millis();
}

void loop() {
  const unsigned long now = millis();
  const float dt = (now - lastMs) / 1000.0f;
  if (dt < LOOP_DT_S) return; // 20ms Takt
  lastMs = now;

  // 1) Inputs
  const InputState in = readInputs();

  // 2) IK
  const IKResult ik = inverseKinematics(in.target, L1, L2, h, Y_IS_NEGATIVE, ELBOW_UP);

  // 3) Map -> Servo
  const ServoAngles targetServos = mapToServos(ik.q, in.gripper01);

  // 4) Motion smoothing
  const ServoAngles cur = motion.stepToward(targetServos, dt, MAX_SPEED_DEG_PER_S);

  // 5) Output
  writeServos(cur);

  // Debug (sparsam)
  static int n=0;
  if (++n >= 20) { // ~ alle 0.4s
    n=0;
    Serial.print("ok="); Serial.print(ik.ok ? "1":"0");
    Serial.print("  x="); Serial.print(in.target.x, 3);
    Serial.print(" y="); Serial.print(in.target.y, 3);
    Serial.print(" z="); Serial.print(in.target.z, 3);
    Serial.print(" | q(deg)=");
    Serial.print(ik.q.q1 * 180.0f/PI, 1); Serial.print(",");
    Serial.print(ik.q.q2 * 180.0f/PI, 1); Serial.print(",");
    Serial.print(ik.q.q3 * 180.0f/PI, 1);
    Serial.println();
  }
}