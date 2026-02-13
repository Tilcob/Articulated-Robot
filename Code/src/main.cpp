#include <Servo.h>

#include "Config.h"
#include "Types.h"
#include "Input.h"
#include "Kinematics.h"
#include "Mapping.h"
#include "Motion.h"
#include "Trajectory.h"

Servo servoBase, servoShoulder, servoElbow, servoGripper;
MotionController motion;

static unsigned long lastMs = 0;

static Vec3 goalXYZ{};
static float goalGripper01 = 0.0f;
static bool hasGoal = false;

static Trajectory traj;
static bool trajMode = false;

static bool btnWasDown = false;
static unsigned long pressStartMs = 0;

static bool nearf(float const a, float const b, float const eps) {
  return fabsf(a - b) <= eps;
}

static bool arrived(const ServoAngles& cur, const ServoAngles& tgt) {
  constexpr float eps = 1.0f; // degrees tolerance
  return nearf(cur.baseDeg, tgt.baseDeg, eps) &&
         nearf(cur.shoulderDeg, tgt.shoulderDeg, eps) &&
         nearf(cur.elbowDeg, tgt.elbowDeg, eps) &&
         nearf(cur.gripperDeg, tgt.gripperDeg, eps);
}

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
  initInputs();

  goalXYZ = Vec3{L1 + L2, 0.0f, h};
  goalGripper01 = 0.5f;
  hasGoal = true;
}

void loop() {
  const unsigned long now = millis();
  const float dt = (now - lastMs) / 1000.0f;
  if (dt < LOOP_DT_S) return; // 20ms Takt
  lastMs = now;

  // 1) Inputs
  const InputState in = readInputs();

  // Long press (>700ms) -> Trajectory mode (play waypoints)
  if (in.commitDown && !btnWasDown) {
    btnWasDown = true;
    pressStartMs = now;
  }
  if (!in.commitDown && btnWasDown) {
    btnWasDown = false;
    const unsigned long heldMs = now - pressStartMs;

    if (heldMs > 700) {
      trajMode = true;
      traj.reset();
      Serial.println("TRAJ: start");
    } else {
      // Short press -> latch pot target
      trajMode = false;
      goalXYZ = in.target;
      goalGripper01 = in.gripper01;
      hasGoal = true;

      Serial.print("COMMIT x="); Serial.print(goalXYZ.x, 3);
      Serial.print(" y=");       Serial.print(goalXYZ.y, 3);
      Serial.print(" z=");       Serial.print(goalXYZ.z, 3);
      Serial.println();
    }
  }

  // Choose an active target
  Vec3 targetXYZ{};
  float targetGrip01 = 0.5f;

  if (trajMode) {
    if (traj.finished()) {
      trajMode = false;
      Serial.println("TRAJ: finished");
      return;
    }
    const Waypoint& wp = traj.current();
    targetXYZ = wp.p;
    targetGrip01 = wp.gripper01;
  } else {
    if (!hasGoal) return;
    targetXYZ = goalXYZ;
    targetGrip01 = goalGripper01;
  }

  // IK (use active target)
  const IKResult ik = inverseKinematics(targetXYZ, L1, L2, h, ELBOW_UP);

  // Map -> Servo
  const ServoAngles targetServos = mapToServos(ik.q, targetGrip01);

  // Motion smoothing
  const ServoAngles cur = motion.stepToward(targetServos, dt, MAX_SPEED_DEG_PER_S);

  writeServos(cur);

  // Advance trajectory when arrived at the current waypoint
  if (trajMode && arrived(cur, targetServos)) {
    traj.advanceIfArrived(true);
  }
}