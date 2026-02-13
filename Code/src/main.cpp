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

static Vec3 committedTargetPosition{};
static float committedGripper = 0.0f;
static bool hasCommitted = false;

static Trajectory traj;
static bool trajMode = false;

static bool btnWasDown = false;
static unsigned long pressStartMs = 0;

static bool nearf(float const a, float const b, float const eps) {
  return fabsf(a - b) <= eps;
}

static bool arrived(const ServoAngles& cur, const ServoAngles& tgt) {
  constexpr float degTolerance = 1.0f; // degrees tolerance
  return nearf(cur.baseDeg, tgt.baseDeg, degTolerance) &&
         nearf(cur.shoulderDeg, tgt.shoulderDeg, degTolerance) &&
         nearf(cur.elbowDeg, tgt.elbowDeg, degTolerance) &&
         nearf(cur.gripperDeg, tgt.gripperDeg, degTolerance);
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

  ServoAngles start{};
  start.baseDeg     = BASE_ZERO_DEG;
  start.shoulderDeg = SHOULDER_ZERO_DEG;
  start.elbowDeg    = ELBOW_ZERO_DEG;
  start.gripperDeg  = 60.0f;

  motion.setCurrent(start);
  writeServos(start);

  lastMs = millis();
  initInputs();

  committedTargetPosition = Vec3{L1 + L2, 0.0f, h};
  committedGripper = 0.5f;
  hasCommitted = true;
}

void loop() {
  const unsigned long now = millis();
  const float dt = (now - lastMs) / 1000.0f;
  if (dt < LOOP_DT_S) return;
  lastMs = now;


  const InputState inputState = readInputs();

  // Long press (>700ms) -> Trajectory mode (play waypoints)
  if (inputState.commitDown && !btnWasDown) {
    btnWasDown = true;
    pressStartMs = now;
  }
  if (!inputState.commitDown && btnWasDown) {
    btnWasDown = false;
    const unsigned long heldMs = now - pressStartMs;

    if (heldMs > 700) {
      trajMode = true;
      traj.reset();
      Serial.println("TRAJ: start");
    } else {
      // Short press -> latch potentiometer target
      trajMode = false;
      committedTargetPosition = inputState.target;
      committedGripper = inputState.gripper01;
      hasCommitted = true;

      Serial.print("COMMIT x="); Serial.print(committedTargetPosition.x, 3);
      Serial.print(" y=");       Serial.print(committedTargetPosition.y, 3);
      Serial.print(" z=");       Serial.print(committedTargetPosition.z, 3);
      Serial.println();
    }
  }

  Vec3 target{};
  float targetGripper = 0.5f;

  if (trajMode) {
    if (traj.finished()) {
      trajMode = false;
      Serial.println("TRAJ: finished");
      return;
    }
    const Waypoint& wp = traj.current();
    target = wp.targetPos;
    targetGripper = wp.gripper;
  } else {
    if (!hasCommitted) return;
    target = committedTargetPosition;
    targetGripper = committedGripper;
  }

  const IKResult result = inverseKinematics(target, L1, L2, h, ELBOW_UP);
  const ServoAngles targetAngles = mapToServos(result.q, targetGripper);
  const ServoAngles cur = motion.stepToward(targetAngles, dt, MAX_SPEED_DEG_PER_S);

  writeServos(cur);

  if (trajMode && arrived(cur, targetAngles)) {
    traj.advanceIfArrived(true);
  }
}