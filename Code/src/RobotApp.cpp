#include "RobotApp.h"
#include <Servo.h>

#include "Config.h"
#include "Types.h"
#include "Input.h"
#include "Kinematics.h"
#include "Mapping.h"
#include "Motion.h"
#include "Trajectory.h"

#include "ServoIO.h"
#include "SerialControl.h"

static Servo servoBase, servoShoulder, servoElbow, servoGripper;
static MotionController motion;

static unsigned long lastMs = 0;

static Vec3 committedTargetPosition{};
static float committedGripper = 0.0f;
static bool hasCommitted = false;

static Trajectory traj;
static bool trajMode = false;

static bool btnWasDown = false;
static unsigned long pressStartMs = 0;
static constexpr unsigned long LONG_PRESS_MS = 700;

static Angles lastCommandQ{};
static Vec3   lastTcpPosBase{};
static Mat3   lastRBaseTcp{};
static bool   havePose = false;

// Homing
static bool homingFlag = true;
static ServoAngles homeServo{};

// Debug
static bool debugPrint = false;
static unsigned long lastDbgMs = 0;
static constexpr unsigned long DBG_PERIOD_MS = 200;

static float calDeltaTime() {
  const unsigned long now = millis();
  const float dt = (now - lastMs)/1000.0f;
  lastMs = now;
  return constrain(dt, LOOP_DT_S, 0.05f);
}

static void homing(const float dt) {
  const ServoAngles cur = motion.stepToward(homeServo, dt, MAX_SPEED_DEG_PER_S, MAX_ACC_DEG_PER_S2);
  writeServos(servoBase, servoShoulder, servoElbow, servoGripper, cur);

  if (arrived(cur, homeServo)) {
    homingFlag = false;

    lastCommandQ = Angles{0,0,0};
    lastTcpPosBase = forwardKinematics(lastCommandQ, L1, L2, h);
    lastRBaseTcp = tcpRotationBase(lastCommandQ);
    havePose = true;

    Serial.println("HOMING done.");
  }
}

static void handleButtonPress(const InputState &inputState, const float now = millis()) {
  if (inputState.commitPressed) {
    btnWasDown = true;
    pressStartMs = now;
  }
  if (btnWasDown && !inputState.commitDown) {
    const unsigned long pressMs = now - pressStartMs;
    btnWasDown = false;

    if (pressMs >= LONG_PRESS_MS) {
      traj.reset();
      trajMode = true;
      hasCommitted = true;
      Serial.println("TRAJ start");
    } else {
      trajMode = false;
      committedTargetPosition = inputState.target;
      committedGripper = inputState.gripper01;
      hasCommitted = true;
      Serial.println("COMMIT manual target");
    }
  }
}

void robotSetup() {
  Serial.begin(115200);

  attachServos(servoBase, servoShoulder, servoElbow, servoGripper);

  ServoAngles start{};
  start.baseDeg     = BASE_ZERO_DEG;
  start.shoulderDeg = SHOULDER_ZERO_DEG;
  start.elbowDeg    = ELBOW_ZERO_DEG;
  start.gripperDeg  = 60;

  motion.setCurrent(start);
  writeServos(servoBase, servoShoulder, servoElbow, servoGripper, start);

  constexpr Angles homeQ{0,0,0};
  homeServo = mapToServos(homeQ, 0.5f);
  homingFlag = true;
  Serial.println("HOMING: moving to q=(0,0,0)");

  lastMs = millis();
  initInputs();

  committedTargetPosition = Vec3{(L1 + L2) * 0.8f, 0, h};
  committedGripper = 0.5f;
  hasCommitted = false;
}

void log_debug_information(const Vec3 &target, const IKResult &result) {
  const unsigned long nowMs = millis();
  if (nowMs - lastDbgMs >= DBG_PERIOD_MS) {
    lastDbgMs = nowMs;
    const Vec3 fk = lastTcpPosBase;
    Serial.print("DBG err: ");
    Serial.print(target.x - fk.x,4); Serial.print(" ");
    Serial.print(target.y - fk.y,4); Serial.print(" ");
    Serial.print(target.z - fk.z,4);
    Serial.print(" | q: ");
    Serial.print(result.q.q1*180/PI,1); Serial.print(" ");
    Serial.print(result.q.q2*180/PI,1); Serial.print(" ");
    Serial.print(result.q.q3*180/PI,1);
    Serial.println();
  }
}

void robotLoop() {
  const float dt = calDeltaTime();

  if (homingFlag) {
    homing(dt);
    return;
  }

  const InputState inputState = readInputs();

  // Long/Short press handling
  handleButtonPress(inputState);

  // Serial commit
  Vec3 serialPos{};
  float serialGrip = committedGripper;
  if (readSerialTcpCommand(debugPrint, havePose, lastTcpPosBase, lastRBaseTcp, serialPos, serialGrip)) {
    trajMode = false;
    committedTargetPosition = serialPos;
    committedGripper = serialGrip;
    hasCommitted = true;
    Serial.println("COMMIT (serial)");
  }

  if (!hasCommitted) return;

  Vec3 target = committedTargetPosition;
  float targetGrip = committedGripper;

  if (trajMode) {
    if (traj.finished()) {
      trajMode = false;
      Serial.println("TRAJ done");
    } else {
      const Waypoint& wp = traj.current();
      target = wp.targetPos;
      targetGrip = wp.gripper;
    }
  }

  static bool wsWarned = false;
  const float r = sqrtf(target.x*target.x + target.y*target.y);
  if (r < R_MIN || r > R_MAX || target.z < Z_MIN || target.z > Z_MAX) {
    if (!wsWarned) Serial.println("WS ERR");
    wsWarned = true;
    return;
  }
  wsWarned = false;

  const IKResult result = inverseKinematics(target, L1, L2, h, ELBOW_UP);
  if (!result.ok) Serial.println("IK WARN: clamped");

  lastCommandQ = result.q;
  lastTcpPosBase = forwardKinematics(result.q, L1, L2, h);
  lastRBaseTcp = tcpRotationBase(result.q);
  havePose = true;

  if (debugPrint) {
    log_debug_information(target, result);
  }

  const ServoAngles targetAngles = mapToServos(result.q, targetGrip);
  const ServoAngles cur = motion.stepToward(targetAngles, dt, MAX_SPEED_DEG_PER_S, MAX_ACC_DEG_PER_S2);
  writeServos(servoBase, servoShoulder, servoElbow, servoGripper, cur);

  if (trajMode && traj.advanceIfArrived(arrived(cur, targetAngles)) && traj.finished()) {
    trajMode = false;
    Serial.println("TRAJ done");
  }
}
