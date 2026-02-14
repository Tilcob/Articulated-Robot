#include <Servo.h>
#include <ctype.h>

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
static constexpr unsigned long LONG_PRESS_MS = 700;

static Angles lastCommandQ{};
static Vec3   lastTcpPosBase{};
static Mat3   lastRBaseTcp{};
static bool   havePose = false;

// ---- Homing ------------------------------------------------------------
static bool homing = true;
static ServoAngles homeServo{};

// ---- Debug -------------------------------------------------------------
static bool debugPrint = false;
static unsigned long lastDbgMs = 0;
static constexpr unsigned long DBG_PERIOD_MS = 200;

// ------------------------------------------------------------------------

static bool nearf(float a, float b, float eps) {
  return fabsf(a - b) <= eps;
}

static bool arrived(const ServoAngles& cur, const ServoAngles& tgt) {
  constexpr float tol = 1.0f;
  return nearf(cur.baseDeg, tgt.baseDeg, tol) &&
         nearf(cur.shoulderDeg, tgt.shoulderDeg, tol) &&
         nearf(cur.elbowDeg, tgt.elbowDeg, tol) &&
         nearf(cur.gripperDeg, tgt.gripperDeg, tol);
}

static void writeServos(const ServoAngles& a) {
  servoBase.write((int)a.baseDeg);
  servoShoulder.write((int)a.shoulderDeg);
  servoElbow.write((int)a.elbowDeg);
  servoGripper.write((int)a.gripperDeg);
}

// ------------------------------------------------------------------------
// SERIAL INPUT
// ------------------------------------------------------------------------

static bool readSerialTcp(Vec3& outPos, float& outGrip01) {
  static char buf[80];
  static uint8_t len = 0;

  while (Serial.available()) {
    const char c = Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      buf[len] = '\0';
      len = 0;

      char* p = buf;
      while (*p && isspace((unsigned char)*p)) p++;

      // ---- Commands ----------------------------------------------------
      if (*p == 'h' || *p == 'H') {
        Serial.println("Commands:");
        Serial.println("  x y z [g]      -> absolute Base-KS");
        Serial.println("  t dx dy dz [g] -> relative TCP-KS");
        Serial.println("  d              -> toggle debug");
        return false;
      }

      if (*p == 'd' || *p == 'D') {
        debugPrint = !debugPrint;
        Serial.print("DEBUG: ");
        Serial.println(debugPrint ? "ON" : "OFF");
        return false;
      }

      bool tcpRelative = false;
      if (*p == 't' || *p == 'T') {
        tcpRelative = true;
        p++;
        while (*p && isspace((unsigned char)*p)) p++;
      }

      char* end;
      double xd = strtod(p, &end);
      if (end == p) return false;
      p = end;

      double yd = strtod(p, &end);
      if (end == p) return false;
      p = end;

      double zd = strtod(p, &end);
      if (end == p) return false;
      p = end;

      double gd = outGrip01;
      if (*p != '\0')
        gd = strtod(p, &end);

      Vec3 parsed{(float)xd,(float)yd,(float)zd};
      float g = constrain((float)gd,0.0f,1.0f);
      outGrip01 = g;

      if (tcpRelative) {
        if (!havePose) {
          Serial.println("ERR: no pose yet");
          return false;
        }
        Vec3 d_base = mul(lastRBaseTcp, parsed);
        outPos = add(lastTcpPosBase, d_base);
      } else {
        outPos = parsed;
      }

      return true;
    }

    if (len < sizeof(buf)-1)
      buf[len++] = c;
    else
      len = 0;
  }
  return false;
}

// ------------------------------------------------------------------------

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
  start.gripperDeg  = 60;

  motion.setCurrent(start);
  writeServos(start);

  constexpr Angles homeQ{0,0,0};
  homeServo = mapToServos(homeQ, 0.5f);
  homing = true;

  Serial.println("HOMING: moving to q=(0,0,0)");

  lastMs = millis();
  initInputs();

  // FIX 1: keinen Default commit außerhalb des Workspace setzen
  committedTargetPosition = Vec3{(L1 + L2) * 0.8f, 0, h}; // sicher innerhalb R_MAX
  committedGripper = 0.5f;
  hasCommitted = false; // erst fahren nach Button/Serial
}

// ------------------------------------------------------------------------

void loop() {

  unsigned long now = millis();
  float dt = (now - lastMs)/1000.0f;
  if (dt < LOOP_DT_S) return;
  lastMs = now;

  // ---- Homing ----------------------------------------------------------
  if (homing) {
    ServoAngles cur = motion.stepToward(homeServo, dt, MAX_SPEED_DEG_PER_S);
    writeServos(cur);

    if (arrived(cur, homeServo)) {
      homing = false;

      lastCommandQ = Angles{0,0,0};
      lastTcpPosBase = forwardKinematics(lastCommandQ, L1, L2, h);
      lastRBaseTcp = tcpRotationBase(lastCommandQ);
      havePose = true;

      Serial.println("HOMING done.");
    }
    return;
  }

  InputState inputState = readInputs();

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

  // FIX 2: Button-Commit wirklich verwenden
  if (inputState.commitPressed) {
    trajMode = false;
    committedTargetPosition = inputState.target;
    committedGripper = inputState.gripper01;
    hasCommitted = true;
    Serial.println("COMMIT (potis)");
  }

  Vec3 serialPos{};
  float serialGrip = committedGripper;

  if (readSerialTcp(serialPos, serialGrip)) {
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

  IKResult result = inverseKinematics(target, L1, L2, h, ELBOW_UP);
  if (!result.ok) {
    Serial.println("IK ERR");
    return;
  }

  lastCommandQ = result.q;
  lastTcpPosBase = forwardKinematics(result.q, L1, L2, h);
  lastRBaseTcp = tcpRotationBase(result.q);
  havePose = true;

  if (debugPrint) {
    unsigned long nowMs = millis();
    if (nowMs - lastDbgMs >= DBG_PERIOD_MS) {
      lastDbgMs = nowMs;

      Vec3 fk = lastTcpPosBase;
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

  ServoAngles targetAngles = mapToServos(result.q, targetGrip);
  ServoAngles cur = motion.stepToward(targetAngles, dt, MAX_SPEED_DEG_PER_S);
  writeServos(cur);

  if (trajMode && traj.advanceIfArrived(arrived(cur, targetAngles)) && traj.finished()) {
    trajMode = false;
    Serial.println("TRAJ done");
  }
}
