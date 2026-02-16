#include "RobotApp.h"
#include <Arduino.h>
#include <Servo.h>
#include "Config.h"
#include "Input.h"
#include "Kinematics.h"
#include "Mapping.h"
#include "Motion.h"
#include "SerialControl.h"
#include "ServoIO.h"
#include "Trajectory.h"
#include "Types.h"
#include "Workspace.h"

namespace {
	struct Pose {
		Angles q{};
		Vec3 tcpPosBase{};
		Mat3 rBaseTcp{};
		bool valid{false};
	};

	class RobotApp {
	public:
		void setup();
		void loop();

	private:
		float calcDt_();
		void runHoming_(float dt);
		void handleButton_(const InputState &in, unsigned long nowMs);
		void handleSerial_();
		void updateTargetFromTrajectory_(float dt);
		void clampTargetToWorkspace_();
		void solveIK_();
		void updatePoseFromQ_();
		void debugLog_();
		void enterTrajectoryMode_();
		void commitManualTarget_(const InputState &in);
		void commitSerialTarget_(const Vec3 &pos, float grip01);

		Servo servoBase_;
		Servo servoShoulder_;
		Servo servoElbow_;
		Servo servoGripper_;
		MotionController motion_;
		SerialControl serial_;

		unsigned long lastMs_{0};

		Vec3 committedTargetPos_{};
		float committedGrip01_{0.5f};
		bool hasCommit_{false};

		Vec3 targetPos_{};
		float targetGrip01_{0.5f};

		Trajectory traj_;
		bool trajMode_{false};

		bool btnWasDown_{false};
		unsigned long pressStartMs_{0};
		static constexpr unsigned long LONG_PRESS_MS = 700;

		Pose pose_;

		bool homing_{true};
		ServoAngles homeServo_{};

		IKResult ik_{};
		ServoAngles targetServo_{};

		bool debugPrint_{false};
		unsigned long lastDbgMs_{0};
		static constexpr unsigned long DBG_PERIOD_MS = 200;
		bool wsWarned_{false};
		bool ikWarned_{false};
	};

	float RobotApp::calcDt_() {
		const unsigned long now = millis();
		const float dt = (now - lastMs_) / 1000.0f;
		lastMs_ = now;
		return constrain(dt, cfg::LOOP_DT_S, 0.05f);
	}

	void RobotApp::runHoming_(const float dt) {
		const ServoAngles cur = motion_.stepToward(homeServo_, dt, cfg::MAX_SPEED_DEG_PER_S, cfg::MAX_ACC_DEG_PER_S2);
		writeServos(servoBase_, servoShoulder_, servoElbow_, servoGripper_, cur);

		if (!arrived(cur, homeServo_)) return;

		homing_ = false;

		pose_.q = Angles{0, 0, 0};
		pose_.tcpPosBase = forwardKinematics(pose_.q, cfg::L1, cfg::L2, cfg::h);
		pose_.rBaseTcp = tcpRotationBase(pose_.q);
		pose_.valid = true;

		Serial.println("HOMING done.");
	}

	void RobotApp::enterTrajectoryMode_() {
		traj_.reset();
		trajMode_ = true;
		hasCommit_ = true;
		Serial.println("TRAJ start");
	}

	void RobotApp::commitManualTarget_(const InputState &in) {
		trajMode_ = false;
		committedTargetPos_ = in.target;
		committedGrip01_ = in.gripper01;
		hasCommit_ = true;
		Serial.println("COMMIT manual target");
	}

	void RobotApp::commitSerialTarget_(const Vec3 &pos, float grip01) {
		trajMode_ = false;
		committedTargetPos_ = pos;
		committedGrip01_ = grip01;
		hasCommit_ = true;
		Serial.println("COMMIT (serial)");
	}

	void RobotApp::handleButton_(const InputState &in, const unsigned long nowMs) {
		if (in.commitPressed) {
			btnWasDown_ = true;
			pressStartMs_ = nowMs;
		}

		if (!btnWasDown_ || in.commitDown) return;

		const unsigned long pressMs = nowMs - pressStartMs_;
		btnWasDown_ = false;

		if (pressMs >= LONG_PRESS_MS) enterTrajectoryMode_();
		else commitManualTarget_(in);
	}

	void RobotApp::handleSerial_() {
		Vec3 serialPos{};
		float serialGrip = committedGrip01_;
		if (!serial_.poll(debugPrint_, pose_.valid, pose_.tcpPosBase, pose_.rBaseTcp, serialPos, serialGrip)) return;
		commitSerialTarget_(serialPos, serialGrip);
	}

	void RobotApp::updateTargetFromTrajectory_(const float dt) {
		if (!trajMode_) return;

		const Waypoint wp = traj_.sample(dt);
		targetPos_ = wp.targetPos;
		targetGrip01_ = wp.gripper;

		if (!traj_.finished()) return;
		trajMode_ = false;
		Serial.println("TRAJ done");
	}

	void RobotApp::clampTargetToWorkspace_() {
		const WorkspaceResult ws = clampToWorkspace(targetPos_);
		if (!ws.wasOutOfBounds) {
			wsWarned_ = false;
			return;
		}

		if (!wsWarned_) Serial.println("WS clamp");
		wsWarned_ = true;
		targetPos_ = ws.clamped;
	}

	void RobotApp::solveIK_() {
		ik_ = inverseKinematics(targetPos_, cfg::L1, cfg::L2, cfg::h, cfg::ELBOW_UP);
		if (!ik_.ok) {
			if (!ikWarned_) Serial.println("IK WARN: clamped");
			ikWarned_ = true;
		} else {
			ikWarned_ = false;
		}
	}

	void RobotApp::updatePoseFromQ_() {
		pose_.q = ik_.q;
		pose_.tcpPosBase = forwardKinematics(pose_.q, cfg::L1, cfg::L2, cfg::h);
		pose_.rBaseTcp = tcpRotationBase(pose_.q);
		pose_.valid = true;
	}

	void RobotApp::debugLog_() {
		if (!debugPrint_) return;
		const unsigned long nowMs = millis();
		if (nowMs - lastDbgMs_ < DBG_PERIOD_MS) return;
		lastDbgMs_ = nowMs;

		const Vec3 fk = pose_.tcpPosBase;
		Serial.print("DBG err: ");
		Serial.print(targetPos_.x - fk.x, 4);
		Serial.print(" ");
		Serial.print(targetPos_.y - fk.y, 4);
		Serial.print(" ");
		Serial.print(targetPos_.z - fk.z, 4);
		Serial.print(" | q: ");
		Serial.print(ik_.q.q1 * 180 / PI, 1);
		Serial.print(" ");
		Serial.print(ik_.q.q2 * 180 / PI, 1);
		Serial.print(" ");
		Serial.print(ik_.q.q3 * 180 / PI, 1);
		Serial.println();
	}

	void RobotApp::setup() {
		Serial.begin(115200);

		attachServos(servoBase_, servoShoulder_, servoElbow_, servoGripper_);

		Serial.println("CALIB MODE: type: b <deg>, s <deg>, e <deg>, p=print, x=exit");
		ServoAngles cal{};
		cal.baseDeg = 90;
		cal.shoulderDeg = 90;
		cal.elbowDeg = 90;
		cal.gripperDeg = 60;
		writeServos(servoBase_, servoShoulder_, servoElbow_, servoGripper_, cal);

		while (cfg::ENABLE_CALIBRATION_MODE) {
			if (!Serial.available()) continue;
			const char cmd = Serial.read();
			if (cmd == '\n' || cmd == '\r' || cmd == ' ') continue;

			if (cmd == 'x') break;
			if (cmd == 'p') {
				Serial.print("B=");
				Serial.print(cal.baseDeg);
				Serial.print(" S=");
				Serial.print(cal.shoulderDeg);
				Serial.print(" E=");
				Serial.println(cal.elbowDeg);
				continue;
			}

			float v = Serial.parseFloat();
			v = constrain(v, cfg::SERVO_MIN_DEG, cfg::SERVO_MAX_DEG);

			if (cmd == 'b') cal.baseDeg = v;
			if (cmd == 's') cal.shoulderDeg = v;
			if (cmd == 'e') cal.elbowDeg = v;

			writeServos(servoBase_, servoShoulder_, servoElbow_, servoGripper_, cal);
		}

		ServoAngles start{};
		start.baseDeg = cfg::BASE_ZERO_DEG;
		start.shoulderDeg = cfg::SHOULDER_ZERO_DEG;
		start.elbowDeg = cfg::ELBOW_ZERO_DEG;
		start.gripperDeg = 60;

		motion_.setCurrent(start);
		writeServos(servoBase_, servoShoulder_, servoElbow_, servoGripper_, start);

		constexpr Angles homeQ{0, 0, 0};
		homeServo_ = mapToServos(homeQ, 0.5f);
		homing_ = true;
		Serial.println("HOMING: moving to q=(0,0,0)");

		lastMs_ = millis();
		initInputs();

		committedTargetPos_ = Vec3{(cfg::L1 + cfg::L2) * 0.8f, 0, cfg::h};
		committedGrip01_ = 0.5f;
		hasCommit_ = false;
	}

	void RobotApp::loop() {
		const float dt = calcDt_();

		if (homing_) {
			runHoming_(dt);
			return;
		}

		const InputState in = readInputs();
		handleButton_(in, millis());
		handleSerial_();

		if (!hasCommit_) return;

		targetPos_ = committedTargetPos_;
		targetGrip01_ = committedGrip01_;

		updateTargetFromTrajectory_(dt);
		clampTargetToWorkspace_();
		solveIK_();
		updatePoseFromQ_();
		debugLog_();

		targetServo_ = mapToServos(ik_.q, targetGrip01_);
		const ServoAngles cur = motion_.stepToward(targetServo_, dt, cfg::MAX_SPEED_DEG_PER_S, cfg::MAX_ACC_DEG_PER_S2);
		writeServos(servoBase_, servoShoulder_, servoElbow_, servoGripper_, cur);
	}

	RobotApp g_app;
}

void robotSetup() { g_app.setup(); }
void robotLoop() { g_app.loop(); }
