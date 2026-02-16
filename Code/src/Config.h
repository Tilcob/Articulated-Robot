#pragma once
#include <Arduino.h>

static constexpr float DEG2RAD = PI / 180.0f;

static constexpr float qLimitFromServoDeg(const float servoDeg, const float zero, const int sign) {
	// servoDeg = zeroDeg + sign * jointDeg  => jointDeg = (servoDeg - zeroDeg)/sign
	return ((servoDeg - zero) / static_cast<float>(sign)) * DEG2RAD;
}

static constexpr float L1 = 0.080f;  // [m]
static constexpr float L2 = 0.068f;  // [m]
static constexpr float h  = 0.027f;  // [m]

// Workspace limits
static constexpr float R_MIN = 0.0005f;
static constexpr float R_MAX = L1 + L2;
static constexpr float Z_MIN = h - 0.02f;
static constexpr float Z_MAX = h + L1 + L2;

static constexpr bool ELBOW_UP = false;

// Momentary button (Taster). Wiring: pin -> button -> GND (uses INPUT_PULLUP).
static constexpr uint8_t PIN_BTN_COMMIT = 2;


static constexpr uint8_t PIN_POT_X = A0;
static constexpr uint8_t PIN_POT_Y = A1;
static constexpr uint8_t PIN_POT_Z = A2;
static constexpr uint8_t PIN_POT_G = A3;

// ADC
static constexpr int ADC_MIN = 0;
static constexpr int ADC_MAX = 1023;

static constexpr uint8_t PIN_SERVO_BASE     = 3;
static constexpr uint8_t PIN_SERVO_SHOULDER = 5;
static constexpr uint8_t PIN_SERVO_ELBOW    = 6;
static constexpr uint8_t PIN_SERVO_GRIPPER  = 9;

static constexpr float SERVO_MIN_DEG = 0.0f;
static constexpr float SERVO_MAX_DEG = 180.0f;

static constexpr float BASE_ZERO_DEG     = 0.0f;
static constexpr float SHOULDER_ZERO_DEG = 0.0f;
static constexpr float ELBOW_ZERO_DEG    = 0.0f;

static constexpr int BASE_SIGN     = +1;
static constexpr int SHOULDER_SIGN = +1;
static constexpr int ELBOW_SIGN    = +1;

static constexpr float Q1_A = qLimitFromServoDeg(SERVO_MIN_DEG, BASE_ZERO_DEG, BASE_SIGN);
static constexpr float Q1_B = qLimitFromServoDeg(SERVO_MAX_DEG, BASE_ZERO_DEG, BASE_SIGN);
static constexpr float Q1_MIN = (Q1_A < Q1_B) ? Q1_A : Q1_B;
static constexpr float Q1_MAX = (Q1_A < Q1_B) ? Q1_B : Q1_A;

static constexpr float Q2_A = qLimitFromServoDeg(SERVO_MIN_DEG, SHOULDER_ZERO_DEG, SHOULDER_SIGN);
static constexpr float Q2_B = qLimitFromServoDeg(SERVO_MAX_DEG, SHOULDER_ZERO_DEG, SHOULDER_SIGN);
static constexpr float Q2_MIN = (Q2_A < Q2_B) ? Q2_A : Q2_B;
static constexpr float Q2_MAX = (Q2_A < Q2_B) ? Q2_B : Q2_A;

static constexpr float Q3_A = qLimitFromServoDeg(SERVO_MIN_DEG, ELBOW_ZERO_DEG, ELBOW_SIGN);
static constexpr float Q3_B = qLimitFromServoDeg(SERVO_MAX_DEG, ELBOW_ZERO_DEG, ELBOW_SIGN);
static constexpr float Q3_MIN = (Q3_A < Q3_B) ? Q3_A : Q3_B;
static constexpr float Q3_MAX = (Q3_A < Q3_B) ? Q3_B : Q3_A;

static constexpr float MAX_SPEED_DEG_PER_S = 60.0f;  // max speed
static constexpr float LOOP_DT_S = 0.02f;            // 20ms
static constexpr float MAX_ACC_DEG_PER_S2 = 120.0f;

// Debug: print a warning (rate-limited) if any servo command saturates at 0..180.
static constexpr bool ENABLE_SERVO_SAT_WARN = true;
static constexpr unsigned long SERVO_SAT_WARN_PERIOD_MS = 1000;

// Trajectory timing (time per segment between two waypoints)
static constexpr float TRAJ_SEG_TIME_S = .5f;

static constexpr bool ENABLE_CALIBRATION_MODE = false;