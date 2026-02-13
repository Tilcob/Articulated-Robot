#pragma once
#include <Arduino.h>

static constexpr float L1 = 0.120f;  // [m]
static constexpr float L2 = 0.100f;  // [m]
static constexpr float h  = 0.014f;  // [m]

// Workspace limits
static constexpr float R_MIN = 0.05f;
static constexpr float R_MAX = (L1 + L2) * 0.95f;
static constexpr float Z_MIN = h - 0.02f;
static constexpr float Z_MAX = h + (L1 + L2) * 0.80f;

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

static constexpr float BASE_ZERO_DEG     = 90.0f;
static constexpr float SHOULDER_ZERO_DEG = 45.0f;
static constexpr float ELBOW_ZERO_DEG    = 45.0f;

static constexpr int BASE_SIGN     = +1;
static constexpr int SHOULDER_SIGN = -1;
static constexpr int ELBOW_SIGN    = +1;

static constexpr float Q1_MIN = -PI;
static constexpr float Q1_MAX = +PI;

static constexpr float Q2_MIN = -PI/2;
static constexpr float Q2_MAX = +PI/2;

static constexpr float Q3_MIN = -PI*3/4;
static constexpr float Q3_MAX = +PI*3/4;

static constexpr float MAX_SPEED_DEG_PER_S = 180.0f;  // max speed
static constexpr float LOOP_DT_S = 0.02f;            // 20ms