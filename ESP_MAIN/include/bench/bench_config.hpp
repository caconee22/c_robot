#pragma once
#include <Arduino.h>

namespace config { namespace bench {
// Lift wheels for MOTOR / MANUAL first. Units are permille, not percent.
constexpr int16_t OUTPUT_LIMIT = 200;
constexpr int16_t MOTOR_TEST_DUTY = 120;
constexpr uint32_t MOTOR_STEP_MS = 3000;
constexpr uint32_t MOTOR_GAP_MS = 500;
constexpr uint32_t MANUAL_HOLD_MS = 250;
constexpr uint32_t COMMAND_VALID_MS = 80;
constexpr int16_t TURN_DUTY = 120;
constexpr int16_t FOLLOW_DUTY = 160;
constexpr int16_t STEERING_MAX = 40;
constexpr uint16_t CENTER_X = 960;
constexpr uint16_t CENTER_DEADBAND = 100;
constexpr uint16_t TURN_ONLY_ERROR = 300;
constexpr uint16_t STOP_BOX_HEIGHT = 420;
constexpr uint32_t ESCAPE_MOVE_MS = 250;
constexpr uint32_t ESCAPE_TURN_MS = 250;
constexpr uint32_t LOG_PERIOD_MS = 100;
constexpr uint32_t LOG_FRAME_TIMEOUT_MS = 80;
constexpr uint32_t CONSOLE_TIMEOUT_MS = 250;
constexpr uint8_t CONSOLE_BYTES_PER_LOOP = 64;
static_assert(OUTPUT_LIMIT > 0 && OUTPUT_LIMIT <= 300, "Keep bench limit low");
static_assert(MOTOR_STEP_MS > 0 && MANUAL_HOLD_MS > 0 && COMMAND_VALID_MS > 0 &&
              CENTER_X > 0 && CENTER_X < 1920 && STOP_BOX_HEIGHT > 0 && STOP_BOX_HEIGHT <= 1080,
              "Invalid bench timing or camera settings");
static_assert(MOTOR_TEST_DUTY > 0 && MOTOR_TEST_DUTY <= OUTPUT_LIMIT &&
              TURN_DUTY > 0 && TURN_DUTY <= OUTPUT_LIMIT &&
              FOLLOW_DUTY > 0 && FOLLOW_DUTY + STEERING_MAX <= OUTPUT_LIMIT,
              "Bench duties exceed limit");
} }
