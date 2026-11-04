#pragma once
#include <Arduino.h>

namespace config { namespace bench {
// Lift wheels for MOTOR / MANUAL first. Units are permille, not percent.
constexpr int16_t OUTPUT_LIMIT = 400;
constexpr int16_t MOTOR_TEST_DUTY = 120;
constexpr uint32_t MOTOR_STEP_MS = 3000;
constexpr uint32_t MOTOR_GAP_MS = 500;
constexpr uint32_t MANUAL_HOLD_MS = 250;
constexpr uint32_t COMMAND_VALID_MS = 80;
// 11/04: at 260 one wheel stalls and the robot pivots (looks like forward/back);
// 300 spins in place on both wheels. Below ~250 nothing turns.
constexpr int16_t TURN_DUTY = 300;
constexpr int16_t FOLLOW_DUTY = 160;
constexpr int16_t STEERING_MAX = 40;
constexpr uint16_t CENTER_X = 960;
constexpr uint16_t CENTER_DEADBAND = 100;
constexpr uint16_t TURN_ONLY_ERROR = 300;
// ALIGN only: wider center window against left/right swinging (11/04).
constexpr uint16_t ALIGN_DEADBAND = 200;
// ALIGN only: after losing the tower, spin toward the side it was last seen
// for about one turn, then stop. Time-based guess, measure one turn at TURN_DUTY.
constexpr uint32_t SEARCH_TURN_MS = 3000;
// ALIGN spin output ramps with the tower's distance from center: MIN at the
// window edge (ALIGN_DEADBAND) up to MAX at ALIGN_FULL_ERROR px and beyond.
constexpr int16_t ALIGN_MIN_DUTY = 250;
constexpr int16_t ALIGN_MAX_DUTY = 350;
constexpr uint16_t ALIGN_FULL_ERROR = 700;
// Extra permille for one side when spinning (ALIGN and its search). The command is
// always equal and opposite, so forward/back drift means one side is weaker:
// raise the weak side here after the DRIVE 250 -250 / -250 250 check (BENCH_TESTS).
constexpr int16_t LEFT_SPIN_TRIM = 0;
constexpr int16_t RIGHT_SPIN_TRIM = 0;
constexpr uint16_t STOP_BOX_HEIGHT = 420;
// Physical SW2 held this long while stopped selects ALIGN; the release then starts it.
constexpr uint32_t ALIGN_HOLD_MS = 3000;
constexpr uint32_t ESCAPE_MOVE_MS = 250;
constexpr uint32_t ESCAPE_TURN_MS = 250;
constexpr uint32_t LOG_PERIOD_MS = 100;
constexpr uint32_t LOG_FRAME_TIMEOUT_MS = 80;
constexpr uint32_t CONSOLE_TIMEOUT_MS = 250;
constexpr uint8_t CONSOLE_BYTES_PER_LOOP = 64;
static_assert(OUTPUT_LIMIT > 0 && OUTPUT_LIMIT <= 400, "Keep bench limit low");
static_assert(MOTOR_STEP_MS > 0 && MANUAL_HOLD_MS > 0 && COMMAND_VALID_MS > 0 &&
              CENTER_X > 0 && CENTER_X < 1920 && STOP_BOX_HEIGHT > 0 && STOP_BOX_HEIGHT <= 1080,
              "Invalid bench timing or camera settings");
static_assert(MOTOR_TEST_DUTY > 0 && MOTOR_TEST_DUTY <= OUTPUT_LIMIT &&
              TURN_DUTY > 0 && TURN_DUTY <= OUTPUT_LIMIT &&
              FOLLOW_DUTY > 0 && FOLLOW_DUTY + STEERING_MAX <= OUTPUT_LIMIT,
              "Bench duties exceed limit");
static_assert(ALIGN_MIN_DUTY > 0 && ALIGN_MIN_DUTY <= ALIGN_MAX_DUTY &&
              ALIGN_FULL_ERROR > ALIGN_DEADBAND && ALIGN_FULL_ERROR <= CENTER_X &&
              LEFT_SPIN_TRIM >= 0 && RIGHT_SPIN_TRIM >= 0 &&
              ALIGN_MAX_DUTY + LEFT_SPIN_TRIM <= OUTPUT_LIMIT &&
              ALIGN_MAX_DUTY + RIGHT_SPIN_TRIM <= OUTPUT_LIMIT &&
              TURN_DUTY + LEFT_SPIN_TRIM <= OUTPUT_LIMIT &&
              TURN_DUTY + RIGHT_SPIN_TRIM <= OUTPUT_LIMIT, "Align spin exceeds limit");
} }
