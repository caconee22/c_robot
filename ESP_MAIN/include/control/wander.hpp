#pragma once

#include <Arduino.h>

#include "robot_types.hpp"

// Contract
// Role: blind driving pattern for when the Raspberry Pi gives no usable frames.
//   Alternates a forward arc and an in-place turn, each with a random length,
//   duty, side and arc, so the robot never repeats one fixed motion (judges can
//   count a repeated motion as a broken robot).
// Called from: RobotFsm only (reset on entering Wander, intent every pass).
//   Pure logic: no hardware, no logging, never blocks.
// Failure: none. Floor hazards and safety stay above it in RobotFsm.
// Values: start values, not yet driven on the field.

namespace config {
namespace wander {
// No usable frame for this long before wandering (short UART gaps keep the spin search).
constexpr uint32_t VISION_LOST_MS = 1000;
constexpr uint16_t FORWARD_DUTY_MIN = 380;
constexpr uint16_t FORWARD_DUTY_MAX = 480;
constexpr int16_t FORWARD_STEERING_MAX = 90;   // random arc, both sides
constexpr uint32_t FORWARD_MS_MIN = 300;
constexpr uint32_t FORWARD_MS_MAX = 900;
constexpr uint16_t TURN_DUTY_MIN = 300;        // spin floor from the 11/04 bench
constexpr uint16_t TURN_DUTY_MAX = 380;
constexpr uint32_t TURN_MS_MIN = 200;
constexpr uint32_t TURN_MS_MAX = 700;
}  // namespace wander
}  // namespace config

namespace Wander {
void reset(uint32_t nowMs);
DriveIntent intent(uint32_t nowMs);
}  // namespace Wander
