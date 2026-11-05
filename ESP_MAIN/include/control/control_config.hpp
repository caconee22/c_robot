#pragma once

#include <Arduino.h>

#include "sensors/color_sensor_types.hpp"

namespace config {
namespace control {

// Aggressive V1 profile. All values are permille and are still clamped by
// MotorController::DEFAULT_OUTPUT_LIMIT.
// 11/04 bench (V2, 3S pack): below ~250 the robot does not spin in place and
// at 260 one wheel stalls so it pivots (looks like driving forward/back);
// 300 spins cleanly. Spin duties stay at or above that floor.
constexpr uint16_t SEARCH_DUTY = 300;
constexpr uint16_t TARGET_CONFIRM_DUTY = 300;
// Align spin ramps with the tower's distance from center: MIN just outside the
// center window, MAX at ALIGN_FULL_ERROR px and beyond (bench ALIGN: less
// overshoot and left/right swinging than one fixed duty).
constexpr uint16_t ALIGN_MIN_DUTY = 280;
constexpr uint16_t ALIGN_MAX_DUTY = 380;
constexpr uint16_t ALIGN_FULL_ERROR = 700;
constexpr uint16_t APPROACH_DUTY = 450;
constexpr uint16_t ATTACK_CHARGE_DUTY = 650;
constexpr uint16_t SPOKE_INSERT_DUTY = 480;
constexpr uint16_t PUSH_DUTY = 650;
constexpr uint16_t BACKOFF_DUTY = 450;
constexpr uint16_t ZONE_ESCAPE_DUTY = 550;
constexpr int16_t APPROACH_STEERING_MAX = 120;
constexpr int16_t PUSH_STEERING_MAX = 60;

constexpr uint16_t TARGET_CENTER_X = 960;
// 11/04 bench ALIGN: +-100 px swung left/right, +-200 px settled.
constexpr uint16_t TARGET_CENTER_DEADBAND = 200;
// Attack needs a narrower center than align, but +-75 px was tighter than the
// align could hold; approach steering closes the rest on the way in.
constexpr uint16_t ATTACK_CENTER_DEADBAND = 150;
constexpr uint16_t ATTACK_BOX_HEIGHT = 420;
constexpr uint8_t TARGET_CONFIRM_FRAMES = 2;
// After losing the tower, keep spinning toward the side it was last seen
// (bench ALIGN found it again this way), then search starts on that side too.
constexpr uint32_t LOST_TARGET_HOLD_MS = 1500;
constexpr uint32_t SEARCH_DIRECTION_PERIOD_MS = 1200;

// Time-based nose-dive and spoke sequence. Tune on a lifted-wheel rig first.
constexpr uint32_t ATTACK_CHARGE_MS = 220;
constexpr uint32_t DIVE_BRAKE_MS = 90;
constexpr uint32_t SPOKE_INSERT_MS = 350;
constexpr uint32_t BLIND_PUSH_HOLD_MS = 500;
constexpr uint32_t BACKOFF_MS = 260;

constexpr uint32_t ZONE_ESCAPE_REVERSE_MS = 360;
constexpr uint32_t ZONE_ESCAPE_TURN_MS = 320;
// Front and rear both on a hazard color (robot inside a groove): brake for a
// short time, then drive out opposite to the end that entered last. Flip the
// direction if the robot is still inside after each GROOVE_EXIT_FLIP_MS.
// Start values; not yet verified on the real field.
constexpr uint32_t BOTH_HAZARD_BRAKE_MS = 150;
constexpr uint32_t GROOVE_EXIT_FLIP_MS = 1500;
// A sensor pressed onto the floor (dark) or right against it (saturated) gives
// an invalid sample, and its color is cleared at once. If it read a hazard just
// before, keep treating it as a hazard this long (11/05; e.g. a nose dive at a
// groove edge). Start value, not yet measured on the field.
constexpr uint32_t HAZARD_CONTACT_HOLD_MS = 300;
constexpr uint32_t COMMAND_VALID_MS = 80;
static_assert(ALIGN_MIN_DUTY <= ALIGN_MAX_DUTY && ALIGN_FULL_ERROR > TARGET_CENTER_DEADBAND &&
              ATTACK_CENTER_DEADBAND <= TARGET_CENTER_DEADBAND, "Inconsistent align settings");

constexpr uint8_t colorBit(FloorColor color) {
  return 1U << static_cast<uint8_t>(color);
}

// Black is the normal floor. Red and blue grooves are hazards.
// Yellow is not a hazard (11/04 decision).
constexpr uint8_t HAZARD_COLOR_MASK =
    colorBit(FloorColor::Red) | colorBit(FloorColor::Blue);

}  // namespace control
}  // namespace config
