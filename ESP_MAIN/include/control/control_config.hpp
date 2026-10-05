#pragma once

#include <Arduino.h>

#include "sensors/color_sensor_types.hpp"

namespace config {
namespace control {

// Aggressive V1 profile. All values are permille and are still clamped by
// MotorController::DEFAULT_OUTPUT_LIMIT.
constexpr uint16_t SEARCH_DUTY = 300;
constexpr uint16_t TARGET_CONFIRM_DUTY = 240;
constexpr uint16_t ALIGN_DUTY = 280;
constexpr uint16_t APPROACH_DUTY = 450;
constexpr uint16_t ATTACK_CHARGE_DUTY = 650;
constexpr uint16_t SPOKE_INSERT_DUTY = 480;
constexpr uint16_t PUSH_DUTY = 650;
constexpr uint16_t BACKOFF_DUTY = 450;
constexpr uint16_t ZONE_ESCAPE_DUTY = 550;
constexpr int16_t APPROACH_STEERING_MAX = 120;
constexpr int16_t PUSH_STEERING_MAX = 60;

constexpr uint16_t TARGET_CENTER_X = 960;
constexpr uint16_t TARGET_CENTER_DEADBAND = 140;
constexpr uint16_t ATTACK_CENTER_DEADBAND = 75;
constexpr uint16_t ATTACK_BOX_HEIGHT = 420;
constexpr uint8_t TARGET_CONFIRM_FRAMES = 2;
constexpr uint32_t LOST_TARGET_HOLD_MS = 260;
constexpr uint32_t SEARCH_DIRECTION_PERIOD_MS = 1200;

// Time-based nose-dive and spoke sequence. Tune on a lifted-wheel rig first.
constexpr uint32_t ATTACK_CHARGE_MS = 220;
constexpr uint32_t DIVE_BRAKE_MS = 90;
constexpr uint32_t SPOKE_INSERT_MS = 350;
constexpr uint32_t BLIND_PUSH_HOLD_MS = 500;
constexpr uint32_t BACKOFF_MS = 260;

constexpr uint32_t ZONE_ESCAPE_REVERSE_MS = 360;
constexpr uint32_t ZONE_ESCAPE_TURN_MS = 320;
constexpr uint32_t COMMAND_VALID_MS = 80;

constexpr uint8_t colorBit(FloorColor color) {
  return 1U << static_cast<uint8_t>(color);
}

// Black is the normal floor. Colored grooves are treated as hazards until the
// final scoring strategy defines a different meaning for each color.
constexpr uint8_t HAZARD_COLOR_MASK =
    colorBit(FloorColor::Red) | colorBit(FloorColor::Yellow) |
    colorBit(FloorColor::Blue);

}  // namespace control
}  // namespace config
