#pragma once

#include "control/floor_hazard.hpp"
#include "robot_types.hpp"

// Contract
// Role: go around the red center circle while locked on a far tower (11/10).
//   AlignFsm starts it instead of the normal escape when the first hazard is
//   red, the tower was seen within LOCK_RECENT_MS and its box is shorter than
//   TOWER_MAX_HEIGHT (far away: the circle is between us and the opponent).
//   Steps (time based): short straight reverse -> turn in place away from the
//   circle -> forward arc hugging the circle -> done, AlignFsm aligns and
//   charges again (lock-on kept). Touching red again in the arc restarts it,
//   so the robot works its way around the circle edge.
//   Rear hazard while reversing, or front and rear at once -> NeedEscape
//   (AlignFsm hands over to the normal escape).
// Called from: AlignFsm only. Pure logic: no hardware, never blocks.
// Values: start values for 3S, not yet driven on the field. TOWER_MAX_HEIGHT
//   is in packet pixels (1920x1080, the "h" in the Pi tracking viewer).

namespace config {
namespace circle {
constexpr uint16_t TOWER_MAX_HEIGHT = 250;
constexpr uint32_t LOCK_RECENT_MS = 300;
constexpr uint16_t BACK_DUTY = 450;    // 11/10: back from the +20 % try
constexpr uint32_t BACK_MS = 150;
constexpr uint16_t TURN_DUTY = 400;     // ~35-40 deg on 3S; 11/10 back from the +20 % try
constexpr uint32_t TURN_MS = 180;
constexpr uint16_t ARC_DUTY = 400;     // 11/10: back from the +20 % try
constexpr int16_t ARC_STEERING = 60;    // toward the circle side, to hug its edge
constexpr uint32_t ARC_MS = 600;
}  // namespace circle
}  // namespace config

enum class BypassResult : uint8_t { Running, Done, NeedEscape };

namespace CircleBypass {
// turnRight: true when the circle is on the left (front left hit first).
void start(uint32_t nowMs, bool turnRight);
void cancel();
bool active();
DriveIntent update(uint32_t nowMs, const FloorHazardInfo& floor, BypassResult& result);
}  // namespace CircleBypass
