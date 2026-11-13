#pragma once

#include "control/floor_hazard.hpp"
#include "robot_types.hpp"

// Contract
// Role: go around the red center circle while locked on a far tower (11/10).
//   AlignFsm starts it instead of the normal escape when the first hazard is
//   red, the tower was seen within LOCK_RECENT_MS and its box is shorter than
//   TOWER_MAX_HEIGHT (far away: the circle is between us and the opponent).
//   Steps: straight reverse (at least BACK_MS, until the front is clear for
//   BACK_CLEAR_MS, at most BACK_MAX_MS) -> turn in place away from the
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
// 11/13 match field (학현): the groove has a lip, 0.15 s back left the front
// still over the red, so it turned on the edge and hit it again. Now: back at
// least BACK_MS, and keep backing until the front is off the red for
// BACK_CLEAR_MS (a sensor lifted over the lip can read the red as black/unknown
// early, hence the minimum), at most BACK_MAX_MS.
constexpr uint16_t BACK_DUTY = 600;    // 11/13: 450 -> 600
constexpr uint32_t BACK_MS = 180;      // minimum back time (11/13: 150 -> 400 too far -> 180)
constexpr uint32_t BACK_CLEAR_MS = 80;    // front clear this long before turning (11/13: 200 -> 80)
constexpr uint32_t BACK_MAX_MS = 1200;    // never back longer than this
constexpr uint16_t TURN_DUTY = 400;     // 11/13: 0.25 s ~50 deg on 3S (tune on the field)
constexpr uint32_t TURN_MS = 250;      // 11/13: 180 -> 250
constexpr uint16_t ARC_DUTY = 450;     // 11/13: 400 -> 450
constexpr int16_t ARC_STEERING = 60;    // toward the circle side, to hug its edge
constexpr uint32_t ARC_MS = 800;       // 11/13: 600 -> 800
static_assert(BACK_MS <= BACK_MAX_MS, "minimum back time must fit the maximum");
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
