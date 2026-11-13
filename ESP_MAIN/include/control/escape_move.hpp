#pragma once

#include "control/floor_hazard.hpp"
#include "robot_types.hpp"

// Contract
// Role: floor escape for AlignFsm (11/10). The way out is decided by the side
//   that went onto the color FIRST (FloorHazard), not by the direction the
//   robot was commanding: when the opponent pushes us in, the commanded
//   direction and the real motion differ. That decision is kept when the
//   robot ends up fully on the color (pushed all the way in), with a higher
//   duty because the opponent is still pushing.
//     front both  -> reverse                front left/right -> reverse + turn
//     rear both   -> forward                rear left/right  -> forward, nose
//                                                               away from it
//     left side   -> forward, hard right    right side       -> forward, hard left
//     unclear     -> reverse if we were charging, else forward
//   Reverse turn sign set on the field 11/10 (front left -> -TURN_STEERING).
//   Still on the color after FLIP_MS one way -> flip direction (10 s rule).
//   Ends CLEAR_MS after every corner is clear.
// Does not touch the target memory: AlignFsm keeps its lock-on.
// Called from: AlignFsm only. Pure logic: no hardware, never blocks.
// Values: start values, not yet driven on the field.

namespace config {
namespace escape {
constexpr uint16_t REVERSE_DUTY = 650;   // 11/13 match field: climbing out over the groove lip takes longer (450 -> 650)
constexpr uint16_t FORWARD_DUTY = 650;   // 11/13: same as REVERSE_DUTY (450 -> 650)
constexpr int16_t TURN_STEERING = 150;   // front corner first, while reversing
constexpr int16_t REAR_STEERING = 120;   // rear corner first, while going forward
constexpr int16_t SIDE_STEERING = 250;   // pushed in sideways
// Front and rear both on the color (pushed all the way in): push back harder.
constexpr uint16_t PUSHED_DUTY = 750;    // 11/13: above REVERSE/FORWARD 650 (was 620); duty + steering capped at MOTOR_LIMIT
constexpr uint16_t MOTOR_LIMIT = 1000;   // motor DEFAULT_OUTPUT_LIMIT (1000 since 11/12): duty + steering stays under it
constexpr uint32_t CLEAR_MS = 200;       // keep moving this long after the floor is clear (11/13: 200 -> 500 backed off too far -> 200; duty stays 650)
constexpr uint32_t FLIP_MS = 2500;       // 11/13: 1500 -> 2500, the lip makes the way out slower; do not flip too early
}  // namespace escape
}  // namespace config

namespace EscapeMove {
// wasCharging: the robot was driving forward at the tower (for an unclear entry).
// straightBack (11/11): reverse without the turn, so the robot keeps facing an
// opponent it just pushed onto the color and can charge again at once
// (AlignFsm decides when; see config::control::STRAIGHT_*).
void start(uint32_t nowMs, const FloorHazardInfo& floor, bool wasCharging = false,
           bool straightBack = false);
void cancel();
bool active();
// Advances the escape; returns Brake and ends (active() false) once clear.
DriveIntent update(uint32_t nowMs, const FloorHazardInfo& floor);
}  // namespace EscapeMove
