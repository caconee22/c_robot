#pragma once

#include "control/floor_hazard.hpp"
#include "robot_types.hpp"

// Contract
// Role: floor escape for AlignFsm (11/10). Front corner hit first -> back off
//   while the nose turns away from the side that saw the hazard first (front
//   left first -> nose swings right). Rear hit first -> drive forward.
//   Ends CLEAR_MS after every corner is clear. Front and rear at once (inside
//   a groove): brake BOTH_BRAKE_MS, then drive out opposite to the way the
//   robot was moving, flipping every FLIP_MS (never park in a groove: 10 s rule).
//   Still on a hazard after FLIP_MS in one direction -> flip the direction.
// Does not touch the target memory: AlignFsm keeps its lock-on.
// Called from: AlignFsm only. Pure logic: no hardware, never blocks.
// Values: start values, not yet driven on the field.

namespace config {
namespace escape {
constexpr uint16_t REVERSE_DUTY = 450;
constexpr int16_t TURN_STEERING = 150;   // nose swing while backing off
constexpr uint16_t FORWARD_DUTY = 450;   // rear-first escape
constexpr uint32_t CLEAR_MS = 200;       // keep moving this long after the floor is clear
constexpr uint32_t BOTH_BRAKE_MS = 150;
constexpr uint32_t FLIP_MS = 1500;
}  // namespace escape
}  // namespace config

namespace EscapeMove {
void start(uint32_t nowMs, const FloorHazardInfo& floor);
void cancel();
bool active();
// Advances the escape; returns Brake and ends (active() false) once clear.
DriveIntent update(uint32_t nowMs, const FloorHazardInfo& floor);
}  // namespace EscapeMove
