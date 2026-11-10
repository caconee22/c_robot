#pragma once

#include "robot_types.hpp"

// Contract
// Role: floor perception for the FSM (11/10). Which corners stand on a hazard
//   color (red/blue, config::control::HAZARD_COLOR_MASK; yellow is not one)
//   and which corner saw it FIRST, so the escape turns away from that side.
//   A sensor that read a hazard and is then blinded by floor contact (invalid
//   sample: dark or saturated) keeps counting for HAZARD_CONTACT_HOLD_MS.
//   The first-hit memory is kept until every corner has been clear for
//   FIRST_CLEAR_MS, so a flickering edge keeps one direction.
// Called from: AlignFsm once per pass. Pure logic: no hardware, no motion.
// Corners: bit0 front left, bit1 front right, bit2 rear left, bit3 rear right.
// Limits (field conditions): 3/10/20 mm gaps change the readings; at 20 mm a
//   colored floor can read black (late or no detection), 3 mm can saturate,
//   and color edges mix. A corner whose sensor is off reads nothing.

namespace config {
namespace floor_hazard {
constexpr uint32_t FIRST_CLEAR_MS = 300;
// Corners that reach the color within this time of the first one count as
// "first" together (two sensors of one side rarely confirm in the same pass).
constexpr uint32_t FIRST_WINDOW_MS = 60;
}  // namespace floor_hazard
}  // namespace config

// Side that went onto the color first = the deepest end (11/10: decides the
// way out, also when the robot is pushed all the way in).
enum class HazardSide : uint8_t {
  None,
  FrontLeft, FrontRight, FrontBoth,   // front first: we drove (or were pushed) forward in
  RearLeft, RearRight, RearBoth,      // rear first: pushed backward in
  Left, Right,                        // one side first: pushed sideways in
  Unclear,                            // diagonal or 3+ corners at once
};

struct FloorHazardInfo {
  uint8_t mask = 0;                      // corners on a hazard now
  HazardSide first = HazardSide::None;   // side that saw it first (kept, see above)
  FloorColor firstColor = FloorColor::Unknown;
  bool front() const { return (mask & 0x03U) != 0; }
  bool rear() const { return (mask & 0x0CU) != 0; }
};

namespace FloorHazard {
void reset();
FloorHazardInfo update(uint32_t nowMs, const SensorSnapshot& sensors);
}  // namespace FloorHazard
