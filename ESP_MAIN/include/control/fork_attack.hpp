#pragma once

#include <Arduino.h>

#include "robot_types.hpp"

// Contract
// Role: fork attack for AlignFsm (11/10). While charging, when the tower is
//   close (box height >= TRIGGER_HEIGHT) and centered, run a short timed move:
//     1. BRAKE   brake hard for BRAKE_MS: the moving robot pitches forward and
//                the fork drops toward the floor
//     2. INSERT  drive in at INSERT_DUTY for INSERT_MS: slide the fork under
//     3. BURST   jump to BURST_DUTY (near the limit) for BURST_MS: lift/push
//   then AlignFsm keeps charging. Steering toward the tower while it is seen.
//   COOLDOWN_MS after one attack before the next (no repeated braking mid-push).
//   Floor escape and safety cancel it (AlignFsm). Lock-on is kept.
// Called from: AlignFsm only. Pure logic: no hardware, never blocks.
// Values: start values for 3S, not yet driven on the field. TRIGGER_HEIGHT is
//   in packet pixels (1920x1080, the "h" in the Pi tracking viewer).

namespace config {
namespace fork {
constexpr bool ENABLED = true;
constexpr uint16_t TRIGGER_HEIGHT = 400;
constexpr uint16_t TRIGGER_CENTER = 200;   // px off center at most (11/10 log: long pushes sat ~160 px off)
constexpr uint32_t BRAKE_MS = 90;
constexpr uint16_t INSERT_DUTY = 480;
constexpr uint32_t INSERT_MS = 250;
constexpr uint16_t BURST_DUTY = 650;
constexpr int16_t STEERING_MAX = 60;       // BURST_DUTY + STEERING_MAX stays at the 710 limit
constexpr uint32_t BURST_MS = 800;
constexpr uint32_t COOLDOWN_MS = 1500;
static_assert(BURST_DUTY + STEERING_MAX <= 710 && INSERT_DUTY + STEERING_MAX <= 710,
              "fork attack must fit under the motor limit");
}  // namespace fork
}  // namespace config

enum class ForkPhase : uint8_t { Idle, Brake, Insert, Burst };

namespace ForkAttack {
void start(uint32_t nowMs);
void cancel();
// Current phase; Idle once finished or cancelled. Also tracks the cooldown.
ForkPhase phase(uint32_t nowMs);
bool ready(uint32_t nowMs);   // not running and the cooldown has passed
// towerError: px off center (0 when the tower is not seen this pass).
DriveIntent intent(uint32_t nowMs, int32_t towerError);
}  // namespace ForkAttack
