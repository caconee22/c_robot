#pragma once

#include <Arduino.h>

#include "robot_types.hpp"

// Contract
// Role: fixed opening move right after SW2 starts a run (11/10): turn left
//   ~45 deg, drive forward to the left of the field center, turn left ~90 deg
//   more to face the opponent, then hand over to the align/search FSM.
//   Open loop: every step is a fixed duty for a fixed time (no IMU, no
//   encoder). The angles and distance are tuned on the field by changing the
//   times below; the battery voltage and floor change them.
// Called from: AlignFsm only (start on each new run, intent every pass).
//   Pure logic: no hardware, never blocks.
// Failure: none here. Safety (SW1/SW2, watchdog, tilt) stays outside; floor
//   colors are NOT checked during the move.
// Values: start values, not yet driven on the field.

namespace config {
namespace opening {
constexpr bool ENABLED = true;
// 11/10: false = after the move the robot just brakes and stays (tracker off,
// for tuning the move alone). true = align/search toward the tower afterwards.
constexpr bool TRACK_AFTER = true;
constexpr uint16_t TURN_DUTY = 400;   // 11/10 3S logs: 400 x 1.167 gave 43-44 deg for the 45 deg turn, 350 only ~30 deg
// 11/10 14:08 log (3S, trim R900): 220 ms gave ~34 deg, 440 ms ~70 deg -> 290/565.
// 11/11: new ESP chip on V2 = empty NVS, trim back to 1000/1000, and the turns
// went too far -> back to 220/440 (gave 43-44 deg on 3S without trim, 11/10).
// 11/11 21:21 log (3S, R920): 220 ms gave ~34 deg -> 255 ms.
constexpr uint32_t TURN_LEFT_45_MS = 255;
constexpr uint16_t FORWARD_DUTY = 400;
constexpr uint32_t FORWARD_MS = 2000;   // 11/10 field: 900 went half the distance -> 2 s
constexpr uint32_t TURN_LEFT_90_MS = 440;
// Brake between steps so one step's momentum does not carry into the next.
constexpr uint32_t PAUSE_MS = 150;
// Battery compensation (11/10): the times above were tuned on a 4S pack. The
// duties are scaled by the pack type the power-on battery check found
// (BatteryMonitor cell count, result Ok only), not by the measured voltage:
// 4S x1.0, 3S x THREE_CELL_SCALE (4/3: same nominal motor voltage).
// Battery check off or not Ok -> no scaling (x1.0, the 4S values).
constexpr uint16_t FOUR_CELL_SCALE_PERMILLE = 1000;
// 11/10 field on 3S: 1333 went too far, 1000 a little short -> halfway.
constexpr uint16_t THREE_CELL_SCALE_PERMILLE = 1167;
constexpr uint16_t DUTY_MAX = 1000;   // motor DEFAULT_OUTPUT_LIMIT (11/12; opening duties unchanged)
static_assert(TURN_DUTY * THREE_CELL_SCALE_PERMILLE / 1000 <= DUTY_MAX &&
              FORWARD_DUTY * THREE_CELL_SCALE_PERMILLE / 1000 <= DUTY_MAX,
              "scaled opening duty must stay under the motor limit");
}  // namespace opening
}  // namespace config

namespace OpeningMove {
// Cell count from the power-on battery check (3 or 4), 0 = unknown. Read at start().
void setBatteryCells(uint8_t cells);
uint16_t scalePermille();             // duty scale of the current/last run
uint8_t batteryCells();               // last value given to setBatteryCells
void start(uint32_t nowMs);
void cancel();
bool active(uint32_t nowMs);          // false once every step has run (or cancelled)
DriveIntent intent(uint32_t nowMs);   // call only while active()
}  // namespace OpeningMove
