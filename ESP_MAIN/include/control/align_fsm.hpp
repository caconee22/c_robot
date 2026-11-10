#pragma once

#include "robot_types.hpp"

// Contract
// Role: match FSM rebuilt step by step (11/10), used instead of RobotFsm when
//   config::control::ALIGN_ONLY_FSM is true. One pass picks, in this order:
//   1. safety: motors not allowed -> Brake (cancels the opening and escape)
//   2. floor escape (control/escape_move.hpp): a hazard color on any corner
//      starts it, also during the opening move (which is then cancelled).
//      The target memory is kept (lock-on), no new confirmation afterwards.
//   3. opening move (control/opening_move.hpp) once at the start of each run
//   4. charge: tower centered for CHARGE_CONFIRM_FRAMES new frames -> drive at
//      it with steering; back to align when it drifts past CHARGE_ABORT_ERROR;
//      a detector drop shorter than CHARGE_LOST_HOLD_MS keeps driving straight
//   4b. fork attack (control/fork_attack.hpp): charging at a close centered
//      tower -> brake, slide the fork under, burst; then keep charging
//   5. align: turn in place toward the tower (duty ramps with the error)
//   6. lost: brake ALIGN_LOST_HOLD_MS, then spin toward the side last seen
//   config::opening::TRACK_AFTER false stops after the opening move instead
//   (escape still active); config::control::CHARGE_ENABLED false keeps the
//   robot turning in place only.
// Called from: loop(). Pure logic: no GPIO, I2C, UART, never blocks.
// Safety: SW1/SW2, the command watchdog and the tilt stop stay in their own
//   modules. Floor sensing limits: see control/floor_hazard.hpp.
// Values: config::control, config::opening, config::escape (start values).
class AlignFsm {
 public:
  static void begin();
  static DriveIntent update(uint32_t nowMs, const SensorSnapshot& sensors,
                            const VisionSnapshot& vision, const SafetyStatus& safety);
  static RobotState state();
  // Hazard color that started the current escape (for the LED).
  static FloorColor escapeColor();
};
