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
constexpr uint16_t SEARCH_DUTY = 500;   // 11/13 match day (학현): too slow, stuck turning in the groove (300 -> 500)
constexpr uint16_t TARGET_CONFIRM_DUTY = 300;
// Align spin ramps with the tower's distance from center: MIN just outside the
// center window, MAX at ALIGN_FULL_ERROR px and beyond (bench ALIGN: less
// overshoot and left/right swinging than one fixed duty).
constexpr uint16_t ALIGN_MIN_DUTY = 350;   // 11/13: turn out of the groove (280 -> 350)
constexpr uint16_t ALIGN_MAX_DUTY = 450;   // 11/13: (380 -> 450); 455 rocked on 11/10, lower first if it rocks
constexpr uint16_t ALIGN_FULL_ERROR = 700;
constexpr uint16_t APPROACH_DUTY = 450;
constexpr uint16_t ATTACK_CHARGE_DUTY = 650;
constexpr uint16_t SPOKE_INSERT_DUTY = 480;
constexpr uint16_t PUSH_DUTY = 650;
constexpr uint16_t BACKOFF_DUTY = 450;
constexpr uint16_t ZONE_ESCAPE_DUTY = 550;
constexpr int16_t APPROACH_STEERING_MAX = 120;
constexpr int16_t PUSH_STEERING_MAX = 60;
static_assert(PUSH_DUTY + PUSH_STEERING_MAX <= 710 && ATTACK_CHARGE_DUTY + PUSH_STEERING_MAX <= 710,
              "push steering must fit under motor DEFAULT_OUTPUT_LIMIT");

constexpr uint16_t TARGET_CENTER_X = 960;
// 11/04 bench ALIGN: +-100 px swung left/right, +-200 px settled.
constexpr uint16_t TARGET_CENTER_DEADBAND = 200;
// Attack needs a narrower center than align, but +-75 px was tighter than the
// align could hold; approach steering closes the rest on the way in.
constexpr uint16_t ATTACK_CENTER_DEADBAND = 150;
// 11/09 field run: 420 only charged when already very close; lowered so the
// charge starts farther out. Start value, measure the box height at the
// wanted charge distance with the real camera.
constexpr uint16_t ATTACK_BOX_HEIGHT = 280;
// Push gives up and re-confirms below this box height. Kept at the old
// 420 / 2 so lowering the charge height does not change the push.
constexpr uint16_t PUSH_MIN_BOX_HEIGHT = 210;
constexpr uint8_t TARGET_CONFIRM_FRAMES = 2;
// After losing the tower, keep spinning toward the side it was last seen
// (bench ALIGN found it again this way), then search starts on that side too.
constexpr uint32_t LOST_TARGET_HOLD_MS = 1500;
// 11/09: search keeps spinning one way (toward the side the tower was last
// seen) instead of swinging left/right, and keeps spinning when the Pi goes
// silent instead of the random forward/turn Wander. true restores Wander.
constexpr bool WANDER_WHEN_VISION_LOST = false;

// 11/10 restart: true runs AlignFsm (control/align_fsm.hpp) instead of
// RobotFsm: turn in place toward the tower and search by spinning, never
// forward or backward. false = the full match FSM below.
constexpr bool ALIGN_ONLY_FSM = true;
// AlignFsm: tower missing this long (detector drop) -> brake, then search.
constexpr uint32_t ALIGN_LOST_HOLD_MS = 300;
// AlignFsm charge (11/10, start values for 3S): tower within
// ATTACK_CENTER_DEADBAND for CHARGE_CONFIRM_FRAMES new frames -> drive at it.
// AlignFsm align damping (11/10, the +20 % outputs rocked left/right):
// stop early when the tower's image speed says it reaches the window within
// ALIGN_LEAD_MS (camera + UART delay and coasting), and once settled only turn
// again past ALIGN_RESUME_ERROR. Camera only, no IMU (AGENTS: no IMU control).
constexpr uint32_t ALIGN_LEAD_MS = 80;
constexpr uint16_t ALIGN_RESUME_ERROR = 260;
static_assert(ALIGN_RESUME_ERROR > TARGET_CENTER_DEADBAND, "resume band must be wider than the window");
constexpr bool CHARGE_ENABLED = true;
constexpr uint16_t CHARGE_DUTY = 800;      // 11/12: charge/push state higher, always (500 -> 800)
constexpr int16_t CHARGE_STEERING_MAX = 150;   // 11/12: with CHARGE 800 (120)
constexpr uint8_t CHARGE_CONFIRM_FRAMES = 2;
constexpr uint16_t CHARGE_ABORT_ERROR = 400;     // px off center -> back to align
constexpr uint32_t CHARGE_LOST_HOLD_MS = 300;    // keep going straight through a drop
static_assert(CHARGE_DUTY + CHARGE_STEERING_MAX <= 1000, "charge must fit under the motor limit");
// Push-through (11/11, 학현): opponent close (tower box at least
// PUSH_THROUGH_MIN_HEIGHT, seen within LOCKON_RECENT_MS) while charging or in
// the fork attack, and a hazard (red or blue) on the FRONT sensors only ->
// keep pushing PUSH_THROUGH_MS longer before backing out, to shove the
// opponent fully in. Rear or front+rear hazard, or a far tower -> escape at
// once as before. 0 = off. Start 0.1 s and lengthen on the field; every 0.1 s
// at the charge duty carries the front further over the groove edge.
constexpr uint32_t PUSH_THROUGH_MS = 0;   // 11/12 (학현): off (was 100); 0 = escape at once
constexpr uint16_t PUSH_THROUGH_MIN_HEIGHT = 300;   // 11/11 log: pushing at h 300-420
// Straight back-out (11/11, 학현): when the escape starts while charging or in
// the fork attack, the opponent is close (tower seen within LOCKON_RECENT_MS)
// and the hazard is on the front only, back straight out (no turn) so the
// robot keeps facing the opponent it pushed in and can hit again at once.
// Red (center circle) needs a closer opponent than blue, so an opponent just
// beyond the circle is not chased back and forth. STRAIGHT_REPEAT_MAX
// straight escapes from the same front side within STRAIGHT_REPEAT_WINDOW_MS
// -> the next one turns as before (breaks a back-and-forth loop).
constexpr bool STRAIGHT_BACK_ENABLED = true;
constexpr uint16_t STRAIGHT_MIN_HEIGHT = 300;       // blue (corner)
constexpr uint16_t STRAIGHT_MIN_HEIGHT_RED = 400;   // red (center circle)
constexpr uint8_t STRAIGHT_REPEAT_MAX = 3;
constexpr uint32_t STRAIGHT_REPEAT_WINDOW_MS = 4000;

// Lock-on escape (11/09): a hazard on the front sensors only while the robot
// is tracking the opponent (align, approach, attack or push, tower seen within
// LOCKON_RECENT_MS) means it is pushing the opponent into the zone. Back
// straight out without turning, keep the target, then push again.
// Start values, not yet driven on the field.
constexpr uint32_t LOCKON_RECENT_MS = 300;
constexpr uint32_t LOCKON_BACKOFF_MS = 360;
constexpr uint16_t LOCKON_BACKOFF_DUTY = 550;

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
