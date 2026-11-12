#pragma once

#include <Arduino.h>

namespace config {
namespace motor {

// Public motor commands use permille: -1000 (full reverse) to +1000
// (full forward). PWM details stay inside the motor module.
constexpr int16_t COMMAND_MAX = 1000;

// Standard BTS7960 DC motor setting: 20 kHz (50 us period), 8-bit duty
// resolution (0..255). The aggressive V1 profile allows up to 65 percent.
constexpr uint32_t PWM_FREQUENCY_HZ = 20000;
constexpr uint8_t PWM_RESOLUTION_BITS = 8;
constexpr uint32_t PWM_MAX_DUTY = (1UL << PWM_RESOLUTION_BITS) - 1UL;
constexpr int16_t ABSOLUTE_OUTPUT_LIMIT = COMMAND_MAX;
// 11/08 (학현): 650 -> 710 so PUSH/ATTACK_CHARGE 650 + PUSH_STEERING_MAX 60
// is not clipped. Not yet verified on the robot (current, heat, traction).
// 11/12 (학현): 710 -> 1000 only so the charge can go higher; every other
// duty is still under 710 and unchanged.
constexpr int16_t DEFAULT_OUTPUT_LIMIT = 1000;

constexpr uint32_t DEFAULT_COMMAND_TIMEOUT_MS = 100;
constexpr uint32_t WATCHDOG_POLL_MS = 5;
constexpr uint16_t DIRECTION_CHANGE_DEADTIME_US = 100;

constexpr uint8_t MOTOR1_LPWM_CHANNEL = 0;
constexpr uint8_t MOTOR1_RPWM_CHANNEL = 1;
constexpr uint8_t MOTOR2_LPWM_CHANNEL = 2;
constexpr uint8_t MOTOR2_RPWM_CHANNEL = 3;

// SIDES_SWAPPED: driver 1 (MOTOR1 pins) turns the RIGHT wheel and driver 2 the
// left one. LEFT/RIGHT_MOTOR_INVERTED then still name driver 1 / driver 2.
// PC tests build with MOTOR_PLAIN_MAPPING=1 (no swap, no inversion).
#if defined(HW_VERSION) && HW_VERSION == 2
// 2026-11-04 V2 rebuilt wiring: with both inverted (11/01 setting) both wheels ran backward again, so neither is inverted.
constexpr bool LEFT_MOTOR_INVERTED = false;
constexpr bool RIGHT_MOTOR_INVERTED = false;
constexpr bool SIDES_SWAPPED = false;
#elif defined(MOTOR_PLAIN_MAPPING) && MOTOR_PLAIN_MAPPING
constexpr bool LEFT_MOTOR_INVERTED = false;
constexpr bool RIGHT_MOTOR_INVERTED = false;
constexpr bool SIDES_SWAPPED = false;
#else
// 2026-11-10 old V1 robot, wheels lifted: "forward" spun right and "turn right"
// drove backward -> driver 1 drives the right wheel reversed, driver 2 the
// left wheel. Pins unchanged (pins.hpp); only the mapping.
constexpr bool LEFT_MOTOR_INVERTED = true;    // driver 1 (right wheel here)
constexpr bool RIGHT_MOTOR_INVERTED = false;  // driver 2 (left wheel here)
constexpr bool SIDES_SWAPPED = true;
#endif

// Whole-robot output scale, applied with the straight-drive trim after the
// limit (MotorController::apply), so every FSM and bench duty shrinks alike.
// 11/10: the old V1 robot has a faster gear ratio and ran too fast with the V2
// duties -> V1 x0.6 (start value, tune on the old robot). V2 stays x1.0.
// PC tests build with MOTOR_OUTPUT_SCALE_PERMILLE=1000 so duties are checked unscaled.
#if defined(MOTOR_OUTPUT_SCALE_PERMILLE)
constexpr uint16_t OUTPUT_SCALE_PERMILLE = MOTOR_OUTPUT_SCALE_PERMILLE;
#elif defined(HW_VERSION) && HW_VERSION == 2
constexpr uint16_t OUTPUT_SCALE_PERMILLE = 1000;
#else
// 11/10 (2nd): the old robot has 24 V motors run from a 4S pack (14.8-16.8 V),
// so the V2 duties were too weak -> x1.6 (~24 / 14.8). Applied after the
// runtime output limit; the result is clamped to COMMAND_MAX (100 %), so on
// V1 the limit (710, bench 400) is in "24 V motor" terms, not raw PWM.
constexpr uint16_t OUTPUT_SCALE_PERMILLE = 1600;
#endif
static_assert(OUTPUT_SCALE_PERMILLE > 0 && OUTPUT_SCALE_PERMILLE <= 2000, "output scale 0..x2");
#if defined(HW_VERSION) && HW_VERSION == 2
static_assert(OUTPUT_SCALE_PERMILLE <= 1000, "V2 output scale only reduces");
#endif

enum class BrakeStrategy : uint8_t {
  // Confirmed safe fallback: PWM and enable signals are all LOW.
  Coast,
  // Experimental BTS7960 combinations. Select only after a lifted-wheel test.
  BothPwmLowEnableHigh,
  BothPwmHighEnableHigh,
};

// Selected for the nose-dive test. Verify this electrical combination with
// lifted wheels before allowing floor operation.
constexpr BrakeStrategy BRAKE_STRATEGY = BrakeStrategy::BothPwmLowEnableHigh;

}  // namespace motor
}  // namespace config
