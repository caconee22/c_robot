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
constexpr int16_t DEFAULT_OUTPUT_LIMIT = 650;

constexpr uint32_t DEFAULT_COMMAND_TIMEOUT_MS = 100;
constexpr uint32_t WATCHDOG_POLL_MS = 5;
constexpr uint16_t DIRECTION_CHANGE_DEADTIME_US = 100;

constexpr uint8_t MOTOR1_LPWM_CHANNEL = 0;
constexpr uint8_t MOTOR1_RPWM_CHANNEL = 1;
constexpr uint8_t MOTOR2_LPWM_CHANNEL = 2;
constexpr uint8_t MOTOR2_RPWM_CHANNEL = 3;

// Change these after checking whether positive output moves the robot forward.
constexpr bool LEFT_MOTOR_INVERTED = false;
constexpr bool RIGHT_MOTOR_INVERTED = false;

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
