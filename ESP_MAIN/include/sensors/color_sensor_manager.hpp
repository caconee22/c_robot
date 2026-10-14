#pragma once

#include "sensors/color_sensor_types.hpp"

// Contract
// Role: TCA9548A channel select, AS7341/TCS34725 init and round-robin raw
//   reads in short register steps (no waiting for integration).
// Called from: SensorManager only. Each Wire transfer is bounded by
//   config::sensors::I2C_TIMEOUT_MS.
// Failure: I2C/timeout disables that channel until reboot; saturation or no
//   light only invalidates that sample. No re-init, no retry loop.
// Settings: sensor_config.hpp (exposure, gain, LED current are start values).
class ColorSensorManager {
 public:
  static bool begin();
  static void update(uint32_t nowMs);

  static ColorRawSample sample(uint8_t sensorIndex);
  static ColorSensorStatus status();
  static bool available(uint8_t sensorIndex);
  static ColorSensorModel model();
  static uint32_t settingsSignature();
};
