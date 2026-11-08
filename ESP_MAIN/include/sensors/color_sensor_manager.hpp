#pragma once

#include "sensors/color_sensor_types.hpp"

// Contract
// Role: TCA9548A channel select, AS7341/TCS34725 init and round-robin raw
//   reads in short register steps (no waiting for integration).
// Called from: SensorManager only. Each Wire transfer is bounded by
//   config::sensors::I2C_TIMEOUT_MS.
// Failure: I2C/timeout/init failure disables that channel; it is re-initialized
//   at most SENSOR_RETRY_LIMIT times, once per SENSOR_RETRY_PERIOD_MS, in two
//   short steps (power on, configure) so loop() never waits. After that it
//   stays off until reboot. Saturation or no light only invalidates that sample.
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
