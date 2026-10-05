#pragma once

#include "sensors/color_sensor_types.hpp"

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
