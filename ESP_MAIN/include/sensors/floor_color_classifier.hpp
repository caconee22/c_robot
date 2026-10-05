#pragma once

#include "sensors/color_sensor_types.hpp"

class FloorColorClassifier {
 public:
  static FloorColorResult classify(
      uint8_t sensorIndex, const ColorRawSample& sample,
      const ColorCalibrationData& calibration);
};
