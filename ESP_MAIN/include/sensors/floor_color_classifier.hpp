#pragma once

#include "sensors/color_sensor_types.hpp"

// Contract
// Role: pure function from one raw sample to Red/Yellow/Blue/Black/Unknown.
//   Colors by ratio, black also by brightness. Must handle 3/10/20 mm gaps.
// Called from: SensorManager only. No I2C, UART or motor access.
class FloorColorClassifier {
 public:
  static FloorColorResult classify(
      uint8_t sensorIndex, const ColorRawSample& sample,
      const ColorCalibrationData& calibration);
};
