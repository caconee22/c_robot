#pragma once

#include "sensors/color_sensor_types.hpp"

// Contract
// Role: load, validate, save and erase the color calibration blob in NVS
//   (namespace "color-cal", key "data"). The format is shared by the
//   production firmware (read only) and the calibration firmware (writes).
// Called from: SensorManager::begin() in production; ColorCalibration in the
//   separate calibration firmware. Boot only for load(); never in a loop.
// Failure: missing, corrupt or mismatched data is cleared and valid() is
//   false. Classification then falls back to the temporary thresholds.
class CalibrationStore {
 public:
  static bool load();
  static bool save(const ColorCalibrationData& value);
  static bool erase();

  // Fills a new blob header for the current sensor model and settings.
  static void initialize(ColorCalibrationData& value);
  static bool isValid(const ColorCalibrationData& value);

  static bool valid();
  static uint32_t generation();
  static const ColorCalibrationData& data();
};
