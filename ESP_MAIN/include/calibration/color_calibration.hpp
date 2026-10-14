#pragma once

#include "sensors/color_sensor_types.hpp"

enum class CalibrationState : uint8_t {
  Idle,
  WaitingForReady,
  Countdown,
  Measuring,
  WaitingForAccept,
  Completed,
  Error,
};

struct ColorCalibrationStatus {
  CalibrationState state = CalibrationState::Idle;
  uint8_t step = 0;
  uint8_t sensorIndex = 0;
  FloorColor color = FloorColor::Red;
  CalibrationPose pose = CalibrationPose::Level;
  uint16_t sampleCount = 0;
  bool active = false;
  bool storedDataValid = false;
};

// Contract
// Role: UART0 guided calibration (4 sensors x 4 colors x 3 poses). Saves
//   through CalibrationStore. Not linked into the production firmware.
// Called from: calibration_main.cpp only (PlatformIO env color_calibration).
//   That firmware never initializes the motors; their pins stay LOW.
// Rule: call CalibrationStore::load() before begin().
class ColorCalibration {
 public:
  static void begin();
  static void update(uint32_t nowMs);

  static bool active();
  static ColorCalibrationStatus status();
};
