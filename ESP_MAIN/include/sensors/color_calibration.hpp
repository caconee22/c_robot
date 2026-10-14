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
// Role: UART0 guided calibration (4 sensors x 4 colors x 3 poses) and NVS
//   storage. Not part of match driving.
// Called from: loop(); while active() the loop brakes and skips the FSM.
// Rule: runs only while waiting for start; SW2 aborts it.
class ColorCalibration {
 public:
  static void begin();

  // allowed is true only while the robot is waiting for its start button.
  static void update(uint32_t nowMs, bool allowed);

  static bool active();
  static bool hasStoredData();
  static uint32_t generation();
  static const ColorCalibrationData& data();
  static ColorCalibrationStatus status();
};
