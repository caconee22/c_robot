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
  uint8_t sensorMask = 0;  // bit n = sensor n+1 is measured in this step
  FloorColor color = FloorColor::Red;
  CalibrationPose pose = CalibrationPose::Level;
  uint16_t sampleCount = 0;
  bool active = false;
  bool storedDataValid = false;
};

// Contract
// Role: UART0 guided calibration, 12 steps: black/red/blue x (level all 4,
//   lifted all 4, pressed front pair, pressed rear pair). Yellow is not
//   calibrated. Saves through CalibrationStore. Not linked into the match firmware.
// Called from: the bench firmware (bench_v2_app.cpp), which owns the UART0
//   console, passes calibration lines to command() and calls tick() every
//   loop; it brakes the motors for as long as active() is true.
//   update() reads UART0 itself for a host that has no console of its own.
// Rule: call CalibrationStore::load() before begin(). begin() prints nothing.
// Output: Serial.printf prompts; they may wait for UART FIFO room, which is
//   acceptable only because the motors are braked while calibrating.
class ColorCalibration {
 public:
  static constexpr uint8_t STEP_COUNT = 12;
  static void begin();
  static void update(uint32_t nowMs);  // pollSerial + tick
  static void command(const char* line, uint32_t nowMs);  // one console line
  static void tick(uint32_t nowMs);     // countdown / measuring progress

  static bool active();
  static ColorCalibrationStatus status();
};
