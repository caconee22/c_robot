#pragma once

#include <Arduino.h>

struct ButtonStatus {
  bool emergencyLatched = false;
  bool startPressed = false;
  bool startPressQualified = false;
  uint32_t pendingStartEvents = 0;
  uint32_t lastStartChangeMs = 0;
};

class ButtonInterrupts {
 public:
  static void begin();
  static void update(uint32_t nowMs);

  static bool emergencyLatched();
  static bool consumeEmergencyEvent();
  static bool consumeStartEvent();
  static bool startPressed();
  // Discards the current SW2 gesture. Used when calibration owns the robot.
  static void cancelStartGesture();
  static ButtonStatus status();

  // Permanently stops normal execution. Recovery requires a hardware reset.
  [[noreturn]] static void enterEmergencyStopLoop();
};
