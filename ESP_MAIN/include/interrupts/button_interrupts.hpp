#pragma once

#include <Arduino.h>

struct ButtonStatus {
  bool emergencyLatched = false;
  bool startPressed = false;
  bool startPressQualified = false;
  uint32_t pendingStartEvents = 0;
  uint32_t lastStartChangeMs = 0;
};

// Contract
// Role: SW1 ISR latches a permanent drive inhibit and cuts BTS7960 EN at once;
//   SW2 edges are debounced in update() (config::switches::DEBOUNCE_MS) and
//   one start event is produced on a qualified press then release.
// Called from: ISR (internal), update() from SystemManager. SystemManager is
//   the only consumer of start events.
// Safety code: yes. SW1 latch is cleared only by reboot.
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
