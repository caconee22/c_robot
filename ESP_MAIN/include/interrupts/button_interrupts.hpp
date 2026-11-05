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
// Role: SW1 held LOW for config::switches::HOLD_MS latches a permanent drive
//   inhibit and cuts BTS7960 EN (11/04: shorter presses came from the robot's
//   own impacts). The SW1 ISR only marks edges so a bounce restarts the hold.
//   SW2 edges are debounced in update() (DEBOUNCE_MS); a press held HOLD_MS
//   then released produces one start event.
// Called from: ISR (internal), update() from SystemManager, pollEmergencyHold()
//   also from the motor watchdog task so SW1 still works if the loop stalls.
//   SystemManager is the only consumer of start events and turns a held SW2
//   into triggerEmergencyStop() while running.
// Safety code: yes. SW1 latch is cleared only by reboot.
class ButtonInterrupts {
 public:
  static void begin();
  static void update(uint32_t nowMs);

  static bool emergencyLatched();
  static bool consumeEmergencyEvent();
  static bool consumeStartEvent();
  static bool startPressed();
  // How long SW2 has been stably pressed, 0 when released.
  static uint32_t startHeldMs(uint32_t nowMs);
  // SW1 hold check. Safe from the main loop and the motor watchdog task.
  static void pollEmergencyHold(uint32_t nowMs);
  // Same permanent drive inhibit/EN cut as SW1, normal context only.
  static void triggerEmergencyStop();
  // Discards the current SW2 gesture. Used when calibration owns the robot.
  static void cancelStartGesture();
  static ButtonStatus status();

#if defined(BENCH_SOFTWARE_SWITCHES) && BENCH_SOFTWARE_SWITCHES
  // Bench-only input injection; physical LOW always remains effective.
  // SW2 still passes through the existing debounce and SystemManager.
  static void benchSetStartPressed(bool pressed);
  // Same permanent drive inhibit/EN cut as physical SW1, normal context only.
  static void benchEmergencyStop();
#endif

  // Permanently stops normal execution. Recovery requires a hardware reset.
  [[noreturn]] static void enterEmergencyStopLoop();
};
