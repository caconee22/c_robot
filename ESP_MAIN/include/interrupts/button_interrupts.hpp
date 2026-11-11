#pragma once

#include <Arduino.h>

struct ButtonStatus {
  bool emergencyLatched = false;
  bool startPressed = false;         // SW2 debounced
  bool startPressQualified = false;  // SW2 press that may become a start
  bool safetyPressed = false;        // SW1 debounced
  bool dualGesture = false;          // both pressed: no start until both released
  bool resetArmed = false;           // in e-stop and both seen released
  uint32_t pendingStartEvents = 0;
  uint32_t lastStartChangeMs = 0;
};

// Contract
// Role (11/11, 학현 decision, replaces the permanent SW1 latch):
//   - Stopped: SW1 and SW2 are both start buttons. A press held HOLD_MS then
//     released gives one start event. A press that was already down at boot
//     (or after a reset) counts only after it was released once.
//   - Running: either button held HOLD_MS is an emergency stop. SW1 is also
//     checked by pollEmergencyHold() from the motor watchdog task, so it still
//     stops if the main loop stalls (armed only while running).
//   - Emergency stop: motors off, EN cut. Both buttons released, then both
//     pressed within DUAL_PRESS_WINDOW_MS and held HOLD_MS -> soft reset back
//     to "waiting for start". No hardware is re-initialized.
//   - Both pressed at once never starts the robot, and after a reset the
//     buttons must be released and pressed again for a start.
// Called from: ISR (edge marks only), update() from SystemManager and the
//   e-stop wait loop, pollEmergencyHold() also from the motor watchdog task.
// Safety code: yes.
class ButtonInterrupts {
 public:
  static void begin();
  static void update(uint32_t nowMs);

  static bool emergencyLatched();
  static bool consumeStartEvent();
  static bool startPressed();
  // How long SW2 has been stably pressed, 0 when released.
  static uint32_t startHeldMs(uint32_t nowMs);
  // Longest stable hold of SW1 or SW2 (running e-stop check).
  static uint32_t anyHeldMs(uint32_t nowMs);
  // SystemManager: true only while running. SW1 hold trips only when armed.
  static void setEmergencyArmed(bool armed);
  // SW1 hold check. Safe from the main loop and the motor watchdog task.
  static void pollEmergencyHold(uint32_t nowMs);
  // Emergency stop (SW2 / software / tilt): motors off, EN cut, latched until
  // the two-button soft reset. Normal context only.
  static void triggerEmergencyStop();
  // Discards the current start gesture. Used when calibration owns the robot.
  static void cancelStartGesture();
  static ButtonStatus status();

#if defined(BENCH_SOFTWARE_SWITCHES) && BENCH_SOFTWARE_SWITCHES
  // Bench-only input injection; physical LOW always remains effective.
  // SW2 still passes through the existing debounce and SystemManager.
  static void benchSetStartPressed(bool pressed);
  // Same emergency stop as a physical button while running, normal context only.
  static void benchEmergencyStop();
#endif

  // Emergency stop: brakes, cuts EN, prints ESTOP every 3 s and waits here
  // until the two-button soft reset, then clears the latch and returns.
  // The caller (SystemManager) resets the rest of the robot state.
  static void waitInEmergencyStop();
};
