#include "bench_v2/software_switches.hpp"
#if defined(BENCH_SOFTWARE_SWITCHES) && BENCH_SOFTWARE_SWITCHES
#include "config.hpp"
#include "interrupts/button_interrupts.hpp"
#include "pins.hpp"
#include "system/system_manager.hpp"

namespace {
bool pressing = false;
uint32_t pressedMs = 0;
constexpr uint32_t PRESS_TIMEOUT_MS = 250;
bool startHealthy() {
  const auto status = SystemManager::status();
  return !status.startHeld && status.applicationHealthy &&
         status.motorsHealthy && status.sensorsHealthy &&
         !ButtonInterrupts::emergencyLatched();
}
}

void BenchSoftwareSwitches::begin() {
  pressing = false;
  ButtonInterrupts::benchSetStartPressed(false);
}

bool BenchSoftwareSwitches::pressStart(uint32_t nowMs) {
  if (pressing || !startHealthy() || ButtonInterrupts::startPressed() ||
      digitalRead(pins::START_SWITCH) == LOW) return false;
  pressing = true;
  pressedMs = nowMs;
  ButtonInterrupts::benchSetStartPressed(true);
  return true;
}

void BenchSoftwareSwitches::update(uint32_t nowMs) {
  if (!startHealthy()) { cancelStart(); return; }
  if (pressing && nowMs - pressedMs >= PRESS_TIMEOUT_MS) {
    cancelStart();  // do not keep a start request pending across a long loop stall
    return;
  }
  if (pressing && nowMs - pressedMs >= config::switches::DEBOUNCE_MS + 20 &&
      ButtonInterrupts::status().startPressQualified) {
    pressing = false;
    ButtonInterrupts::benchSetStartPressed(false);
  }
}

void BenchSoftwareSwitches::cancelStart() {
  if (!pressing) return;
  pressing = false;
  ButtonInterrupts::benchSetStartPressed(false);
  ButtonInterrupts::cancelStartGesture();
}

void BenchSoftwareSwitches::emergencyStop() {
  cancelStart();
  ButtonInterrupts::benchEmergencyStop();
}
#endif
