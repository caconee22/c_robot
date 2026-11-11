#include "app/boot_mode.hpp"

#include "hardware/board_io.hpp"
#include "pins.hpp"

namespace {
bool bothPressed() {
  return digitalRead(pins::SAFETY_SWITCH) == LOW && digitalRead(pins::START_SWITCH) == LOW;
}
}  // namespace

bool BootMode::benchRequested() {
  BoardIo::setEarlySafeState();   // motors off before anything waits
  pinMode(pins::SAFETY_SWITCH, INPUT_PULLUP);
  pinMode(pins::START_SWITCH, INPUT_PULLUP);
  delay(config::boot_mode::POLL_MS);   // let the pull-ups settle
  const uint32_t startMs = millis();
  while (bothPressed()) {
    if (millis() - startMs >= config::boot_mode::BENCH_HOLD_MS) return true;
    delay(config::boot_mode::POLL_MS);
  }
  return false;
}
