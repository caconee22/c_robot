#pragma once

// Contract
// Role: drive PWM/EN pins LOW as early as possible, then set pin modes.
// Called from: BootManager only. Pins come from pins.hpp (V1, do not change).
// Safety code: yes.
class BoardIo {
 public:
  static void setEarlySafeState();
  static void configurePins();
};
