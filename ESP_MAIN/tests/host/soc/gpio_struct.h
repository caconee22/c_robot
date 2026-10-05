#pragma once
#include "Arduino.h"
struct ClearRegister {
  int base;
  void operator=(uint32_t mask) {
    for (int i = 0; i < 32; ++i) if (mask & (1U << i)) fake::pins[base+i] = LOW;
  }
};
struct FakeGpio {
  ClearRegister out_w1tc{0};
  struct { ClearRegister val{32}; } out1_w1tc;
};
extern FakeGpio GPIO;
