#pragma once
#include "Arduino.h"
#define NEO_GRB 0
#define NEO_KHZ800 0
struct Adafruit_NeoPixel {
  Adafruit_NeoPixel(size_t, uint16_t, int) {}
  void begin() {}
  void clear() {}
  void show() {}
  void setPixelColor(size_t, uint32_t) {}
  uint32_t Color(uint8_t r, uint8_t g, uint8_t b) { return (r<<16) | (g<<8) | b; }
};
