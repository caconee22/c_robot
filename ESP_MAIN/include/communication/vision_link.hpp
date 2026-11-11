#pragma once

#include "communication/huskylens_link.hpp"
#include "communication/raspberry_link.hpp"
#include "config.hpp"

// Contract
// Role: picks the camera source for UART1 at build time so main.cpp, the
//   bench and BootManager do not care which one is fitted:
//   VISION_HUSKYLENS=0 (default) Raspberry Pi packets (V2 robot),
//   VISION_HUSKYLENS=1 HuskyLens color blocks (old V1 robot, 11/10).
//   Both give the same VisionSnapshot. Only one is started (they share UART1).
// Called from: BootManager::begin (begin), loop / bench (poll, latest, status).
namespace VisionLink {
inline void begin() {
  if (config::uart::VISION_FROM_HUSKYLENS) HuskyLensLink::begin(); else RaspberryLink::begin();
}
inline void poll() {
  if (config::uart::VISION_FROM_HUSKYLENS) HuskyLensLink::poll(); else RaspberryLink::poll();
}
inline VisionSnapshot latest() {
  return config::uart::VISION_FROM_HUSKYLENS ? HuskyLensLink::latest() : RaspberryLink::latest();
}
inline RaspberryLinkStatus status() {
  return config::uart::VISION_FROM_HUSKYLENS ? HuskyLensLink::status() : RaspberryLink::status();
}
}  // namespace VisionLink
