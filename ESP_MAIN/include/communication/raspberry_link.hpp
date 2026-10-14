#pragma once

#include "robot_types.hpp"

struct RaspberryLinkStatus {
  bool initialized = false;
  bool linkValid = false;
  bool frameFresh = false;
  uint8_t lastSequence = 0;
  uint32_t lastPacketMs = 0;
  uint32_t lastFrameMs = 0;
  uint32_t validPackets = 0;
  uint32_t invalidPackets = 0;
  uint32_t duplicatePackets = 0;
};

// Contract
// Role: parse the fixed 14-byte UART1 packet (vision_protocol.hpp, no CRC by
//   design) into VisionSnapshot with linkValid / frameFresh.
// Called from: poll() once per loop pass, at most
//   config::uart::RASPBERRY_MAX_BYTES_PER_POLL bytes, never blocks.
// Failure: missing Pi is expected; it only clears linkValid/frameFresh and
//   never becomes a system fault.
class RaspberryLink {
 public:
  static void begin();
  static void poll();
  static VisionSnapshot latest();
  static RaspberryLinkStatus status();
};
