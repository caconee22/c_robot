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

class RaspberryLink {
 public:
  static void begin();
  static void poll();
  static VisionSnapshot latest();
  static RaspberryLinkStatus status();
};
