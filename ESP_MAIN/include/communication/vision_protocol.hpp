#pragma once

#include <Arduino.h>

namespace vision_protocol {

constexpr uint8_t HEADER_1 = 0xAA;
constexpr uint8_t HEADER_2 = 0x55;
constexpr uint8_t VERSION = 0x01;
constexpr uint8_t PAYLOAD_LENGTH = 10;
constexpr size_t FRAME_LENGTH = 4 + PAYLOAD_LENGTH;

constexpr uint16_t FRAME_WIDTH = 1920;
constexpr uint16_t FRAME_HEIGHT = 1080;

enum StatusFlag : uint8_t {
  TargetValid = 1U << 0,
  TrackStable = 1U << 1,
  MultipleTargets = 1U << 2,
  BoxClipped = 1U << 3,
  CameraOk = 1U << 4,
  PipelineOk = 1U << 5,
};

constexpr uint8_t KNOWN_STATUS_MASK =
    TargetValid | TrackStable | MultipleTargets | BoxClipped | CameraOk |
    PipelineOk;

// Values are included in the InvalidPacket event as value1.
enum class PacketError : uint8_t {
  None = 0,
  Version = 1,
  Length = 2,
  ReservedStatusBits = 3,
  TargetStatusConflict = 4,
  NoTargetDataNonZero = 5,
  CoordinateRange = 6,
  BoxSizeRange = 7,
  SequenceReuse = 8,
  InterByteTimeout = 9,
};

}  // namespace vision_protocol
