#include "communication/raspberry_link.hpp"

#include <Arduino.h>
#include <string.h>

#include "communication/vision_protocol.hpp"
#include "config.hpp"
#include "pins.hpp"
#include "system/event_logger.hpp"

namespace {

using vision_protocol::PacketError;

uint8_t receiveBuffer[vision_protocol::FRAME_LENGTH] = {};
size_t receiveCount = 0;
uint32_t lastByteMs = 0;
uint8_t lastPayload[vision_protocol::PAYLOAD_LENGTH] = {};
bool lastPayloadKnown = false;
bool sequenceKnown = false;
VisionSnapshot latestVision;
RaspberryLinkStatus linkStatus;

uint16_t readLe16(const uint8_t* data) {
  return static_cast<uint16_t>(data[0] |
                               (static_cast<uint16_t>(data[1]) << 8U));
}

void invalid(PacketError error, int32_t detail = 0) {
  ++linkStatus.invalidPackets;
  EventLogger::log(EventId::InvalidPacket, static_cast<int32_t>(error),
                   detail);
}

PacketError validate(const uint8_t* payload) {
  const uint8_t flags = payload[1];
  const bool target = (flags & vision_protocol::TargetValid) != 0;
  const bool targetFlags =
      (flags & (vision_protocol::TrackStable |
                vision_protocol::MultipleTargets |
                vision_protocol::BoxClipped)) != 0;
  const uint16_t x = readLe16(payload + 2);
  const uint16_t y = readLe16(payload + 4);
  const uint16_t width = readLe16(payload + 6);
  const uint16_t height = readLe16(payload + 8);

  if ((flags & ~vision_protocol::KNOWN_STATUS_MASK) != 0) {
    return PacketError::ReservedStatusBits;
  }
  if (!target) {
    if (targetFlags) return PacketError::TargetStatusConflict;
    return (x == 0 && y == 0 && width == 0 && height == 0)
               ? PacketError::None
               : PacketError::NoTargetDataNonZero;
  }
  if ((flags & vision_protocol::CameraOk) == 0 ||
      (flags & vision_protocol::PipelineOk) == 0) {
    return PacketError::TargetStatusConflict;
  }
  if (x >= vision_protocol::FRAME_WIDTH ||
      y >= vision_protocol::FRAME_HEIGHT) {
    return PacketError::CoordinateRange;
  }
  if (width == 0 || height == 0 || width > vision_protocol::FRAME_WIDTH ||
      height > vision_protocol::FRAME_HEIGHT) {
    return PacketError::BoxSizeRange;
  }
  return PacketError::None;
}

void accept(const uint8_t* payload, uint32_t nowMs) {
  if (sequenceKnown &&
      nowMs - linkStatus.lastPacketMs > config::uart::RASPBERRY_TIMEOUT_MS) {
    sequenceKnown = false;
    lastPayloadKnown = false;
  }
  const uint8_t sequence = payload[0];
  if (sequenceKnown && sequence == linkStatus.lastSequence) {
    if (!lastPayloadKnown ||
        memcmp(payload, lastPayload, vision_protocol::PAYLOAD_LENGTH) != 0) {
      invalid(PacketError::SequenceReuse, sequence);
      return;
    }
    ++linkStatus.validPackets;
    ++linkStatus.duplicatePackets;
    linkStatus.lastPacketMs = nowMs;
    latestVision.receivedMs = nowMs;
    return;
  }

  const uint8_t flags = payload[1];
  latestVision.receivedMs = nowMs;
  latestVision.frameUpdatedMs = nowMs;
  latestVision.sequence = sequence;
  latestVision.statusBits = flags;
  latestVision.targetX = readLe16(payload + 2);
  latestVision.targetY = readLe16(payload + 4);
  latestVision.targetWidth = readLe16(payload + 6);
  latestVision.targetHeight = readLe16(payload + 8);
  latestVision.targetValid = (flags & vision_protocol::TargetValid) != 0;
  latestVision.trackStable = (flags & vision_protocol::TrackStable) != 0;
  latestVision.multipleTargets =
      (flags & vision_protocol::MultipleTargets) != 0;
  latestVision.boxClipped = (flags & vision_protocol::BoxClipped) != 0;
  latestVision.cameraOk = (flags & vision_protocol::CameraOk) != 0;
  latestVision.pipelineOk = (flags & vision_protocol::PipelineOk) != 0;

  sequenceKnown = true;
  linkStatus.lastSequence = sequence;
  linkStatus.lastPacketMs = nowMs;
  linkStatus.lastFrameMs = nowMs;
  ++linkStatus.validPackets;
  memcpy(lastPayload, payload, vision_protocol::PAYLOAD_LENGTH);
  lastPayloadKnown = true;
}

void resynchronize() {
  for (size_t i = 1; i < receiveCount; ++i) {
    if (receiveBuffer[i] != vision_protocol::HEADER_1) continue;
    const size_t remaining = receiveCount - i;
    if (remaining >= 2 && receiveBuffer[i + 1] != vision_protocol::HEADER_2) continue;
    if (remaining >= 3 && receiveBuffer[i + 2] != vision_protocol::VERSION) continue;
    if (remaining >= 4 && receiveBuffer[i + 3] != vision_protocol::PAYLOAD_LENGTH) continue;
    memmove(receiveBuffer, receiveBuffer + i, remaining);
    receiveCount = remaining;
    return;
  }
  receiveCount = 0;
}

void completeFrame(uint32_t nowMs) {
  const uint8_t* payload = receiveBuffer + 4;
  const PacketError error = validate(payload);
  if (error == PacketError::None) {
    accept(payload, nowMs);
    receiveCount = 0;
  } else {
    invalid(error);
    resynchronize();
  }
}

void consumeByte(uint8_t value, uint32_t nowMs) {
  lastByteMs = nowMs;
  if (receiveCount == 0) {
    if (value == vision_protocol::HEADER_1) {
      receiveBuffer[receiveCount++] = value;
    }
    return;
  }
  if (receiveCount == 1) {
    if (value == vision_protocol::HEADER_2) {
      receiveBuffer[receiveCount++] = value;
    } else if (value != vision_protocol::HEADER_1) {
      receiveCount = 0;
    }
    return;
  }

  receiveBuffer[receiveCount++] = value;
  if (receiveCount == 3 && receiveBuffer[2] != vision_protocol::VERSION) {
    invalid(PacketError::Version, receiveBuffer[2]);
    resynchronize();
  } else if (receiveCount == 4 &&
             receiveBuffer[3] != vision_protocol::PAYLOAD_LENGTH) {
    invalid(PacketError::Length, receiveBuffer[3]);
    resynchronize();
  } else if (receiveCount == vision_protocol::FRAME_LENGTH) {
    completeFrame(nowMs);
  }
}

void updateAge(uint32_t nowMs, VisionSnapshot& vision,
               RaspberryLinkStatus* status = nullptr) {
  const bool linkValid = linkStatus.validPackets > 0 &&
      nowMs - linkStatus.lastPacketMs <= config::uart::RASPBERRY_TIMEOUT_MS;
  const bool frameFresh = sequenceKnown &&
      nowMs - linkStatus.lastFrameMs <= config::uart::RASPBERRY_FRAME_STALE_MS;

  vision.linkValid = linkValid;
  vision.frameFresh = frameFresh;
  if (!linkValid || !frameFresh) {
    vision.targetValid = false;
    vision.trackStable = false;
  }
  if (status != nullptr) {
    status->linkValid = linkValid;
    status->frameFresh = frameFresh;
  }
}

}  // namespace

void RaspberryLink::begin() {
  Serial1.begin(config::uart::RASPBERRY_BAUD, SERIAL_8N1, pins::UART1_RX,
                pins::UART1_TX);
  latestVision = VisionSnapshot{};
  linkStatus = RaspberryLinkStatus{};
  linkStatus.initialized = true;
  receiveCount = 0;
  lastByteMs = 0;
  lastPayloadKnown = false;
  sequenceKnown = false;
}

void RaspberryLink::poll() {
  uint32_t nowMs = millis();
  if (receiveCount > 0 &&
      nowMs - lastByteMs > config::uart::RASPBERRY_INTER_BYTE_TIMEOUT_MS) {
    invalid(PacketError::InterByteTimeout,
            static_cast<int32_t>(receiveCount));
    receiveCount = 0;
  }

  size_t processed = 0;
  while (processed < config::uart::RASPBERRY_MAX_BYTES_PER_POLL &&
         Serial1.available() > 0) {
    const int value = Serial1.read();
    if (value < 0) break;
    consumeByte(static_cast<uint8_t>(value), millis());
    ++processed;
  }
  VisionSnapshot vision = latest();
  EventLogger::valueChanged(EventId::VisionStatusChanged, LogSource::Communication,
                           (vision.linkValid ? 1 : 0) | (vision.frameFresh ? 2 : 0));
}

VisionSnapshot RaspberryLink::latest() {
  VisionSnapshot result = latestVision;
  updateAge(millis(), result);
  return result;
}

RaspberryLinkStatus RaspberryLink::status() {
  RaspberryLinkStatus result = linkStatus;
  VisionSnapshot vision = latestVision;
  updateAge(millis(), vision, &result);
  return result;
}
