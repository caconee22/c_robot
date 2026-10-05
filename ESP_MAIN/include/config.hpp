#pragma once

#include <Arduino.h>

namespace config {

namespace uart {
constexpr uint32_t EVENT_LOG_BAUD = 57600;
constexpr uint32_t RASPBERRY_BAUD = 115200;
constexpr uint32_t RASPBERRY_TIMEOUT_MS = 120;
constexpr uint32_t RASPBERRY_FRAME_STALE_MS = 120;
constexpr uint32_t RASPBERRY_INTER_BYTE_TIMEOUT_MS = 20;
constexpr size_t RASPBERRY_MAX_BYTES_PER_POLL = 128;
}  // namespace uart

namespace switches {
constexpr uint32_t DEBOUNCE_MS = 40;
constexpr uint32_t ESTOP_LOG_PERIOD_MS = 3000;
}  // namespace switches

namespace led {
constexpr uint32_t UPDATE_PERIOD_MS = 20;
constexpr uint8_t INTERNAL_BRIGHTNESS = 32;
constexpr uint8_t EXTERNAL_BRIGHTNESS = 64;
}  // namespace led

namespace event_log {
constexpr uint32_t DEFAULT_REPEAT_LIMIT_MS = 1000;
constexpr uint8_t MAX_RECORDS_PER_PROCESS = 2;
constexpr size_t QUEUE_SIZE = 32;
constexpr size_t TEXT_LENGTH = 40;
constexpr size_t TRACKER_SIZE = 32;
}  // namespace event_log

}  // namespace config
