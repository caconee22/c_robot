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

// Power-on battery check (V2 only, BatteryMonitor). Pack voltage must stay in
// CELL_COUNT x [CELL_MIN_MV, CELL_MAX_MV] for OK_SAMPLES_REQUIRED samples in a
// row within CHECK_WINDOW_MS, or the robot will not start. Default: 4S LiPo.
// Divider and thresholds are not yet verified on the V2 board.
namespace battery {
constexpr uint32_t CELL_COUNT = 4;
constexpr uint32_t CELL_MIN_MV = 3300;   // Below: empty or fewer cells.
constexpr uint32_t CELL_MAX_MV = 4250;   // Above: more cells than expected.
constexpr uint32_t MISSING_BELOW_MV = 5000;  // Pack below this = no battery.
constexpr uint32_t DIVIDER_RATIO = 11;   // 100 kOhm / 10 kOhm on V2.
constexpr uint32_t CHECK_WINDOW_MS = 3000;
constexpr uint32_t SAMPLE_PERIOD_MS = 20;
constexpr uint8_t OK_SAMPLES_REQUIRED = 5;
}  // namespace battery

}  // namespace config
