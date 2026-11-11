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
// UART1 camera source, set by the build environment (communication/vision_link.hpp):
// 0 = Raspberry Pi (V2 robot), 1 = HuskyLens (old V1 robot, 11/10). With the
// HuskyLens the ESP->Pi run log is off, because UART1 TX goes to the HuskyLens.
#ifndef VISION_HUSKYLENS
#define VISION_HUSKYLENS 0
#endif
constexpr bool VISION_FROM_HUSKYLENS = VISION_HUSKYLENS != 0;
}  // namespace uart

namespace switches {
constexpr uint32_t DEBOUNCE_MS = 40;
// 11/04: the robot's own slams pressed SW1/SW2, so a button acts only after
// being held this long (SW1 stop, SW2 start, SW2 stop while running).
constexpr uint32_t HOLD_MS = 500;
constexpr uint32_t ESTOP_LOG_PERIOD_MS = 3000;
// 11/11 (학현): SW1 and SW2 both start while stopped, both stop while running,
// and both held HOLD_MS together leave the emergency stop (soft reset). The
// two presses count as "together" when they start at most this far apart.
constexpr uint32_t DUAL_PRESS_WINDOW_MS = 300;
}  // namespace switches

namespace led {
constexpr uint32_t UPDATE_PERIOD_MS = 20;
// V2 has only this LED; 32 was too dim to read under field lights (not yet seen on the board).
constexpr uint8_t INTERNAL_BRIGHTNESS = 96;
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
// n x [CELL_MIN_MV, CELL_MAX_MV] for some n in CELL_COUNT_MIN..CELL_COUNT_MAX
// (3S and 4S LiPo both used), OK_SAMPLES_REQUIRED samples in a row within
// CHECK_WINDOW_MS, or the robot will not start.
// Divider and thresholds are not yet verified on the V2 board.
namespace battery {
constexpr uint32_t CELL_COUNT_MIN = 3;
constexpr uint32_t CELL_COUNT_MAX = 4;
constexpr uint32_t CELL_MIN_MV = 3300;   // Below: empty or fewer cells.
constexpr uint32_t CELL_MAX_MV = 4250;   // Above: more cells than expected.
constexpr uint32_t MISSING_BELOW_MV = 5000;  // Pack below this = no battery.
constexpr uint32_t DIVIDER_RATIO = 11;   // 100 kOhm / 10 kOhm on V2.
constexpr uint32_t CHECK_WINDOW_MS = 3000;
constexpr uint32_t SAMPLE_PERIOD_MS = 20;
constexpr uint8_t OK_SAMPLES_REQUIRED = 5;
}  // namespace battery

}  // namespace config
