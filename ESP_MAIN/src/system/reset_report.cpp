#include "system/reset_report.hpp"

#include <esp_attr.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <stddef.h>

#include "communication/run_log.hpp"

namespace {
constexpr uint32_t MAGIC = 0xC0B07E13;

struct LastState {
  uint32_t magic;
  uint32_t uptimeMs;
  uint16_t duty;
  uint8_t robotState;
  uint8_t intentType;
  uint8_t running;
  uint8_t stopped;   // emergency stop latched (noteStopped), cleared by the next loop pass
  uint8_t check;   // XOR of the bytes above, so power-on garbage is not trusted
};

RTC_NOINIT_ATTR LastState last;

// This boot's frame, repeated every REPEAT_MS until the first start, so a Pi
// that boots later (or restarts its receiver) still gets it.
uint16_t frame[4] = {};
bool repeating = false;
uint32_t lastSentMs = 0;
bool resume = false;

uint8_t checksum(const LastState& s) {
  const auto* p = reinterpret_cast<const uint8_t*>(&s);
  uint8_t x = 0x5A;
  for (size_t i = 0; i < offsetof(LastState, check); ++i) x ^= p[i];
  return x;
}
}  // namespace

bool ResetReport::abnormal(uint8_t reason) {
  // esp_reset_reason_t: 3 SW, 4 PANIC, 5 INT_WDT, 6 TASK_WDT, 7 WDT, 9 BROWNOUT
  return reason == 3 || reason == 4 || reason == 5 || reason == 6 || reason == 7 || reason == 9;
}

void ResetReport::begin(uint8_t reason, uint32_t nowMs) {
  const bool valid = last.magic == MAGIC && last.check == checksum(last);
  // esp_reset_reason_t 5 INT_WDT, 6 TASK_WDT, 7 WDT
  // 11/13 (2nd, 학현): any watchdog reset starts driving at once, also when the
  // RTC copy was lost or it was waiting; only an emergency stop blocks it.
  resume = config::reset_report::RESUME_AFTER_WATCHDOG && !(valid && last.stopped) &&
           (reason == 5 || reason == 6 || reason == 7);
  // Boot frame: a reset reason | boot id << 8 (random, so the Pi records each
  // boot once even though the frame repeats), b previous robot state (+0x100 when it was
  // driving; 0xFFFF = no previous state), c previous uptime in 0.1 s (capped),
  // d previous drive intent << 12 | duty (0..4095).
  const uint16_t b = valid ? static_cast<uint16_t>(last.robotState | (last.running ? 0x100 : 0)) : 0xFFFF;
  const uint32_t tenths = valid ? last.uptimeMs / 100 : 0;
  const uint16_t c = tenths > 0xFFFF ? 0xFFFF : static_cast<uint16_t>(tenths);
  const uint16_t d = valid ? static_cast<uint16_t>((last.intentType & 0x0F) << 12 | (last.duty & 0x0FFF)) : 0;
  const uint16_t bootId = static_cast<uint16_t>(esp_random() & 0xFF);
  frame[0] = static_cast<uint16_t>(reason | bootId << 8);
  frame[1] = b;
  frame[2] = c;
  frame[3] = d;
  RunLog::push(RunLogKind::Boot, frame[0], frame[1], frame[2], frame[3]);
  repeating = true;
  lastSentMs = nowMs;
  last.magic = 0;   // the RTC copy is used for this boot only
}

void ResetReport::update(uint32_t nowMs, bool started) {
  if (!repeating) return;
  if (started) { repeating = false; return; }   // first start: the Pi had its chance
  if (nowMs - lastSentMs < config::reset_report::REPEAT_MS) return;
  lastSentMs = nowMs;
  RunLog::push(RunLogKind::Boot, frame[0], frame[1], frame[2], frame[3]);
}

void ResetReport::record(uint32_t nowMs, bool running, uint8_t robotState, uint8_t intentType, uint16_t duty) {
  last.uptimeMs = nowMs;
  last.duty = duty;
  last.robotState = robotState;
  last.intentType = intentType;
  last.running = running ? 1 : 0;
  last.stopped = 0;
  last.magic = MAGIC;
  last.check = checksum(last);
}

void ResetReport::noteStopped() {
  if (last.magic != MAGIC) return;
  last.running = 0;
  last.stopped = 1;
  last.check = checksum(last);
}

bool ResetReport::resumeRequested() { return resume; }

void ResetReport::armWatchdog() {
  esp_task_wdt_init(config::reset_report::WDT_TIMEOUT_S, true);   // reconfigures the running TWDT
  esp_task_wdt_add(nullptr);                                     // this (loop) task
}

void ResetReport::feedWatchdog() { esp_task_wdt_reset(); }
