#pragma once

#include <Arduino.h>

// Contract
// Role: tell the Pi why the ESP32 restarted (11/13, 학현: in the match the ESP
//   went dark mid-run and nothing recorded why). Every loop pass the drive
//   state is copied into RTC no-init RAM (a few bytes, no flash). At the next
//   boot that copy (when it survived the reset) and esp_reset_reason() go to
//   the Pi as a run-log Boot frame (RunLogKind::Boot), so the Pi writes the
//   reason and what the robot was doing just before it. The frame repeats
//   every REPEAT_MS until the first start, because the Pi may still be booting
//   (it shares the power that just dropped); a random boot id lets the Pi
//   record each boot once.
//   RTC RAM survives software, panic, watchdog and (on the ESP-IDF brownout
//   interrupt path) brownout restarts, but not a power cut: then only the
//   reason is sent (no previous state). Not verified on the real board.
// Called from: setup() (begin, after BootManager so UART1 and the reset reason
//   are ready; armWatchdog at the end) and loop() (feedWatchdog, update,
//   record every pass; plain RAM writes). SystemManager calls noteStopped when
//   the emergency stop latches.
// Auto resume (11/13, 학현): a watchdog reset starts the run (11/13 2nd: also
//   when waiting or the RTC copy was lost; not after an emergency stop)
//   at once, skipping the opening move. Health checks and the battery
//   start hold still apply (SystemManager). Not for power-on, reset button,
//   brownout or panic. Safety code: the resume decision, yes.
// Failure: Pi missing -> the frame is simply lost (run log policy). No flash,
//   never touches motors or safety. Safety code: no.

namespace config {
namespace reset_report {
constexpr uint32_t REPEAT_MS = 2000;   // Boot frame repeat until the first start
// 11/13 (학현): loop/idle task watchdog shortened from the 5 s default, and a
// watchdog restart while driving starts the run again at once (no button).
constexpr uint32_t WDT_TIMEOUT_S = 1;
constexpr bool RESUME_AFTER_WATCHDOG = true;
}  // namespace reset_report
}  // namespace config

namespace ResetReport {
// esp_reset_reason_t values that are not a normal power-on or reset button:
// software restart, panic, interrupt/task/other watchdog, brownout.
bool abnormal(uint8_t reason);
// Reads and clears the RTC copy, then queues the Boot frame.
void begin(uint8_t reason, uint32_t nowMs);
// Every loop pass: repeats the Boot frame every REPEAT_MS until started
// (motors allowed for the first time), then never again this boot.
void update(uint32_t nowMs, bool started);
void record(uint32_t nowMs, bool running, uint8_t robotState, uint8_t intentType, uint16_t duty);
// Emergency stop latched: the RTC copy says "not driving", so a later
// watchdog restart never resumes a run that was stopped on purpose.
void noteStopped();
// True when this boot came from a watchdog reset (interrupt/task/other WDT),
// unless the RTC copy says an emergency stop was latched (11/13 2nd: also when
// it was waiting or the copy was lost): main() then starts the run without a
// button (SystemManager::requestAutoStart).
bool resumeRequested();
// Match firmware only: task watchdog WDT_TIMEOUT_S with panic (reset +
// core dump), the loop task subscribed. feedWatchdog() once per loop pass.
void armWatchdog();
void feedWatchdog();
}  // namespace ResetReport
