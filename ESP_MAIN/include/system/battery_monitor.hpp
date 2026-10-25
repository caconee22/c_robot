#pragma once

#include <Arduino.h>

enum class BatteryCheckResult : uint8_t {
  NotAvailable,  // V1 board: no battery voltage input, check skipped.
  Checking,      // Within the power-on window, waiting for a good voltage.
  Ok,
  Missing,       // Pack voltage almost zero (no battery, USB power only).
  Low,           // No allowed cell count fits: empty pack or too few cells.
  High,          // Above the largest allowed pack: too many cells.
};

struct BatteryStatus {
  BatteryCheckResult result = BatteryCheckResult::NotAvailable;
  uint32_t packMillivolts = 0;  // Last measured pack voltage.
  uint8_t okSamples = 0;
  uint8_t cells = 0;             // Detected cell count once Ok, else 0.
};

// Contract
// Role: power-on battery check on the V2 board. Within CHECK_WINDOW_MS after
//   begin(), the pack voltage must fit one allowed cell count (3S or 4S)
//   for OK_SAMPLES_REQUIRED samples in a row. Otherwise the battery is
//   reported as missing / wrong and the robot is not allowed to start.
// Called from: setup() (begin) and loop() (update). One ADC read at most every
//   SAMPLE_PERIOD_MS; never waits.
// Failure: SystemManager::setApplicationHealthy(false, ...) once, with the pack
//   millivolts as detail. Motors stay stopped until reboot (existing policy).
//   After the check passes, the voltage is no longer watched.
// Safety code: no (it only reports through SystemManager).
class BatteryMonitor {
 public:
  static void begin(uint32_t nowMs);
  static void update(uint32_t nowMs);
  static BatteryStatus status();
  static const char* resultName(BatteryCheckResult result);
};
