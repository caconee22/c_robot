#include "system/battery_monitor.hpp"

#include "config.hpp"
#include "pins.hpp"
#include "system/event_logger.hpp"
#include "system/system_manager.hpp"

namespace {

BatteryStatus current;
#if HW_VERSION == 2
uint32_t checkStartedMs = 0;
uint32_t lastSampleMs = 0;
bool sampled = false;
uint8_t lastCells = 0;

constexpr uint32_t packMaxMv() {
  return config::battery::CELL_COUNT_MAX * config::battery::CELL_MAX_MV;
}

// Cell count whose per-cell range holds the pack voltage, or 0 if none does
// (e.g. 12.8-13.1 V: too high for 3S, too low for 4S).
uint8_t cellsFor(uint32_t packMv) {
  for (uint32_t n = config::battery::CELL_COUNT_MIN; n <= config::battery::CELL_COUNT_MAX; ++n) {
    if (packMv >= n * config::battery::CELL_MIN_MV && packMv <= n * config::battery::CELL_MAX_MV)
      return static_cast<uint8_t>(n);
  }
  return 0;
}

void finish(BatteryCheckResult result) {
  current.result = result;
  if (result == BatteryCheckResult::Ok) SystemManager::holdStart(false);
  const int32_t mv = static_cast<int32_t>(current.packMillivolts);
  EventLogger::logText(static_cast<uint16_t>(EventId::Custom),
                       result == BatteryCheckResult::Ok ? LogSeverity::Info
                                                        : LogSeverity::Error,
                       LogSource::System, BatteryMonitor::resultName(result), mv,
                       current.cells, 0);
  if (result == BatteryCheckResult::Ok) return;
  const FaultCode fault = result == BatteryCheckResult::Missing
                              ? FaultCode::Disconnected
                          : result == BatteryCheckResult::High
                              ? FaultCode::OverLimit
                              : FaultCode::Range;
  SystemManager::setApplicationHealthy(false, fault, mv);
}
#endif  // HW_VERSION == 2

}  // namespace

void BatteryMonitor::begin(uint32_t nowMs) {
  current = BatteryStatus{};
#if HW_VERSION == 2
  analogSetPinAttenuation(pins::BATTERY_ADC, ADC_11db);
  current.result = BatteryCheckResult::Checking;
  SystemManager::holdStart(true);  // No SW2 start before the pack is judged.
  checkStartedMs = nowMs;
  sampled = false;
  lastCells = 0;
#else
  (void)nowMs;  // V1 has no battery voltage input.
#endif
}

void BatteryMonitor::update(uint32_t nowMs) {
#if HW_VERSION == 2
  if (current.result != BatteryCheckResult::Checking) return;
  if (sampled && nowMs - lastSampleMs < config::battery::SAMPLE_PERIOD_MS) return;
  sampled = true;
  lastSampleMs = nowMs;

  const uint32_t pinMv = analogReadMilliVolts(pins::BATTERY_ADC);
  current.packMillivolts = pinMv * config::battery::DIVIDER_RATIO;
  // Consecutive samples must agree on the same cell count.
  const uint8_t cells = cellsFor(current.packMillivolts);
  current.okSamples = cells && cells == lastCells ? current.okSamples + 1 : (cells ? 1 : 0);
  lastCells = cells;

  if (current.okSamples >= config::battery::OK_SAMPLES_REQUIRED) {
    current.cells = cells;
    finish(BatteryCheckResult::Ok);
    return;
  }
  if (nowMs - checkStartedMs < config::battery::CHECK_WINDOW_MS) return;
  if (current.packMillivolts < config::battery::MISSING_BELOW_MV) {
    finish(BatteryCheckResult::Missing);
  } else if (current.packMillivolts > packMaxMv()) {
    finish(BatteryCheckResult::High);
  } else {
    finish(BatteryCheckResult::Low);
  }
#else
  (void)nowMs;
#endif
}

BatteryStatus BatteryMonitor::status() { return current; }

const char* BatteryMonitor::resultName(BatteryCheckResult result) {
  switch (result) {
    case BatteryCheckResult::NotAvailable: return "battery_not_checked";
    case BatteryCheckResult::Checking: return "battery_checking";
    case BatteryCheckResult::Ok: return "battery_ok";
    case BatteryCheckResult::Missing: return "battery_missing";
    case BatteryCheckResult::Low: return "battery_low_or_wrong";
    case BatteryCheckResult::High: return "battery_too_high";
    default: return "battery_unknown";
  }
}
