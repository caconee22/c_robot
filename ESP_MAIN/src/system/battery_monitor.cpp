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

constexpr uint32_t packMinMv() {
  return config::battery::CELL_COUNT * config::battery::CELL_MIN_MV;
}
constexpr uint32_t packMaxMv() {
  return config::battery::CELL_COUNT * config::battery::CELL_MAX_MV;
}

void finish(BatteryCheckResult result) {
  current.result = result;
  const int32_t mv = static_cast<int32_t>(current.packMillivolts);
  EventLogger::logText(static_cast<uint16_t>(EventId::Custom),
                       result == BatteryCheckResult::Ok ? LogSeverity::Info
                                                        : LogSeverity::Error,
                       LogSource::System, BatteryMonitor::resultName(result), mv,
                       config::battery::CELL_COUNT, 0);
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
  checkStartedMs = nowMs;
  sampled = false;
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
  const bool inRange = current.packMillivolts >= packMinMv() &&
                       current.packMillivolts <= packMaxMv();
  current.okSamples = inRange ? current.okSamples + 1 : 0;

  if (current.okSamples >= config::battery::OK_SAMPLES_REQUIRED) {
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
