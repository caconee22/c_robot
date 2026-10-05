#include "sensors/sensor_manager.hpp"

#include <Arduino.h>
#include <string.h>
#include "sensors/color_calibration.hpp"
#include "sensors/color_sensor_manager.hpp"
#include "sensors/floor_color_classifier.hpp"
#include "sensors/sensor_config.hpp"
#include "system/event_logger.hpp"

namespace {
SensorSnapshot latestSnapshot;
uint32_t lastColorSequence[COLOR_SENSOR_COUNT] = {};

struct ColorConfirmation {
  FloorColor candidate = FloorColor::Unknown;
  uint8_t count = 0;
};

ColorConfirmation confirmations[COLOR_SENSOR_COUNT];
uint32_t calibrationGeneration = 0;
ColorSensorError reportedError[COLOR_SENSOR_COUNT] = {};

FaultCode faultFor(ColorSensorError error) {
  if (error == ColorSensorError::Initialization) return FaultCode::Initialization;
  if (error == ColorSensorError::Timeout) return FaultCode::Timeout;
  if (error == ColorSensorError::Range) return FaultCode::Range;
  return FaultCode::Communication;
}

void reportErrors() {
  const auto status = ColorSensorManager::status();
  for (uint8_t i = 0; i < COLOR_SENSOR_COUNT; ++i) {
    if (status.error[i] == reportedError[i]) continue;
    if (reportedError[i] != ColorSensorError::None) {
      EventLogger::faultChanged(LogSource::Sensor, faultFor(reportedError[i]), false, i + 1);
    }
    if (status.error[i] != ColorSensorError::None) {
      EventLogger::faultChanged(LogSource::Sensor, faultFor(status.error[i]), true, i + 1);
    }
    reportedError[i] = status.error[i];
  }
}

FloorColorResult confirmColor(uint8_t sensorIndex,
                              const FloorColorResult& measured) {
  ColorConfirmation& filter = confirmations[sensorIndex];
  if (!measured.reliable || measured.color == FloorColor::Unknown) {
    filter = ColorConfirmation{};
    return FloorColorResult{};
  }

  if (filter.candidate == measured.color) {
    if (filter.count < UINT8_MAX) ++filter.count;
  } else {
    filter.candidate = measured.color;
    filter.count = 1;
  }

  if (filter.count < config::sensors::COLOR_CONFIRM_COUNT) {
    FloorColorResult pending;
    pending.timestampMs = measured.timestampMs;
    pending.brightness = measured.brightness;
    pending.consecutiveMatches = filter.count;
    return pending;
  }

  FloorColorResult confirmed = measured;
  confirmed.consecutiveMatches = filter.count;
  return confirmed;
}
}

bool SensorManager::begin() {
  latestSnapshot = SensorSnapshot{};
  memset(lastColorSequence, 0, sizeof(lastColorSequence));
  for (ColorConfirmation& confirmation : confirmations) {
    confirmation = ColorConfirmation{};
  }
  ColorSensorManager::begin();
  ColorCalibration::begin();
  calibrationGeneration = ColorCalibration::generation();
  for (auto& error : reportedError) error = ColorSensorError::None;
  reportErrors();
  latestSnapshot.colorSensorReadyMask =
      ColorSensorManager::status().readyMask;
  latestSnapshot.colorCalibrationValid =
      ColorCalibration::hasStoredData();
  // Color sensors are optional at boot. Missing sensors remain UNKNOWN and
  // are logged once. Failed channels require reboot after a wiring repair.
  return true;
}

void SensorManager::update() {
  ColorSensorManager::update(millis());
  // I2C transfers take real time: a just-published sample must not appear
  // to be from the future when testing its age with unsigned subtraction.
  const uint32_t nowMs = millis();
  latestSnapshot.timestampMs = nowMs;
  reportErrors();
  if (calibrationGeneration != ColorCalibration::generation()) {
    calibrationGeneration = ColorCalibration::generation();
    for (size_t i = 0; i < COLOR_SENSOR_COUNT; ++i) {
      confirmations[i] = ColorConfirmation{};
      latestSnapshot.floorColor[i] = FloorColorResult{};
      lastColorSequence[i] = ColorSensorManager::sample(i).sequence;
    }
  }
  latestSnapshot.colorSensorReadyMask =
      ColorSensorManager::status().readyMask;
  latestSnapshot.colorCalibrationValid =
      ColorCalibration::hasStoredData();
  for (uint8_t index = 0; index < COLOR_SENSOR_COUNT; ++index) {
    const ColorRawSample raw = ColorSensorManager::sample(index);
    latestSnapshot.colorRaw[index] = raw;
    if (!raw.valid || nowMs - raw.timestampMs > config::sensors::SAMPLE_STALE_MS) {
      confirmations[index] = ColorConfirmation{};
      latestSnapshot.floorColor[index] = FloorColorResult{};
      lastColorSequence[index] = raw.sequence;
      continue;
    }
    if (raw.sequence == lastColorSequence[index]) {
      continue;
    }
    lastColorSequence[index] = raw.sequence;
    const FloorColorResult measured = FloorColorClassifier::classify(
        index, raw, ColorCalibration::data());
    latestSnapshot.floorColor[index] = confirmColor(index, measured);
  }
}

SensorSnapshot SensorManager::snapshot() { return latestSnapshot; }
