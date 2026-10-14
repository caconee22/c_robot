#include "sensors/calibration_store.hpp"

#include <Preferences.h>
#include <cmath>

#include "sensors/color_sensor_manager.hpp"
#include "sensors/sensor_config.hpp"

namespace {
using std::isfinite;

constexpr uint32_t CALIBRATION_MAGIC = 0x43414C31UL;
constexpr uint16_t CALIBRATION_VERSION = 2;
constexpr char NVS_NAMESPACE[] = "color-cal";
constexpr char NVS_KEY[] = "data";

ColorCalibrationData storedData;
bool storedValid = false;
uint32_t dataGeneration = 0;

void clearData(ColorCalibrationData& value) {
  // Clear small pieces, not a whole 4.6 KB temporary on the task stack.
  value.magic = 0;
  value.version = 0;
  value.sensorModel = 0;
  value.sensorCount = 0;
  value.settingsSignature = 0;
  value.ratioMarginPermille = 0;
  value.brightnessMarginPermille = 0;
  for (auto& sensor : value.reference) {
    for (auto& color : sensor) {
      for (auto& reference : color) reference = ColorCalibrationReference{};
    }
  }
}

bool allReferencesValid(const ColorCalibrationData& value) {
  for (size_t sensor = 0; sensor < COLOR_SENSOR_COUNT; ++sensor) {
    for (size_t color = 0; color < FLOOR_COLOR_COUNT; ++color) {
      for (size_t pose = 0; pose < CALIBRATION_POSE_COUNT; ++pose) {
        const auto& ref = value.reference[sensor][color][pose];
        if (!ref.valid || ref.sampleCount < config::sensors::CALIBRATION_MIN_SAMPLES ||
            !isfinite(ref.brightnessMean) || !isfinite(ref.brightnessStdDev) ||
            ref.brightnessMean < 0 || ref.brightnessStdDev < 0 ||
            ref.brightnessMin > ref.brightnessMax) return false;
        for (size_t i = 0; i < COLOR_CHANNEL_COUNT; ++i) {
          if (!isfinite(ref.normalizedMean[i]) || !isfinite(ref.normalizedStdDev[i]) ||
              ref.normalizedMean[i] < 0 || ref.normalizedStdDev[i] < 0) return false;
        }
      }
    }
  }
  return true;
}

}  // namespace

bool CalibrationStore::isValid(const ColorCalibrationData& value) {
  return value.magic == CALIBRATION_MAGIC &&
         value.version == CALIBRATION_VERSION &&
         value.sensorModel ==
             static_cast<uint8_t>(ColorSensorManager::model()) &&
         value.sensorCount == COLOR_SENSOR_COUNT &&
         value.settingsSignature == ColorSensorManager::settingsSignature() &&
         value.ratioMarginPermille > 0 && value.ratioMarginPermille <= 1000 &&
         value.brightnessMarginPermille > 0 && value.brightnessMarginPermille <= 1000 &&
         allReferencesValid(value);
}

void CalibrationStore::initialize(ColorCalibrationData& value) {
  clearData(value);
  value.magic = CALIBRATION_MAGIC;
  value.version = CALIBRATION_VERSION;
  value.sensorModel = static_cast<uint8_t>(ColorSensorManager::model());
  value.sensorCount = COLOR_SENSOR_COUNT;
  value.settingsSignature = ColorSensorManager::settingsSignature();
  value.ratioMarginPermille =
      config::sensors::DEFAULT_RATIO_MARGIN_PERMILLE;
  value.brightnessMarginPermille =
      config::sensors::DEFAULT_BRIGHTNESS_MARGIN_PERMILLE;
}

bool CalibrationStore::load() {
  clearData(storedData);
  storedValid = false;
  ++dataGeneration;
  Preferences preferences;
  if (!preferences.begin(NVS_NAMESPACE, true)) return false;
  const size_t length = preferences.getBytesLength(NVS_KEY);
  const size_t read =
      length == sizeof(storedData)
          ? preferences.getBytes(NVS_KEY, &storedData, sizeof(storedData))
          : 0;
  preferences.end();
  if (read != sizeof(storedData) || !isValid(storedData)) {
    clearData(storedData);
    return false;
  }
  storedValid = true;
  return true;
}

bool CalibrationStore::save(const ColorCalibrationData& value) {
  if (!isValid(value)) return false;
  Preferences preferences;
  if (!preferences.begin(NVS_NAMESPACE, false)) return false;
  const size_t written = preferences.putBytes(NVS_KEY, &value, sizeof(value));
  preferences.end();
  if (written != sizeof(value)) return false;
  storedData = value;
  storedValid = true;
  ++dataGeneration;
  return true;
}

bool CalibrationStore::erase() {
  Preferences preferences;
  if (!preferences.begin(NVS_NAMESPACE, false)) return false;
  const bool removed = preferences.getBytesLength(NVS_KEY) == 0 ||
                       preferences.remove(NVS_KEY);
  preferences.end();
  if (!removed) return false;
  clearData(storedData);
  storedValid = false;
  ++dataGeneration;
  return true;
}

bool CalibrationStore::valid() { return storedValid; }

uint32_t CalibrationStore::generation() { return dataGeneration; }

const ColorCalibrationData& CalibrationStore::data() { return storedData; }
