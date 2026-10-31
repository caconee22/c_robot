#include "calibration/color_calibration.hpp"

#include <Arduino.h>
#include <ctype.h>
#include <cmath>
#include <string.h>

#include "sensors/calibration_store.hpp"
#include "sensors/color_sensor_manager.hpp"
#include "sensors/sensor_config.hpp"

namespace {
constexpr uint8_t TOTAL_STEPS =
    COLOR_SENSOR_COUNT * FLOOR_COLOR_COUNT * CALIBRATION_POSE_COUNT;
struct MeasurementAccumulator {
  uint32_t count = 0;
  uint16_t featureMask = 0;
  double mean[COLOR_CHANNEL_COUNT] = {};
  double m2[COLOR_CHANNEL_COUNT] = {};
  double brightnessMean = 0.0;
  double brightnessM2 = 0.0;
  uint16_t brightnessMin = UINT16_MAX;
  uint16_t brightnessMax = 0;
};

ColorCalibrationData workingData;
ColorCalibrationReference pendingReference;
MeasurementAccumulator accumulator;
ColorCalibrationStatus latestStatus;

char inputLine[64] = {};
size_t inputLength = 0;
uint32_t stateStartedMs = 0;
uint32_t lastPreviewMs = 0;
uint32_t lastSampleSequence = 0;
int8_t lastCountdownNumber = -1;
bool discardInputLine = false;

uint8_t stepSensor(uint8_t step) {
  return step / (FLOOR_COLOR_COUNT * CALIBRATION_POSE_COUNT);
}

FloorColor stepColor(uint8_t step) {
  return static_cast<FloorColor>(
      (step / CALIBRATION_POSE_COUNT) % FLOOR_COLOR_COUNT);
}

CalibrationPose stepPose(uint8_t step) {
  return static_cast<CalibrationPose>(step % CALIBRATION_POSE_COUNT);
}

void setStepFields() {
  latestStatus.sensorIndex = stepSensor(latestStatus.step);
  latestStatus.color = stepColor(latestStatus.step);
  latestStatus.pose = stepPose(latestStatus.step);
}

void printStepPrompt() {
  setStepFields();
  Serial.printf("\n[CAL] Step %u/%u, sensor %u %s\n",
                static_cast<unsigned>(latestStatus.step + 1),
                static_cast<unsigned>(TOTAL_STEPS),
                static_cast<unsigned>(latestStatus.sensorIndex + 1),
                sensorCornerName(latestStatus.sensorIndex));
  Serial.printf("[CAL] Floor=%s, position=%s\n",
                floorColorName(latestStatus.color),
                calibrationPoseName(latestStatus.pose));
  Serial.printf("[CAL] Set sensor-to-floor gap to %u mm (manual).\n",
                config::sensors::CALIBRATION_HEIGHT_MM[static_cast<uint8_t>(latestStatus.pose)]);
  if (latestStatus.pose == CalibrationPose::Level) {
    Serial.println("[CAL] Keep the robot level.");
  } else if (latestStatus.pose == CalibrationPose::Lifted) {
    Serial.println("[CAL] Lift this corner away from the floor.");
  } else {
    Serial.println("[CAL] Press this corner toward the floor.");
  }
  Serial.println("[CAL] Type READY when the position is stable.");
}

void printRaw(const ColorRawSample& sample) {
  if (!sample.valid) {
    Serial.printf("[CAL][PREVIEW] S%u NOT_READY\n",
                  static_cast<unsigned>(latestStatus.sensorIndex + 1));
    return;
  }

  if (ColorSensorManager::model() == ColorSensorModel::AS7341) {
    Serial.printf(
        "[CAL][PREVIEW] S%u F2=%u F3=%u F5=%u F6=%u F7=%u C=%u\n",
        static_cast<unsigned>(latestStatus.sensorIndex + 1),
        sample.channel[1], sample.channel[2], sample.channel[4],
        sample.channel[5], sample.channel[6], sample.channel[8]);
  } else {
    Serial.printf("[CAL][PREVIEW] S%u R=%u G=%u B=%u C=%u\n",
                  static_cast<unsigned>(latestStatus.sensorIndex + 1),
                  sample.channel[0], sample.channel[1], sample.channel[2],
                  sample.channel[8]);
  }
}

void resetAccumulator() {
  accumulator = MeasurementAccumulator{};
  pendingReference = ColorCalibrationReference{};
  latestStatus.sampleCount = 0;
  lastSampleSequence = 0;
}

void addSample(const ColorRawSample& sample) {
  if (!sample.valid || sample.sequence == lastSampleSequence) return;
  lastSampleSequence = sample.sequence;
  ++accumulator.count;
  accumulator.featureMask = sample.featureMask;

  for (size_t index = 0; index < COLOR_CHANNEL_COUNT; ++index) {
    if ((sample.featureMask & (1U << index)) == 0) continue;
    const double delta = sample.normalized[index] - accumulator.mean[index];
    accumulator.mean[index] += delta / accumulator.count;
    const double delta2 = sample.normalized[index] - accumulator.mean[index];
    accumulator.m2[index] += delta * delta2;
  }

  const double brightnessDelta =
      sample.brightness - accumulator.brightnessMean;
  accumulator.brightnessMean += brightnessDelta / accumulator.count;
  const double brightnessDelta2 =
      sample.brightness - accumulator.brightnessMean;
  accumulator.brightnessM2 += brightnessDelta * brightnessDelta2;
  accumulator.brightnessMin =
      min(accumulator.brightnessMin, sample.brightness);
  accumulator.brightnessMax =
      max(accumulator.brightnessMax, sample.brightness);
  latestStatus.sampleCount = static_cast<uint16_t>(
      min(accumulator.count, static_cast<uint32_t>(UINT16_MAX)));
}

void finishMeasurement() {
  if (accumulator.count < config::sensors::CALIBRATION_MIN_SAMPLES) {
    Serial.printf("[CAL] Too few samples: %lu. Type READY to retry.\n",
                  static_cast<unsigned long>(accumulator.count));
    latestStatus.state = CalibrationState::WaitingForReady;
    resetAccumulator();
    return;
  }

  const double divisor = accumulator.count > 1 ? accumulator.count - 1 : 1;
  for (size_t index = 0; index < COLOR_CHANNEL_COUNT; ++index) {
    pendingReference.normalizedMean[index] = accumulator.mean[index];
    pendingReference.normalizedStdDev[index] =
        sqrt(accumulator.m2[index] / divisor);
  }
  pendingReference.brightnessMean = accumulator.brightnessMean;
  pendingReference.brightnessStdDev =
      sqrt(accumulator.brightnessM2 / divisor);
  pendingReference.brightnessMin = accumulator.brightnessMin;
  pendingReference.brightnessMax = accumulator.brightnessMax;
  pendingReference.sampleCount = latestStatus.sampleCount;
  pendingReference.valid = true;

  Serial.printf(
      "[CAL][RESULT] samples=%u brightness=%.1f min=%u max=%u noise=%.2f\n",
      pendingReference.sampleCount, pendingReference.brightnessMean,
      pendingReference.brightnessMin, pendingReference.brightnessMax,
      pendingReference.brightnessStdDev);
  Serial.println("[CAL] Type ACCEPT, RETRY, BACK or ABORT.");
  latestStatus.state = CalibrationState::WaitingForAccept;
}

void startCalibration() {
  CalibrationStore::initialize(workingData);
  if (CalibrationStore::valid()) {
    workingData.ratioMarginPermille =
        CalibrationStore::data().ratioMarginPermille;
    workingData.brightnessMarginPermille =
        CalibrationStore::data().brightnessMarginPermille;
  }
  latestStatus.active = true;
  latestStatus.state = CalibrationState::WaitingForReady;
  latestStatus.step = 0;
  resetAccumulator();
  Serial.println("\n[CAL] Calibration started. Motors are braked until it ends.");
  Serial.println("[CAL] IMU is not used during calibration.");
  Serial.println("[CAL] Commands: READY ACCEPT RETRY BACK STATUS ABORT");
  Serial.println("[CAL] Margin: CAL MARGIN <ratio_percent> <brightness_percent>");
  printStepPrompt();
}

void abortCalibration(const char* reason) {
  Serial.printf("[CAL] Calibration aborted: %s\n", reason);
  latestStatus.active = false;
  latestStatus.state = CalibrationState::Idle;
  latestStatus.sampleCount = 0;
}

void acceptMeasurement() {
  const uint8_t sensor = stepSensor(latestStatus.step);
  const uint8_t color = static_cast<uint8_t>(stepColor(latestStatus.step));
  const uint8_t pose = static_cast<uint8_t>(stepPose(latestStatus.step));
  workingData.reference[sensor][color][pose] = pendingReference;

  ++latestStatus.step;
  resetAccumulator();
  if (latestStatus.step >= TOTAL_STEPS) {
    latestStatus.state = CalibrationState::Completed;
    Serial.println("\n[CAL] All measurements completed.");
    Serial.println("[CAL] Type CAL SAVE to store, or ABORT to discard.");
    return;
  }
  latestStatus.state = CalibrationState::WaitingForReady;
  printStepPrompt();
}

void printStatus() {
  Serial.printf(
      "[CAL][STATUS] active=%u stored=%u state=%u step=%u/%u "
      "sensor=%u %s color=%s pose=%s samples=%u margin=%u/%u%%\n",
      latestStatus.active ? 1 : 0,
      latestStatus.storedDataValid ? 1 : 0,
      static_cast<unsigned>(latestStatus.state),
      static_cast<unsigned>(min(static_cast<uint8_t>(latestStatus.step + 1), TOTAL_STEPS)),
      static_cast<unsigned>(TOTAL_STEPS),
      static_cast<unsigned>(latestStatus.sensorIndex + 1),
      sensorCornerName(latestStatus.sensorIndex),
      floorColorName(latestStatus.color),
      calibrationPoseName(latestStatus.pose), latestStatus.sampleCount,
      workingData.ratioMarginPermille / 10,
      workingData.brightnessMarginPermille / 10);
}

bool parseMargins(const char* text, unsigned& ratio, unsigned& brightness) {
  unsigned values[2] = {};
  for (uint8_t index = 0; index < 2; ++index) {
    unsigned& value = values[index];
    while (*text == ' ') ++text;
    if (*text < '0' || *text > '9') return false;
    uint8_t digits = 0;
    while (*text >= '0' && *text <= '9') {
      if (++digits > 3) return false;
      value = value * 10 + (*text++ - '0');
    }
    if (value < 1 || value > 100) return false;
    if (index == 0 && *text != ' ') return false;
  }
  while (*text == ' ') ++text;
  if (*text != '\0') return false;
  ratio = values[0]; brightness = values[1];
  return true;
}

void handleCommand(const char* command, uint32_t nowMs) {
  if (strcmp(command, "CAL STATUS") == 0 || strcmp(command, "STATUS") == 0) {
    printStatus();
    return;
  }

  if (!latestStatus.active) {
    if (strcmp(command, "CAL START") == 0) {
      if (ColorSensorManager::status().readyMask != 0x0F) {
        Serial.printf("[CAL] Rejected: sensor ready mask is 0x%02X.\n",
                      ColorSensorManager::status().readyMask);
      } else {
        startCalibration();
      }
    } else if (strcmp(command, "CAL ERASE") == 0) {
      if (CalibrationStore::erase()) {
        latestStatus.storedDataValid = false;
        Serial.println("[CAL] Stored calibration erased.");
      } else {
        Serial.println("[CAL] NVS erase failed.");
      }
    }
    return;
  }

  if (strcmp(command, "ABORT") == 0) {
    abortCalibration("USER");
    return;
  }

  unsigned ratioPercent = 0;
  unsigned brightnessPercent = 0;
  if (strncmp(command, "CAL MARGIN", 10) == 0) {
    if (command[10] != ' ' ||
        !parseMargins(command + 11, ratioPercent, brightnessPercent)) {
      Serial.println("[CAL] Margin range is 1..100 percent.");
    } else {
      workingData.ratioMarginPermille = ratioPercent * 10;
      workingData.brightnessMarginPermille = brightnessPercent * 10;
      Serial.printf("[CAL] Margin set to ratio=%u%% brightness=%u%%.\n",
                    ratioPercent, brightnessPercent);
    }
    return;
  }

  if (strcmp(command, "READY") == 0 &&
      latestStatus.state == CalibrationState::WaitingForReady) {
    resetAccumulator();
    latestStatus.state = CalibrationState::Countdown;
    stateStartedMs = nowMs;
    lastCountdownNumber = -1;
    return;
  }

  if (strcmp(command, "ACCEPT") == 0 &&
      latestStatus.state == CalibrationState::WaitingForAccept) {
    acceptMeasurement();
    return;
  }

  if (strcmp(command, "RETRY") == 0 &&
      latestStatus.state == CalibrationState::WaitingForAccept) {
    resetAccumulator();
    latestStatus.state = CalibrationState::WaitingForReady;
    Serial.println("[CAL] Measurement discarded. Type READY to retry.");
    return;
  }

  if (strcmp(command, "BACK") == 0 &&
      (latestStatus.state == CalibrationState::WaitingForReady ||
       latestStatus.state == CalibrationState::WaitingForAccept)) {
    if (latestStatus.step > 0) --latestStatus.step;
    resetAccumulator();
    latestStatus.state = CalibrationState::WaitingForReady;
    printStepPrompt();
    return;
  }

  if (strcmp(command, "CAL SAVE") == 0 &&
      (latestStatus.state == CalibrationState::Completed ||
       latestStatus.state == CalibrationState::Error)) {
    if (CalibrationStore::save(workingData)) {
      latestStatus.storedDataValid = true;
      latestStatus.active = false;
      latestStatus.state = CalibrationState::Idle;
      Serial.println("[CAL] Calibration saved to NVS.");
    } else {
      latestStatus.state = CalibrationState::Error;
      Serial.println("[CAL] NVS save failed. Type ABORT or retry CAL SAVE.");
    }
    return;
  }

  Serial.println("[CAL] Command is not valid in the current state.");
}

void pollSerial(uint32_t nowMs) {
  size_t processed = 0;
  while (processed++ < 64 && Serial.available() > 0) {
    const int incoming = Serial.read();
    if (incoming == '\r') continue;
    if (incoming == '\n') {
      if (!discardInputLine && inputLength > 0) {
        inputLine[inputLength] = '\0';
        handleCommand(inputLine, nowMs);
        inputLength = 0;
      }
      inputLength = 0;
      discardInputLine = false;
      continue;
    }
    if (discardInputLine) continue;
    if (incoming < 32 || incoming > 126) {
      inputLength = 0;
      discardInputLine = true;
      continue;
    }
    if (inputLength + 1 < sizeof(inputLine)) {
      inputLine[inputLength++] =
          static_cast<char>(toupper(static_cast<unsigned char>(incoming)));
    } else {
      inputLength = 0;
      discardInputLine = true;
    }
  }
}

void updateActiveState(uint32_t nowMs) {
  const ColorRawSample current =
      ColorSensorManager::sample(latestStatus.sensorIndex);

  if (latestStatus.state == CalibrationState::WaitingForReady &&
      nowMs - lastPreviewMs >=
          config::sensors::CALIBRATION_PREVIEW_PERIOD_MS) {
    lastPreviewMs = nowMs;
    printRaw(current);
  }

  if (latestStatus.state == CalibrationState::Countdown) {
    const uint32_t elapsed = nowMs - stateStartedMs;
    const int8_t number = static_cast<int8_t>(
        3 - min(elapsed / 1000U, static_cast<uint32_t>(2)));
    if (number != lastCountdownNumber) {
      lastCountdownNumber = number;
      Serial.printf("[CAL] Starting in %d...\n", number);
    }
    if (elapsed >= config::sensors::CALIBRATION_COUNTDOWN_MS) {
      latestStatus.state = CalibrationState::Measuring;
      stateStartedMs = nowMs;
      resetAccumulator();
      lastSampleSequence = current.sequence;
      Serial.println("[CAL] Measuring for 3 seconds. Do not move.");
    }
    return;
  }

  if (latestStatus.state == CalibrationState::Measuring) {
    addSample(current);
    if (nowMs - stateStartedMs >= config::sensors::CALIBRATION_MEASURE_MS) {
      finishMeasurement();
    }
  }
}

}  // namespace

void ColorCalibration::begin() {
  latestStatus = ColorCalibrationStatus{};
  latestStatus.storedDataValid = CalibrationStore::valid();
  workingData = CalibrationStore::data();
  setStepFields();
  inputLength = 0;
  discardInputLine = false;
}

void ColorCalibration::update(uint32_t nowMs) {
  pollSerial(nowMs);
  tick(nowMs);
}

void ColorCalibration::command(const char* line, uint32_t nowMs) {
  char upper[sizeof(inputLine)];
  size_t length = 0;
  for (; line[length] && length + 1 < sizeof(upper); ++length)
    upper[length] = static_cast<char>(toupper(static_cast<unsigned char>(line[length])));
  upper[length] = '\0';
  handleCommand(upper, nowMs);
}

void ColorCalibration::tick(uint32_t nowMs) {
  if (latestStatus.active) updateActiveState(nowMs);
}

bool ColorCalibration::active() { return latestStatus.active; }

ColorCalibrationStatus ColorCalibration::status() { return latestStatus; }
