#include "calibration/color_calibration.hpp"

#include <Arduino.h>
#include <ctype.h>
#include <cmath>
#include <string.h>

#include "sensors/calibration_store.hpp"
#include "sensors/color_sensor_manager.hpp"
#include "sensors/sensor_config.hpp"

namespace {
// 11/04 decision: yellow is not calibrated (not a hazard, no need to tell it
// apart). Level and lifted measure all four corners at once; pressed tilts the
// robot, so front and rear pairs are measured separately.
struct Step {
  FloorColor color;
  CalibrationPose pose;
  uint8_t sensorMask;
};
constexpr uint8_t ALL_SENSORS = 0x0F, FRONT_SENSORS = 0x03, REAR_SENSORS = 0x0C;
constexpr Step STEPS[] = {
    {FloorColor::Black, CalibrationPose::Level, ALL_SENSORS},
    {FloorColor::Black, CalibrationPose::Lifted, ALL_SENSORS},
    {FloorColor::Black, CalibrationPose::Pressed, FRONT_SENSORS},
    {FloorColor::Black, CalibrationPose::Pressed, REAR_SENSORS},
    {FloorColor::Red, CalibrationPose::Level, ALL_SENSORS},
    {FloorColor::Red, CalibrationPose::Lifted, ALL_SENSORS},
    {FloorColor::Red, CalibrationPose::Pressed, FRONT_SENSORS},
    {FloorColor::Red, CalibrationPose::Pressed, REAR_SENSORS},
    {FloorColor::Blue, CalibrationPose::Level, ALL_SENSORS},
    {FloorColor::Blue, CalibrationPose::Lifted, ALL_SENSORS},
    {FloorColor::Blue, CalibrationPose::Pressed, FRONT_SENSORS},
    {FloorColor::Blue, CalibrationPose::Pressed, REAR_SENSORS},
};
constexpr uint8_t TOTAL_STEPS = sizeof(STEPS) / sizeof(STEPS[0]);
static_assert(TOTAL_STEPS == ColorCalibration::STEP_COUNT, "Update STEP_COUNT");
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
ColorCalibrationReference pendingReference[COLOR_SENSOR_COUNT];
MeasurementAccumulator accumulators[COLOR_SENSOR_COUNT];
ColorCalibrationStatus latestStatus;

char inputLine[64] = {};
size_t inputLength = 0;
uint32_t stateStartedMs = 0;
uint32_t lastPreviewMs = 0;
uint32_t lastSampleSequence[COLOR_SENSOR_COUNT] = {};
int8_t lastCountdownNumber = -1;
bool discardInputLine = false;

bool inStep(uint8_t sensor) {
  return (latestStatus.sensorMask & (1U << sensor)) != 0;
}

void setStepFields() {
  const Step& step = STEPS[min(latestStatus.step, static_cast<uint8_t>(TOTAL_STEPS - 1))];
  latestStatus.sensorMask = step.sensorMask;
  latestStatus.color = step.color;
  latestStatus.pose = step.pose;
}

void printStepPrompt() {
  setStepFields();
  Serial.printf("\n[CAL] Step %u/%u, sensors",
                static_cast<unsigned>(latestStatus.step + 1),
                static_cast<unsigned>(TOTAL_STEPS));
  for (uint8_t sensor = 0; sensor < COLOR_SENSOR_COUNT; ++sensor) {
    if (inStep(sensor)) Serial.printf(" %u", static_cast<unsigned>(sensor + 1));
  }
  Serial.println();
  Serial.printf("[CAL] Floor=%s, position=%s\n",
                floorColorName(latestStatus.color),
                calibrationPoseName(latestStatus.pose));
  Serial.printf("[CAL] Set sensor-to-floor gap to %u mm (manual).\n",
                config::sensors::CALIBRATION_HEIGHT_MM[static_cast<uint8_t>(latestStatus.pose)]);
  if (latestStatus.pose == CalibrationPose::Level) {
    Serial.println("[CAL] Keep the robot level.");
  } else if (latestStatus.pose == CalibrationPose::Lifted) {
    Serial.println("[CAL] Raise the whole robot level by 10 mm.");
  } else if (latestStatus.sensorMask == FRONT_SENSORS) {
    Serial.println("[CAL] Tilt the robot forward onto the front sensors.");
  } else {
    Serial.println("[CAL] Tilt the robot backward onto the rear sensors.");
  }
  Serial.println("[CAL] Type READY when the position is stable.");
}

void printRaw(uint8_t sensor, const ColorRawSample& sample) {
  if (!sample.valid) {
    Serial.printf("[CAL][PREVIEW] S%u NOT_READY\n", static_cast<unsigned>(sensor + 1));
    return;
  }

  if (ColorSensorManager::model() == ColorSensorModel::AS7341) {
    Serial.printf(
        "[CAL][PREVIEW] S%u F2=%u F3=%u F5=%u F6=%u F7=%u C=%u\n",
        static_cast<unsigned>(sensor + 1),
        sample.channel[1], sample.channel[2], sample.channel[4],
        sample.channel[5], sample.channel[6], sample.channel[8]);
  } else {
    Serial.printf("[CAL][PREVIEW] S%u R=%u G=%u B=%u C=%u\n",
                  static_cast<unsigned>(sensor + 1),
                  sample.channel[0], sample.channel[1], sample.channel[2],
                  sample.channel[8]);
  }
}

void resetAccumulator() {
  for (uint8_t sensor = 0; sensor < COLOR_SENSOR_COUNT; ++sensor) {
    accumulators[sensor] = MeasurementAccumulator{};
    pendingReference[sensor] = ColorCalibrationReference{};
    lastSampleSequence[sensor] = 0;
  }
  latestStatus.sampleCount = 0;
}

// sampleCount reports the sensor with the fewest samples in this step.
void updateSampleCount() {
  uint32_t fewest = UINT32_MAX;
  for (uint8_t sensor = 0; sensor < COLOR_SENSOR_COUNT; ++sensor) {
    if (inStep(sensor)) fewest = min(fewest, accumulators[sensor].count);
  }
  latestStatus.sampleCount = static_cast<uint16_t>(
      min(fewest, static_cast<uint32_t>(UINT16_MAX)));
}

void addSample(uint8_t sensor, const ColorRawSample& sample) {
  if (!sample.valid || sample.sequence == lastSampleSequence[sensor]) return;
  lastSampleSequence[sensor] = sample.sequence;
  MeasurementAccumulator& accumulator = accumulators[sensor];
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
}

void finishMeasurement() {
  updateSampleCount();
  if (latestStatus.sampleCount < config::sensors::CALIBRATION_MIN_SAMPLES) {
    Serial.printf("[CAL] Too few samples: %u. Type READY to retry.\n",
                  static_cast<unsigned>(latestStatus.sampleCount));
    latestStatus.state = CalibrationState::WaitingForReady;
    resetAccumulator();
    return;
  }

  for (uint8_t sensor = 0; sensor < COLOR_SENSOR_COUNT; ++sensor) {
    if (!inStep(sensor)) continue;
    const MeasurementAccumulator& accumulator = accumulators[sensor];
    ColorCalibrationReference& reference = pendingReference[sensor];
    const double divisor = accumulator.count > 1 ? accumulator.count - 1 : 1;
    for (size_t index = 0; index < COLOR_CHANNEL_COUNT; ++index) {
      reference.normalizedMean[index] = accumulator.mean[index];
      reference.normalizedStdDev[index] = sqrt(accumulator.m2[index] / divisor);
    }
    reference.brightnessMean = accumulator.brightnessMean;
    reference.brightnessStdDev = sqrt(accumulator.brightnessM2 / divisor);
    reference.brightnessMin = accumulator.brightnessMin;
    reference.brightnessMax = accumulator.brightnessMax;
    reference.sampleCount = static_cast<uint16_t>(
        min(accumulator.count, static_cast<uint32_t>(UINT16_MAX)));
    reference.valid = true;

    Serial.printf(
        "[CAL][RESULT] S%u samples=%u brightness=%.1f min=%u max=%u noise=%.2f\n",
        static_cast<unsigned>(sensor + 1), reference.sampleCount,
        reference.brightnessMean, reference.brightnessMin,
        reference.brightnessMax, reference.brightnessStdDev);
  }
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
  const uint8_t color = static_cast<uint8_t>(latestStatus.color);
  const uint8_t pose = static_cast<uint8_t>(latestStatus.pose);
  for (uint8_t sensor = 0; sensor < COLOR_SENSOR_COUNT; ++sensor) {
    if (inStep(sensor)) workingData.reference[sensor][color][pose] = pendingReference[sensor];
  }

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
      "sensors=0x%X color=%s pose=%s samples=%u margin=%u/%u%%\n",
      latestStatus.active ? 1 : 0,
      latestStatus.storedDataValid ? 1 : 0,
      static_cast<unsigned>(latestStatus.state),
      static_cast<unsigned>(min(static_cast<uint8_t>(latestStatus.step + 1), TOTAL_STEPS)),
      static_cast<unsigned>(TOTAL_STEPS),
      static_cast<unsigned>(latestStatus.sensorMask),
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
  if (latestStatus.state == CalibrationState::WaitingForReady &&
      nowMs - lastPreviewMs >=
          config::sensors::CALIBRATION_PREVIEW_PERIOD_MS) {
    lastPreviewMs = nowMs;
    // All four, not only this step's sensors, so the GUI can compare corners.
    for (uint8_t sensor = 0; sensor < COLOR_SENSOR_COUNT; ++sensor) {
      printRaw(sensor, ColorSensorManager::sample(sensor));
    }
  }

  if (latestStatus.state == CalibrationState::Countdown) {
    const uint32_t elapsed = nowMs - stateStartedMs;
    const uint32_t remainingMs = config::sensors::CALIBRATION_COUNTDOWN_MS -
        min(elapsed, config::sensors::CALIBRATION_COUNTDOWN_MS);
    const int8_t number = static_cast<int8_t>(max((remainingMs + 999U) / 1000U, 1U));
    if (number != lastCountdownNumber) {
      lastCountdownNumber = number;
      Serial.printf("[CAL] Starting in %d...\n", number);
    }
    if (elapsed >= config::sensors::CALIBRATION_COUNTDOWN_MS) {
      latestStatus.state = CalibrationState::Measuring;
      stateStartedMs = nowMs;
      resetAccumulator();
      // Count only samples taken after the countdown.
      for (uint8_t sensor = 0; sensor < COLOR_SENSOR_COUNT; ++sensor) {
        lastSampleSequence[sensor] = ColorSensorManager::sample(sensor).sequence;
      }
      Serial.printf("[CAL] Measuring for %lu ms. Do not move.\n",
                    static_cast<unsigned long>(config::sensors::CALIBRATION_MEASURE_MS));
    }
    return;
  }

  if (latestStatus.state == CalibrationState::Measuring) {
    for (uint8_t sensor = 0; sensor < COLOR_SENSOR_COUNT; ++sensor) {
      if (inStep(sensor)) addSample(sensor, ColorSensorManager::sample(sensor));
    }
    updateSampleCount();
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
