#pragma once

#include <Arduino.h>

constexpr size_t COLOR_SENSOR_COUNT = 4;
constexpr size_t COLOR_CHANNEL_COUNT = 10;
constexpr size_t FLOOR_COLOR_COUNT = 4;
constexpr size_t CALIBRATION_POSE_COUNT = 3;

enum class ColorSensorModel : uint8_t {
  AS7341 = 1,
  TCS34725 = 2,
};

enum class SensorCorner : uint8_t {
  FrontLeft = 0,
  FrontRight = 1,
  RearLeft = 2,
  RearRight = 3,
};

enum class FloorColor : uint8_t {
  Red = 0,
  Yellow = 1,
  Blue = 2,
  Black = 3,
  Unknown = 255,
};

enum class CalibrationPose : uint8_t {
  Level = 0,
  Lifted = 1,
  Pressed = 2,
};

struct ColorRawSample {
  uint32_t timestampMs = 0;
  uint32_t sequence = 0;
  uint16_t channel[COLOR_CHANNEL_COUNT] = {};
  float normalized[COLOR_CHANNEL_COUNT] = {};
  uint16_t featureMask = 0;
  uint16_t brightness = 0;
  bool valid = false;
};

enum class ColorSensorError : uint8_t {
  None,
  Initialization,
  I2c,
  Timeout,
  Range,
};

struct ColorSensorStatus {
  ColorSensorModel model = ColorSensorModel::AS7341;
  uint8_t readyMask = 0;
  uint8_t lastReadSensor = 0;
  uint32_t successfulReads = 0;
  uint32_t failedReads = 0;
  ColorSensorError error[COLOR_SENSOR_COUNT] = {};
};

struct ColorCalibrationReference {
  float normalizedMean[COLOR_CHANNEL_COUNT] = {};
  float normalizedStdDev[COLOR_CHANNEL_COUNT] = {};
  float brightnessMean = 0.0f;
  float brightnessStdDev = 0.0f;
  uint16_t brightnessMin = 0;
  uint16_t brightnessMax = 0;
  uint16_t sampleCount = 0;
  bool valid = false;
};

struct ColorCalibrationData {
  uint32_t magic = 0;
  uint16_t version = 0;
  uint8_t sensorModel = 0;
  uint8_t sensorCount = 0;
  uint32_t settingsSignature = 0;
  uint16_t ratioMarginPermille = 0;
  uint16_t brightnessMarginPermille = 0;
  ColorCalibrationReference
      reference[COLOR_SENSOR_COUNT][FLOOR_COLOR_COUNT]
               [CALIBRATION_POSE_COUNT];
};

struct FloorColorResult {
  FloorColor color = FloorColor::Unknown;
  uint16_t brightness = 0;
  uint8_t confidence = 0;
  uint8_t consecutiveMatches = 0;
  CalibrationPose matchedPose = CalibrationPose::Level;
  uint32_t timestampMs = 0;
  bool reliable = false;
};

const char* sensorCornerName(uint8_t sensorIndex);
const char* floorColorName(FloorColor color);
const char* calibrationPoseName(CalibrationPose pose);
