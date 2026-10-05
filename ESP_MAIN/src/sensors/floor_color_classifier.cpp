#include "sensors/floor_color_classifier.hpp"

#include <cmath>

#include "sensors/color_sensor_manager.hpp"
#include "sensors/sensor_config.hpp"

namespace {
using std::isfinite;

constexpr float MINIMUM_SCALE = 0.005f;
constexpr float MAXIMUM_MATCH_SCORE = 1.0f;
constexpr float MINIMUM_WINNER_GAP = 0.08f;

float maximum(float a, float b) { return a > b ? a : b; }

uint8_t dominanceConfidence(float winner, float runnerUp) {
  if (winner <= 0.0f) return 0;
  const float separation = (winner - runnerUp) / winner;
  return static_cast<uint8_t>(
      constrain(static_cast<int>(50.0f + separation * 100.0f), 0, 100));
}

FloorColorResult classifyFallback(const ColorRawSample& sample) {
  FloorColorResult result;
  result.timestampMs = sample.timestampMs;
  result.brightness = sample.brightness;

  const bool as7341 = ColorSensorManager::model() == ColorSensorModel::AS7341;
  const uint16_t blackLimit =
      as7341 ? config::sensors::FALLBACK_AS7341_BLACK_CLEAR_MAX
             : config::sensors::FALLBACK_TCS34725_BLACK_CLEAR_MAX;
  if (sample.brightness <= blackLimit) {
    result.color = FloorColor::Black;
    result.confidence = 70;
    result.reliable = true;
    return result;
  }

  float red = 0.0f;
  float green = 0.0f;
  float blue = 0.0f;
  float yellow = 0.0f;
  if (as7341) {
    blue = (sample.normalized[1] + sample.normalized[2]) * 0.5f;
    green = (sample.normalized[3] + sample.normalized[4]) * 0.5f;
    yellow = (sample.normalized[4] + sample.normalized[5]) * 0.5f;
    red = (sample.normalized[6] + sample.normalized[7]) * 0.5f;
  } else {
    red = sample.normalized[0];
    green = sample.normalized[1];
    blue = sample.normalized[2];
    yellow = (red + green) * 0.5f;
  }

  const float rgRatio = green > 0.0001f ? red / green : 99.0f;
  if (red > green * config::sensors::FALLBACK_RED_DOMINANCE &&
      red > blue * config::sensors::FALLBACK_RED_DOMINANCE) {
    result.color = FloorColor::Red;
    result.confidence = dominanceConfidence(red, maximum(green, blue));
  } else if (yellow > blue * config::sensors::FALLBACK_YELLOW_OVER_BLUE &&
      rgRatio >= config::sensors::FALLBACK_YELLOW_RG_MIN &&
      rgRatio <= config::sensors::FALLBACK_YELLOW_RG_MAX) {
    result.color = FloorColor::Yellow;
    result.confidence = dominanceConfidence(yellow, blue);
  } else if (blue > red * config::sensors::FALLBACK_BLUE_DOMINANCE &&
             blue > green * config::sensors::FALLBACK_BLUE_DOMINANCE) {
    result.color = FloorColor::Blue;
    result.confidence = dominanceConfidence(blue, maximum(red, green));
  } else {
    return result;
  }

  result.reliable = true;
  return result;
}

float ratioScore(const ColorRawSample& sample,
                 const ColorCalibrationReference& reference,
                 float margin) {
  float squared = 0.0f;
  uint8_t used = 0;
  for (size_t index = 0; index < COLOR_CHANNEL_COUNT; ++index) {
    if ((sample.featureMask & (1U << index)) == 0) continue;
    const float tolerance = maximum(
        maximum(fabsf(reference.normalizedMean[index]) * margin,
                reference.normalizedStdDev[index] * 3.0f),
        MINIMUM_SCALE);
    const float normalizedError =
        (sample.normalized[index] - reference.normalizedMean[index]) /
        tolerance;
    squared += normalizedError * normalizedError;
    ++used;
  }
  if (used == 0) return INFINITY;
  return sqrtf(squared / used);
}

float brightnessScore(const ColorRawSample& sample,
                      const ColorCalibrationReference& reference,
                      float margin) {
  const float spread = maximum(
      maximum(reference.brightnessMean * margin,
              reference.brightnessStdDev * 3.0f),
      1.0f);
  const float low = maximum(0.0f, reference.brightnessMin - spread);
  const float high = reference.brightnessMax + spread;
  const float value = sample.brightness;
  if (value >= low && value <= high) return 0.0f;
  return value < low ? (low - value) / spread : (value - high) / spread;
}

}  // namespace

FloorColorResult FloorColorClassifier::classify(
    uint8_t sensorIndex, const ColorRawSample& sample,
    const ColorCalibrationData& calibration) {
  FloorColorResult result;
  result.timestampMs = sample.timestampMs;
  result.brightness = sample.brightness;

  if (!sample.valid || sensorIndex >= COLOR_SENSOR_COUNT || sample.featureMask == 0) {
    return result;
  }
  for (size_t i = 0; i < COLOR_CHANNEL_COUNT; ++i) {
    if ((sample.featureMask & (1U << i)) != 0 &&
        (!isfinite(sample.normalized[i]) || sample.normalized[i] < 0)) {
      return result;
    }
  }
  if (calibration.magic == 0 ||
      calibration.sensorModel !=
          static_cast<uint8_t>(ColorSensorManager::model()) ||
      calibration.settingsSignature !=
          ColorSensorManager::settingsSignature()) {
    return classifyFallback(sample);
  }

  const float ratioMargin =
      maximum(calibration.ratioMarginPermille / 1000.0f, 0.01f);
  const float brightnessMargin =
      maximum(calibration.brightnessMarginPermille / 1000.0f, 0.01f);

  float bestScore = INFINITY;
  float secondScore = INFINITY;
  FloorColor bestColor = FloorColor::Unknown;
  CalibrationPose bestPose = CalibrationPose::Level;

  for (uint8_t color = 0; color < FLOOR_COLOR_COUNT; ++color) {
    float colorBestScore = INFINITY;
    CalibrationPose colorBestPose = CalibrationPose::Level;
    for (uint8_t pose = 0; pose < CALIBRATION_POSE_COUNT; ++pose) {
      const ColorCalibrationReference& reference =
          calibration.reference[sensorIndex][color][pose];
      if (!reference.valid) continue;

      const float ratio = ratioScore(sample, reference, ratioMargin);
      const float brightness =
          brightnessScore(sample, reference, brightnessMargin);
      const bool black = color == static_cast<uint8_t>(FloorColor::Black);
      const float score = black ? ratio * 0.45f + brightness * 0.55f
                                : ratio * 0.80f + brightness * 0.20f;
      if (score < colorBestScore) {
        colorBestScore = score;
        colorBestPose = static_cast<CalibrationPose>(pose);
      }
    }

    // Different height references for the same color are not competitors.
    // The winner gap is checked only between different colors.
    if (colorBestScore < bestScore) {
      secondScore = bestScore;
      bestScore = colorBestScore;
      bestColor = static_cast<FloorColor>(color);
      bestPose = colorBestPose;
    } else if (colorBestScore < secondScore) {
      secondScore = colorBestScore;
    }
  }

  if (bestColor == FloorColor::Unknown || !isfinite(bestScore) || bestScore > MAXIMUM_MATCH_SCORE ||
      (isfinite(secondScore) && secondScore - bestScore < MINIMUM_WINNER_GAP)) {
    return result;
  }

  const float confidence = 100.0f * (1.0f - bestScore);
  result.color = bestColor;
  result.matchedPose = bestPose;
  result.confidence = static_cast<uint8_t>(
      constrain(static_cast<int>(confidence + 0.5f), 0, 100));
  result.reliable = true;
  return result;
}
