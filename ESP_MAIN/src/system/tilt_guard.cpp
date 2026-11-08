#include "system/tilt_guard.hpp"

#include <math.h>

namespace {
using namespace config::tilt;

float level[3] = {};
uint8_t levelSamples = 0;
uint32_t lastSampleMs = 0;
bool tilted = false;
uint32_t tiltedSinceMs = 0;
float angleDeg = 0.0f;

float length(const float v[3]) { return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }
}  // namespace

void TiltGuard::begin() {
  levelSamples = 0;
  lastSampleMs = 0;
  tilted = false;
  angleDeg = 0.0f;
}

bool TiltGuard::update(uint32_t nowMs, bool running, const ImuStatus& status,
                       const ImuSample& sample) {
  if (!status.dataFresh) {
    tilted = false;  // cannot judge; start the hold again once data returns
    return false;
  }
  const bool newSample = sample.timestampMs != lastSampleMs;
  lastSampleMs = sample.timestampMs;
  const float accel[3] = {static_cast<float>(sample.accel[0]), static_cast<float>(sample.accel[1]),
                          static_cast<float>(sample.accel[2])};
  if (!running) {
    tilted = false;
    if (!newSample) return false;
    // Running average of the resting gravity vector (start position is level).
    const float weight = levelSamples < LEVEL_SAMPLES ? 1.0f / (levelSamples + 1) : 0.1f;
    for (uint8_t i = 0; i < 3; ++i) level[i] += (accel[i] - level[i]) * weight;
    if (levelSamples < LEVEL_SAMPLES) ++levelSamples;
    return false;
  }
  if (levelSamples < LEVEL_SAMPLES) {
    // IMU came up only after the start: take this sample as level.
    for (uint8_t i = 0; i < 3; ++i) level[i] = accel[i];
    levelSamples = LEVEL_SAMPLES;
  }
  const float a = length(accel), l = length(level);
  if (a < 0.2f * l || l <= 0.0f) return tilted && nowMs - tiltedSinceMs >= HOLD_MS;  // free fall or crash spike: skip
  const float cosine = (accel[0] * level[0] + accel[1] * level[1] + accel[2] * level[2]) / (a * l);
  angleDeg = acosf(fmaxf(-1.0f, fminf(1.0f, cosine))) * 57.29578f;
  if (angleDeg <= LIMIT_DEG) {
    tilted = false;
    return false;
  }
  if (!tilted) {
    tilted = true;
    tiltedSinceMs = nowMs;
  }
  return nowMs - tiltedSinceMs >= HOLD_MS;
}

float TiltGuard::lastAngleDeg() { return angleDeg; }
