#include "control/pose_estimate.hpp"

#include <math.h>

namespace {
using namespace config::pose;

Pose current;
bool wasRunning = false;
float bias[3] = {};
float up[3] = {0.0f, 0.0f, 1.0f};
bool learned = false;
uint32_t lastImuMs = 0;
bool imuKnown = false;
uint32_t lastMs = 0;
bool timeKnown = false;

void learnAtRest(const ImuSample& sample) {
  const float w = learned ? BIAS_WEIGHT : 1.0f;
  float g[3];
  for (uint8_t i = 0; i < 3; ++i) {
    bias[i] += (sample.gyro[i] - bias[i]) * w;
    g[i] = sample.accel[i];
  }
  const float norm = sqrtf(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]);
  if (norm > config::imu::ACCEL_LSB_PER_G / 2) {
    for (uint8_t i = 0; i < 3; ++i) up[i] += (g[i] / norm - up[i]) * w;
  }
  learned = true;
}
}  // namespace

void PoseEstimate::reset() {
  current = Pose{};
  wasRunning = false;
  learned = false;
  imuKnown = false;
  timeKnown = false;
  for (float& b : bias) b = 0.0f;
  up[0] = up[1] = 0.0f;
  up[2] = 1.0f;
}

void PoseEstimate::update(uint32_t nowMs, bool running, const ImuStatus& imu,
                          const ImuSample& sample, int16_t leftPermille, int16_t rightPermille) {
  const bool newSample = imu.dataFresh && (!imuKnown || sample.timestampMs != lastImuMs);
  if (!running) {
    wasRunning = false;
    if (newSample) learnAtRest(sample);
    if (newSample) { lastImuMs = sample.timestampMs; imuKnown = true; }
    current.imuOk = imu.dataFresh;
    return;
  }
  if (!wasRunning) {   // SW2: this is (0, 0), heading 0
    wasRunning = true;
    current = Pose{};
    timeKnown = false;
  }
  current.imuOk = imu.dataFresh && learned;

  // Heading: gyro along "up", bias removed; degrees per second from raw counts.
  if (newSample) {
    if (imuKnown && learned) {
      const uint32_t dtMs = sample.timestampMs - lastImuMs;
      if (dtMs > 0 && dtMs <= MAX_STEP_MS) {
        float rate = 0.0f;
        for (uint8_t i = 0; i < 3; ++i) rate += (sample.gyro[i] - bias[i]) * up[i];
        rate = rate * 10.0f / config::imu::GYRO_LSB_PER_10DPS;
        current.headingDeg += rate * dtMs / 1000.0f;
      }
    }
    lastImuMs = sample.timestampMs;
    imuKnown = true;
  }

  // Position: commanded speed along the heading.
  if (timeKnown) {
    const uint32_t dtMs = nowMs - lastMs;
    if (dtMs > 0 && dtMs <= MAX_STEP_MS) {
      const float speed = (leftPermille + rightPermille) * 0.5f * MM_PER_S_PER_PERMILLE;
      const float rad = current.headingDeg * 0.0174533f;
      current.xMm += speed * cosf(rad) * dtMs / 1000.0f;
      current.yMm += speed * sinf(rad) * dtMs / 1000.0f;
    }
  }
  lastMs = nowMs;
  timeKnown = true;
}

Pose PoseEstimate::pose() { return current; }
