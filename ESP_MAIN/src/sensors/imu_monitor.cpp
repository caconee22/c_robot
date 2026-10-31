#include "sensors/imu_monitor.hpp"

#if HW_VERSION == 2

#include <Wire.h>
#include <cstring>

namespace {
using namespace config::imu;

constexpr uint8_t REG_SMPLRT_DIV = 0x19;
constexpr uint8_t REG_INT_ENABLE = 0x38;
constexpr uint8_t REG_INT_STATUS = 0x3A;  // followed by 14 data bytes from 0x3B
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_WHO_AM_I = 0x75;
constexpr uint8_t DATA_READY = 0x01;
constexpr uint8_t BLOCK_SIZE = 15;
// SMPLRT_DIV 9 (100 Hz), CONFIG DLPF 44 Hz, GYRO ±1000 dps, ACCEL ±8 g.
constexpr uint8_t CONFIG_BLOCK[] = {9, 0x03, 0x10, 0x10};

ImuStatus current;
ImuSample sample;
uint32_t stateSinceMs = 0, lastPollMs = 0;
uint8_t consecutiveErrors = 0;
bool resetSent = false;
uint8_t wakeStep = 0;  // 0 WHO_AM_I, 1 config block, 2 INT_ENABLE
uint8_t lastRaw[14] = {};

bool write(uint8_t reg, const uint8_t* data, size_t size) {
  Wire.beginTransmission(ADDRESS);
  Wire.write(reg);
  Wire.write(data, size);
  return Wire.endTransmission() == 0;
}

bool read(uint8_t reg, uint8_t* data, uint8_t size) {
  Wire.beginTransmission(ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(ADDRESS, size) != size) return false;
  for (uint8_t i = 0; i < size; ++i) data[i] = static_cast<uint8_t>(Wire.read());
  return true;
}

void enter(ImuState state, uint32_t nowMs) {
  current.state = state;
  stateSinceMs = nowMs;
}

void busError(uint32_t nowMs) {
  ++current.busErrors;
  current.lastError = ImuError::Bus;
  if (++consecutiveErrors >= BUS_ERRORS_TO_FAULT) enter(ImuState::Fault, nowMs);
}

int16_t word(const uint8_t* p) { return static_cast<int16_t>((p[0] << 8) | p[1]); }

void poll(uint32_t nowMs) {
  uint8_t block[BLOCK_SIZE];
  if (!read(REG_INT_STATUS, block, BLOCK_SIZE)) { busError(nowMs); return; }
  consecutiveErrors = 0;
  if (!(block[0] & DATA_READY)) return;  // reading INT_STATUS clears the flag
  const uint8_t* raw = block + 1;
  for (uint8_t i = 0; i < 3; ++i) {
    sample.accel[i] = word(raw + 2 * i);
    sample.gyro[i] = word(raw + 8 + 2 * i);
  }
  sample.temperature = word(raw + 6);
  sample.timestampMs = nowMs;
  ++current.samples;
  current.lastSampleMs = nowMs;
  if (current.samples == 1 || memcmp(raw, lastRaw, sizeof(lastRaw)) != 0) {
    memcpy(lastRaw, raw, sizeof(lastRaw));
    current.lastChangeMs = nowMs;
  }
}
}  // namespace

void ImuMonitor::begin(uint32_t nowMs) {
  current = ImuStatus{};
  requestReset(nowMs);
  current.resets = 0;  // the power-on reset is not an operator reset
}

void ImuMonitor::requestReset(uint32_t nowMs) {
  const uint32_t errors = current.busErrors, resets = current.resets + 1;
  current = ImuStatus{};
  current.busErrors = errors;
  current.resets = resets;
  sample = ImuSample{};
  consecutiveErrors = 0;
  memset(lastRaw, 0, sizeof(lastRaw));
  resetSent = false;  // the reset write happens in update(); this call never touches the bus
  enter(ImuState::Resetting, nowMs);
}

void ImuMonitor::update(uint32_t nowMs) {
  switch (current.state) {
    case ImuState::Off:
      break;
    case ImuState::Resetting: {
      if (!resetSent) {
        const uint8_t reset = 0x80;  // DEVICE_RESET
        if (write(REG_PWR_MGMT_1, &reset, 1)) { consecutiveErrors = 0; resetSent = true; stateSinceMs = nowMs; }
        else busError(nowMs);
        break;
      }
      if (nowMs - stateSinceMs < RESET_WAIT_MS) break;
      const uint8_t wake = 0x01;  // clock from gyro X PLL, sleep off
      if (write(REG_PWR_MGMT_1, &wake, 1)) { consecutiveErrors = 0; wakeStep = 0; enter(ImuState::Waking, nowMs); }
      else busError(nowMs);
      break;
    }
    case ImuState::Waking: {
      if (nowMs - stateSinceMs < WAKE_WAIT_MS) break;
      // One short transaction per update: WHO_AM_I, then config, then DATA_RDY_EN.
      bool ok = false;
      if (wakeStep == 0) {
        uint8_t who = 0;
        ok = read(REG_WHO_AM_I, &who, 1);
        if (ok) {
          current.whoAmI = who;
          // 0x68 is genuine; 0x70/0x72/0x98 are common MPU6050-compatible clones.
          if (who != 0x68 && who != 0x70 && who != 0x72 && who != 0x98) {
            current.lastError = ImuError::WhoAmI;
            enter(ImuState::Fault, nowMs);
            break;
          }
        }
      } else if (wakeStep == 1) {
        ok = write(REG_SMPLRT_DIV, CONFIG_BLOCK, sizeof(CONFIG_BLOCK));
      } else {
        const uint8_t dataReadyEnable = DATA_READY;  // INT_STATUS reports samples; INT pin unused
        ok = write(REG_INT_ENABLE, &dataReadyEnable, 1);
      }
      if (!ok) { busError(nowMs); break; }
      consecutiveErrors = 0;
      if (++wakeStep < 3) break;
      enter(ImuState::Ready, nowMs);
      lastPollMs = nowMs;
      current.lastSampleMs = current.lastChangeMs = nowMs;
      break;
    }
    case ImuState::Ready:
      if (nowMs - lastPollMs >= POLL_PERIOD_MS) {
        lastPollMs = nowMs;
        poll(nowMs);
      }
      break;
    case ImuState::Fault:
      break;
  }
  current.dataFresh = current.state == ImuState::Ready && current.samples > 0 &&
                      nowMs - current.lastSampleMs <= STALE_MS &&
                      nowMs - current.lastChangeMs <= FROZEN_MS;
}

ImuStatus ImuMonitor::status() { return current; }
ImuSample ImuMonitor::latest() { return sample; }

#else  // V1 has no IMU: keep the API linkable and permanently Off.

void ImuMonitor::begin(uint32_t) {}
void ImuMonitor::requestReset(uint32_t) {}
void ImuMonitor::update(uint32_t) {}
ImuStatus ImuMonitor::status() { return ImuStatus{}; }
ImuSample ImuMonitor::latest() { return ImuSample{}; }

#endif

const char* ImuMonitor::stateName(ImuState state) {
  switch (state) {
    case ImuState::Off: return "OFF";
    case ImuState::Resetting: return "RESETTING";
    case ImuState::Waking: return "WAKING";
    case ImuState::Ready: return "READY";
    case ImuState::Fault: return "FAULT";
  }
  return "UNKNOWN";
}
