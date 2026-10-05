#include "sensors/color_sensor_manager.hpp"

#include <Wire.h>

#include "pins.hpp"
#include "sensors/sensor_config.hpp"

namespace {

using namespace config::sensors;

#if COLOR_SENSOR_MODEL == COLOR_SENSOR_AS7341
constexpr uint8_t SENSOR_ADDRESS = 0x39;
// Same SMUX routing as Adafruit AS7341 1.4.1: F1..F4 / F5..F8, Clear, NIR.
constexpr uint8_t SMUX_LOW[20] = {
    0x30, 0x01, 0, 0, 0, 0x42, 0, 0, 0x50, 0,
    0, 0, 0x20, 0x04, 0, 0x30, 0x01, 0x50, 0, 0x06};
constexpr uint8_t SMUX_HIGH[20] = {
    0, 0, 0, 0x40, 0x02, 0, 0x10, 0x03, 0x50, 0x10,
    0x03, 0, 0, 0, 0x24, 0, 0, 0x50, 0, 0x06};
#elif COLOR_SENSOR_MODEL == COLOR_SENSOR_TCS34725
constexpr uint8_t SENSOR_ADDRESS = 0x29;
#else
#error "Unsupported COLOR_SENSOR_MODEL"
#endif

// Each visit performs a small I2C step; integration waits never block loop().
enum class ReadPhase : uint8_t { Start, LowSmux, LowData, HighSmux, HighData };
struct SensorRead {
  ReadPhase phase = ReadPhase::Start;
  uint32_t phaseStartedMs = 0;
  uint16_t low[6] = {};
  bool saturated = false;
};

ColorRawSample samples[::COLOR_SENSOR_COUNT];
SensorRead reads[::COLOR_SENSOR_COUNT];
ColorSensorStatus latestStatus;
uint8_t nextSensor = 0;
uint32_t lastSlotMs = 0;

bool selectChannel(uint8_t channel) {
  if (channel > 7) return false;
  Wire.beginTransmission(TCA9548A_ADDRESS);
  Wire.write(static_cast<uint8_t>(1U << channel));
  return Wire.endTransmission() == 0;
}

bool writeRegister(uint8_t reg, const uint8_t* data, size_t size) {
  Wire.beginTransmission(SENSOR_ADDRESS);
  Wire.write(reg);
  Wire.write(data, size);
  return Wire.endTransmission() == 0;
}

bool writeRegister(uint8_t reg, uint8_t value) {
  return writeRegister(reg, &value, 1);
}

bool readRegister(uint8_t reg, uint8_t* data, size_t size) {
  Wire.beginTransmission(SENSOR_ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(SENSOR_ADDRESS, static_cast<uint8_t>(size)) != size) {
    return false;
  }
  for (size_t i = 0; i < size; ++i) data[i] = static_cast<uint8_t>(Wire.read());
  return true;
}

bool readByte(uint8_t reg, uint8_t& value) {
  return readRegister(reg, &value, 1);
}

#if COLOR_SENSOR_MODEL == COLOR_SENSOR_AS7341
bool readSpectrum(uint16_t* words, bool& saturated) {
  // ASTATUS latches all six ADCs and reports analog/digital saturation.
  uint8_t bytes[13];
  if (!readRegister(0x94, bytes, sizeof(bytes))) return false;
  saturated = saturated || (bytes[0] & 0x80) != 0;
  for (size_t i = 0; i < 6; ++i) {
    words[i] = bytes[i * 2 + 1] |
               (static_cast<uint16_t>(bytes[i * 2 + 2]) << 8);
  }
  return true;
}
#else
bool readWords(uint8_t reg, uint16_t* words, size_t count) {
  uint8_t bytes[12];
  if (!readRegister(reg, bytes, count * 2)) return false;
  for (size_t i = 0; i < count; ++i) {
    words[i] = bytes[i * 2] | (static_cast<uint16_t>(bytes[i * 2 + 1]) << 8);
  }
  return true;
}
#endif

void failSensor(uint8_t index, ColorSensorError error) {
  latestStatus.error[index] = error;
  latestStatus.readyMask &= ~(1U << index);
  samples[index].valid = false;
  samples[index].timestampMs = millis();
  ++samples[index].sequence;
  ++latestStatus.failedReads;
  // Bus/timeout faults stay disabled until reboot. No reset/retry machinery.
}

bool initializeSensor(uint8_t index) {
  if (!selectChannel(COLOR_SENSOR_TCA_CHANNELS[index])) return false;
  uint8_t id = 0;
#if COLOR_SENSOR_MODEL == COLOR_SENSOR_AS7341
  if (!readByte(0x92, id) || (id & 0xFC) != 0x24) return false;
  if (!writeRegister(0x80, 0x01)) return false;
  delay(1);  // Initial internal setup is about 300 us; boot only.
  return writeRegister(0xA9, 0x00) &&  // High register bank.
         writeRegister(0x81, AS7341_INTEGRATION_ATIME) &&
         writeRegister(0xCA, static_cast<uint8_t>(AS7341_INTEGRATION_ASTEP)) &&
         writeRegister(0xCB, static_cast<uint8_t>(AS7341_INTEGRATION_ASTEP >> 8)) &&
         writeRegister(0xAA, static_cast<uint8_t>(AS7341_GAIN));
#else
  if (!readByte(0x92, id) || (id != 0x44 && id != 0x4D)) return false;
  if (!writeRegister(0x80, 0x01)) return false;
  delay(3);  // Power-on oscillator startup, boot only.
  return writeRegister(0x81, static_cast<uint8_t>(TCS34725_INTEGRATION)) &&
         writeRegister(0x8F, static_cast<uint8_t>(TCS34725_GAIN));
#endif
}

void publishSample(uint8_t index, ColorRawSample& sample, uint16_t limit) {
  sample.timestampMs = millis();
  sample.sequence = samples[index].sequence + 1;
  sample.brightness = sample.channel[8];
  sample.valid = sample.brightness > 0;
#if COLOR_SENSOR_MODEL == COLOR_SENSOR_AS7341
  if (reads[index].saturated || reads[index].low[4] >= limit ||
      reads[index].low[5] >= limit) sample.valid = false;
#endif
  for (size_t i = 0; i < COLOR_CHANNEL_COUNT; ++i) {
    if (sample.channel[i] >= limit) sample.valid = false;
  }
  if (sample.valid) {
#if COLOR_SENSOR_MODEL == COLOR_SENSOR_AS7341
    sample.featureMask = 0x02FFU;
    for (size_t i = 0; i < 8; ++i) {
      // F1..F4 and F5..F8 are acquired at different times.
      const uint16_t clear = i < 4 ? reads[index].low[4] : sample.brightness;
      if (clear == 0) { sample.valid = false; break; }
      sample.normalized[i] = static_cast<float>(sample.channel[i]) / clear;
    }
    sample.normalized[9] = static_cast<float>(sample.channel[9]) / sample.brightness;
#else
    sample.featureMask = 0x0007U;
    for (size_t i = 0; i < 3; ++i) {
      sample.normalized[i] = static_cast<float>(sample.channel[i]) / sample.brightness;
    }
#endif
  }
  if (!sample.valid) sample.featureMask = 0;
  samples[index] = sample;
  latestStatus.error[index] = sample.valid ? ColorSensorError::None
                                           : ColorSensorError::Range;
  if (sample.valid) ++latestStatus.successfulReads;
  else ++latestStatus.failedReads;
  reads[index].phase = ReadPhase::Start;
}

#if COLOR_SENSOR_MODEL == COLOR_SENSOR_AS7341
bool configureSmux(bool high) {
  return writeRegister(0x80, 0x01) && writeRegister(0xAF, 0x10) &&
         writeRegister(0x00, high ? SMUX_HIGH : SMUX_LOW, 20) &&
         writeRegister(0x80, 0x11);
}

bool stepSensor(uint8_t index, uint32_t nowMs) {
  SensorRead& read = reads[index];
  uint8_t status = 0;
  if (read.phase == ReadPhase::Start) {
    read.saturated = false;
    if (!configureSmux(false)) return false;
    read.phase = ReadPhase::LowSmux;
    read.phaseStartedMs = millis();
  } else if (read.phase == ReadPhase::LowSmux ||
             read.phase == ReadPhase::HighSmux) {
    if (!readByte(0x80, status)) return false;
    if ((status & 0x10) == 0) {
      if (!writeRegister(0x80, 0x03)) return false;
      read.phase = read.phase == ReadPhase::LowSmux ? ReadPhase::LowData
                                                   : ReadPhase::HighData;
      read.phaseStartedMs = millis();
    } else if (nowMs - read.phaseStartedMs >= SMUX_TIMEOUT_MS) {
      failSensor(index, ColorSensorError::Timeout);
    }
  } else {
    if (!readByte(0xA3, status)) return false;
    if ((status & 0x40) == 0) {
      if (nowMs - read.phaseStartedMs >=
          AS7341_INTEGRATION_MS + CONVERSION_TIMEOUT_MARGIN_MS) {
        failSensor(index, ColorSensorError::Timeout);
      }
      return true;
    }
    if (read.phase == ReadPhase::LowData) {
      if (!readSpectrum(read.low, read.saturated) || !configureSmux(true)) return false;
      read.phase = ReadPhase::HighSmux;
      read.phaseStartedMs = millis();
    } else {
      uint16_t high[6];
      if (!readSpectrum(high, read.saturated) || !writeRegister(0x80, 0x01)) return false;
      ColorRawSample sample;
      for (size_t i = 0; i < 4; ++i) {
        sample.channel[i] = read.low[i];
        sample.channel[i + 4] = high[i];
      }
      sample.channel[8] = high[4];
      sample.channel[9] = high[5];
      const uint32_t counts = (AS7341_INTEGRATION_ATIME + 1UL) *
                               (AS7341_INTEGRATION_ASTEP + 1UL);
      publishSample(index, sample, static_cast<uint16_t>(min(counts, uint32_t{65535})));
    }
  }
  return true;
}
#else
bool stepSensor(uint8_t index, uint32_t nowMs) {
  SensorRead& read = reads[index];
  if (read.phase == ReadPhase::Start) {
    if (!writeRegister(0x80, 0x03)) return false;
    read.phase = ReadPhase::LowData;
    read.phaseStartedMs = millis();
    return true;
  }
  if (nowMs - read.phaseStartedMs < TCS34725_INTEGRATION_MS + 1) return true;
  uint8_t status = 0;
  if (!readByte(0x93, status)) return false;
  if ((status & 0x01) == 0) {
    if (nowMs - read.phaseStartedMs >=
        TCS34725_INTEGRATION_MS + CONVERSION_TIMEOUT_MARGIN_MS) {
      failSensor(index, ColorSensorError::Timeout);
    }
    return true;
  }
  uint16_t rgbc[4];
  if (!readWords(0xB4, rgbc, 4) || !writeRegister(0x80, 0x01)) return false;
  ColorRawSample sample;
  sample.channel[8] = rgbc[0];
  for (size_t i = 0; i < 3; ++i) sample.channel[i] = rgbc[i + 1];
  const uint32_t counts = (256UL - static_cast<uint8_t>(TCS34725_INTEGRATION)) * 1024UL;
  publishSample(index, sample, static_cast<uint16_t>(min(counts, uint32_t{65535})));
  return true;
}
#endif

}  // namespace

bool ColorSensorManager::begin() {
  latestStatus = ColorSensorStatus{};
  latestStatus.model = static_cast<ColorSensorModel>(COLOR_SENSOR_MODEL);
  Wire.begin(pins::I2C_SDA, pins::I2C_SCL);
  Wire.setClock(config::sensors::I2C_FREQUENCY_HZ);
  Wire.setTimeOut(config::sensors::I2C_TIMEOUT_MS);
  for (uint8_t i = 0; i < ::COLOR_SENSOR_COUNT; ++i) {
    samples[i] = ColorRawSample{};
    reads[i] = SensorRead{};
    if (initializeSensor(i)) latestStatus.readyMask |= 1U << i;
    else failSensor(i, ColorSensorError::Initialization);
  }
  nextSensor = 0;
  lastSlotMs = millis() - config::sensors::COLOR_SENSOR_SLOT_PERIOD_MS;
  return latestStatus.readyMask == 0x0F;
}

void ColorSensorManager::update(uint32_t nowMs) {
  if (nowMs - lastSlotMs < config::sensors::COLOR_SENSOR_SLOT_PERIOD_MS) return;
  lastSlotMs = nowMs;
  latestStatus.lastReadSensor = nextSensor;
  if (available(nextSensor)) {
    if (!selectChannel(config::sensors::COLOR_SENSOR_TCA_CHANNELS[nextSensor]) ||
        !stepSensor(nextSensor, nowMs)) {
      failSensor(nextSensor, ColorSensorError::I2c);
    }
  }
  nextSensor = (nextSensor + 1) % ::COLOR_SENSOR_COUNT;
}

ColorRawSample ColorSensorManager::sample(uint8_t index) {
  return index < ::COLOR_SENSOR_COUNT ? samples[index] : ColorRawSample{};
}

ColorSensorStatus ColorSensorManager::status() { return latestStatus; }

bool ColorSensorManager::available(uint8_t index) {
  return index < ::COLOR_SENSOR_COUNT && (latestStatus.readyMask & (1U << index)) != 0;
}

ColorSensorModel ColorSensorManager::model() { return latestStatus.model; }

uint32_t ColorSensorManager::settingsSignature() {
  uint32_t signature = 2166136261UL;
  const auto mix = [&signature](uint32_t value) {
    signature = (signature ^ value) * 16777619UL;
  };
  mix(COLOR_SENSOR_MODEL);
#if COLOR_SENSOR_MODEL == COLOR_SENSOR_AS7341
  mix(config::sensors::AS7341_INTEGRATION_ATIME);
  mix(config::sensors::AS7341_INTEGRATION_ASTEP);
  mix(static_cast<uint8_t>(config::sensors::AS7341_GAIN));
#else
  mix(static_cast<uint8_t>(config::sensors::TCS34725_INTEGRATION));
  mix(static_cast<uint8_t>(config::sensors::TCS34725_GAIN));
#endif
  // Routing and per-bank normalization also affect calibration validity.
  mix(2);
  for (uint8_t channel : config::sensors::COLOR_SENSOR_TCA_CHANNELS) mix(channel);
  return signature;
}
