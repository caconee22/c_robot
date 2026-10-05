#include "sensors/color_sensor_manager.hpp"

#include <Wire.h>

#include "pins.hpp"
#include "sensors/sensor_config.hpp"

namespace {

using namespace config::sensors;

#if COLOR_SENSOR_MODEL == COLOR_SENSOR_AS7341
constexpr uint8_t SENSOR_ADDRESS = 0x39;
// Adafruit photodiode positions; ADC0..5 = F2,F3,F5,F6,F7,Clear.
constexpr uint8_t SMUX_FLOOR[20] = {
    0x20, 0, 0, 0, 0x04, 0x01, 0x30, 0x05, 0x60, 0x30,
    0x05, 0, 0x10, 0, 0x40, 0x20, 0, 0x60, 0, 0};
constexpr uint8_t FLOOR_CHANNELS[6] = {1, 2, 4, 5, 6, 8};
#elif COLOR_SENSOR_MODEL == COLOR_SENSOR_TCS34725
constexpr uint8_t SENSOR_ADDRESS = 0x29;
#else
#error "Unsupported COLOR_SENSOR_MODEL"
#endif

// Each visit performs a small I2C step; integration waits never block loop().
enum class ReadPhase : uint8_t { Start, LowSmux, LowData };
struct SensorRead {
  ReadPhase phase = ReadPhase::Start;
  uint32_t phaseStartedMs = 0;
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
  const uint16_t ledMa = AS7341_LED_CURRENT_MA[index];
  if (ledMa < 4 || ledMa > 150 || (ledMa & 1U)) return false;
  if (!writeRegister(0xA9, 0x10) ||  // Low bank: CONFIG and LED.
      !writeRegister(0x70, AS7341_LED_ENABLED ? 0x08 : 0x00) ||
      !writeRegister(0x74, static_cast<uint8_t>(
          (AS7341_LED_ENABLED ? 0x80 : 0) | ((ledMa - 4) / 2)))) return false;
  return writeRegister(0xA9, 0x00) &&  // High register bank.
         writeRegister(0x81, AS7341_INTEGRATION_ATIME) &&
         writeRegister(0xCA, static_cast<uint8_t>(AS7341_INTEGRATION_ASTEP)) &&
         writeRegister(0xCB, static_cast<uint8_t>(AS7341_INTEGRATION_ASTEP >> 8)) &&
         writeRegister(0xAA, static_cast<uint8_t>(AS7341_GAIN)) &&
         writeRegister(0xD6, AS7341_AUTOZERO_INTERVAL) &&
         writeRegister(0xBD, 0x00) &&  // AINT for every completed cycle.
         writeRegister(0xF9, 0x04) &&  // SP_IEN; no external INT wiring needed.
         writeRegister(0x93, 0x08);
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
  sample.valid = sample.brightness >= MINIMUM_CLEAR_COUNTS && sample.brightness > 0;
#if COLOR_SENSOR_MODEL == COLOR_SENSOR_AS7341
  if (reads[index].saturated) sample.valid = false;
#endif
  for (size_t i = 0; i < COLOR_CHANNEL_COUNT; ++i) {
    if (sample.channel[i] >= limit) sample.valid = false;
  }
  if (sample.valid) {
#if COLOR_SENSOR_MODEL == COLOR_SENSOR_AS7341
    sample.featureMask = 0x0076U;
    for (size_t i = 0; i < 8; ++i) {
      if ((sample.featureMask & (1U << i)) != 0)
        sample.normalized[i] = static_cast<float>(sample.channel[i]) / sample.brightness;
    }
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
#if COLOR_SENSOR_MODEL == COLOR_SENSOR_TCS34725
  reads[index].phase = ReadPhase::Start;
#endif
}

#if COLOR_SENSOR_MODEL == COLOR_SENSOR_AS7341
bool configureSmux() {
  return writeRegister(0x80, 0x01) && writeRegister(0xAF, 0x10) &&
         writeRegister(0x00, SMUX_FLOOR, 20) &&
         writeRegister(0x80, 0x11);
}

bool stepSensor(uint8_t index, uint32_t nowMs) {
  SensorRead& read = reads[index];
  uint8_t status = 0;
  if (read.phase == ReadPhase::Start) {
    read.saturated = false;
    if (!configureSmux()) return false;
    read.phase = ReadPhase::LowSmux;
    read.phaseStartedMs = millis();
  } else if (read.phase == ReadPhase::LowSmux) {
    if (!readByte(0x80, status)) return false;
    if ((status & 0x10) == 0) {
      if (!writeRegister(0x80, 0x03)) return false;
      read.phase = ReadPhase::LowData;
      read.phaseStartedMs = millis();
    } else if (nowMs - read.phaseStartedMs >= SMUX_TIMEOUT_MS) {
      failSensor(index, ColorSensorError::Timeout);
    }
  } else {
    // AVALID alone cannot identify a new cycle in continuous mode.
    if (!readByte(0x93, status)) return false;
    if ((status & 0x08) == 0) {
      if (nowMs - read.phaseStartedMs >=
          AS7341_INTEGRATION_MS + CONVERSION_TIMEOUT_MARGIN_MS) {
        failSensor(index, ColorSensorError::Timeout);
      }
      return true;
    }
    // Also avoid recounting a cycle that completed during a previous burst.
    if (nowMs - read.phaseStartedMs < AS7341_INTEGRATION_MS) return true;
    uint16_t words[6];
    read.saturated = false;
    if (!writeRegister(0x93, 0x08) || !readSpectrum(words, read.saturated)) return false;
    ColorRawSample sample;
    for (size_t i = 0; i < 6; ++i) sample.channel[FLOOR_CHANNELS[i]] = words[i];
    const uint32_t counts = (AS7341_INTEGRATION_ATIME + 1UL) *
                           (AS7341_INTEGRATION_ASTEP + 1UL);
    publishSample(index, sample, static_cast<uint16_t>(min(counts, uint32_t{65535})));
    read.phaseStartedMs = millis();
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
  for (uint8_t visit = 0; visit < COLOR_SENSOR_VISITS_PER_UPDATE; ++visit) {
    latestStatus.lastReadSensor = nextSensor;
    if (available(nextSensor)) {
      if (!selectChannel(COLOR_SENSOR_TCA_CHANNELS[nextSensor]) ||
          !stepSensor(nextSensor, millis())) {
        failSensor(nextSensor, ColorSensorError::I2c);
      }
    }
    nextSensor = (nextSensor + 1) % ::COLOR_SENSOR_COUNT;
  }
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
  mix(config::sensors::AS7341_LED_ENABLED);
  mix(config::sensors::AS7341_AUTOZERO_INTERVAL);
  for (uint16_t current : config::sensors::AS7341_LED_CURRENT_MA) mix(current);
#else
  mix(static_cast<uint8_t>(config::sensors::TCS34725_INTEGRATION));
  mix(static_cast<uint8_t>(config::sensors::TCS34725_GAIN));
#endif
  // Routing and per-bank normalization also affect calibration validity.
  mix(3);  // Single-exposure floor routing.
  for (uint8_t channel : config::sensors::COLOR_SENSOR_TCA_CHANNELS) mix(channel);
  return signature;
}
