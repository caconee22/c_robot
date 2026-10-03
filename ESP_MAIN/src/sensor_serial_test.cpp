#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_AS7341.h>

#include "pins.hpp"

namespace {

constexpr uint32_t DEBUG_BAUD = 115200;
constexpr uint32_t PRINT_INTERVAL_MS = 100;
constexpr uint32_t I2C_FREQUENCY_HZ = 400000;
constexpr uint8_t TCA9548A_ADDRESS = 0x70;
constexpr uint8_t AS7341_COUNT = 4;

// Change this list if the physical TCA9548A channel order is different.
constexpr uint8_t AS7341_TCA_CHANNELS[AS7341_COUNT] = {0, 1, 2, 3};

Adafruit_AS7341 colorSensors[AS7341_COUNT];
bool colorSensorReady[AS7341_COUNT] = {false, false, false, false};
uint32_t lastPrintMs = 0;

bool selectTcaChannel(uint8_t channel) {
  if (channel > 7) {
    return false;
  }

  Wire.beginTransmission(TCA9548A_ADDRESS);
  Wire.write(1U << channel);
  return Wire.endTransmission() == 0;
}

void disableAllTcaChannels() {
  Wire.beginTransmission(TCA9548A_ADDRESS);
  Wire.write(0);
  Wire.endTransmission();
}

void initializeDigitalInputs() {
  pinMode(pins::IR1, INPUT);
  pinMode(pins::IR2, INPUT);
  pinMode(pins::IR3, INPUT);
  pinMode(pins::IR4, INPUT);
  pinMode(pins::IR5_RESERVED, INPUT);
  pinMode(pins::IR6_RESERVED, INPUT);
  pinMode(pins::IR7_RESERVED, INPUT);
  pinMode(pins::IR8_RESERVED, INPUT);

  pinMode(pins::SAFETY_SWITCH, INPUT_PULLUP);
  pinMode(pins::START_SWITCH, INPUT_PULLUP);
}

void initializeColorSensors() {
  Serial.println("AS7341 initialization:");

  for (uint8_t index = 0; index < AS7341_COUNT; ++index) {
    const uint8_t channel = AS7341_TCA_CHANNELS[index];
    Serial.printf("  AS%u / TCA channel %u: ", index + 1, channel);

    if (!selectTcaChannel(channel)) {
      Serial.println("TCA channel select failed");
      continue;
    }

    if (!colorSensors[index].begin()) {
      Serial.println("sensor not found");
      continue;
    }

    // About 10 ms integration time, allowing all four sensors to be read
    // within the 100 ms reporting interval.
    colorSensors[index].setATIME(9);
    colorSensors[index].setASTEP(359);
    colorSensors[index].setGain(AS7341_GAIN_16X);
    colorSensorReady[index] = true;
    Serial.println("OK");
  }

  disableAllTcaChannels();
}

void printDigitalInputs(uint32_t sampleMs) {
  Serial.printf(
      "[%10lu ms] IR1=%d IR2=%d IR3=%d IR4=%d "
      "IR5=%d IR6=%d IR7=%d IR8=%d SW1_SAFE=%d SW2_START=%d\n",
      static_cast<unsigned long>(sampleMs), digitalRead(pins::IR1),
      digitalRead(pins::IR2), digitalRead(pins::IR3),
      digitalRead(pins::IR4), digitalRead(pins::IR5_RESERVED),
      digitalRead(pins::IR6_RESERVED), digitalRead(pins::IR7_RESERVED),
      digitalRead(pins::IR8_RESERVED), digitalRead(pins::SAFETY_SWITCH),
      digitalRead(pins::START_SWITCH));
}

void printColorSensor(uint8_t index) {
  const uint8_t channel = AS7341_TCA_CHANNELS[index];
  Serial.printf("  AS%u(CH%u): ", index + 1, channel);

  if (!colorSensorReady[index]) {
    Serial.println("NOT_READY");
    return;
  }

  if (!selectTcaChannel(channel)) {
    Serial.println("TCA_ERROR");
    return;
  }

  if (!colorSensors[index].readAllChannels()) {
    Serial.println("READ_ERROR");
    return;
  }

  Serial.printf(
      "F1_415=%u F2_445=%u F3_480=%u F4_515=%u "
      "F5_555=%u F6_590=%u F7_630=%u F8_680=%u CLEAR=%u NIR=%u\n",
      colorSensors[index].getChannel(AS7341_CHANNEL_415nm_F1),
      colorSensors[index].getChannel(AS7341_CHANNEL_445nm_F2),
      colorSensors[index].getChannel(AS7341_CHANNEL_480nm_F3),
      colorSensors[index].getChannel(AS7341_CHANNEL_515nm_F4),
      colorSensors[index].getChannel(AS7341_CHANNEL_555nm_F5),
      colorSensors[index].getChannel(AS7341_CHANNEL_590nm_F6),
      colorSensors[index].getChannel(AS7341_CHANNEL_630nm_F7),
      colorSensors[index].getChannel(AS7341_CHANNEL_680nm_F8),
      colorSensors[index].getChannel(AS7341_CHANNEL_CLEAR),
      colorSensors[index].getChannel(AS7341_CHANNEL_NIR));
}

void printAllSensors(uint32_t sampleMs) {
  printDigitalInputs(sampleMs);

  for (uint8_t index = 0; index < AS7341_COUNT; ++index) {
    printColorSensor(index);
  }

  disableAllTcaChannels();
  Serial.println();
}

}  // namespace

void setup() {
  Serial.begin(DEBUG_BAUD, SERIAL_8N1, pins::UART0_RX, pins::UART0_TX);
  initializeDigitalInputs();

  Wire.begin(pins::I2C_SDA, pins::I2C_SCL);
  Wire.setClock(I2C_FREQUENCY_HZ);

  Serial.println();
  Serial.println("ESP32-S3 all-sensor serial test");
  Serial.println("Digital values are raw: switches are active when the value is 0.");
  Serial.println("Motor control pins are not initialized or changed by this test.");
  initializeColorSensors();
  Serial.println();

  lastPrintMs = millis() - PRINT_INTERVAL_MS;
}

void loop() {
  const uint32_t nowMs = millis();
  if (nowMs - lastPrintMs < PRINT_INTERVAL_MS) {
    return;
  }

  lastPrintMs = nowMs;
  printAllSensors(nowMs);
}
