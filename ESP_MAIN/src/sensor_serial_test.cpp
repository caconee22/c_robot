#include <Arduino.h>

#include "pins.hpp"
#include "sensors/color_sensor_manager.hpp"
#include "sensors/color_sensor_types.hpp"
#include "sensors/sensor_config.hpp"

namespace {

constexpr uint32_t DEBUG_BAUD = 115200;
constexpr uint32_t PRINT_INTERVAL_MS = 100;

uint32_t lastPrintMs = 0;

void initializeInputs() {
  pinMode(pins::SAFETY_SWITCH, INPUT_PULLUP);
  pinMode(pins::START_SWITCH, INPUT_PULLUP);
}

void printColorSensor(uint8_t index) {
  const ColorRawSample sample = ColorSensorManager::sample(index);
  Serial.printf("  S%u %-11s: ", static_cast<unsigned>(index + 1),
                sensorCornerName(index));
  if (!ColorSensorManager::available(index)) {
    Serial.printf("DISABLED error=%u\n", static_cast<unsigned>(ColorSensorManager::status().error[index]));
    return;
  }
  if (!sample.valid) {
    Serial.printf("INVALID error=%u ", static_cast<unsigned>(ColorSensorManager::status().error[index]));
    // Keep raw counts visible for close-range saturation / weak-light tuning.
  }

  if (ColorSensorManager::model() == ColorSensorModel::AS7341) {
    Serial.printf(
        "F2=%u F3=%u F5=%u F6=%u F7=%u C=%u seq=%lu age=%lums\n",
        sample.channel[1], sample.channel[2], sample.channel[4],
        sample.channel[5], sample.channel[6], sample.channel[8],
        static_cast<unsigned long>(sample.sequence),
        static_cast<unsigned long>(millis() - sample.timestampMs));
  } else {
    Serial.printf("R=%u G=%u B=%u C=%u\n", sample.channel[0],
                  sample.channel[1], sample.channel[2], sample.channel[8]);
  }
}

}  // namespace

void setup() {
  Serial.begin(DEBUG_BAUD, SERIAL_8N1, pins::UART0_RX, pins::UART0_TX);
  initializeInputs();

  Serial.println();
  Serial.println("ESP32-S3 all-sensor serial test");
  Serial.println("Sensor order: 1 FL, 2 FR, 3 RL, 4 RR.");
  Serial.println("Only the four color sensors are tested in V1.");
  Serial.println("Motor pins are not initialized or changed by this test.");
  const bool ready = ColorSensorManager::begin();
  const ColorSensorStatus status = ColorSensorManager::status();
  if (status.model == ColorSensorModel::AS7341) {
    Serial.printf("AS7341 continuous F2/F3/F5/F6/F7/C exposure=%luus gainCode=%u LED=%u\n",
                  static_cast<unsigned long>(config::sensors::AS7341_EXPOSURE_US),
                  static_cast<unsigned>(config::sensors::AS7341_GAIN),
                  config::sensors::AS7341_LED_ENABLED ? 1 : 0);
  }
  Serial.printf("Color model=%s readyMask=0x%02X allReady=%u\n",
                status.model == ColorSensorModel::AS7341 ? "AS7341"
                                                         : "TCS34725",
                status.readyMask, ready ? 1 : 0);
  lastPrintMs = millis() - PRINT_INTERVAL_MS;
}

void loop() {
  const uint32_t nowMs = millis();
  ColorSensorManager::update(nowMs);
  if (nowMs - lastPrintMs < PRINT_INTERVAL_MS) return;
  lastPrintMs = nowMs;

  Serial.printf("[%10lu ms] SW1_SAFE=%d SW2_START=%d\n",
                static_cast<unsigned long>(nowMs),
                digitalRead(pins::SAFETY_SWITCH),
                digitalRead(pins::START_SWITCH));
  for (uint8_t index = 0; index < COLOR_SENSOR_COUNT; ++index) {
    printColorSensor(index);
  }
  Serial.println();
  delay(1);
}
