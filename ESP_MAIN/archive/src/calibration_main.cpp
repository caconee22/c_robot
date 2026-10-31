// ARCHIVED 2026-10-31: calibration now runs inside robot_bench_v2_manual.
// Separate color calibration firmware (PlatformIO env color_calibration).
// It never initializes the motors: every BTS7960 PWM/EN pin is held LOW.
// Upload the production firmware again after CAL SAVE.

#include <Arduino.h>

#include "calibration/color_calibration.hpp"
#include "config.hpp"
#include "hardware/board_io.hpp"
#include "pins.hpp"
#include "sensors/calibration_store.hpp"
#include "sensors/color_sensor_manager.hpp"

void setup() {
  BoardIo::setEarlySafeState();
  Serial.begin(config::uart::EVENT_LOG_BAUD, SERIAL_8N1, pins::UART0_RX,
               pins::UART0_TX);
  ColorSensorManager::begin();
  CalibrationStore::load();
  ColorCalibration::begin();
  Serial.printf("[CAL] Stored calibration: %s\n",
                ColorCalibration::status().storedDataValid ? "VALID" : "NONE");
  Serial.println("[CAL] Motors are not initialized in this firmware.");
  Serial.println("[CAL] Type CAL START to begin, CAL ERASE to delete.");
  Serial.printf("[CAL] Sensor ready mask: 0x%02X\n",
                ColorSensorManager::status().readyMask);
}

void loop() {
  ColorSensorManager::update(millis());
  ColorCalibration::update(millis());
  delay(1);
}
