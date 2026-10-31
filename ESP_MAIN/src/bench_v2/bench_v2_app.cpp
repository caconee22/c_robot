#include "bench_v2/bench_v2_app.hpp"
#include "hardware/reset_manager.hpp"
#if HW_VERSION != 2
#error V2 bench requires HW_VERSION=2
#endif
#include <Arduino.h>
#include <cstdio>
#include <cstring>

#include "app/boot_manager.hpp"
#include "bench/bench_config.hpp"
#include "bench/bench_controller.hpp"
#include "communication/raspberry_link.hpp"
#include "motor/motor_controller.hpp"
#include "sensors/color_sensor_manager.hpp"
#include "sensors/sensor_manager.hpp"
#include "system/battery_monitor.hpp"
#include "system/event_logger.hpp"
#include "system/status_led.hpp"
#include "system/system_manager.hpp"

namespace {
using namespace config::bench;
char input[48];
uint8_t inputLength = 0;
bool discardLine = false, logging = false;
uint32_t lastInputMs = 0, lastLogMs = 0;
uint8_t logRow = 5;
bool batteryReported = false;

void stop() {
  SystemManager::requestStop();
  MotorController::brake();
  BenchController::select(BenchController::mode());
}

void reply(const char* line) {
  const size_t size = strlen(line);
  if (Serial.availableForWrite() >= static_cast<int>(size))
    Serial.write(reinterpret_cast<const uint8_t*>(line), size);
}
void help() {
  reply("MODE STOP|MOTOR|MANUAL|DATA|ALIGN|FOLLOW|AVOID\r\n");
  reply("SW2 press/release to arm; DRIVE -200..200, repeat <250ms\r\n");
  reply("STOP / LOG ON|OFF / STATUS / HELP; UART0 57600\r\n");
}

void execute(const char* line) {
  if (!strcmp(line, "STOP")) { stop(); reply("OK STOP\r\n"); return; }
  if (!strcmp(line, "HELP")) { stop(); help(); return; }
  if (!strcmp(line, "LOG ON")) { logging = true; return; }
  if (!strcmp(line, "LOG OFF")) { logging = false; logRow = 5; return; }
  if (!strcmp(line, "STATUS")) { logging = true; lastLogMs = millis() - LOG_PERIOD_MS; return; }
  if (!strncmp(line, "MODE ", 5)) {
    for (uint8_t i = 0; i <= static_cast<uint8_t>(BenchMode::Avoid); ++i) {
      const auto mode = static_cast<BenchMode>(i);
      if (strcmp(line + 5, BenchController::modeName(mode))) continue;
      stop(); BenchController::select(mode);
      if (mode == BenchMode::Data) logging = true;
      EventLogger::modeChanged(i);
      reply("OK MODE; SW2 required for motion\r\n");
      return;
    }
  }
  if (!strncmp(line, "DRIVE ", 6)) {
    // Bound input before scanf: overflowing integer strings are rejected.
    size_t position = 6;
    int values[2] = {};
    bool valid = true;
    for (int axis = 0; axis < 2; ++axis) {
      while (line[position] == ' ') ++position;
      const bool negative = line[position] == '-';
      if (negative || line[position] == '+') ++position;
      uint8_t digits = 0;
      while (line[position] >= '0' && line[position] <= '9') {
        if (++digits > 3) { valid = false; break; }
        values[axis] = values[axis] * 10 + line[position++] - '0';
      }
      if (!digits || !valid) { valid = false; break; }
      if (negative) values[axis] = -values[axis];
      if (axis == 0 && line[position] != ' ') { valid = false; break; }
    }
    while (line[position] == ' ') ++position;
    if (valid && line[position] == '\0' &&
        BenchController::setManual(values[0], values[1], millis(), SystemManager::motorAllowed())) {
      return;  // Avoid acknowledgements on a fast manual command stream.
    }
  }
  stop(); reply("ERR command rejected; stopped\r\n");
}

void console() {
  if (inputLength && millis() - lastInputMs >= CONSOLE_TIMEOUT_MS) {
    stop(); inputLength = 0; discardLine = true;
  }
  uint8_t budget = CONSOLE_BYTES_PER_LOOP;
  while (budget-- && Serial.available()) {
    const char c = static_cast<char>(Serial.read());
    lastInputMs = millis();
    if (c == '\r' || c == '\n') {
      if (!discardLine && inputLength) { input[inputLength] = 0; execute(input); }
      inputLength = 0; discardLine = false;
    } else if (!discardLine) {
      if (static_cast<size_t>(inputLength) + 1 >= sizeof(input) || c < 32 || c > 126) {
        stop(); inputLength = 0; discardLine = true;
      } else input[inputLength++] = c;
    }
  }
}

// One line once the power-on battery check decides (V2 only), for comparing
// with a multimeter. Skipped until UART has room; never waits.
void reportBattery() {
  const auto battery = BatteryMonitor::status();
  if (batteryReported || battery.result == BatteryCheckResult::Checking ||
      battery.result == BatteryCheckResult::NotAvailable) return;
  char line[64];
  const int length = snprintf(line, sizeof(line), "BAT %s %lumV %uS\r\n",
      BatteryMonitor::resultName(battery.result),
      static_cast<unsigned long>(battery.packMillivolts), battery.cells);
  if (length > 0 && static_cast<size_t>(length) < sizeof(line) && Serial.availableForWrite() >= length) {
    Serial.write(reinterpret_cast<const uint8_t*>(line), length);
    batteryReported = true;
  }
}

void telemetry(uint32_t now) {
  if (!logging) return;
  if (logRow >= 5) {
    if (now - lastLogMs < LOG_PERIOD_MS) return;
    lastLogMs = now; logRow = 0;
  }
  if (now - lastLogMs >= LOG_FRAME_TIMEOUT_MS) { logRow = 5; return; }
  char line[256];
  int length = 0;
  const auto sensors = SensorManager::snapshot();
  if (logRow == 0) {
    const auto vision = RaspberryLink::latest();
    const auto motor = MotorController::status();
    length = snprintf(line, sizeof(line), "D,%lu,%s,%u,%d,%d,%u,%u,%u,%u,%u,%u\r\n",
        static_cast<unsigned long>(now), BenchController::modeName(BenchController::mode()),
        SystemManager::motorAllowed() ? 1 : 0, motor.appliedLeftPermille,
        motor.appliedRightPermille, vision.linkValid ? 1 : 0, vision.frameFresh ? 1 : 0,
        vision.targetValid ? 1 : 0, vision.targetX, vision.targetHeight,
        BenchController::hazardMask(sensors));
  } else {
    const uint8_t i = logRow - 1;
    const auto& raw = sensors.colorRaw[i];
    const auto& floor = sensors.floorColor[i];
    const bool as = ColorSensorManager::model() == ColorSensorModel::AS7341;
    length = snprintf(line, sizeof(line), "S,%lu,%u,%lu,%u,%s,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
        static_cast<unsigned long>(now), static_cast<unsigned>(i + 1),
        static_cast<unsigned long>(raw.sequence), raw.valid ? 1 : 0,
        floorColorName(floor.color), floor.reliable ? 1 : 0, raw.brightness,
        raw.channel[as ? 1 : 0], raw.channel[as ? 2 : 1], raw.channel[as ? 4 : 2],
        raw.channel[as ? 5 : 8], raw.channel[as ? 6 : 8],
        static_cast<unsigned>(ColorSensorManager::status().error[i]));
  }
  // Append fields only: legacy D/S columns retain their original positions.
  if (length > 0 && static_cast<size_t>(length) < sizeof(line) - 2 && logRow < 4) {
    length -= 2;  // Replace CRLF with the diagnostic suffix.
    const auto sys = SystemManager::status();
    const auto motor = MotorController::status();
    const auto battery = BatteryMonitor::status();
    const auto led = StatusLed::status().internal;
    const auto link = RaspberryLink::status();
    int added = 0;
    if (logRow == 0)
      added = snprintf(line + length, sizeof(line) - length, ",SYSTEM,%u,%u,%u,%u,%u,%lu,%u,%lu\r\n",
          static_cast<unsigned>(sys.state),
          motor.emergencyStopLatched ? 1 : 0, sys.runRequested ? 1 : 0,
          sys.motorAllowed ? 1 : 0, sys.rearmRequired ? 1 : 0,
          static_cast<unsigned long>(sys.startEventCount),
          static_cast<unsigned>(ResetManager::reason()), static_cast<unsigned long>(sys.faultFlags));
    else if (logRow == 1)
      added = snprintf(line + length, sizeof(line) - length, ",MOTOR,%d,%d,%u,%u,%u,%lu,%lu\r\n",
          motor.requestedLeftPermille, motor.requestedRightPermille,
          motor.outputsActive ? 1 : 0, static_cast<unsigned>(motor.lastResult),
          static_cast<unsigned>(motor.stopMode), static_cast<unsigned long>(motor.commandValidUntilMs),
          static_cast<unsigned long>(motor.watchdogStopCount));
    else if (logRow == 2)
      added = snprintf(line + length, sizeof(line) - length, ",LINK,%u,%lu,%lu,%lu,%lu,%lu\r\n",
          link.lastSequence,
          static_cast<unsigned long>(link.lastPacketMs), static_cast<unsigned long>(link.lastFrameMs),
          static_cast<unsigned long>(link.validPackets), static_cast<unsigned long>(link.invalidPackets),
          static_cast<unsigned long>(link.duplicatePackets));
    else
      added = snprintf(line + length, sizeof(line) - length, ",POWER_LED,%u,%lu,%u,%u,%u,%u,%u,%u,%u\r\n",
          static_cast<unsigned>(battery.result),
          static_cast<unsigned long>(battery.packMillivolts), battery.cells, battery.okSamples,
          static_cast<unsigned>(led.state), led.outputOn ? 1 : 0, led.red, led.green, led.blue);

    if (added <= 0 || static_cast<size_t>(added) >= sizeof(line) - length) return;
    length += added;
  }
  // Telemetry is best-effort; never wait for UART space while driving.
  if (length > 0 && static_cast<size_t>(length) < sizeof(line) && Serial.availableForWrite() >= length) {
    Serial.write(reinterpret_cast<const uint8_t*>(line), length);
    ++logRow;
  }
}
}

void BenchV2App::begin() {
  const auto boot = BootManager::begin();
  if (!boot.safeToRun) {
    while (true) { BootManager::staySafe(); StatusLed::update(millis()); delay(10); }
  }
  MotorController::setOutputLimit(OUTPUT_LIMIT);
  BenchController::begin();
  BatteryMonitor::begin(millis());
  batteryReported = false;
  inputLength = 0; discardLine = false; logging = true; logRow = 5;
  lastInputMs = lastLogMs = millis();
  help();
}

void BenchV2App::update() {
  SystemManager::update(millis());
  RaspberryLink::poll();
  SensorManager::update();
  BatteryMonitor::update(millis());
  SystemManager::update(millis());
  console();
  const auto command = BenchController::update(millis(), SystemManager::motorAllowed(),
      SensorManager::snapshot(), RaspberryLink::latest());
  if (SystemManager::motorAllowed()) {
    if (!MotorController::apply(command, SystemManager::safetyStatus()) &&
        MotorController::status().lastResult == MotorControlResult::DriverFailure)
      SystemManager::setMotorHealthy(false, FaultCode::Driver);
  } else MotorController::brake();
  if (BenchController::finished()) { stop(); reply("DONE MOTOR\r\n"); }
  StatusLed::update(millis());
  EventLogger::process();
  reportBattery();
  telemetry(millis());
  delay(1);
}
