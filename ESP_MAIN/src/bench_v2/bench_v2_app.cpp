#include "bench_v2/bench_v2_app.hpp"
#include "bench_v2/software_switches.hpp"
#include "bench_v2/i2c_scan.hpp"
#include "hardware/reset_manager.hpp"
#include "interrupts/button_interrupts.hpp"
#if HW_VERSION != 2
#error V2 bench requires HW_VERSION=2
#endif
#include <Arduino.h>
#include <cstdio>
#include <cstring>

#include "app/boot_manager.hpp"
#include "calibration/color_calibration.hpp"
#include "bench/bench_config.hpp"
#include "bench/bench_controller.hpp"
#include "communication/raspberry_link.hpp"
#include "sensors/imu_monitor.hpp"
#include "motor/motor_config.hpp"
#include "motor/motor_controller.hpp"
#include "pins.hpp"
#include "sensors/color_sensor_manager.hpp"
#include "sensors/sensor_manager.hpp"
#include "system/battery_monitor.hpp"
#include "system/battery_setting.hpp"
#include "system/event_logger.hpp"
#include "system/status_led.hpp"
#include "system/system_manager.hpp"

namespace {
using namespace config::bench;
char input[48];
uint8_t inputLength = 0;
bool discardLine = false, logging = false;
uint32_t lastInputMs = 0, lastLogMs = 0;
// Frame rows: 0 = D, 1..4 = S per sensor, 5 = extra row. ROWS means idle.
constexpr uint8_t ROWS = 6;
uint8_t logRow = ROWS;
bool batteryReported = false;
uint32_t lastCalLineMs = 0;

void stop() {
#if defined(BENCH_SOFTWARE_SWITCHES) && BENCH_SOFTWARE_SWITCHES
  BenchSoftwareSwitches::cancelStart();
#endif
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
#if defined(BENCH_SOFTWARE_SWITCHES) && BENCH_SOFTWARE_SWITCHES
  reply("SW1: permanent ESTOP; SW2: debounced start press/release (bench only)\r\n");
#endif
  reply("MODE STOP|MOTOR|MANUAL|DATA|ALIGN|FOLLOW|AVOID\r\n");
  reply("SW2 press/release to arm; DRIVE -400..400, repeat <250ms\r\n");
  reply("SW2 held 3s while stopped: ALIGN (turn to center the tower)\r\n");
  reply("STOP / LOG ON|OFF / STATUS / HELP / IMU RESET; UART0 57600\r\n");
  reply("CAL START / CAL ERASE / CAL STATUS: floor color calibration\r\n");
  reply("I2C SCAN: main bus + TCA9548A channels\r\n");
  reply("BAT CHECK ON|OFF: power-on battery check, saved, next boot (match too)\r\n");
}

// BATCHECK,<1|0>: the saved setting, for the GUI.
void replyBatteryCheck() {
  reply(BatterySetting::checkEnabled() ? "BATCHECK,1\r\n" : "BATCHECK,0\r\n");
}

bool startsWith(const char* line, const char* prefix) { return !strncmp(line, prefix, strlen(prefix)); }

void execute(const char* line) {
#if defined(BENCH_SOFTWARE_SWITCHES) && BENCH_SOFTWARE_SWITCHES
  // Emergency stop must work even while calibration owns the console.
  if (!strcmp(line, "SW1")) {
    BenchSoftwareSwitches::emergencyStop();
    SystemManager::update(millis());  // enters the existing permanent ESTOP loop
    return;
  }
  if (!strcmp(line, "SW2")) {
    const auto mode = BenchController::mode();
    const bool allowed = !ColorCalibration::active() &&
        mode != BenchMode::Stop && mode != BenchMode::Data;
    if (allowed && BenchSoftwareSwitches::pressStart(millis()))
      reply("OK SW2 press/release queued\r\n");
    else reply("ERR SW2 blocked: mode/battery/fault/calibration/button busy\r\n");
    return;
  }
#endif
  // While calibrating every line belongs to the calibration; motors stay braked.
  if (ColorCalibration::active()) { ColorCalibration::command(line, millis()); return; }
  if (!strcmp(line, "BAT CHECK ON") || !strcmp(line, "BAT CHECK OFF")) {
    // OFF also applies now (a failed check stops blocking SW2; nothing starts by
    // itself). ON waits for the next power-on so a run in progress is never cut.
    const bool on = !strcmp(line, "BAT CHECK ON");
    if (!on) BatteryMonitor::stopCheck();
    if (BatterySetting::setCheckEnabled(on))
      reply(on ? "OK BAT CHECK on from the next power-on\r\n" : "OK BAT CHECK off now and saved\r\n");
    else reply("ERR BAT CHECK not saved (NVS)\r\n");
    replyBatteryCheck();
    return;
  }
  if (!strcmp(line, "BAT CHECK")) { replyBatteryCheck(); return; }
  if (!strcmp(line, "I2C SCAN")) {
    stop();  // motors stay braked; a new SW2 is needed after the scan
    BenchI2cScan::start(millis());
    reply("OK I2C SCAN\r\n");
    return;
  }
  if (startsWith(line, "CAL ")) {
    // CAL STATUS only reports; anything else may start calibration, so stop first.
    if (strcmp(line, "CAL STATUS")) { stop(); logging = false; logRow = ROWS; }  // quiet console for prompts
    ColorCalibration::command(line, millis());
    return;
  }
  if (!strcmp(line, "STOP")) { stop(); reply("OK STOP\r\n"); return; }
  if (!strcmp(line, "HELP")) { stop(); help(); return; }
  if (!strcmp(line, "LOG ON")) { logging = true; return; }
  // IMU reset only touches the IMU, so motion is not stopped.
  if (!strcmp(line, "IMU RESET")) { ImuMonitor::requestReset(millis()); reply("OK IMU RESET\r\n"); return; }
  if (!strcmp(line, "LOG OFF")) { logging = false; logRow = ROWS; return; }
  if (!strcmp(line, "STATUS")) { logging = true; lastLogMs = millis() - LOG_PERIOD_MS; return; }
  if (!strncmp(line, "MODE ", 5)) {
    for (uint8_t i = 0; i <= static_cast<uint8_t>(BenchMode::Avoid); ++i) {
      const auto mode = static_cast<BenchMode>(i);
      if (strcmp(line + 5, BenchController::modeName(mode))) continue;
      stop(); BenchController::select(mode);
      if (mode == BenchMode::Data) logging = true;
      EventLogger::modeChanged(i);
      char ok[48];
      snprintf(ok, sizeof(ok), "OK MODE %s; SW2 required\r\n", BenchController::modeName(mode));
      reply(ok);
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

// Every analog input on the V2 board, read one per loop so no loop waits on
// eleven conversions. Order matches the ADC row. GPIO19/20 (motor 2 IS) are
// the USB pins: the first analog read hands them to the ADC.
constexpr gpio_num_t ANALOG_PINS[] = {
    pins::MOTOR1_L_IS, pins::MOTOR1_R_IS, pins::MOTOR2_L_IS, pins::MOTOR2_R_IS,
    pins::IR_INPUTS[0], pins::IR_INPUTS[1], pins::IR_INPUTS[2],
    pins::IR_INPUTS[3], pins::IR_INPUTS[4], pins::IR_INPUTS[5], pins::BATTERY_ADC};
constexpr uint8_t ANALOG_COUNT = sizeof(ANALOG_PINS) / sizeof(ANALOG_PINS[0]);
uint16_t analogMv[ANALOG_COUNT] = {};
uint8_t analogNext = 0;

void sampleAnalog() {
  analogMv[analogNext] = static_cast<uint16_t>(analogReadMilliVolts(ANALOG_PINS[analogNext]));
  analogNext = (analogNext + 1) % ANALOG_COUNT;
}

// Each frame is D, S1..S4 and one extra row. UART0 has only its 128-byte
// hardware FIFO (no TX ring buffer), so every line stays under 128 bytes and
// the extra rows take turns: IMU every other frame, pins and ADC every 8th,
// the rest every 16th.
enum class Extra : uint8_t { Imu, System, Motor, Link, Power, Pins, Analog };
constexpr Extra EXTRA_ORDER[] = {Extra::Imu, Extra::System, Extra::Imu, Extra::Pins,
                                 Extra::Imu, Extra::Analog, Extra::Imu, Extra::Motor,
                                 Extra::Imu, Extra::Pins, Extra::Imu, Extra::Analog,
                                 Extra::Imu, Extra::Link, Extra::Imu, Extra::Power};
uint8_t extraIndex = 0;

int extraLine(char* line, size_t size, uint32_t now, Extra kind) {
  const auto ms = static_cast<unsigned long>(now);
  switch (kind) {
    case Extra::Imu: {
      const auto imu = ImuMonitor::status();
      const auto v = ImuMonitor::latest();
      return snprintf(line, size, "I,%lu,%s,%02X,%u,%lu,%lu,%d,%d,%d,%d,%d,%d,%u,%02X\r\n", ms,
          ImuMonitor::stateName(imu.state), imu.whoAmI, imu.dataFresh ? 1 : 0,
          static_cast<unsigned long>(imu.samples), static_cast<unsigned long>(imu.busErrors),
          v.accel[0], v.accel[1], v.accel[2], v.gyro[0], v.gyro[1], v.gyro[2],
          static_cast<unsigned>(imu.stale), imu.powerMgmt);
    }
    case Extra::System: {
      const auto sys = SystemManager::status();
      return snprintf(line, size, "SYSTEM,%lu,%u,%u,%u,%u,%u,%lu,%u,%lu\r\n", ms,
          static_cast<unsigned>(sys.state), MotorController::status().emergencyStopLatched ? 1 : 0,
          sys.runRequested ? 1 : 0, sys.motorAllowed ? 1 : 0, sys.rearmRequired ? 1 : 0,
          static_cast<unsigned long>(sys.startEventCount),
          static_cast<unsigned>(ResetManager::reason()), static_cast<unsigned long>(sys.faultFlags));
    }
    case Extra::Motor: {
      const auto motor = MotorController::status();
      return snprintf(line, size, "MOTOR,%lu,%d,%d,%u,%u,%u,%lu,%lu\r\n", ms,
          motor.requestedLeftPermille, motor.requestedRightPermille,
          motor.outputsActive ? 1 : 0, static_cast<unsigned>(motor.lastResult),
          static_cast<unsigned>(motor.stopMode), static_cast<unsigned long>(motor.commandValidUntilMs),
          static_cast<unsigned long>(motor.watchdogStopCount));
    }
    case Extra::Link: {
      const auto link = RaspberryLink::status();
      return snprintf(line, size, "LINK,%lu,%u,%lu,%lu,%lu,%lu,%lu\r\n", ms, link.lastSequence,
          static_cast<unsigned long>(link.lastPacketMs), static_cast<unsigned long>(link.lastFrameMs),
          static_cast<unsigned long>(link.validPackets), static_cast<unsigned long>(link.invalidPackets),
          static_cast<unsigned long>(link.duplicatePackets));
    }
    case Extra::Pins: {
      // Actual pin levels, read back from the hardware (OUTPUT keeps the input
      // buffer on): SW1/SW2 1 = pressed, EN 1 = HIGH, PWM = LEDC duty 0..255.
      // PIN,ms,sw1,sw2,en1,en2,lpwm1,rpwm1,lpwm2,rpwm2,sda,scl
      return snprintf(line, size, "PIN,%lu,%d,%d,%d,%d,%lu,%lu,%lu,%lu,%d,%d\r\n", ms,
          digitalRead(pins::SAFETY_SWITCH) == LOW ? 1 : 0,
          digitalRead(pins::START_SWITCH) == LOW ? 1 : 0,
          digitalRead(pins::MOTOR1_EN), digitalRead(pins::MOTOR2_EN),
          static_cast<unsigned long>(ledcRead(config::motor::MOTOR1_LPWM_CHANNEL)),
          static_cast<unsigned long>(ledcRead(config::motor::MOTOR1_RPWM_CHANNEL)),
          static_cast<unsigned long>(ledcRead(config::motor::MOTOR2_LPWM_CHANNEL)),
          static_cast<unsigned long>(ledcRead(config::motor::MOTOR2_RPWM_CHANNEL)),
          digitalRead(pins::I2C_SDA), digitalRead(pins::I2C_SCL));
    }
    case Extra::Analog:
      // ADC,ms,IS_L1,IS_R1,IS_L2,IS_R2,IR1..IR6,BAT pin, all in mV at the GPIO.
      return snprintf(line, size, "ADC,%lu,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n", ms,
          analogMv[0], analogMv[1], analogMv[2], analogMv[3], analogMv[4], analogMv[5],
          analogMv[6], analogMv[7], analogMv[8], analogMv[9], analogMv[10]);
    case Extra::Power: {
      const auto battery = BatteryMonitor::status();
      const auto led = StatusLed::status().internal;
      return snprintf(line, size, "POWER_LED,%lu,%u,%lu,%u,%u,%u,%u,%u,%u,%u\r\n", ms,
          static_cast<unsigned>(battery.result), static_cast<unsigned long>(battery.packMillivolts),
          battery.cells, battery.okSamples, static_cast<unsigned>(led.state),
          led.outputOn ? 1 : 0, led.red, led.green, led.blue);
    }
  }
  return 0;
}

// While calibrating: one short machine-readable line every 250 ms for the GUI.
// CAL,ms,state,step(0-based),sensorMask(bit n = sensor n+1),color,pose,samples,stored
void calibrationLine(uint32_t now) {
  if (!ColorCalibration::active() || now - lastCalLineMs < 250) return;
  const auto cal = ColorCalibration::status();
  char line[96];
  const int length = snprintf(line, sizeof(line), "CAL,%lu,%u,%u,%u,%s,%s,%u,%u\r\n",
      static_cast<unsigned long>(now), static_cast<unsigned>(cal.state), cal.step, cal.sensorMask,
      floorColorName(cal.color), calibrationPoseName(cal.pose), cal.sampleCount, cal.storedDataValid ? 1 : 0);
  if (length > 0 && static_cast<size_t>(length) < sizeof(line) && Serial.availableForWrite() >= length) {
    Serial.write(reinterpret_cast<const uint8_t*>(line), length);
    lastCalLineMs = now;
  }
}

// SW2 held ALIGN_HOLD_MS while stopped picks ALIGN, so the field needs no PC:
// the release is the normal start gesture. While running a 0.5 s hold is the
// emergency stop (SystemManager), so this never fires then.
void alignHold(uint32_t now) {
  if (SystemManager::running() || ColorCalibration::active() ||
      BenchController::mode() == BenchMode::Align ||
      ButtonInterrupts::startHeldMs(now) < ALIGN_HOLD_MS) return;
  BenchController::select(BenchMode::Align);
  EventLogger::modeChanged(static_cast<uint8_t>(BenchMode::Align));
  reply("OK MODE ALIGN (SW2 3s); release SW2 to start\r\n");
}

void telemetry(uint32_t now) {
  if (!logging) return;
  if (logRow >= ROWS) {
    if (now - lastLogMs < LOG_PERIOD_MS) return;
    lastLogMs = now; logRow = 0;
    extraIndex = (extraIndex + 1) % (sizeof(EXTRA_ORDER) / sizeof(EXTRA_ORDER[0]));
  }
  if (now - lastLogMs >= LOG_FRAME_TIMEOUT_MS) { logRow = ROWS; return; }
  char line[128];
  int length = 0;
  const auto sensors = SensorManager::snapshot();
  if (logRow == 0) {
    const auto vision = RaspberryLink::latest();
    const auto motor = MotorController::status();
    // D,ms,mode,allowed,left,right,link,fresh,target,x,height,hazard,y,width,flags,seq
    // (y..seq appended 11/04 for the GUI camera box; old readers ignore them)
    length = snprintf(line, sizeof(line), "D,%lu,%s,%u,%d,%d,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
        static_cast<unsigned long>(now), BenchController::modeName(BenchController::mode()),
        SystemManager::motorAllowed() ? 1 : 0, motor.appliedLeftPermille,
        motor.appliedRightPermille, vision.linkValid ? 1 : 0, vision.frameFresh ? 1 : 0,
        vision.targetValid ? 1 : 0, vision.targetX, vision.targetHeight,
        BenchController::hazardMask(sensors), vision.targetY, vision.targetWidth,
        vision.statusBits, vision.sequence);
  } else if (logRow == ROWS - 1) {
    length = extraLine(line, sizeof(line), now, EXTRA_ORDER[extraIndex]);
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
  // A line that does not fit is skipped, not retried forever.
  if (length <= 0 || static_cast<size_t>(length) >= sizeof(line)) { ++logRow; return; }
  // Telemetry is best-effort; never wait for UART space while driving.
  if (Serial.availableForWrite() >= length) {
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
  ImuMonitor::begin(millis());
  ColorCalibration::begin();  // CalibrationStore was loaded by SensorManager::begin
#if defined(BENCH_SOFTWARE_SWITCHES) && BENCH_SOFTWARE_SWITCHES
  BenchSoftwareSwitches::begin();
#endif
  batteryReported = false;
  inputLength = 0; discardLine = false; logging = true; logRow = ROWS;
  extraIndex = sizeof(EXTRA_ORDER) / sizeof(EXTRA_ORDER[0]) - 1;  // first frame shows the IMU
  lastInputMs = lastLogMs = millis();
  help();
  replyBatteryCheck();
}

void BenchV2App::update() {
#if defined(BENCH_SOFTWARE_SWITCHES) && BENCH_SOFTWARE_SWITCHES
  BenchSoftwareSwitches::update(millis());
#endif
  SystemManager::update(millis());
  alignHold(millis());
  RaspberryLink::poll();
  SensorManager::update();
  ColorCalibration::tick(millis());
  ImuMonitor::update(millis());
  BatteryMonitor::update(millis());
  BenchI2cScan::step(millis());
  SystemManager::update(millis());
  console();
  const auto command = BenchController::update(millis(), SystemManager::motorAllowed(),
      SensorManager::snapshot(), RaspberryLink::latest());
  if (SystemManager::motorAllowed() && !ColorCalibration::active()) {
    if (!MotorController::apply(command, SystemManager::safetyStatus()) &&
        MotorController::status().lastResult == MotorControlResult::DriverFailure)
      SystemManager::setMotorHealthy(false, FaultCode::Driver);
  } else MotorController::brake();
  if (BenchController::finished()) { stop(); reply("DONE MOTOR\r\n"); }
  StatusLed::update(millis());
  EventLogger::process();
  reportBattery();
  sampleAnalog();
  BenchI2cScan::report();
  telemetry(millis());
  calibrationLine(millis());
  delay(1);
}
