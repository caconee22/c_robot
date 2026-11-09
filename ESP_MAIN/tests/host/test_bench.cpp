#include <iostream>
#include <stdexcept>
#include "Arduino.h"
#include "Wire.h"
#include "pins.hpp"
#include "bench/bench_controller.hpp"
#include "bench/bench_config.hpp"
#include "motor/motor_controller.hpp"
#include "motor/motor_trim.hpp"
#include "Preferences.h"
#include "system/system_manager.hpp"
#include "system/battery_monitor.hpp"
#include "system/status_led.hpp"
#include "sensors/imu_monitor.hpp"
#include "calibration/color_calibration.hpp"
#include "interrupts/button_interrupts.hpp"
#include "motor/bts7960_driver.hpp"
#include "sensors/color_sensor_manager.hpp"

// 1 when testing robot_bench_v2_manual (bench_v2/), 0 for the legacy bench.
#ifndef BENCH_V2_MANUAL
#define BENCH_V2_MANUAL 0
#endif
#include "sensors/sensor_config.hpp"

void setup();
void loop();
namespace {
unsigned checks = 0;
#define CHECK(expression) do { ++checks; if (!(expression)) throw std::runtime_error(std::to_string(__LINE__)+" "+#expression); } while (0)
bool stopped(const MotorCommand& command) { return !command.leftPermille && !command.rightPermille; }
VisionSnapshot target(uint32_t now = 0) {
  VisionSnapshot v;
  v.linkValid=v.frameFresh=v.targetValid=v.cameraOk=v.pipelineOk=true;
  v.targetX=960; v.targetHeight=200; v.targetWidth=100; v.frameUpdatedMs=now;
  return v;
}
SensorSnapshot floor(uint32_t now, FloorColor color = FloorColor::Black) {
  SensorSnapshot s; s.colorSensorReadyMask=15;
  for (auto& item:s.floorColor) { item.reliable=true; item.color=color; item.timestampMs=now; }
  return s;
}
void logicTests() {
  SensorSnapshot s; auto v=target(); BenchController::begin();
  CHECK(stopped(BenchController::update(0,true,s,v)));
  BenchController::select(static_cast<BenchMode>(255));
  CHECK(BenchController::mode()==BenchMode::Stop);
  CHECK(stopped(BenchController::update(0,true,s,v)));
  BenchController::select(BenchMode::Data); CHECK(stopped(BenchController::update(0,true,s,v)));
  BenchController::select(BenchMode::Motor);
  auto c=BenchController::update(100,true,s,v); CHECK(c.leftPermille==120 && c.rightPermille==0);
  CHECK(stopped(BenchController::update(3100,true,s,v)));
  CHECK(BenchController::update(3600,true,s,v).leftPermille==-120);
  CHECK(BenchController::update(7100,true,s,v).rightPermille==120);
  CHECK(BenchController::update(10600,true,s,v).rightPermille==-120);
  CHECK(stopped(BenchController::update(14100,true,s,v))); CHECK(BenchController::finished());
  BenchController::select(BenchMode::Manual);
  CHECK(!BenchController::setManual(100,100,0,false));
  CHECK(BenchController::setManual(-200,200,100,true));
  CHECK(BenchController::update(349,true,s,v).leftPermille==-200);
  CHECK(stopped(BenchController::update(350,true,s,v)));
  CHECK(!BenchController::setManual(401,0,360,true));
  CHECK(stopped(BenchController::update(361,true,s,v)));
  CHECK(BenchController::setManual(100,100,UINT32_MAX-100,true));
  CHECK(!stopped(BenchController::update(50,true,s,v)));
  CHECK(stopped(BenchController::update(149,true,s,v)));
  BenchController::select(BenchMode::Align);
  CHECK(stopped(BenchController::update(0,true,s,v)));
  v.targetX=1300; c=BenchController::update(0,true,s,v); CHECK(c.leftPermille>0 && c.rightPermille<0);
  v.targetX=600; c=BenchController::update(0,true,s,v); CHECK(c.leftPermille<0 && c.rightPermille>0);
  v.frameFresh=false; CHECK(stopped(BenchController::update(0,true,s,v)));
  // Output ramps: 250 just outside the window, 350 far off, never forward/back.
  v=target(); v.targetX=960+201; c=BenchController::update(0,true,s,v);
  CHECK(c.leftPermille==250 && c.rightPermille==-250);
  v.targetX=960-450; c=BenchController::update(0,true,s,v); CHECK(c.leftPermille==-300 && c.rightPermille==300);
  v.targetX=1919; c=BenchController::update(0,true,s,v); CHECK(c.leftPermille==350 && c.rightPermille==-350);
  v.targetX=5; c=BenchController::update(0,true,s,v); CHECK(c.leftPermille==-350 && c.rightPermille==350);
  // Wider ALIGN window: 150 px off center is centered enough.
  v=target(); v.targetX=960+150; CHECK(stopped(BenchController::update(0,true,s,v)));
  // Tower lost (Pi fine, no target): spin toward the side last seen, ~one turn, then stop.
  v.targetX=1500; c=BenchController::update(10,true,s,v); CHECK(c.leftPermille>0 && c.rightPermille<0);
  v.targetValid=false; v.targetX=0; v.targetHeight=0;
  c=BenchController::update(20,true,s,v); CHECK(c.leftPermille==300 && c.rightPermille==-300);
  c=BenchController::update(20+config::bench::SEARCH_TURN_MS-1,true,s,v); CHECK(c.leftPermille>0);
  CHECK(stopped(BenchController::update(20+config::bench::SEARCH_TURN_MS,true,s,v)));
  v=target(); v.targetX=400; c=BenchController::update(4000,true,s,v); CHECK(c.leftPermille<0);  // found again
  v.targetValid=false; v.targetX=0; v.targetHeight=0;
  c=BenchController::update(4010,true,s,v); CHECK(c.leftPermille<0 && c.rightPermille>0);  // lost on the left
  v.linkValid=false; CHECK(stopped(BenchController::update(4020,true,s,v)));  // no Pi: no blind search
  BenchController::select(BenchMode::Align); v=target(); v.targetValid=false; v.targetX=0; v.targetHeight=0;
  CHECK(stopped(BenchController::update(0,true,s,v)));  // never seen since start: stay still
  v=target(); BenchController::select(BenchMode::Follow);
  c=BenchController::update(0,true,s,v); CHECK(c.leftPermille==160 && c.rightPermille==160);
  v.targetX=1100; c=BenchController::update(0,true,s,v); CHECK(c.leftPermille>c.rightPermille);
  v.targetHeight=420; CHECK(stopped(BenchController::update(0,true,s,v)));
  v=target(); v.boxClipped=true; CHECK(stopped(BenchController::update(0,true,s,v)));
  BenchController::select(BenchMode::Avoid); v=target(); s=floor(0);
  CHECK(!stopped(BenchController::update(0,true,s,v)));
  s.floorColor[0].reliable=false; CHECK(stopped(BenchController::update(0,true,s,v)));
  s=floor(0); s.colorSensorReadyMask=7; CHECK(stopped(BenchController::update(0,true,s,v)));
  s=floor(0); CHECK(stopped(BenchController::update(config::sensors::SAMPLE_STALE_MS+1,true,s,v)));
  s=floor(1000); s.floorColor[0].color=FloorColor::Red;
  c=BenchController::update(1000,true,s,v); CHECK(c.leftPermille<0 && c.rightPermille<0);
  c=BenchController::update(1250,true,s,v); CHECK(c.leftPermille>0 && c.rightPermille<0);
  s=floor(1500); CHECK(!stopped(BenchController::update(1500,true,s,v)));
  BenchController::select(BenchMode::Avoid); s=floor(0); s.floorColor[2].color=FloorColor::Blue;
  c=BenchController::update(0,true,s,v); CHECK(c.leftPermille>0 && c.rightPermille>0);
  s.floorColor[0].color=FloorColor::Yellow; CHECK(!stopped(BenchController::update(1,true,s,v)));  // yellow is not a hazard
  s.floorColor[0].color=FloorColor::Red; CHECK(stopped(BenchController::update(1,true,s,v)));
  CHECK(stopped(BenchController::update(2,false,s,v)));
}
void ticks(uint32_t duration) {
  const uint32_t start=millis();
  while (millis()-start<duration) { fake::advance(1); loop(); }
}
void send(const std::string& line) { Serial.inject(line+"\n"); ticks(5); }
void startButton() {
  fake::level(pins::START_SWITCH,LOW); ticks(config::switches::HOLD_MS+60);
  fake::level(pins::START_SWITCH,HIGH); ticks(50);
}
void camera(uint16_t x, uint16_t height, uint8_t sequence = 1) {
  std::vector<uint8_t> packet{0xAA,0x55,1,10,sequence,0x31};
  for (uint16_t value : {x, uint16_t{540}, uint16_t{100}, height}) {
    packet.push_back(value & 255); packet.push_back(value >> 8);
  }
  Serial1.inject(packet); ticks(8);
}
#if BENCH_V2_MANUAL
void i2cScanTests() {
  fake::reset();
  for (unsigned i=0;i<4;++i) fake::floor(i,3);
  setup(); ticks(200);
  Serial.tx.clear(); send("I2C SCAN"); ticks(1500);
  CHECK(Serial.tx.find("OK I2C SCAN")!=std::string::npos);
  CHECK(Serial.tx.find("I2C MAIN 68 70" "\r\n")!=std::string::npos);
  CHECK(Serial.tx.find("I2C MUX 70")!=std::string::npos);
  CHECK(Serial.tx.find("I2C CH 0 39")!=std::string::npos && Serial.tx.find("I2C CH 3 39")!=std::string::npos);
  CHECK(Serial.tx.find("I2C CH 4" "\r\n")!=std::string::npos);  // empty channel
  CHECK(Serial.tx.find("I2C DONE")!=std::string::npos);
  // The scan leaves the floor sensors working: they reselect their own channel.
  ticks(500); CHECK(ColorSensorManager::status().readyMask==0x0F);
  // Without the mux: reported as missing, no channel lines.
  fake::tcaConnected=false; Serial.tx.clear(); send("I2C SCAN"); ticks(500);
  CHECK(Serial.tx.find("I2C MUX NONE")!=std::string::npos && Serial.tx.find("I2C CH")==std::string::npos);
  CHECK(Serial.tx.find("I2C DONE")!=std::string::npos);
  fake::tcaConnected=true;
}
// Software SW2 holds past the 0.5 s button hold, then releases and debounces.
constexpr uint32_t SW2_MS = config::switches::DEBOUNCE_MS*2 + config::switches::HOLD_MS + 60;
void softwareSwitchTests() {
  fake::reset();
  for (unsigned i=0;i<4;++i) fake::floor(i,3);
  fake::adcMillivolts[pins::BATTERY_ADC]=0;
  setup();
  send("MODE MANUAL"); send("SW2"); ticks(120);
  CHECK(!SystemManager::motorAllowed());  // pending battery check cannot be bypassed
  CHECK(Serial.tx.find("ERR SW2 blocked")!=std::string::npos);
  fake::adcMillivolts[pins::BATTERY_ADC]=15200U/11U; ticks(150);
  CHECK(BatteryMonitor::status().result==BatteryCheckResult::Ok);
  CHECK(!SystemManager::motorAllowed());  // old command was discarded
  send("SW2"); ticks(25); CHECK(!SystemManager::motorAllowed());
  fake::advance(80); loop();  // a delayed loop must still qualify the press before release
  ticks(SW2_MS); CHECK(SystemManager::motorAllowed());
  CHECK(SystemManager::status().startEventCount==1);
  send("DRIVE 100 100"); CHECK(MotorController::status().outputsActive);
  send("STOP"); CHECK(!SystemManager::motorAllowed());
  send("SW2"); ticks(20); send("STOP"); ticks(150);
  CHECK(!SystemManager::motorAllowed());
  CHECK(SystemManager::status().startEventCount==1);
  send("SW2"); ticks(20); send("MODE FOLLOW"); ticks(150);
  CHECK(!SystemManager::motorAllowed());  // mode changes cancel the old gesture
  send("MODE MANUAL");
  fake::level(pins::START_SWITCH,LOW); ticks(config::switches::HOLD_MS+60);
  send("SW2"); CHECK(Serial.tx.find("ERR SW2 blocked")!=std::string::npos);
  send("STOP"); fake::level(pins::START_SWITCH,HIGH); ticks(60);
  CHECK(!SystemManager::motorAllowed());
  send("SW2"); send("SW2"); ticks(SW2_MS);
  CHECK(SystemManager::motorAllowed());
  CHECK(SystemManager::status().startEventCount==2);  // repeat does not queue another start
  send("STOP");
  send("CAL START"); CHECK(ColorCalibration::active());
  send("SW2"); ticks(150); CHECK(!SystemManager::motorAllowed());
  send("ABORT"); ticks(100); CHECK(!SystemManager::motorAllowed());
  send("MODE MOTOR"); send("SW2"); ticks(SW2_MS);
  CHECK(SystemManager::motorAllowed() && MotorController::status().outputsActive);
  send("STOP");
  send("MODE MANUAL"); send("SW2");
  fake::advance(SW2_MS+300); loop(); ticks(130);
  CHECK(!SystemManager::motorAllowed());  // a stale virtual press must not start later
  SystemManager::setApplicationHealthy(false);
  send("MODE MANUAL"); send("SW2"); ticks(130);
  CHECK(!SystemManager::motorAllowed());
  SystemManager::setApplicationHealthy(true); ticks(100);
  CHECK(!SystemManager::motorAllowed());
  send("SW2"); ticks(SW2_MS); CHECK(SystemManager::motorAllowed());
  send("DRIVE 100 100"); CHECK(MotorController::status().outputsActive);
  // SW1 command enters the same permanent loop as the physical switch.
  fake::stopOnDelay=true;
  Serial.inject("SW1\n");
  try { loop(); CHECK(false); } catch (const fake::StopLoop&) {}
  CHECK(ButtonInterrupts::emergencyLatched());
  CHECK(!SystemManager::motorAllowed() && !MotorController::status().outputsActive);
  CHECK(!Bts7960Driver::clearDriveInhibit());
  CHECK(Serial.tx.find("ESTOP")!=std::string::npos);
}

void softwareEmergencyDuringCalibrationTest() {
  fake::reset();
  for (unsigned i=0;i<4;++i) fake::floor(i,3);
  setup(); ticks(300); send("CAL START");
  CHECK(ColorCalibration::active());
  fake::stopOnDelay=true;
  Serial.inject("SW1\n");
  try { loop(); CHECK(false); } catch (const fake::StopLoop&) {}
  CHECK(ButtonInterrupts::emergencyLatched());
  CHECK(!Bts7960Driver::clearDriveInhibit());
  CHECK(!SystemManager::motorAllowed());
}

void calibrationTests() {
  fake::reset();
  for (unsigned i=0;i<4;++i) fake::floor(i,3);
  setup(); ticks(300);
  send("CAL START");
  CHECK(ColorCalibration::active());
  CHECK(Serial.tx.find("[CAL] Calibration started")!=std::string::npos);
  // While calibrating, motion commands go to the calibration and the motors stay braked.
  send("MODE MANUAL"); startButton(); send("DRIVE 100 100");
  CHECK(ColorCalibration::active() && !MotorController::status().outputsActive);
  Serial.tx.clear(); ticks(300);
  CHECK(Serial.tx.find("\nCAL,")!=std::string::npos || Serial.tx.rfind("CAL,",0)==0);
  CHECK(Serial.tx.find("\nD,")==std::string::npos);  // telemetry is quiet during calibration
  send("ready");  // lower case is accepted
  ticks(config::sensors::CALIBRATION_COUNTDOWN_MS + config::sensors::CALIBRATION_MEASURE_MS + 300);
  CHECK(Serial.tx.find("[CAL][RESULT]")!=std::string::npos);
  CHECK(ColorCalibration::status().state==CalibrationState::WaitingForAccept);
  CHECK(!MotorController::status().outputsActive);
  send("ACCEPT"); CHECK(ColorCalibration::status().step==1);
  send("ABORT"); CHECK(!ColorCalibration::active());
  // Back to normal bench use: a new SW2 gesture is needed, then DRIVE works.
  send("MODE MANUAL"); startButton(); send("DRIVE 100 100");
  CHECK(MotorController::status().outputsActive);
  send("STOP");
}
void imuTests() {
  fake::reset();
  setup(); ticks(350);  // IMU row every other frame; READY after ~110 ms
  auto imu=ImuMonitor::status();
  CHECK(imu.state==ImuState::Ready && imu.whoAmI==0x68);
  CHECK(imu.samples>0 && imu.dataFresh && imu.resets==0);
  CHECK(fake::imu.resets==1 && fake::imu.awake);
  CHECK(Serial.tx.find("I,")!=std::string::npos && Serial.tx.find(",READY,68,1,")!=std::string::npos);
  CHECK(ImuMonitor::latest().accel[0]!=0);
  // Values stop changing: data-ready still arrives but the sensor is not fresh.
  fake::imu.frozen=true; ticks(config::imu::FROZEN_MS+50);
  CHECK(!ImuMonitor::status().dataFresh && ImuMonitor::status().state==ImuState::Ready);
  CHECK(ImuMonitor::status().stale==ImuStale::Frozen);
  fake::imu.frozen=false; ticks(30); CHECK(ImuMonitor::status().dataFresh);
  CHECK(ImuMonitor::status().stale==ImuStale::Fresh);
  ticks(600); CHECK(ImuMonitor::status().powerMgmt==0x01);  // running, read back
  CHECK(Serial.tx.find(",0,01\r\n")!=std::string::npos);  // I row: fresh, PWR_MGMT_1 0x01
  // The chip resets by itself (brown-out): it sleeps, no data-ready, no bus error.
  { const auto errors=ImuMonitor::status().busErrors;
    std::fill(fake::imu.reg,fake::imu.reg+128,0); fake::imu.reg[0x6B]=0x40; fake::imu.awake=false;
    ticks(700); imu=ImuMonitor::status();
    CHECK(imu.state==ImuState::Ready && imu.stale==ImuStale::NoData && imu.powerMgmt==0x40);
    CHECK(imu.busErrors==errors);
    send("IMU RESET"); ticks(200); CHECK(ImuMonitor::status().dataFresh); }
  // Bus loss: Fault after a few errors, no automatic retry.
  fake::imu.connected=false; ticks(100);
  imu=ImuMonitor::status();
  CHECK(imu.state==ImuState::Fault && imu.lastError==ImuError::Bus && !imu.dataFresh);
  const auto errors=imu.busErrors; ticks(200);
  CHECK(ImuMonitor::status().busErrors==errors);
  // Operator reset brings it back once the bus works again.
  fake::imu.connected=true; send("IMU RESET");
  CHECK(Serial.tx.find("OK IMU RESET")!=std::string::npos);
  ticks(200); imu=ImuMonitor::status();
  CHECK(imu.state==ImuState::Ready && imu.resets==2 && imu.dataFresh);  // 2nd operator reset
  // A device that is not an MPU6050 is reported, not used.
  fake::imu.whoAmI=0x12; send("IMU RESET"); ticks(200);
  imu=ImuMonitor::status();
  CHECK(imu.state==ImuState::Fault && imu.lastError==ImuError::WhoAmI && imu.whoAmI==0x12);
  // The IMU never affects the motors or the safety state.
  CHECK(!MotorController::status().outputsActive && SystemManager::status().applicationHealthy);
}
#endif
#if HW_VERSION == 2
void batteryCheckSwitchTest() {
  // Check fails (no pack), then OFF in the same boot: SW2 works without a reboot.
  fake::reset();
  fake::adcMillivolts[pins::BATTERY_ADC]=0;
  setup(); ticks(3100);
  CHECK(BatteryMonitor::status().result==BatteryCheckResult::Missing);
  send("MODE MANUAL"); send("SW2"); ticks(SW2_MS); CHECK(!SystemManager::motorAllowed());
  send("BAT CHECK OFF");
  CHECK(BatteryMonitor::status().result==BatteryCheckResult::Disabled);
  CHECK(!SystemManager::motorAllowed());                  // nothing starts by itself
  send("MODE MANUAL"); send("SW2"); ticks(SW2_MS); CHECK(SystemManager::motorAllowed());
  send("STOP"); send("BAT CHECK ON");
  fake::reset(); setup(); ticks(30);
  Serial.tx.clear(); send("BAT CHECK OFF");
  CHECK(Serial.tx.find("BATCHECK,0")!=std::string::npos);
  fake::adcMillivolts[pins::BATTERY_ADC]=0;   // no pack: would fail the check
  setup(); ticks(3100);                       // next power-on reads the saved switch
  CHECK(BatteryMonitor::status().result==BatteryCheckResult::Disabled);
  CHECK(SystemManager::status().applicationHealthy);
  send("MODE MANUAL"); send("SW2"); ticks(SW2_MS); CHECK(SystemManager::motorAllowed());
  send("STOP");
  Serial.tx.clear(); send("BAT CHECK ON");
  CHECK(Serial.tx.find("BATCHECK,1")!=std::string::npos);
}
void trimCommandTest() {
  fake::reset(); setup(); ticks(30);
  Serial.tx.clear(); send("TRIM"); CHECK(Serial.tx.find("TRIM,1000,1000")!=std::string::npos);
  Serial.tx.clear(); send("TRIM 950 1000");                 // RAM, applies at once
  CHECK(Serial.tx.find("TRIM,950,1000")!=std::string::npos && MotorTrim::left()==950);
  CHECK(fake::nvsU16.find("trimL")==fake::nvsU16.end());     // not saved yet
  Serial.tx.clear(); send("TRIM 700 1000");                 // out of range: rejected, unchanged
  CHECK(Serial.tx.find("ERR TRIM")!=std::string::npos && MotorTrim::left()==950);
  send("TRIM 950"); CHECK(MotorTrim::left()==950);           // malformed
  send("TRIM SAVE"); CHECK(fake::nvsU16["trimL"]==950 && fake::nvsU16["trimR"]==1000);
  send("TRIM RESET"); CHECK(MotorTrim::left()==1000);
  setup(); ticks(30); CHECK(MotorTrim::left()==950);         // next power-on reads the saved trim
}
void batteryFailureTest() {
  fake::reset();
  fake::adcMillivolts[pins::BATTERY_ADC] = 0;
  setup(); ticks(3100);
  CHECK(BatteryMonitor::status().result == BatteryCheckResult::Missing);
  CHECK(!SystemManager::status().applicationHealthy);
  send("MODE MANUAL"); startButton(); send("DRIVE 200 200");
  CHECK(!SystemManager::motorAllowed());
  CHECK(!MotorController::status().outputsActive);
}
#endif
void integrationTests() {
  fake::reset();
  for (unsigned i=0;i<4;++i) fake::floor(i,3);
  setup(); ticks(30);
  CHECK(MotorController::outputLimit()==400); CHECK(!SystemManager::motorAllowed());
  send("MODE MANUAL"); send("DRIVE 100 100");
  CHECK(!SystemManager::motorAllowed()); CHECK(!MotorController::status().outputsActive);
  fake::level(pins::START_SWITCH,LOW); ticks(10);
  send("MODE FOLLOW"); ticks(50);
  fake::level(pins::START_SWITCH,HIGH); ticks(50);
  CHECK(!SystemManager::motorAllowed());
  send("MODE MANUAL");
  startButton(); CHECK(SystemManager::motorAllowed());
  send("DRIVE 100 -100"); CHECK(MotorController::status().appliedLeftPermille==100);
  ticks(260); CHECK(!MotorController::status().outputsActive);
  send("DRIVE 150 150"); CHECK(MotorController::status().outputsActive);
#if BENCH_V2_MANUAL
  // PIN row reports the real pin levels: EN HIGH, forward PWM 150 permille = 38/255,
  // on RPWM because no V2 motor is inverted since the 11/04 rewiring (motor_config.hpp).
  Serial.tx.clear();
  for (int i=0;i<12;++i) { send("DRIVE 150 150"); ticks(100); }
  CHECK(Serial.tx.find(",0,0,1,1,0,38,0,38,")!=std::string::npos);
  CHECK(Serial.tx.find("PIN,")!=std::string::npos);
  // ADC row: every analog pin in mV at the GPIO; IS motor 2 left is GPIO20.
  fake::adcMillivolts[pins::MOTOR2_L_IS]=1234; fake::adcMillivolts[pins::IR_INPUTS[5]]=567;
  Serial.tx.clear();
  for (int i=0;i<20;++i) { send("DRIVE 150 150"); ticks(100); }
  CHECK(Serial.tx.find("ADC,")!=std::string::npos);
  CHECK(Serial.tx.find(",1234,")!=std::string::npos && Serial.tx.find(",567,")!=std::string::npos);
#endif
  send("MODE ALIGN"); CHECK(!SystemManager::motorAllowed()); CHECK(!MotorController::status().outputsActive);
  startButton(); ticks(20); CHECK(!MotorController::status().outputsActive);  // No Pi target.
  camera(1300,200); CHECK(MotorController::status().appliedLeftPermille>0);
  CHECK(MotorController::status().appliedRightPermille<0);
  ticks(130); CHECK(!MotorController::status().outputsActive);
  // SW2 held 3 s while stopped picks ALIGN and the release starts it; 2.9 s does not.
  send("MODE STOP");
  fake::level(pins::START_SWITCH,LOW); ticks(2900); fake::level(pins::START_SWITCH,HIGH); ticks(50);
  CHECK(BenchController::mode()==BenchMode::Stop);
  CHECK(SystemManager::motorAllowed());  // a normal start in STOP mode: nothing moves
  camera(1300,200,5); CHECK(!MotorController::status().outputsActive);
  send("STOP");
  fake::level(pins::START_SWITCH,LOW); ticks(3100);
  CHECK(BenchController::mode()==BenchMode::Align); CHECK(!SystemManager::motorAllowed());
  fake::level(pins::START_SWITCH,HIGH); ticks(50); CHECK(SystemManager::motorAllowed());
  camera(600,200,6); CHECK(MotorController::status().appliedLeftPermille<0);
  CHECK(MotorController::status().appliedRightPermille>0);
  camera(960,200,7); CHECK(!MotorController::status().outputsActive);  // centered: stop
  send("MODE FOLLOW"); startButton(); camera(960,200,2);
  CHECK(MotorController::status().appliedLeftPermille==160);
  CHECK(MotorController::status().appliedRightPermille==160);
  camera(960,420,3); CHECK(!MotorController::status().outputsActive);
  send("MODE AVOID"); startButton(); camera(960,200,4);
  CHECK(MotorController::status().appliedLeftPermille==160);
  fake::sensors[0].connected=false; ticks(20); CHECK(!MotorController::status().outputsActive);
  send("MODE MANUAL"); startButton(); send("DRIVE 100 100");
  send("DRIVE 99999999999999999999999999999999999999999999999999999 1");
  CHECK(!SystemManager::motorAllowed()); CHECK(!MotorController::status().outputsActive);
  send("MODE MANUAL"); startButton(); send("DRIVE 100 100");
  Serial.inject("DRIVE 20"); ticks(300); CHECK(!SystemManager::motorAllowed());
  send("0 200"); CHECK(!MotorController::status().outputsActive);  // Discard remainder.
  send("MODE DATA"); ticks(150); CHECK(Serial.tx.find("D,")!=std::string::npos);
  CHECK(Serial.tx.find("S,")!=std::string::npos);
  // The D row carries the whole Pi target for the GUI camera box: ...,x,height,hazard,y,width,flags,seq
  camera(700,300,9); ticks(150);
  CHECK(Serial.tx.find(",1,1,1,700,300,0,540,100,49,9\r\n")!=std::string::npos);
#if BENCH_V2_MANUAL
  ticks(900);  // extra rows take turns over 8 frames
  CHECK(Serial.baud == 57600);
  CHECK(Serial.tx.find("\nSYSTEM,") != std::string::npos);
  CHECK(Serial.tx.find("\nMOTOR,") != std::string::npos);
  CHECK(Serial.tx.find("\nLINK,") != std::string::npos);
  CHECK(Serial.tx.find("\nPOWER_LED,") != std::string::npos);
  CHECK(Serial.tx.find("\nI,") != std::string::npos);
  // Every telemetry line fits the 128-byte UART0 hardware FIFO.
  for (size_t a = 0, b; (b = Serial.tx.find('\n', a)) != std::string::npos; a = b + 1) CHECK(b - a + 1 < 128);
  CHECK(BatteryMonitor::status().result == BatteryCheckResult::Ok);
  CHECK(StatusLed::status().initialized);
  send("LOG OFF");
  Serial.tx.clear(); ticks(150);
  CHECK(Serial.tx.find("D,") == std::string::npos);
  send("LOG ON"); ticks(150);
  CHECK(Serial.tx.find("D,") != std::string::npos);
#endif
  Serial.capacity=0; const auto before=millis(); loop(); CHECK(millis()-before<=5);
#if BENCH_V2_MANUAL
  Serial.tx.clear();
  send("MODE MANUAL"); startButton(); send("DRIVE 100 -100");
  CHECK(MotorController::status().outputsActive);
  ticks(260);
  CHECK(!MotorController::status().outputsActive);
  CHECK(Serial.tx.empty());
#endif
  Serial.capacity=4096;
  send("MODE MANUAL"); startButton(); send("DRIVE 100 100");
  // SW1 acts only after the 0.5 s hold (impacts pressed it on the robot).
  fake::level(pins::SAFETY_SWITCH,LOW); loop(); CHECK(!ButtonInterrupts::emergencyLatched());
  fake::advance(config::switches::HOLD_MS);
  fake::stopOnDelay=true;
  try { loop(); CHECK(false); } catch (const fake::StopLoop&) {}
  CHECK(ButtonInterrupts::emergencyLatched());
  CHECK(MotorController::status().emergencyStopLatched);
}
}
int main(int argc, char** argv) {
  try {
#if HW_VERSION == 2
    if (argc > 1 && std::string(argv[1]) == "--battery-missing") batteryFailureTest();
    else
#else
    (void)argc; (void)argv;
#endif
    { logicTests(); integrationTests();
#if BENCH_V2_MANUAL
      softwareSwitchTests();
      i2cScanTests();
      softwareEmergencyDuringCalibrationTest();
      imuTests();
      calibrationTests();
      batteryCheckSwitchTest();
      trimCommandTest();
#endif
    }
    std::cout<<"BENCH MODEL "<<COLOR_SENSOR_MODEL<<": "<<checks<<" checks passed\n";
  } catch (const std::exception& e) { std::cerr<<"FAIL "<<e.what()<<"\n"; return 1; }
}
