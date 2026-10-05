#include "Arduino.h"
#include "Wire.h"
#include "Preferences.h"
#include "app/boot_manager.hpp"
#include "communication/raspberry_link.hpp"
#include "communication/vision_protocol.hpp"
#include "control/control_config.hpp"
#include "control/drive_logic.hpp"
#include "control/robot_fsm.hpp"
#include "interrupts/button_interrupts.hpp"
#include "motor/motor_controller.hpp"
#include "pins.hpp"
#include "sensors/color_calibration.hpp"
#include "sensors/color_sensor_manager.hpp"
#include "sensors/floor_color_classifier.hpp"
#include "sensors/sensor_config.hpp"
#include "sensors/sensor_manager.hpp"
#include "system/event_logger.hpp"
#include "system/status_led.hpp"
#include "system/system_manager.hpp"
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>

void setup();
void loop();
namespace {
int assertions=0, groups=0;
#define CHECK(condition) do { ++assertions; if (!(condition)) throw std::runtime_error(std::string(__func__)+":"+std::to_string(__LINE__)+" " #condition); } while(false)

SafetyStatus running() { SafetyStatus s; s.state=SafetyState::Running; s.motorAllowed=true; return s; }
void fixture() {
  fake::reset(); EventLogger::begin(); MotorController::begin();
  ButtonInterrupts::begin(); StatusLed::begin(); SystemManager::begin();
}
void pressStart() {
  fake::level(pins::START_SWITCH,LOW); SystemManager::update(millis());
  fake::advance(40); SystemManager::update(millis());
  fake::level(pins::START_SWITCH,HIGH); SystemManager::update(millis());
  fake::advance(40); SystemManager::update(millis());
}
std::vector<uint8_t> packet(uint8_t seq, uint16_t x=960, uint16_t height=300, uint8_t flags=0x33) {
  const uint16_t y=flags&1 ? 500 : 0, width=flags&1 ? 200 : 0;
  if(!(flags&1)) { x=0; height=0; }
  std::vector<uint8_t> p{0xAA,0x55,1,10,seq,flags};
  for(uint16_t value : {x,y,width,height}) { p.push_back(value&255); p.push_back(value>>8); }
  return p;
}
void feed(const std::vector<uint8_t>& bytes) {
  Serial1.inject(bytes); while(Serial1.available()) RaspberryLink::poll();
}
VisionSnapshot target(uint8_t seq, uint32_t frameMs, uint16_t x=960, uint16_t height=300) {
  VisionSnapshot v; v.sequence=seq; v.frameUpdatedMs=frameMs; v.receivedMs=frameMs;
  v.targetX=x; v.targetHeight=height; v.targetWidth=200; v.targetY=500;
  v.linkValid=v.frameFresh=v.targetValid=v.cameraOk=v.pipelineOk=true;
  return v;
}
void sensorTicks(uint32_t duration, bool manager=false) {
  for(uint32_t i=0;i<duration;++i) {
    fake::advance(1); const uint32_t before=millis();
    if(manager) SensorManager::update(); else ColorSensorManager::update(millis());
    CHECK(millis()==before);
  }
}
void command(const std::string& text, bool allowed=true) {
  Serial.inject(text+"\n");
  while(Serial.available()) ColorCalibration::update(millis(),allowed);
}

void testMotor() {
  fixture(); CHECK(MotorController::status().initialized);
  CHECK(!MotorController::forward(300,running()));
  CHECK(MotorController::clearEmergencyStop(running()));
  CHECK(MotorController::setTank(-32768,32767,running()));
  auto s=MotorController::status(); CHECK(s.appliedLeftPermille==-650 && s.appliedRightPermille==650);
  CHECK(!MotorController::setOutputLimit(1001)); CHECK(MotorController::setOutputLimit(500));
  CHECK(!MotorController::status().outputsActive);
  CHECK(MotorController::setArcade(300,80,running()));
  s=MotorController::status(); CHECK(s.appliedLeftPermille==380 && s.appliedRightPermille==220);
  fake::advance(99); CHECK(MotorController::status().outputsActive);
  MotorController::checkTimeout(millis()+1); CHECK(!MotorController::status().outputsActive);
  CHECK(MotorController::status().watchdogStopCount==1);
  MotorController::checkTimeout(millis()+100); CHECK(MotorController::status().watchdogStopCount==1);
  // A deadline of zero at millis rollover is an actual deadline.
  fake::now=UINT32_MAX-99; CHECK(MotorController::forward(200,running(),100));
  fake::advance(100); CHECK(!MotorController::status().outputsActive);
  SafetyStatus invalid=running(); invalid.state=SafetyState::WaitingForStart;
  CHECK(!MotorController::forward(200,invalid)); CHECK(!MotorController::clearEmergencyStop(invalid));
}

void testButtonsAndEstop() {
  fixture(); fake::level(pins::START_SWITCH,LOW); ButtonInterrupts::update(millis());
  fake::advance(10); fake::level(pins::START_SWITCH,HIGH); ButtonInterrupts::update(millis());
  fake::advance(50); ButtonInterrupts::update(millis()); CHECK(!ButtonInterrupts::consumeStartEvent());
  pressStart(); CHECK(SystemManager::running());
  CHECK(MotorController::forward(500,running()));
  // Inject the emergency edge while a motor write is in progress.
  fake::writeHook=[] { fake::level(pins::SAFETY_SWITCH,LOW); };
  MotorController::forward(400,running());
  CHECK(ButtonInterrupts::emergencyLatched());
  CHECK(!Bts7960Driver::clearDriveInhibit());
  fake::stopOnDelay=true;
  try { SystemManager::update(millis()); CHECK(false); } catch(const fake::StopLoop&) {}
  fake::stopOnDelay=false;
  for(int i=0;i<4;++i) CHECK(fake::pwm[i]==0);
  CHECK(EventLogger::status().emergencyOnly);
  CHECK(Serial.tx.find("ESTOP")!=std::string::npos);
  fake::level(pins::SAFETY_SWITCH,HIGH);
  CHECK(!MotorController::clearEmergencyStop(running()));
  CHECK(!MotorController::forward(300,running()));
  // Boot with SW2 held: its first release must not start.
  fixture(); fake::level(pins::START_SWITCH,LOW,false); ButtonInterrupts::begin();
  fake::level(pins::START_SWITCH,HIGH); ButtonInterrupts::update(millis());
  fake::advance(40); ButtonInterrupts::update(millis()); CHECK(!ButtonInterrupts::consumeStartEvent());
  pressStart(); CHECK(SystemManager::running());
  // A release bounce inside the debounce window restarts the timer.
  fixture(); fake::level(pins::START_SWITCH,LOW); ButtonInterrupts::update(millis());
  fake::advance(40); ButtonInterrupts::update(millis());
  fake::level(pins::START_SWITCH,HIGH); ButtonInterrupts::update(millis());
  fake::advance(35); fake::level(pins::START_SWITCH,LOW); fake::level(pins::START_SWITCH,HIGH);
  ButtonInterrupts::update(millis()); fake::advance(5); ButtonInterrupts::update(millis());
  CHECK(!ButtonInterrupts::consumeStartEvent()); fake::advance(35); ButtonInterrupts::update(millis());
  CHECK(ButtonInterrupts::consumeStartEvent()); CHECK(!ButtonInterrupts::consumeStartEvent());
}

void testSystem() {
  fixture(); SystemManager::update(millis()); CHECK(SystemManager::state()==SystemState::WaitingForStart);
  pressStart(); CHECK(SystemManager::motorAllowed());
  SystemManager::setMotorHealthy(false,FaultCode::Driver);
  CHECK(!SystemManager::motorAllowed()); CHECK(!SystemManager::status().runRequested);
  pressStart(); CHECK(!SystemManager::status().runRequested);
  SystemManager::setMotorHealthy(true); CHECK(!SystemManager::motorAllowed());
  pressStart(); CHECK(SystemManager::motorAllowed());
  SystemManager::setCalibrationActive(true); CHECK(!SystemManager::motorAllowed());
  pressStart(); CHECK(!SystemManager::status().runRequested);
  SystemManager::setCalibrationActive(false); CHECK(!SystemManager::running());
  pressStart(); CHECK(SystemManager::running()); SystemManager::requestStop(); CHECK(!SystemManager::running());
}

void testProtocol() {
  fixture(); RaspberryLink::begin(); feed(packet(255));
  CHECK(RaspberryLink::latest().targetValid); feed(packet(0)); CHECK(RaspberryLink::status().lastSequence==0);
  feed(packet(0)); CHECK(RaspberryLink::status().duplicatePackets==1);
  feed(packet(0,1000)); CHECK(RaspberryLink::status().invalidPackets==1);
  fake::advance(80); feed(packet(0)); fake::advance(50); feed(packet(0));
  CHECK(RaspberryLink::latest().linkValid && !RaspberryLink::latest().frameFresh);
  fake::advance(121); feed(packet(0,1000)); CHECK(RaspberryLink::latest().targetX==1000);
  feed(packet(1,1920)); CHECK(RaspberryLink::status().invalidPackets==2);
  feed(packet(2,960,300,0xB3)); CHECK(RaspberryLink::status().invalidPackets==3);
  auto partial=packet(3); partial.resize(8); feed(partial); fake::advance(21); RaspberryLink::poll();
  CHECK(RaspberryLink::status().invalidPackets==4);
  auto truncated=packet(4); truncated.erase(truncated.begin()+8); feed(truncated); feed(packet(5));
  CHECK(RaspberryLink::status().lastSequence==5);
  feed(packet(6,0,0,0x30)); CHECK(!RaspberryLink::latest().targetValid);
  fake::now=UINT32_MAX-20; feed(packet(7)); fake::advance(50); CHECK(RaspberryLink::latest().frameFresh);
  fake::advance(80); CHECK(!RaspberryLink::latest().linkValid);
  uint32_t random=12345;
  std::vector<uint8_t> noise;
  for(int i=0;i<50000;++i) { random=random*1664525U+1013904223U; noise.push_back(random>>24); }
  feed(noise); feed(packet(8)); feed(packet(9)); CHECK(RaspberryLink::status().lastSequence==9);
}

void testFsm() {
  fixture(); RobotFsm::begin(); SensorSnapshot sensors; VisionSnapshot missing;
  CHECK(RobotFsm::update(0,sensors,missing,SafetyStatus{}).type==DriveIntentType::Brake);
  CHECK(RobotFsm::update(0,sensors,missing,running()).type==DriveIntentType::SearchLeft);
  RobotFsm::update(10,sensors,target(1,10),running()); CHECK(RobotFsm::state()==RobotState::TargetConfirm);
  RobotFsm::update(11,sensors,target(1,10),running()); CHECK(RobotFsm::state()==RobotState::TargetConfirm);
  auto drive=RobotFsm::update(30,sensors,target(2,30,1100),running());
  CHECK(RobotFsm::state()==RobotState::Approach);
  auto tank=DriveLogic::makeCommand(30,drive); CHECK(tank.leftPermille>tank.rightPermille);
  drive=RobotFsm::update(40,sensors,target(3,40,960,500),running());
  CHECK(drive.type==DriveIntentType::AttackCharge);
  CHECK(RobotFsm::update(259,sensors,missing,running()).type==DriveIntentType::AttackCharge);
  CHECK(RobotFsm::update(260,sensors,missing,running()).type==DriveIntentType::DiveBrake);
  CHECK(RobotFsm::update(349,sensors,missing,running()).type==DriveIntentType::DiveBrake);
  CHECK(RobotFsm::update(350,sensors,missing,running()).type==DriveIntentType::SpokeInsert);
  RobotFsm::update(699,sensors,target(4,699,960,600),running());
  CHECK(RobotFsm::update(700,sensors,missing,running()).type==DriveIntentType::Push);
  CHECK(RobotFsm::update(1198,sensors,missing,running()).type==DriveIntentType::Push);
  CHECK(RobotFsm::update(1199,sensors,missing,running()).type==DriveIntentType::Reverse);
  RobotFsm::update(1459,sensors,missing,running()); CHECK(RobotFsm::state()==RobotState::Search);
  RobotFsm::update(1460,sensors,missing,SafetyStatus{});
  RobotFsm::update(1461,sensors,missing,running()); CHECK(RobotFsm::state()==RobotState::Search);
  // Hazard preemption at every attack phase.
  for(uint32_t attackTime : {40U,260U,350U,700U}) {
    RobotFsm::begin(); RobotFsm::update(10,sensors,target(1,10,960,500),running());
    RobotFsm::update(30,sensors,target(2,30,960,500),running());
    if(attackTime>=260) RobotFsm::update(250,sensors,target(3,250,960,500),running());
    if(attackTime>=350) RobotFsm::update(340,sensors,target(4,340,960,500),running());
    if(attackTime>=700) RobotFsm::update(690,sensors,target(5,690,960,500),running());
    sensors.floorColor[0].color=FloorColor::Red; sensors.floorColor[0].reliable=true;
    CHECK(RobotFsm::update(attackTime,sensors,missing,running()).type==DriveIntentType::Reverse);
    CHECK(RobotFsm::state()==RobotState::ZoneEscape); sensors=SensorSnapshot{};
  }
  for(uint8_t corner=0;corner<4;++corner) {
    RobotFsm::begin(); sensors=SensorSnapshot{};
    sensors.floorColor[corner].color=FloorColor::Blue; sensors.floorColor[corner].reliable=true;
    auto escape=RobotFsm::update(10,sensors,missing,running());
    CHECK(escape.type==(corner<2 ? DriveIntentType::Reverse : DriveIntentType::Approach));
    escape=RobotFsm::update(370,sensors,missing,running());
    CHECK(escape.type==((corner%2)==0 ? DriveIntentType::EscapeRight : DriveIntentType::EscapeLeft));
    RobotFsm::update(690,sensors,missing,running()); CHECK(RobotFsm::state()==RobotState::ZoneEscape);
    sensors=SensorSnapshot{}; RobotFsm::update(1370,sensors,missing,running()); CHECK(RobotFsm::state()==RobotState::Search);
  }
  // millis wrap and frame timestamp zero are valid values.
  sensors=SensorSnapshot{}; RobotFsm::begin();
  RobotFsm::update(UINT32_MAX-10,sensors,target(1,UINT32_MAX-10),running());
  RobotFsm::update(0,sensors,target(2,0),running());
  RobotFsm::update(20,sensors,missing,running()); CHECK(RobotFsm::state()==RobotState::LostTarget);
  RobotFsm::update(260,sensors,missing,running()); CHECK(RobotFsm::state()==RobotState::Search);
}

void testColorsAndErrors() {
  fixture(); CHECK(SensorManager::begin());
  for(uint8_t i=0;i<4;++i) fake::floor(i,i);
  sensorTicks(300,true);
  auto snapshot=SensorManager::snapshot();
  for(uint8_t i=0;i<4;++i) {
    CHECK(snapshot.floorColor[i].reliable); CHECK(static_cast<uint8_t>(snapshot.floorColor[i].color)==i);
    CHECK(snapshot.floorColor[i].consecutiveMatches>=3);
  }
  // Invalid sample clears the old reliable color immediately; no auto reinit.
  fake::sensors[0].connected=false; sensorTicks(20,true);
  CHECK(!SensorManager::snapshot().floorColor[0].reliable);
  CHECK(ColorSensorManager::status().error[0]==ColorSensorError::I2c);
  const uint32_t failures=ColorSensorManager::status().failedReads;
  fake::sensors[0].connected=true; sensorTicks(200,true);
  CHECK(!ColorSensorManager::available(0)); CHECK(ColorSensorManager::status().failedReads==failures);
  CHECK(SensorManager::snapshot().floorColor[1].reliable);
  fixture(); fake::sensors[1].connected=false; SensorManager::begin();
  CHECK(ColorSensorManager::status().error[1]==ColorSensorError::Initialization);
  CHECK(SystemManager::status().sensorsHealthy);
  fixture(); ColorSensorManager::begin(); fake::sensors[0].conversionStuck=true; sensorTicks(300);
  CHECK(ColorSensorManager::status().error[0]==ColorSensorError::Timeout);
#if COLOR_SENSOR_MODEL == COLOR_SENSOR_AS7341
  fixture(); ColorSensorManager::begin(); fake::sensors[0].smuxStuck=true; sensorTicks(100);
  CHECK(ColorSensorManager::status().error[0]==ColorSensorError::Timeout);
  fixture(); SensorManager::begin(); fake::sensors[0].saturated=true;
  sensorTicks(200,true); CHECK(ColorSensorManager::status().error[0]==ColorSensorError::Range);
  CHECK(!SensorManager::snapshot().floorColor[0].reliable);
#endif
  fixture(); SensorManager::begin();
  fake::sensors[0].low.fill(65535); fake::sensors[0].high.fill(65535); fake::sensors[0].rgbc.fill(65535);
  sensorTicks(200,true); CHECK(!SensorManager::snapshot().floorColor[0].reliable);
  CHECK(ColorSensorManager::status().error[0]==ColorSensorError::Range);
  CHECK(ColorSensorManager::available(0));
  auto raw=ColorSensorManager::sample(1);
  raw.normalized[ColorSensorManager::model()==ColorSensorModel::AS7341 ? 1 : 0]=
      std::numeric_limits<float>::quiet_NaN();
  CHECK(!FloorColorClassifier::classify(1,raw,ColorCalibration::data()).reliable);
}

void testThreeSamples() {
  fixture(); SensorManager::begin(); fake::floor(0,0);
  uint32_t oldSequence=0; unsigned newSamples=0;
  for(int i=0;i<400 && newSamples<3;++i) {
    fake::advance(1); SensorManager::update(); auto snapshot=SensorManager::snapshot();
    if(snapshot.colorRaw[0].sequence!=oldSequence) {
      oldSequence=snapshot.colorRaw[0].sequence; ++newSamples;
      CHECK(snapshot.floorColor[0].reliable==(newSamples>=3));
    }
  }
  CHECK(newSamples==3);
  fake::floor(0,2); const uint32_t previousSequence=oldSequence;
  while(ColorSensorManager::sample(0).sequence==previousSequence) { fake::advance(1); SensorManager::update(); }
  CHECK(!SensorManager::snapshot().floorColor[0].reliable);
}

void testContinuousFloorMode() {
#if COLOR_SENSOR_MODEL == COLOR_SENSOR_AS7341
  fixture(); CHECK(SensorManager::begin());
  fake::floor(0,0); sensorTicks(40);
  const auto before=ColorSensorManager::sample(0);
  CHECK(before.valid); CHECK(before.featureMask==0x76);
  CHECK(before.channel[1]==80 && before.channel[2]==80);
  CHECK(before.channel[4]==200 && before.channel[5]==200 && before.channel[6]==700);
  CHECK(before.channel[8]==1000 && before.channel[0]==0 && before.channel[9]==0);
  CHECK(fake::sensors[0].reg[0x80]==3);
  CHECK(fake::sensors[0].reg[0xBD]==0 && fake::sensors[0].reg[0xF9]==4);
  CHECK(fake::sensors[0].reg[0xD6]==255);
  const uint8_t route[20]={0x20,0,0,0,4,1,0x30,5,0x60,0x30,5,0,0x10,0,0x40,0x20,0,0x60,0,0};
  for(unsigned i=0;i<20;++i) CHECK(fake::sensors[0].reg[i]==route[i]);
  const uint32_t start=fake::sensors[0].measurementStarted;
  for(int i=0;i<100;++i) SensorManager::update();
  CHECK(ColorSensorManager::sample(0).sequence==before.sequence);
  // AVALID stays true, but no new completion event means no new sample.
  fake::sensors[0].conversionStuck=true;
  sensorTicks(25);
  CHECK(ColorSensorManager::sample(0).sequence==before.sequence);
  fake::sensors[0].conversionStuck=false; sensorTicks(15);
  CHECK(ColorSensorManager::sample(0).sequence>before.sequence);
  CHECK(fake::sensors[0].measurementStarted==start);  // No stop/restart per read.
  fake::sensors[0].saturated=true; sensorTicks(10);
  CHECK(!ColorSensorManager::sample(0).valid);
  fake::sensors[0].saturated=false; sensorTicks(10);
  CHECK(ColorSensorManager::sample(0).valid);
  auto raw=ColorSensorManager::sample(0);
  raw.brightness=0; raw.valid=false;
  CHECK(!FloorColorClassifier::classify(0,raw,ColorCalibration::data()).reliable);
#endif
}

void testCalibration() {
  fixture(); SensorManager::begin(); command("CAL START",false); CHECK(!ColorCalibration::active());
  command(std::string(80,'X')+"CAL START"); CHECK(!ColorCalibration::active());
  command("CAL START"); CHECK(ColorCalibration::active());
  command("READY"); CHECK(ColorCalibration::status().state==CalibrationState::Countdown);
  command("ABORT"); CHECK(!ColorCalibration::active());
  command("CAL START"); ColorCalibration::update(millis(),false); CHECK(!ColorCalibration::active());
  command("CAL START"); command("CAL MARGIN 20 40");
  for(uint8_t step=0;step<48;++step) {
    auto status=ColorCalibration::status(); fake::floor(status.sensorIndex,static_cast<uint8_t>(status.color));
    // Model a different brightness for level/lifted/pressed references.
    const unsigned scale=status.pose==CalibrationPose::Level ? 2 :
                          status.pose==CalibrationPose::Lifted ? 1 : 3;
    auto& device=fake::sensors[status.sensorIndex];
    for(auto& value:device.low) value=value*scale/4;
    for(auto& value:device.high) value=value*scale/4;
    for(auto& value:device.rgbc) value=value*scale/4;
    command("READY");
    // Two 3-second intervals: countdown then independent samples.
    for(int i=0;i<6100;++i) {
      fake::advance(1); SensorManager::update(); ColorCalibration::update(millis(),true);
    }
    CHECK(ColorCalibration::status().state==CalibrationState::WaitingForAccept);
    CHECK(ColorCalibration::status().sampleCount>=config::sensors::CALIBRATION_MIN_SAMPLES);
    if(step==0) {
      command("RETRY"); CHECK(ColorCalibration::status().state==CalibrationState::WaitingForReady);
      command("READY");
      for(int i=0;i<6100;++i) { fake::advance(1); SensorManager::update(); ColorCalibration::update(millis(),true); }
    }
    command("ACCEPT");
    if(step==0) {
      command("BACK"); CHECK(ColorCalibration::status().step==0);
      command("READY");
      for(int i=0;i<6100;++i) { fake::advance(1); SensorManager::update(); ColorCalibration::update(millis(),true); }
      command("ACCEPT"); CHECK(ColorCalibration::status().step==1);
    }
  }
  CHECK(ColorCalibration::status().state==CalibrationState::Completed);
  command("STATUS"); CHECK(Serial.tx.find("step=48/48")!=std::string::npos);
  fake::nvsWriteOk=false; command("CAL SAVE"); CHECK(ColorCalibration::status().state==CalibrationState::Error);
  fake::nvsWriteOk=true; command("CAL SAVE"); CHECK(ColorCalibration::hasStoredData());
  CHECK(!ColorCalibration::active()); CHECK(!fake::nvs.empty());
  for(uint8_t corner=0;corner<4;++corner) {
    for(uint8_t color=0;color<4;++color) {
      fake::floor(corner,color); sensorTicks(200);
      const auto raw=ColorSensorManager::sample(corner);
      const auto result=FloorColorClassifier::classify(corner,raw,ColorCalibration::data());
      CHECK(result.reliable); CHECK(static_cast<uint8_t>(result.color)==color);
    }
  }
  const auto saved=fake::nvs;
  ColorCalibration::begin(); CHECK(ColorCalibration::hasStoredData());
  fake::nvsWriteOk=false; command("CAL ERASE"); CHECK(ColorCalibration::hasStoredData());
  fake::nvsWriteOk=true; command("CAL ERASE"); CHECK(!ColorCalibration::hasStoredData()); CHECK(fake::nvs.empty());
  fake::nvs=saved; auto corrupt=ColorCalibrationData{}; std::memcpy(&corrupt,fake::nvs.data(),sizeof(corrupt));
  corrupt.settingsSignature^=1; std::memcpy(fake::nvs.data(),&corrupt,sizeof(corrupt));
  ColorCalibration::begin(); CHECK(!ColorCalibration::hasStoredData());
  fake::nvs=saved; std::memcpy(&corrupt,fake::nvs.data(),sizeof(corrupt));
  corrupt.reference[0][0][0].brightnessMean=std::numeric_limits<float>::quiet_NaN();
  std::memcpy(fake::nvs.data(),&corrupt,sizeof(corrupt)); ColorCalibration::begin(); CHECK(!ColorCalibration::hasStoredData());
  fake::nvs=saved; fake::nvs.pop_back(); ColorCalibration::begin(); CHECK(!ColorCalibration::hasStoredData());
  fake::nvs=saved; fake::nvsReadOk=false; ColorCalibration::begin(); CHECK(!ColorCalibration::hasStoredData());
}

void testLogsAndLeds() {
  fixture(); EventLogger::begin();
  CHECK(EventLogger::logText(900,LogSeverity::Error,LogSource::User,
                            "123456789012345678901234567890123456789"));
  CHECK(EventLogger::log(EventId::InvalidPacket,1)); CHECK(!EventLogger::log(EventId::InvalidPacket,1));
  EventLogger::stateChanged(2); CHECK(!EventLogger::stateChanged(2));
  auto before=EventLogger::status(); Serial.capacity=32; EventLogger::process();
  CHECK(EventLogger::status().queued==before.queued);
  Serial.capacity=4096; EventLogger::process(); CHECK(EventLogger::status().written==2);
  StatusLed::begin(); StatusLed::setBoth(LedOwner::Fsm,LedState::Searching);
  StatusLed::setBoth(LedOwner::Sensors,LedState::SensorFault,40);
  fake::advance(20); StatusLed::update(millis()); CHECK(StatusLed::state(LedTarget::Both)==LedState::SensorFault);
  fake::advance(20); StatusLed::update(millis()); CHECK(StatusLed::state(LedTarget::Both)==LedState::Searching);
  StatusLed::setEmergencyStop(); fake::advance(20); StatusLed::update(millis()); CHECK(StatusLed::state(LedTarget::Both)==LedState::EmergencyStop);
  fixture(); fake::now=UINT32_MAX-39; StatusLed::begin();
  StatusLed::setBoth(LedOwner::Fsm,LedState::Searching);
  StatusLed::setBoth(LedOwner::Sensors,LedState::SensorFault,40);
  fake::advance(20); StatusLed::update(millis()); CHECK(StatusLed::state(LedTarget::Both)==LedState::SensorFault);
  fake::advance(20); StatusLed::update(millis()); CHECK(millis()==0);
  CHECK(StatusLed::state(LedTarget::Both)==LedState::Searching);
}

void testRealisticTimingAndBoot() {
  fixture(); fake::transactionDelayMs=1; SensorManager::begin();
  for(uint8_t i=0;i<4;++i) fake::floor(i,i);
  for(int i=0;i<800;++i) { fake::advance(1); SensorManager::update(); }
  for(uint8_t i=0;i<4;++i) CHECK(SensorManager::snapshot().floorColor[i].reliable);
  fake::reset(); for(auto& sensor:fake::sensors) sensor.connected=false;
  const auto boot=BootManager::begin(); CHECK(boot.safeToRun);
  CHECK(SensorManager::snapshot().colorSensorReadyMask==0);
  pressStart(); CHECK(SystemManager::motorAllowed());
  RobotFsm::begin();
  CHECK(RobotFsm::update(millis(),SensorManager::snapshot(),VisionSnapshot{},
                        SystemManager::safetyStatus()).type==DriveIntentType::SearchLeft);
  fake::reset(); fake::pwmSetupOk=false;
  CHECK(!BootManager::begin().safeToRun); CHECK(!SystemManager::motorAllowed());
  fake::reset(); fake::level(pins::SAFETY_SWITCH,LOW,false); fake::stopOnDelay=true;
  try { BootManager::begin(); CHECK(false); } catch(const fake::StopLoop&) {}
  fake::stopOnDelay=false; CHECK(ButtonInterrupts::emergencyLatched());
  CHECK(!SystemManager::motorAllowed()); CHECK(Serial.tx.find("ESTOP")!=std::string::npos);
}

void testMainIntegration() {
  fake::reset(); setup();
  for(uint8_t i=0;i<4;++i) fake::floor(i,3);
  for(int i=0;i<300;++i) loop();
  CHECK(!MotorController::status().outputsActive);
  Serial.inject("CAL START\n"); loop(); CHECK(ColorCalibration::active()); CHECK(!SystemManager::motorAllowed());
  fake::level(pins::START_SWITCH,LOW); for(int i=0;i<50;++i) loop();
  CHECK(!ColorCalibration::active()); CHECK(!SystemManager::motorAllowed());
  fake::level(pins::START_SWITCH,HIGH); for(int i=0;i<50;++i) loop();
  CHECK(!SystemManager::motorAllowed());
  fake::level(pins::START_SWITCH,LOW); for(int i=0;i<50;++i) loop();
  fake::level(pins::START_SWITCH,HIGH); for(int i=0;i<50;++i) loop();
  CHECK(SystemManager::motorAllowed()); CHECK(MotorController::status().outputsActive);
  CHECK(RobotFsm::state()==RobotState::Search);
  Serial1.inject(packet(1)); loop(); Serial1.inject(packet(2)); loop();
  CHECK(RobotFsm::state()==RobotState::Approach);
  fake::advance(110); CHECK(!MotorController::status().outputsActive);
  loop(); CHECK(SystemManager::motorAllowed());  // No restart policy for a brief command expiry.
}

void demo() {
  fixture(); RobotFsm::begin(); SensorSnapshot sensors;
  const char* states[]={"Boot","WaitStart","Search","TargetConfirm","TrackAlign",
    "Approach","ContactCheck","AttackCharge","DiveBrake","SpokeInsert","UndercutDrive",
    "Push","ScoreHold","Backoff","LostTarget","WallEscape","ZoneEscape","StuckRecovery",
    "EmergencyStop","Stop"};
  std::cout<<"time_ms,state,left,right (permille)\n";
  for(uint32_t time=0;time<=1600;time+=20) {
    VisionSnapshot vision;
    if(time>=100 && time<900) vision=target(static_cast<uint8_t>(time/20),time,960,time<300?300:600);
    const auto drive=RobotFsm::update(time,sensors,vision,running());
    const auto motors=DriveLogic::makeCommand(time,drive);
    if(time%100==0 || RobotFsm::state()==RobotState::DiveBrake)
      std::cout<<time<<","<<states[static_cast<uint8_t>(RobotFsm::state())]<<","<<motors.leftPermille<<","<<motors.rightPermille<<"\n";
  }
}
}
int main(int argc,char** argv) {
  try {
    if(argc>1 && std::string(argv[1])=="--demo") { demo(); return 0; }
    struct Test { const char* name; void (*run)(); };
    const Test tests[]={{"motor limit/watchdog/rollover",testMotor},
      {"buttons/ESTOP/race",testButtonsAndEstop},{"system start/fault/calibration",testSystem},
      {"UART loss/duplicates/reboot/fuzz",testProtocol},{"FSM attack/hazards/loss",testFsm},
      {"color sampling/errors/no recovery",testColorsAndErrors},{"three independent samples",testThreeSamples},
      {"continuous single-exposure floor mode",testContinuousFloorMode},
      {"48-step calibration/NVS",testCalibration},{"nonblocking logs/LED priority",testLogsAndLeds},
      {"real main loop integration",testMainIntegration},
      {"I2C time/optional sensors/boot ESTOP",testRealisticTimingAndBoot}};
    for(const auto& test:tests) { test.run(); ++groups; std::cout<<"PASS "<<test.name<<"\n"; }
    std::cout<<"MODEL "<<COLOR_SENSOR_MODEL<<": "<<groups<<" groups, "<<assertions<<" checks passed\n";
  } catch(const std::exception& error) { std::cerr<<"FAIL "<<error.what()<<"\n"; return 1; }
}
