#include "app/boot_manager.hpp"

#include "communication/raspberry_link.hpp"
#include "hardware/board_io.hpp"
#include "hardware/reset_manager.hpp"
#include "interrupts/button_interrupts.hpp"
#include "motor/motor_controller.hpp"
#include "sensors/sensor_manager.hpp"
#include "system/event_logger.hpp"
#include "system/status_led.hpp"
#include "system/system_manager.hpp"

BootResult BootManager::begin() {
  BootResult result;

  BoardIo::setEarlySafeState();
  ResetManager::begin();
  EventLogger::begin();
  EventLogger::log(EventId::BootBegin);
  EventLogger::log(EventId::ResetReason,
                   static_cast<int32_t>(ResetManager::reason()),
                   ResetManager::requiresRearm() ? 1 : 0, 0);
  BoardIo::configurePins();
  const bool motorReady = MotorController::begin();
  ButtonInterrupts::begin();
  StatusLed::begin();
  if (ButtonInterrupts::emergencyLatched()) {
    ButtonInterrupts::enterEmergencyStopLoop();
  }
  const bool sensorsReady = SensorManager::begin();
  if (ButtonInterrupts::emergencyLatched()) {
    ButtonInterrupts::enterEmergencyStopLoop();
  }
  RaspberryLink::begin();
  SystemManager::begin();
  SystemManager::setMotorHealthy(motorReady, FaultCode::Initialization);
  SystemManager::setSensorHealthy(sensorsReady, FaultCode::Initialization);

  result.state = motorReady && sensorsReady ? BootState::Ready
                                            : BootState::Failed;
  result.safeToRun = motorReady && sensorsReady &&
                     !ButtonInterrupts::emergencyLatched();
  result.faultFlags = SystemManager::status().faultFlags;
  EventLogger::log(result.safeToRun ? EventId::BootComplete
                                    : EventId::BootFailed);
  return result;
}

void BootManager::staySafe() {
  if (ButtonInterrupts::emergencyLatched()) {
    ButtonInterrupts::enterEmergencyStopLoop();
  }
  MotorController::emergencyStop();
  StatusLed::setBoth(LedOwner::System, LedState::Fault);
  EventLogger::log(EventId::BootFailed);
  EventLogger::process();
}
