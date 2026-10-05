#include "system/status_led.hpp"

#include <Adafruit_NeoPixel.h>
#include <freertos/FreeRTOS.h>

#include "config.hpp"
#include "pins.hpp"

namespace {

constexpr size_t LED_COUNT = 1;
constexpr size_t TARGET_COUNT = 2;
constexpr size_t OWNER_COUNT = static_cast<size_t>(LedOwner::Count);

constexpr size_t INTERNAL_INDEX = 0;
constexpr size_t EXTERNAL_INDEX = 1;

struct RgbColor {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
};

struct LedStyle {
  RgbColor primary;
  RgbColor secondary;
  uint16_t phaseMs;
};

struct LedRequest {
  LedState state = LedState::Off;
  uint32_t expiresMs = 0;
  uint32_t sequence = 0;
  bool active = false;
  bool expiring = false;
};

struct ResolvedLed {
  LedState state = LedState::Off;
  uint8_t priority = 0;
  uint32_t sequence = 0;
};

Adafruit_NeoPixel internalPixel(
    LED_COUNT, static_cast<uint16_t>(pins::RGB_LED_INTERNAL),
    NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel externalPixel(
    LED_COUNT, static_cast<uint16_t>(pins::RGB_LED_EXTERNAL),
    NEO_GRB + NEO_KHZ800);

portMUX_TYPE ledMux = portMUX_INITIALIZER_UNLOCKED;
LedRequest requests[TARGET_COUNT][OWNER_COUNT];
StatusLedSnapshot latestStatus;
uint32_t nextSequence = 1;
uint32_t lastPhysicalUpdateMs = 0;
uint32_t lastInternalColor = UINT32_MAX;
uint32_t lastExternalColor = UINT32_MAX;

size_t ownerIndex(LedOwner owner) {
  return static_cast<size_t>(owner);
}

bool validOwner(LedOwner owner) { return ownerIndex(owner) < OWNER_COUNT; }

bool validTarget(LedTarget target) {
  return target == LedTarget::Internal || target == LedTarget::External ||
         target == LedTarget::Both;
}

bool deadlineActive(uint32_t nowMs, const LedRequest& request) {
  return !request.expiring ||
         static_cast<int32_t>(nowMs - request.expiresMs) < 0;
}

LedStyle styleFor(LedState state) {
  constexpr RgbColor OFF{0, 0, 0};
  switch (state) {
    case LedState::Booting:
      return {{0, 0, 255}, OFF, 500};
    case LedState::WaitingForStart:
      return {{255, 80, 0}, OFF, 700};
    case LedState::Ready:
      return {{0, 80, 255}, {0, 80, 255}, 0};
    case LedState::Searching:
      return {{255, 0, 0}, OFF, 400};
    case LedState::Tracking:
      return {{0, 255, 0}, {0, 255, 0}, 0};
    case LedState::Aligning:
      return {{255, 180, 0}, {255, 180, 0}, 0};
    case LedState::Approaching:
      return {{0, 255, 180}, {0, 255, 180}, 0};
    case LedState::Charging:
      return {{255, 255, 255}, OFF, 100};
    case LedState::Pushing:
      return {{0, 40, 255}, {0, 40, 255}, 0};
    case LedState::WallEscape:
      return {{255, 0, 255}, OFF, 150};
    case LedState::ZoneEscape:
      return {{255, 70, 0}, OFF, 120};
    case LedState::CommunicationLost:
      return {{150, 0, 255}, OFF, 250};
    case LedState::SensorFault:
      return {{255, 180, 0}, OFF, 200};
    case LedState::MotorFault:
      return {{255, 0, 0}, {255, 60, 0}, 150};
    case LedState::Fault:
      return {{255, 0, 0}, {255, 255, 255}, 150};
    case LedState::EmergencyStop:
      return {{255, 0, 0}, {255, 0, 0}, 0};
    case LedState::Off:
    default:
      return {OFF, OFF, 0};
  }
}

uint8_t scaleChannel(uint8_t channel, uint8_t brightness) {
  const uint32_t scaled =
      (static_cast<uint32_t>(channel) *
           static_cast<uint32_t>(brightness) +
       127U) /
      255U;
  return static_cast<uint8_t>(scaled);
}

RgbColor displayedColor(LedState state, uint32_t nowMs) {
  const LedStyle style = styleFor(state);
  if (style.phaseMs == 0) {
    return style.primary;
  }
  return ((nowMs / style.phaseMs) & 1U) == 0 ? style.primary
                                              : style.secondary;
}

ResolvedLed resolveLocked(size_t targetIndex, uint32_t nowMs) {
  ResolvedLed resolved;
  for (size_t owner = 0; owner < OWNER_COUNT; ++owner) {
    LedRequest& request = requests[targetIndex][owner];
    if (request.active && !deadlineActive(nowMs, request)) {
      request.active = false;
    }
    if (!request.active) {
      continue;
    }

    const uint8_t requestPriority = StatusLed::priority(request.state);
    if (requestPriority > resolved.priority ||
        (requestPriority == resolved.priority &&
         static_cast<int32_t>(request.sequence - resolved.sequence) > 0)) {
      resolved.state = request.state;
      resolved.priority = requestPriority;
      resolved.sequence = request.sequence;
    }
  }
  return resolved;
}

void setRequestLocked(size_t targetIndex, LedOwner owner, LedState state,
                      uint32_t nowMs, uint32_t ttlMs) {
  LedRequest& request = requests[targetIndex][ownerIndex(owner)];
  request.state = state;
  request.expiresMs = ttlMs == 0 ? 0 : nowMs + ttlMs;
  request.expiring = ttlMs != 0;
  request.sequence = nextSequence++;
  request.active = true;
}

void clearRequestLocked(size_t targetIndex, LedOwner owner) {
  requests[targetIndex][ownerIndex(owner)].active = false;
}

void writePixel(Adafruit_NeoPixel& pixel, uint32_t& previousColor,
                const RgbColor& logicalColor, uint8_t brightness,
                LedUnitStatus& unitStatus) {
  const RgbColor output{
      scaleChannel(logicalColor.red, brightness),
      scaleChannel(logicalColor.green, brightness),
      scaleChannel(logicalColor.blue, brightness),
  };
  const uint32_t packed = pixel.Color(output.red, output.green, output.blue);
  if (packed != previousColor) {
    pixel.setPixelColor(0, packed);
    pixel.show();
    previousColor = packed;
  }
  unitStatus.red = output.red;
  unitStatus.green = output.green;
  unitStatus.blue = output.blue;
  unitStatus.outputOn = packed != 0;
}

}  // namespace

void StatusLed::begin() {
  internalPixel.begin();
  externalPixel.begin();
  internalPixel.clear();
  externalPixel.clear();
  internalPixel.show();
  externalPixel.show();

  portENTER_CRITICAL(&ledMux);
  for (size_t target = 0; target < TARGET_COUNT; ++target) {
    for (size_t owner = 0; owner < OWNER_COUNT; ++owner) {
      requests[target][owner] = LedRequest{};
    }
  }
  latestStatus = StatusLedSnapshot{};
  latestStatus.initialized = true;
  nextSequence = 1;
  portEXIT_CRITICAL(&ledMux);

  lastInternalColor = UINT32_MAX;
  lastExternalColor = UINT32_MAX;
  const uint32_t nowMs = millis();
  lastPhysicalUpdateMs = nowMs - config::led::UPDATE_PERIOD_MS;
  setBoth(LedOwner::System, LedState::Booting);
  update(nowMs);
}

bool StatusLed::request(LedOwner owner, LedTarget target, LedState state,
                        uint32_t ttlMs) {
  if (!validOwner(owner) || !validTarget(target)) {
    return false;
  }

  const uint32_t nowMs = millis();
  portENTER_CRITICAL(&ledMux);
  if (target == LedTarget::Internal || target == LedTarget::Both) {
    setRequestLocked(INTERNAL_INDEX, owner, state, nowMs, ttlMs);
  }
  if (target == LedTarget::External || target == LedTarget::Both) {
    setRequestLocked(EXTERNAL_INDEX, owner, state, nowMs, ttlMs);
  }
  portEXIT_CRITICAL(&ledMux);
  return true;
}

void StatusLed::clear(LedOwner owner, LedTarget target) {
  if (!validOwner(owner) || !validTarget(target)) {
    return;
  }

  portENTER_CRITICAL(&ledMux);
  if (target == LedTarget::Internal || target == LedTarget::Both) {
    clearRequestLocked(INTERNAL_INDEX, owner);
  }
  if (target == LedTarget::External || target == LedTarget::Both) {
    clearRequestLocked(EXTERNAL_INDEX, owner);
  }
  portEXIT_CRITICAL(&ledMux);
}

void StatusLed::setInternal(LedOwner owner, LedState state, uint32_t ttlMs) {
  request(owner, LedTarget::Internal, state, ttlMs);
}

void StatusLed::setExternal(LedOwner owner, LedState state, uint32_t ttlMs) {
  request(owner, LedTarget::External, state, ttlMs);
}

void StatusLed::setBoth(LedOwner owner, LedState state, uint32_t ttlMs) {
  request(owner, LedTarget::Both, state, ttlMs);
}

void StatusLed::setEmergencyStop() {
  setBoth(LedOwner::Safety, LedState::EmergencyStop);
}

void StatusLed::update(uint32_t nowMs) {
  if (!latestStatus.initialized ||
      nowMs - lastPhysicalUpdateMs < config::led::UPDATE_PERIOD_MS) {
    return;
  }
  lastPhysicalUpdateMs = nowMs;

  portENTER_CRITICAL(&ledMux);
  const ResolvedLed internal = resolveLocked(INTERNAL_INDEX, nowMs);
  const ResolvedLed external = resolveLocked(EXTERNAL_INDEX, nowMs);
  portEXIT_CRITICAL(&ledMux);

  LedUnitStatus internalStatus;
  internalStatus.state = internal.state;
  internalStatus.priority = internal.priority;
  LedUnitStatus externalStatus;
  externalStatus.state = external.state;
  externalStatus.priority = external.priority;

  const RgbColor internalColor = displayedColor(internal.state, nowMs);
  const RgbColor externalColor = displayedColor(external.state, nowMs);
  writePixel(internalPixel, lastInternalColor, internalColor,
             config::led::INTERNAL_BRIGHTNESS, internalStatus);
  writePixel(externalPixel, lastExternalColor, externalColor,
             config::led::EXTERNAL_BRIGHTNESS, externalStatus);

  portENTER_CRITICAL(&ledMux);
  latestStatus.internal = internalStatus;
  latestStatus.external = externalStatus;
  latestStatus.lastUpdateMs = nowMs;
  portEXIT_CRITICAL(&ledMux);
}

LedState StatusLed::state(LedTarget target) {
  portENTER_CRITICAL(&ledMux);
  const LedUnitStatus internal = latestStatus.internal;
  const LedUnitStatus external = latestStatus.external;
  portEXIT_CRITICAL(&ledMux);

  if (target == LedTarget::Internal) {
    return internal.state;
  }
  if (target == LedTarget::External) {
    return external.state;
  }
  if (external.priority > internal.priority) {
    return external.state;
  }
  return internal.state;
}

uint8_t StatusLed::priority(LedState state) {
  switch (state) {
    case LedState::EmergencyStop:
      return 255;
    case LedState::Fault:
      return 240;
    case LedState::MotorFault:
      return 230;
    case LedState::SensorFault:
      return 220;
    case LedState::ZoneEscape:
      return 200;
    case LedState::WallEscape:
      return 190;
    case LedState::CommunicationLost:
      return 180;
    case LedState::Charging:
      return 150;
    case LedState::Pushing:
      return 140;
    case LedState::Aligning:
      return 130;
    case LedState::Tracking:
      return 120;
    case LedState::Approaching:
      return 110;
    case LedState::Searching:
      return 100;
    case LedState::Ready:
      return 70;
    case LedState::WaitingForStart:
      return 50;
    case LedState::Booting:
      return 40;
    case LedState::Off:
    default:
      return 0;
  }
}

const char* StatusLed::stateName(LedState state) {
  switch (state) {
    case LedState::Off:
      return "off";
    case LedState::Booting:
      return "booting";
    case LedState::WaitingForStart:
      return "waiting_for_start";
    case LedState::Ready:
      return "ready";
    case LedState::Searching:
      return "searching";
    case LedState::Tracking:
      return "tracking";
    case LedState::Aligning:
      return "aligning";
    case LedState::Approaching:
      return "approaching";
    case LedState::Charging:
      return "charging";
    case LedState::Pushing:
      return "pushing";
    case LedState::WallEscape:
      return "wall_escape";
    case LedState::ZoneEscape:
      return "zone_escape";
    case LedState::CommunicationLost:
      return "communication_lost";
    case LedState::SensorFault:
      return "sensor_fault";
    case LedState::MotorFault:
      return "motor_fault";
    case LedState::Fault:
      return "fault";
    case LedState::EmergencyStop:
      return "emergency_stop";
    default:
      return "unknown";
  }
}

StatusLedSnapshot StatusLed::status() {
  portENTER_CRITICAL(&ledMux);
  const StatusLedSnapshot snapshot = latestStatus;
  portEXIT_CRITICAL(&ledMux);
  return snapshot;
}
