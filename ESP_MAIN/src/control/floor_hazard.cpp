#include "control/floor_hazard.hpp"

#include "control/control_config.hpp"

namespace {

uint32_t lastHazardMs[COLOR_SENSOR_COUNT] = {};
bool hazardSeen[COLOR_SENSOR_COUNT] = {};
FloorColor lastHazardColor[COLOR_SENSOR_COUNT] = {};
uint8_t firstMask = 0;
uint32_t firstSinceMs = 0;
FloorColor firstColor = FloorColor::Unknown;
uint32_t lastAnyMs = 0;

bool isHazardColor(const FloorColorResult& result) {
  if (!result.reliable || result.color == FloorColor::Unknown ||
      result.color == FloorColor::Black ||
      static_cast<uint8_t>(result.color) >= FLOOR_COLOR_COUNT) {
    return false;
  }
  const uint8_t bit = 1U << static_cast<uint8_t>(result.color);
  return (config::control::HAZARD_COLOR_MASK & bit) != 0;
}

// Measuring sensor with an invalid sample (dark or saturated contact).
bool contactBlind(const ColorRawSample& raw) { return raw.sequence != 0 && !raw.valid; }

HazardSide sideOf(uint8_t mask) {
  switch (mask) {
    case 0x00: return HazardSide::None;
    case 0x01: return HazardSide::FrontLeft;
    case 0x02: return HazardSide::FrontRight;
    case 0x03: return HazardSide::FrontBoth;
    case 0x04: return HazardSide::RearLeft;
    case 0x08: return HazardSide::RearRight;
    case 0x0C: return HazardSide::RearBoth;
    case 0x05: return HazardSide::Left;
    case 0x0A: return HazardSide::Right;
    default: return HazardSide::Unclear;   // diagonal or 3+ corners
  }
}

FloorColor colorOf(uint8_t mask) {
  for (uint8_t i = 0; i < COLOR_SENSOR_COUNT; ++i) {
    if ((mask & (1U << i)) != 0) return lastHazardColor[i];
  }
  return FloorColor::Unknown;
}

}  // namespace

void FloorHazard::reset() {
  for (bool& seen : hazardSeen) seen = false;
  firstMask = 0;
  firstColor = FloorColor::Unknown;
  lastAnyMs = 0;
}

FloorHazardInfo FloorHazard::update(uint32_t nowMs, const SensorSnapshot& sensors) {
  FloorHazardInfo info;
  for (uint8_t i = 0; i < COLOR_SENSOR_COUNT; ++i) {
    if (isHazardColor(sensors.floorColor[i])) {
      info.mask |= 1U << i;
      lastHazardMs[i] = nowMs;
      lastHazardColor[i] = sensors.floorColor[i].color;
      hazardSeen[i] = true;
    } else if (hazardSeen[i] && contactBlind(sensors.colorRaw[i]) &&
               nowMs - lastHazardMs[i] < config::control::HAZARD_CONTACT_HOLD_MS) {
      info.mask |= 1U << i;   // was a hazard a moment ago, now blinded by contact
    }
  }
  if (info.mask != 0) {
    if (firstMask == 0) {
      firstMask = info.mask;
      firstSinceMs = nowMs;
      firstColor = colorOf(info.mask);
    } else if (nowMs - firstSinceMs < config::floor_hazard::FIRST_WINDOW_MS) {
      firstMask |= info.mask;   // the other sensor of the same side, a pass later
    }
    lastAnyMs = nowMs;
  } else if (firstMask != 0 && nowMs - lastAnyMs >= config::floor_hazard::FIRST_CLEAR_MS) {
    firstMask = 0;
    firstColor = FloorColor::Unknown;
  }
  info.first = sideOf(firstMask);
  info.firstColor = firstColor;
  return info;
}
