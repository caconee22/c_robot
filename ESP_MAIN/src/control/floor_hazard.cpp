#include "control/floor_hazard.hpp"

#include "control/control_config.hpp"

namespace {

uint32_t lastHazardMs[COLOR_SENSOR_COUNT] = {};
bool hazardSeen[COLOR_SENSOR_COUNT] = {};
FloorColor lastHazardColor[COLOR_SENSOR_COUNT] = {};
HazardSide firstSide = HazardSide::None;
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
  const bool fl = (mask & 0x01U) != 0, fr = (mask & 0x02U) != 0;
  if (fl && fr) return HazardSide::FrontBoth;
  if (fl) return HazardSide::FrontLeft;
  if (fr) return HazardSide::FrontRight;
  return (mask & 0x0CU) != 0 ? HazardSide::Rear : HazardSide::None;
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
  firstSide = HazardSide::None;
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
    if (firstSide == HazardSide::None) {
      firstSide = sideOf(info.mask);
      firstColor = colorOf(info.mask);
    }
    lastAnyMs = nowMs;
  } else if (firstSide != HazardSide::None &&
             nowMs - lastAnyMs >= config::floor_hazard::FIRST_CLEAR_MS) {
    firstSide = HazardSide::None;
    firstColor = FloorColor::Unknown;
  }
  info.first = firstSide;
  info.firstColor = firstColor;
  return info;
}
