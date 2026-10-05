#include "system/event_logger.hpp"

#include <string.h>

#include "pins.hpp"

namespace {

struct RateEntry {
  uint32_t key = 0;
  uint32_t lastMs = 0;
  bool used = false;
};

struct ChangeEntry {
  uint32_t key = 0;
  int32_t value = 0;
  bool used = false;
};

EventRecord queue[config::event_log::QUEUE_SIZE];
RateEntry rateEntries[config::event_log::TRACKER_SIZE];
ChangeEntry changeEntries[config::event_log::TRACKER_SIZE];
size_t readIndex = 0;
size_t writeIndex = 0;
size_t queueCount = 0;
size_t nextChangeSlot = 0;
bool initialized = false;
bool emergencyOnly = false;
bool stateKnown = false;
bool modeKnown = false;
uint16_t lastState = 0;
uint16_t lastMode = 0;
uint32_t writtenCount = 0;
uint32_t droppedCount = 0;
uint32_t suppressedCount = 0;

uint32_t mix(uint32_t hash, uint32_t value) {
  return (hash ^ value) * 16777619UL;
}

uint32_t makeKey(uint16_t number, LogSource source, int32_t value,
                 const char* text) {
  uint32_t hash = 2166136261UL;
  hash = mix(hash, number);
  hash = mix(hash, static_cast<uint8_t>(source));
  hash = mix(hash, static_cast<uint32_t>(value));
  if (text != nullptr) {
    while (*text != '\0') hash = mix(hash, static_cast<uint8_t>(*text++));
  }
  return hash;
}

bool repeatAllowed(uint32_t key, uint32_t nowMs, uint32_t intervalMs) {
  if (intervalMs == 0) return true;
  size_t freeSlot = config::event_log::TRACKER_SIZE;
  for (size_t i = 0; i < config::event_log::TRACKER_SIZE; ++i) {
    RateEntry& entry = rateEntries[i];
    if (entry.used && entry.key == key) {
      if (nowMs - entry.lastMs < intervalMs) {
        ++suppressedCount;
        return false;
      }
      entry.lastMs = nowMs;
      return true;
    }
    if (!entry.used && freeSlot == config::event_log::TRACKER_SIZE) {
      freeSlot = i;
    }
  }
  if (freeSlot == config::event_log::TRACKER_SIZE) {
    freeSlot = key % config::event_log::TRACKER_SIZE;
  }
  rateEntries[freeSlot].key = key;
  rateEntries[freeSlot].lastMs = nowMs;
  rateEntries[freeSlot].used = true;
  return true;
}

void copyText(char* destination, const char* source) {
  size_t index = 0;
  while (source != nullptr && *source != '\0' &&
         index + 1 < config::event_log::TEXT_LENGTH) {
    char value = *source++;
    if (value == '\r' || value == '\n') value = ' ';
    if (value == ',') value = ';';
    destination[index++] = value;
  }
  destination[index] = '\0';
}

bool enqueue(uint16_t number, LogSeverity severity, LogSource source,
             int32_t value1, int32_t value2, const char* text,
             uint32_t intervalMs) {
  if (emergencyOnly) {
    ++suppressedCount;
    return false;
  }
  const uint32_t nowMs = millis();
  if (!repeatAllowed(makeKey(number, source, value1, text), nowMs,
                     intervalMs)) {
    return false;
  }
  if (queueCount == config::event_log::QUEUE_SIZE) {
    ++droppedCount;
    return false;
  }

  EventRecord& record = queue[writeIndex];
  record.timestampMs = nowMs;
  record.eventNumber = number;
  record.severity = severity;
  record.source = source;
  record.value1 = value1;
  record.value2 = value2;
  copyText(record.text, text);
  writeIndex = (writeIndex + 1) % config::event_log::QUEUE_SIZE;
  ++queueCount;
  return true;
}

bool changeDetected(uint32_t key, int32_t value, int32_t& previous,
                    bool& known) {
  size_t freeSlot = config::event_log::TRACKER_SIZE;
  for (size_t i = 0; i < config::event_log::TRACKER_SIZE; ++i) {
    ChangeEntry& entry = changeEntries[i];
    if (entry.used && entry.key == key) {
      known = true;
      previous = entry.value;
      if (entry.value == value) {
        ++suppressedCount;
        return false;
      }
      entry.value = value;
      return true;
    }
    if (!entry.used && freeSlot == config::event_log::TRACKER_SIZE) {
      freeSlot = i;
    }
  }
  if (freeSlot == config::event_log::TRACKER_SIZE) {
    freeSlot = nextChangeSlot++ % config::event_log::TRACKER_SIZE;
  }
  changeEntries[freeSlot].key = key;
  changeEntries[freeSlot].value = value;
  changeEntries[freeSlot].used = true;
  known = false;
  previous = 0;
  return true;
}

LogSource sourceFor(EventId id) {
  const uint16_t number = static_cast<uint16_t>(id);
  if (number >= 600 && number < 700) return LogSource::Safety;
  if (number >= 500 && number < 600) return LogSource::Motor;
  if (number >= 400 && number < 500) return LogSource::Sensor;
  if (number >= 300 && number < 400) return LogSource::Communication;
  if (number >= 200 && number < 300) return LogSource::State;
  if (number >= 900 && number < 990) return LogSource::User;
  return LogSource::System;
}

LogSeverity severityFor(EventId id) {
  switch (id) {
    case EventId::BootFailed:
    case EventId::SystemFaultRaised:
    case EventId::MotorFaultRaised:
    case EventId::SensorFaultRaised:
      return LogSeverity::Error;
    case EventId::EmergencyStop:
      return LogSeverity::Fatal;
    case EventId::InvalidPacket:
    case EventId::SensorRangeFault:
    case EventId::MotorCommandRejected:
    case EventId::MotorWatchdogStop:
    case EventId::SafetyBlocked:
      return LogSeverity::Warning;
    default:
      return LogSeverity::Info;
  }
}

uint16_t faultEvent(LogSource source, bool active) {
  if (source == LogSource::Motor) {
    return static_cast<uint16_t>(active ? EventId::MotorFaultRaised
                                        : EventId::MotorFaultCleared);
  }
  if (source == LogSource::Sensor) {
    return static_cast<uint16_t>(active ? EventId::SensorFaultRaised
                                        : EventId::SensorFaultCleared);
  }
  return static_cast<uint16_t>(active ? EventId::SystemFaultRaised
                                      : EventId::SystemFaultCleared);
}

}  // namespace

void EventLogger::begin() {
  Serial.begin(config::uart::EVENT_LOG_BAUD, SERIAL_8N1, pins::UART0_RX,
               pins::UART0_TX);
  memset(queue, 0, sizeof(queue));
  memset(rateEntries, 0, sizeof(rateEntries));
  memset(changeEntries, 0, sizeof(changeEntries));
  readIndex = writeIndex = queueCount = nextChangeSlot = 0;
  emergencyOnly = stateKnown = modeKnown = false;
  writtenCount = droppedCount = suppressedCount = 0;
  initialized = true;
}

bool EventLogger::log(EventId id, int32_t value1, int32_t value2,
                      uint32_t minIntervalMs) {
  return enqueue(static_cast<uint16_t>(id), severityFor(id), sourceFor(id),
                 value1, value2, nullptr, minIntervalMs);
}

bool EventLogger::log(uint16_t number, LogSeverity severity, LogSource source,
                      int32_t value1, int32_t value2,
                      uint32_t minIntervalMs) {
  return enqueue(number, severity, source, value1, value2, nullptr,
                 minIntervalMs);
}

bool EventLogger::logText(uint16_t number, LogSeverity severity,
                          LogSource source, const char* text, int32_t value1,
                          int32_t value2, uint32_t minIntervalMs) {
  return enqueue(number, severity, source, value1, value2, text,
                 minIntervalMs);
}

bool EventLogger::stateChanged(uint16_t value) {
  if (stateKnown && lastState == value) {
    ++suppressedCount;
    return false;
  }
  const int32_t previous = stateKnown ? lastState : -1;
  lastState = value;
  stateKnown = true;
  return enqueue(static_cast<uint16_t>(EventId::StateChanged),
                 LogSeverity::Info, LogSource::State, previous, value,
                 nullptr, 0);
}

bool EventLogger::modeChanged(uint16_t value) {
  if (modeKnown && lastMode == value) {
    ++suppressedCount;
    return false;
  }
  const int32_t previous = modeKnown ? lastMode : -1;
  lastMode = value;
  modeKnown = true;
  return enqueue(static_cast<uint16_t>(EventId::ModeChanged),
                 LogSeverity::Info, LogSource::State, previous, value,
                 nullptr, 0);
}

bool EventLogger::valueChanged(EventId id, LogSource source, int32_t value,
                               int32_t detail) {
  uint32_t key = mix(static_cast<uint16_t>(id), static_cast<uint8_t>(source));
  key = mix(key, static_cast<uint32_t>(detail));
  int32_t previous;
  bool known;
  if (!changeDetected(key, value, previous, known)) return false;
  return enqueue(static_cast<uint16_t>(id), severityFor(id), source,
                 known ? previous : -1, value, nullptr, 0);
}

bool EventLogger::faultChanged(LogSource source, FaultCode fault, bool active,
                               int32_t detail) {
  uint32_t key = mix(0xFA017UL, static_cast<uint8_t>(source));
  key = mix(key, static_cast<uint16_t>(fault));
  key = mix(key, static_cast<uint32_t>(detail));
  int32_t previous;
  bool known;
  if (!changeDetected(key, active ? 1 : 0, previous, known)) return false;
  if (!known && !active) return false;
  return enqueue(faultEvent(source, active),
                 active ? LogSeverity::Error : LogSeverity::Info, source,
                 static_cast<int32_t>(fault), detail, nullptr, 0);
}

void EventLogger::process() {
  if (emergencyOnly) return;
  for (uint8_t count = 0;
       count < config::event_log::MAX_RECORDS_PER_PROCESS && queueCount > 0;
       ++count) {
    const EventRecord& record = queue[readIndex];
    char line[128];
    const int length = snprintf(line, sizeof(line), "EV,%lu,%u,%s,%s,%ld,%ld%s%s\r\n",
                  static_cast<unsigned long>(record.timestampMs),
                  static_cast<unsigned>(record.eventNumber),
                  severityName(record.severity), sourceName(record.source),
                  static_cast<long>(record.value1),
                  static_cast<long>(record.value2),
                  record.text[0] != '\0' ? "," : "", record.text);
    if (length < 0 || static_cast<size_t>(length) >= sizeof(line)) break;
    if (Serial.availableForWrite() < length) break;
    Serial.write(reinterpret_cast<const uint8_t*>(line), length);
    readIndex = (readIndex + 1) % config::event_log::QUEUE_SIZE;
    --queueCount;
    ++writtenCount;
  }
}

EventLoggerStatus EventLogger::status() {
  EventLoggerStatus result;
  result.initialized = initialized;
  result.emergencyOnly = emergencyOnly;
  result.queued = static_cast<uint16_t>(queueCount);
  result.written = writtenCount;
  result.dropped = droppedCount;
  result.suppressed = suppressedCount;
  return result;
}

const char* EventLogger::severityName(LogSeverity severity) {
  switch (severity) {
    case LogSeverity::Debug: return "D";
    case LogSeverity::Info: return "I";
    case LogSeverity::Warning: return "W";
    case LogSeverity::Error: return "E";
    case LogSeverity::Fatal: return "F";
    default: return "?";
  }
}

const char* EventLogger::sourceName(LogSource source) {
  switch (source) {
    case LogSource::System: return "SYS";
    case LogSource::State: return "STATE";
    case LogSource::Safety: return "SAFE";
    case LogSource::Motor: return "MOTOR";
    case LogSource::Sensor: return "SENSOR";
    case LogSource::Communication: return "COM";
    case LogSource::User: return "USER";
    default: return "?";
  }
}

void EventLogger::enterEmergencyMode() {
  emergencyOnly = true;
  readIndex = writeIndex = queueCount = 0;
}

void EventLogger::writeEmergencyStopNow() {
  Serial.println("ESTOP");
  Serial.flush();
}
