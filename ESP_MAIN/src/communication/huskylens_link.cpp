#include "communication/huskylens_link.hpp"

#include "communication/vision_protocol.hpp"
#include "pins.hpp"
#include "system/event_logger.hpp"

namespace {
using namespace config::huskylens;

constexpr uint8_t HEADER_1 = 0x55;
constexpr uint8_t HEADER_2 = 0xAA;
constexpr uint8_t ADDRESS = 0x11;
constexpr uint8_t CMD_REQUEST = 0x20;   // blocks and arrows, like huskylens.request()
constexpr uint8_t CMD_REQUEST_KNOCK = 0x2C;
constexpr uint8_t CMD_RETURN_INFO = 0x29;
constexpr uint8_t CMD_RETURN_BLOCK = 0x2A;
constexpr uint8_t CMD_RETURN_ARROW = 0x2B;
constexpr uint8_t CMD_RETURN_OK = 0x2E;
constexpr uint8_t CMD_RETURN_BUSY = 0x3D;   // a command came before the last one was answered
// Knock search order (HuskyLens Protocol Type: Serial 9600 / 115200 / 1000000).
constexpr uint32_t BAUDS[] = {BAUD, BAUD_OTHER, BAUD_FAST};
constexpr uint8_t BAUD_COUNT = sizeof(BAUDS) / sizeof(BAUDS[0]);
constexpr uint8_t STATUS_OK =
    vision_protocol::CameraOk | vision_protocol::PipelineOk;

// Receive state: one frame at a time (header, address, length, command, data, checksum).
uint8_t frame[5 + MAX_DATA_LENGTH + 1] = {};
size_t frameCount = 0;

// Request state. linkUp = the HuskyLens answered the knock (example begin()).
bool linkUp = false;
bool waiting = false;          // a request is out, answer not complete
uint32_t requestMs = 0;
uint32_t lastKnockMs = 0;
uint8_t knocksAtBaud = 0;      // unanswered knocks at the current baud
uint32_t lastRxMs = 0;         // last byte from the HuskyLens (reply timeout restarts here)
uint32_t busyReplies = 0;      // 0x3D answers (diagnostic)

// Answer being collected: blocks announced by the info frame.
uint16_t blocksExpected = 0;
uint16_t blocksSeen = 0;
uint16_t answerFrame = 0;
bool bestFound = false;
uint8_t matches = 0;
uint16_t bestX = 0, bestY = 0, bestW = 0, bestH = 0;

bool frameKnown = false;
uint16_t lastFrameNumber = 0;
VisionSnapshot latestVision;
RaspberryLinkStatus linkStatus;

// Wiring/baud probe (bench HUSKY PROBE, 11/10): every UART setting in turn,
// knock every 100 ms, count raw bytes and good frames.
struct ProbeConfig { uint32_t baud; bool swapped; };
constexpr ProbeConfig PROBE_CONFIGS[] = {{115200, false}, {115200, true}, {9600, false},
                                         {9600, true}, {1000000, false}, {1000000, true}};
static_assert(sizeof(PROBE_CONFIGS) / sizeof(PROBE_CONFIGS[0]) == PROBE_STEPS,
              "bench prints PROBE_STEPS probe lines");
constexpr uint8_t PROBE_COUNT = sizeof(PROBE_CONFIGS) / sizeof(PROBE_CONFIGS[0]);
constexpr uint32_t PROBE_STEP_MS = 600;
constexpr uint32_t PROBE_KNOCK_MS = 100;
bool probeActive = false;
bool linkOff = false;      // bench HUSKY OFF: UART closed, GPIO17/18 released
bool probeReported = true;
uint8_t probeIndex = 0;
uint32_t probeStepMs = 0, probeKnockMs = 0;
HuskyProbeResult probeResults[PROBE_COUNT];
uint32_t rawBytes = 0;     // every byte read since begin (diagnostic)
uint32_t goodFrames = 0;   // frames with a correct checksum (any command)
uint32_t uartBaud = BAUD;
bool uartSwapped = false;

void openUart(uint32_t baud, bool swapped) {
  Serial1.end();
  Serial1.begin(baud, SERIAL_8N1, swapped ? pins::UART1_TX : pins::UART1_RX,
                swapped ? pins::UART1_RX : pins::UART1_TX);
  while (Serial1.available() > 0) Serial1.read();
  uartBaud = baud;
  uartSwapped = swapped;
}

uint16_t le16(const uint8_t* at) {
  return static_cast<uint16_t>(at[0] | (static_cast<uint16_t>(at[1]) << 8));
}

bool sendFrame(uint8_t command, const uint8_t* data, uint8_t length) {
  uint8_t out[6 + 2] = {HEADER_1, HEADER_2, ADDRESS, length, command};
  const size_t size = 5u + length + 1u;
  if (length > 2 || Serial1.availableForWrite() < static_cast<int>(size)) return false;
  uint8_t sum = 0;
  for (uint8_t i = 0; i < length; ++i) out[5 + i] = data[i];
  for (size_t i = 0; i + 1 < size; ++i) sum = static_cast<uint8_t>(sum + out[i]);
  out[size - 1] = sum;
  Serial1.write(out, size);
  return true;
}

uint16_t scaled(uint16_t value, uint16_t source, uint16_t target) {
  const uint32_t v = static_cast<uint32_t>(value) * target / source;
  return static_cast<uint16_t>(v >= target ? target - 1 : v);
}

// The whole answer is in: publish it the way the Pi link does.
void finishAnswer(uint32_t nowMs) {
  waiting = false;
  ++linkStatus.validPackets;
  linkStatus.lastPacketMs = nowMs;
  latestVision.receivedMs = nowMs;
  if (frameKnown && answerFrame == lastFrameNumber) {
    ++linkStatus.duplicatePackets;   // same camera frame: link alive, not fresh
    return;
  }
  frameKnown = true;
  lastFrameNumber = answerFrame;
  linkStatus.lastSequence = static_cast<uint8_t>(answerFrame);
  linkStatus.lastFrameMs = nowMs;

  VisionSnapshot v;
  v.receivedMs = nowMs;
  v.frameUpdatedMs = nowMs;
  v.sequence = static_cast<uint8_t>(answerFrame);
  v.cameraOk = v.pipelineOk = true;
  uint8_t bits = STATUS_OK;
  if (bestFound) {
    v.targetValid = v.trackStable = true;
    v.targetX = scaled(bestX, SOURCE_WIDTH, vision_protocol::FRAME_WIDTH);
    v.targetY = scaled(bestY, SOURCE_HEIGHT, vision_protocol::FRAME_HEIGHT);
    v.targetWidth = scaled(bestW, SOURCE_WIDTH, vision_protocol::FRAME_WIDTH);
    v.targetHeight = scaled(bestH, SOURCE_HEIGHT, vision_protocol::FRAME_HEIGHT);
    if (v.targetWidth == 0) v.targetWidth = 1;
    if (v.targetHeight == 0) v.targetHeight = 1;
    v.multipleTargets = matches > 1;
    // Box touching the image edge (center +- half size outside the frame).
    v.boxClipped = bestX < bestW / 2 + 1 || bestY < bestH / 2 + 1 ||
                   bestX + bestW / 2 + 1 >= SOURCE_WIDTH || bestY + bestH / 2 + 1 >= SOURCE_HEIGHT;
    bits |= vision_protocol::TargetValid | vision_protocol::TrackStable;
    if (v.multipleTargets) bits |= vision_protocol::MultipleTargets;
    if (v.boxClipped) bits |= vision_protocol::BoxClipped;
  }
  v.statusBits = bits;
  latestVision = v;
}

void handleFrame(uint32_t nowMs) {
  const uint8_t length = frame[3];
  const uint8_t command = frame[4];
  const uint8_t* data = frame + 5;
  if (command == CMD_RETURN_OK) {   // knock answered: linkUp at this baud
    linkUp = true;
    knocksAtBaud = 0;
    waiting = false;
    linkStatus.lastPacketMs = nowMs;
    return;
  }
  if (command == CMD_RETURN_INFO && length >= 6) {
    linkUp = true;   // an answer at all means the baud is right
    blocksExpected = le16(data);
    blocksSeen = 0;
    answerFrame = le16(data + 4);
    bestFound = false;
    matches = 0;
    if (blocksExpected == 0) finishAnswer(nowMs);
    return;
  }
  if (command == CMD_RETURN_BLOCK && length >= 10) {
    if (blocksSeen >= blocksExpected) return;   // block without info: ignore
    ++blocksSeen;
    const uint16_t x = le16(data), y = le16(data + 2), w = le16(data + 4), h = le16(data + 6);
    const uint16_t id = le16(data + 8);
    // ID 0 = seen but not learned (protocol doc): never the tower.
    const bool wanted = id != 0 && (TARGET_ID == 0 || id == TARGET_ID);
    if (wanted && w > 0 && h > 0 && x < SOURCE_WIDTH && y < SOURCE_HEIGHT) {
      ++matches;
      if (!bestFound || static_cast<uint32_t>(w) * h > static_cast<uint32_t>(bestW) * bestH) {
        bestFound = true;
        bestX = x; bestY = y; bestW = w; bestH = h;
      }
    }
    if (blocksSeen >= blocksExpected) finishAnswer(nowMs);
    return;
  }
  if (command == CMD_RETURN_ARROW) {   // line tracking result: counted, not used
    if (blocksSeen < blocksExpected && ++blocksSeen >= blocksExpected) finishAnswer(nowMs);
    return;
  }
  if (command == CMD_RETURN_BUSY) {
    // The HuskyLens was still on the previous command: stop waiting and ask
    // again after REQUEST_PERIOD_MS instead of sitting out the timeout.
    waiting = false;
    blocksExpected = blocksSeen = 0;
    linkStatus.lastPacketMs = nowMs;
    ++busyReplies;
  }
}

void consume(uint8_t value, uint32_t nowMs) {
  if (frameCount == 0 && value != HEADER_1) return;
  if (frameCount == 1 && value != HEADER_2) {
    frameCount = value == HEADER_1 ? 1 : 0;
    if (frameCount) frame[0] = value;
    return;
  }
  frame[frameCount++] = value;
  if (frameCount == 3 && value != ADDRESS) { frameCount = 0; ++linkStatus.invalidPackets; return; }
  if (frameCount == 4 && value > MAX_DATA_LENGTH) { frameCount = 0; ++linkStatus.invalidPackets; return; }
  if (frameCount < 5 || frameCount < 5u + frame[3] + 1u) return;
  uint8_t sum = 0;
  for (size_t i = 0; i + 1 < frameCount; ++i) sum = static_cast<uint8_t>(sum + frame[i]);
  const bool ok = sum == frame[frameCount - 1];
  frameCount = 0;
  if (ok) ++goodFrames;
  if (!ok) {
    ++linkStatus.invalidPackets;
    EventLogger::log(EventId::InvalidPacket, 100, frame[4]);   // 100 = HuskyLens checksum
    return;
  }
  handleFrame(nowMs);
}

void request(uint32_t nowMs) {
  if (waiting) {
    // Like the library's 100 ms per reply: the wait restarts with every byte,
    // so a long 9600-baud answer (info + many blocks) is never cut off.
    const uint32_t since = static_cast<int32_t>(lastRxMs - requestMs) > 0 ? lastRxMs : requestMs;
    if (nowMs - since < RESPONSE_TIMEOUT_MS) return;
    waiting = false;            // no full answer: count it and ask again
    if (linkUp) ++linkStatus.invalidPackets;
    frameCount = 0;
  }
  if (!linkUp) {
    // begin(): knock until OK; after BAUD_SWITCH_KNOCKS silent knocks try the next baud.
    if (lastKnockMs != 0 && nowMs - lastKnockMs < KNOCK_RETRY_MS) return;
    if (knocksAtBaud >= BAUD_SWITCH_KNOCKS) {
      uint8_t next = 0;
      for (uint8_t i = 0; i < BAUD_COUNT; ++i)
        if (BAUDS[i] == uartBaud) next = static_cast<uint8_t>((i + 1) % BAUD_COUNT);
      openUart(BAUDS[next], uartSwapped);   // keeps a swap found by HUSKY PROBE
      knocksAtBaud = 0;
    }
    if (sendFrame(CMD_REQUEST_KNOCK, nullptr, 0)) {
      ++knocksAtBaud;
      lastKnockMs = nowMs == 0 ? 1 : nowMs;
      waiting = true;
      requestMs = nowMs;
    }
    return;
  }
  if (nowMs - requestMs < REQUEST_PERIOD_MS && linkStatus.validPackets > 0) return;
  if (sendFrame(CMD_REQUEST, nullptr, 0)) {
    waiting = true;
    requestMs = nowMs;
  }
}



void updateAge(uint32_t nowMs, VisionSnapshot& v, RaspberryLinkStatus* status = nullptr) {
  const bool linkValid = linkStatus.validPackets > 0 && nowMs - linkStatus.lastPacketMs <= LINK_TIMEOUT_MS;
  const bool fresh = frameKnown && nowMs - linkStatus.lastFrameMs <= FRAME_STALE_MS;
  v.linkValid = linkValid;
  v.frameFresh = fresh;
  if (!linkValid || !fresh) {
    v.targetValid = false;
    v.trackStable = false;
  }
  if (status != nullptr) {
    status->linkValid = linkValid;
    status->frameFresh = fresh;
  }
}
}  // namespace

void HuskyLensLink::begin() {
  Serial1.begin(BAUD, SERIAL_8N1, pins::UART1_RX, pins::UART1_TX);
  uartBaud = BAUD;
  uartSwapped = false;
  probeActive = false;
  probeReported = true;
  rawBytes = goodFrames = 0;
  latestVision = VisionSnapshot{};
  linkStatus = RaspberryLinkStatus{};
  linkStatus.initialized = true;
  frameCount = 0;
  waiting = false;
  requestMs = 0;
  linkUp = false;
  lastKnockMs = 0;
  knocksAtBaud = 0;
  frameKnown = false;
  lastRxMs = 0;
  busyReplies = 0;
  blocksExpected = blocksSeen = 0;
}

void HuskyLensLink::poll() {
  const uint32_t nowMs = millis();
  if (linkOff) return;
  if (probeActive) {
    while (Serial1.available() > 0) {
      const int value = Serial1.read();
      if (value < 0) break;
      ++rawBytes;
      lastRxMs = nowMs;
      consume(static_cast<uint8_t>(value), nowMs);
    }
    HuskyProbeResult& r = probeResults[probeIndex];
    r.bytes = rawBytes;
    r.frames = goodFrames;
    if (nowMs - probeKnockMs >= PROBE_KNOCK_MS) {
      probeKnockMs = nowMs;
      sendFrame(CMD_REQUEST_KNOCK, nullptr, 0);
    }
    if (nowMs - probeStepMs < PROBE_STEP_MS) return;
    if (++probeIndex < PROBE_COUNT) {
      openUart(PROBE_CONFIGS[probeIndex].baud, PROBE_CONFIGS[probeIndex].swapped);
      probeResults[probeIndex] = {PROBE_CONFIGS[probeIndex].baud, PROBE_CONFIGS[probeIndex].swapped, 0, 0};
      rawBytes = goodFrames = 0;
      probeStepMs = probeKnockMs = nowMs;
      return;
    }
    // Done: keep the setting that answered best (until reboot), else the default.
    uint8_t best = 0;
    for (uint8_t i = 1; i < PROBE_COUNT; ++i)
      if (probeResults[i].frames > probeResults[best].frames) best = i;
    if (probeResults[best].frames == 0) best = 0;
    openUart(PROBE_CONFIGS[best].baud, PROBE_CONFIGS[best].swapped);
    probeActive = false;
    probeReported = false;
    frameCount = 0;
    waiting = false;
    linkUp = false;   // knock again at the chosen setting
    lastKnockMs = 0;
    knocksAtBaud = 0;
    return;
  }
  // A HuskyLens that went quiet (unplugged, rebooted): knock again, like begin().
  if (linkUp && nowMs - linkStatus.lastPacketMs > RECONNECT_MS) {
    linkUp = false;
    lastKnockMs = 0;
    knocksAtBaud = 0;
  }
  size_t processed = 0;
  while (processed < MAX_BYTES_PER_POLL && Serial1.available() > 0) {
    const int value = Serial1.read();
    if (value < 0) break;
    ++rawBytes;
    lastRxMs = millis();
    consume(static_cast<uint8_t>(value), millis());
    ++processed;
  }
  request(millis());
  const VisionSnapshot v = latest();
  EventLogger::valueChanged(EventId::VisionStatusChanged, LogSource::Communication,
                            (v.linkValid ? 1 : 0) | (v.frameFresh ? 2 : 0));
}

VisionSnapshot HuskyLensLink::latest() {
  VisionSnapshot result = latestVision;
  updateAge(millis(), result);
  return result;
}

RaspberryLinkStatus HuskyLensLink::status() {
  RaspberryLinkStatus result = linkStatus;
  VisionSnapshot v = latestVision;
  updateAge(millis(), v, &result);
  return result;
}

void HuskyLensLink::startProbe() {
  linkOff = false;
  probeActive = true;
  probeIndex = 0;
  probeResults[0] = {PROBE_CONFIGS[0].baud, PROBE_CONFIGS[0].swapped, 0, 0};
  openUart(PROBE_CONFIGS[0].baud, PROBE_CONFIGS[0].swapped);
  rawBytes = goodFrames = 0;
  frameCount = 0;
  probeStepMs = probeKnockMs = millis();
}

bool HuskyLensLink::probing() { return probeActive; }

uint8_t HuskyLensLink::takeProbeResults(HuskyProbeResult* out, uint8_t max) {
  if (probeActive || probeReported) return 0;
  probeReported = true;
  uint8_t n = 0;
  for (; n < PROBE_COUNT && n < max; ++n) out[n] = probeResults[n];
  return n;
}

uint32_t HuskyLensLink::rawBytesReceived() { return rawBytes; }
uint32_t HuskyLensLink::activeBaud() { return uartBaud; }
bool HuskyLensLink::activeSwapped() { return uartSwapped; }

void HuskyLensLink::setOff(bool off) {
  if (off == linkOff) return;
  linkOff = off;
  probeActive = false;
  frameCount = 0;
  waiting = false;
  if (off) {
    // Stop talking to the HuskyLens: close the UART and leave both pins as
    // plain inputs so nothing drives the HuskyLens RX line.
    Serial1.end();
    pinMode(pins::UART1_TX, INPUT);
    pinMode(pins::UART1_RX, INPUT);
  } else {
    openUart(BAUD, false);
    linkUp = false;
    lastKnockMs = 0;
    knocksAtBaud = 0;
  }
}

bool HuskyLensLink::off() { return linkOff; }

bool HuskyLensLink::connected() { return linkUp; }
uint32_t HuskyLensLink::busyCount() { return busyReplies; }
