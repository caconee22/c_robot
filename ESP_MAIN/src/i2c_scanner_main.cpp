#include <Arduino.h>
#include <Wire.h>
#include <driver/gpio.h>
#include <soc/gpio_struct.h>
#include "hardware/board_io.hpp"
#include "pins.hpp"

// Contract: standalone i2c_scanner_v2 entry point, excluded from match/bench.
// Probe one address per loop on the existing V2 I2C pins. Test main bus and
// each channel of responding 0x70..0x77 mux candidates. No drive commands,
// retries, sensor setup, or pin remapping. Motor outputs stay LOW throughout.
namespace {
bool muxFound[8] = {};
uint8_t address = 1, muxIndex = 0, channel = 0, found = 0;
uint8_t consecutiveTimeouts = 0;
enum class Phase { Main, Select, Channel, Wait };
Phase phase = Phase::Main;
uint32_t completedMs = 0;

void inputReport(const char* label, uint8_t mode) {
  pinMode(pins::I2C_SDA, mode);
  pinMode(pins::I2C_SCL, mode);
  delay(5);
  unsigned sdaHigh = 0, sclHigh = 0;
  for (unsigned i = 0; i < 32; ++i) {
    sdaHigh += gpio_get_level(pins::I2C_SDA);
    sclHigh += gpio_get_level(pins::I2C_SCL);
    delayMicroseconds(100);
  }
  Serial.printf("ELECTRICAL %s SDA_HIGH=%u/32 SCL_HIGH=%u/32 GPIO_IN=0x%08lX\r\n",
                label, sdaHigh, sclHigh, (unsigned long)GPIO.in);
}

bool softClockHigh() {
  digitalWrite(pins::I2C_SCL, HIGH);  // open-drain release only
  const uint32_t started = micros();
  while (!digitalRead(pins::I2C_SCL)) {
    if (micros() - started >= 1000) return false;
  }
  delayMicroseconds(10);
  return true;
}

int softProbe(uint8_t target) {
  digitalWrite(pins::I2C_SDA, LOW);
  delayMicroseconds(10);
  digitalWrite(pins::I2C_SCL, LOW);
  const uint8_t data = target << 1;
  for (int bit = 7; bit >= 0; --bit) {
    digitalWrite(pins::I2C_SDA, (data >> bit) & 1);
    delayMicroseconds(10);
    if (!softClockHigh()) return -1;
    digitalWrite(pins::I2C_SCL, LOW);
  }
  digitalWrite(pins::I2C_SDA, HIGH);
  if (!softClockHigh()) return -1;
  const bool ack = !digitalRead(pins::I2C_SDA);
  digitalWrite(pins::I2C_SCL, LOW);
  digitalWrite(pins::I2C_SDA, LOW);
  delayMicroseconds(10);
  if (!softClockHigh()) return -1;
  digitalWrite(pins::I2C_SDA, HIGH);  // STOP
  delayMicroseconds(10);
  return ack ? 1 : 0;
}

void electricalDiagnostics() {
  // Wire must be stopped. These checks use only the existing I2C pins.
  Serial.println("DIAG: Wire OFF; external pull-up, internal pull-up, ADC, then software I2C");
  inputReport("EXTERNAL_PULLUP_ONLY", INPUT);
  inputReport("INTERNAL_PULLUP_ON", INPUT_PULLUP);
  // GPIO11/12 are ADC2 channels 0/1 on ESP32-S3. ADC mode disables digital
  // input/pulls, so restore GPIO mode explicitly after analog measurements.
  analogSetPinAttenuation(pins::I2C_SDA, ADC_11db);
  analogSetPinAttenuation(pins::I2C_SCL, ADC_11db);
  uint32_t sdaRaw = 0, sclRaw = 0, sdaMv = 0, sclMv = 0;
  for (unsigned i = 0; i < 8; ++i) {
    sdaRaw += analogRead(pins::I2C_SDA);
    sclRaw += analogRead(pins::I2C_SCL);
    sdaMv += analogReadMilliVolts(pins::I2C_SDA);
    sclMv += analogReadMilliVolts(pins::I2C_SCL);
  }
  Serial.printf("ADC_NO_PULL SDA raw=%lu/4095 mv=%lu; SCL raw=%lu/4095 mv=%lu\r\n",
                (unsigned long)(sdaRaw / 8), (unsigned long)(sdaMv / 8),
                (unsigned long)(sclRaw / 8), (unsigned long)(sclMv / 8));
  inputReport("RESTORED_INTERNAL_PULLUP", INPUT_PULLUP);
  if (digitalRead(pins::I2C_SDA) && digitalRead(pins::I2C_SCL)) {
    digitalWrite(pins::I2C_SDA, HIGH);
    digitalWrite(pins::I2C_SCL, HIGH);
    gpio_set_direction(pins::I2C_SDA, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_direction(pins::I2C_SCL, GPIO_MODE_INPUT_OUTPUT_OD);
    unsigned devices = 0;
    for (uint8_t target = 1; target < 127; ++target) {
      const int ack = softProbe(target);
      if (ack < 0) { Serial.println("SOFT I2C CLOCK TIMEOUT"); break; }
      if (ack) { Serial.printf("SOFT ACK 0x%02X\r\n", target); ++devices; }
      delay(1);
    }
    Serial.printf("SOFT I2C devices=%u\r\n", devices);
  } else {
    Serial.println("SOFT I2C SKIPPED: released lines not HIGH before Wire.begin");
  }
  inputReport("BEFORE_WIRE", INPUT_PULLUP);
  const bool wireOk = Wire.begin(pins::I2C_SDA, pins::I2C_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(50);  // independent check of previous 5ms timeout setting
  Serial.printf("WIRE_BEGIN=%u SDA=%d SCL=%d timeout=50ms\r\n", wireOk,
                digitalRead(pins::I2C_SDA), digitalRead(pins::I2C_SCL));
}

bool muxSelect(uint8_t index, uint8_t mask) {
  Wire.beginTransmission(0x70 + index);
  Wire.write(mask);
  return Wire.endTransmission() == 0;
}

void banner() {
  Serial.printf("\r\nSCAN MAIN SDA=%u SCL=%u speed=100000 SDA_LEVEL=%d SCL_LEVEL=%d\r\n",
                unsigned(pins::I2C_SDA), unsigned(pins::I2C_SCL),
                digitalRead(pins::I2C_SDA), digitalRead(pins::I2C_SCL));
  const uint32_t pinMv = analogReadMilliVolts(pins::BATTERY_ADC);
  Serial.printf("BATTERY ADC GPIO%u=%lu mV; x11 estimate=%lu mV (not calibrated)\r\n",
                unsigned(pins::BATTERY_ADC), (unsigned long)pinMv,
                (unsigned long)(pinMv * 11));
}

void probe() {
  Wire.beginTransmission(address);
  const uint8_t result = Wire.endTransmission();
  consecutiveTimeouts = result == 5 ? consecutiveTimeouts + 1 : 0;
  if (result == 0) {
    ++found;
    if (phase == Phase::Main) {
      Serial.printf("MAIN ACK 0x%02X\r\n", address);
      if (address >= 0x70 && address <= 0x77) {
        muxFound[address - 0x70] = true;
        // Turn off candidate mux channels immediately, avoiding collisions.
        if (!muxSelect(address - 0x70, 0)) Serial.println("MUX disable failed");
      }
    } else {
      Serial.printf("MUX 0x%02X CH%u ACK 0x%02X (includes main-bus devices)\r\n",
                    0x70 + muxIndex, channel, address);
    }
  } else if (result != 2) {
    Serial.printf("PROBE 0x%02X error=%u SDA=%d SCL=%d\r\n", address, result,
                  digitalRead(pins::I2C_SDA), digitalRead(pins::I2C_SCL));
  }
  if (consecutiveTimeouts >= 3) {
    Serial.println("BUS TIMEOUT: abort cycle after 3 timeouts; check power/wiring/pull-ups");
    banner();
    completedMs = millis();
    phase = Phase::Wait;
  }
  ++address;
}
}

void setup() {
  BoardIo::setEarlySafeState();
  Serial.begin(57600, SERIAL_8N1, pins::UART0_RX, pins::UART0_TX);
  analogSetPinAttenuation(pins::BATTERY_ADC, ADC_11db);
  Serial.println("\r\nI2C SCANNER V2 -- motors disabled, no SW2 start");
  electricalDiagnostics();
  banner();
}

void loop() {
  if (phase == Phase::Main || phase == Phase::Channel) {
    probe();
    if (address == 127) {
      Serial.printf("SCAN END devices=%u\r\n", found);
      address = 1; found = 0;
      if (phase == Phase::Channel) {
        muxSelect(muxIndex, 0);
        if (++channel == 8) { channel = 0; ++muxIndex; }
      }
      phase = Phase::Select;
    }
  } else if (phase == Phase::Select) {
    while (muxIndex < 8 && !muxFound[muxIndex]) ++muxIndex;
    if (muxIndex == 8) {
      Serial.println("CYCLE COMPLETE; next scan in 10 seconds");
      completedMs = millis(); phase = Phase::Wait;
    } else if (muxSelect(muxIndex, 1U << channel)) {
      Serial.printf("SCAN MUX 0x%02X CH%u\r\n", 0x70 + muxIndex, channel);
      phase = Phase::Channel;
    } else {
      Serial.printf("MUX 0x%02X select failed; skipping\r\n", 0x70 + muxIndex);
      ++muxIndex; channel = 0;
    }
  } else if (millis() - completedMs >= 10000) {
    Wire.end();
    electricalDiagnostics();
    muxIndex = 0; channel = 0; address = 1; found = 0; consecutiveTimeouts = 0;
    for (bool& item : muxFound) item = false;
    banner(); phase = Phase::Main;
  }
  delay(1);
}
