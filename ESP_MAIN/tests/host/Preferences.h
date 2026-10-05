#pragma once
#include "Arduino.h"
#include <cstring>
namespace fake {
extern std::vector<uint8_t> nvs;
extern bool nvsWriteOk, nvsReadOk;
}
struct Preferences {
  bool begin(const char*, bool readOnly) { return readOnly ? fake::nvsReadOk : fake::nvsWriteOk; }
  size_t getBytesLength(const char*) { return fake::nvs.size(); }
  size_t getBytes(const char*, void* data, size_t size) {
    size = std::min(size, fake::nvs.size()); std::memcpy(data, fake::nvs.data(), size); return size;
  }
  size_t putBytes(const char*, const void* data, size_t size) {
    if (!fake::nvsWriteOk) return 0;
    auto* bytes = static_cast<const uint8_t*>(data); fake::nvs.assign(bytes, bytes+size); return size;
  }
  bool remove(const char*) { if (!fake::nvsWriteOk) return false; fake::nvs.clear(); return true; }
  void end() {}
};
