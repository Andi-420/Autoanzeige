#pragma once
#include <Arduino.h>
struct Preferences {
  bool begin(const char*, bool) { return true; }
  uint8_t getUChar(const char*, uint8_t d) { return d; }   void putUChar(const char*, uint8_t) {}
  uint16_t getUShort(const char*, uint16_t d) { return d; } void putUShort(const char*, uint16_t) {}
  float getFloat(const char*, float d) { return d; }       void putFloat(const char*, float) {}
  size_t getBytesLength(const char*) { return 0; }
  size_t getBytes(const char*, void*, size_t) { return 0; }
  size_t putBytes(const char*, const void*, size_t n) { return n; }
  bool remove(const char*) { return true; }
};
