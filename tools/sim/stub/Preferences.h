#pragma once
#include <Arduino.h>
struct Preferences{bool begin(const char*,bool){return true;} uint8_t getUChar(const char*,uint8_t d){return d;} void putUChar(const char*,uint8_t){} float getFloat(const char*,float d){return d;} void putFloat(const char*,float){}};
