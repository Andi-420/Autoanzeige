#pragma once
#include <Arduino.h>
struct TwoWire{void begin(int,int){} void setClock(int){} void beginTransmission(int){} int endTransmission(bool=true){return 1;}};
extern TwoWire Wire;
