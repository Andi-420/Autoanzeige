#pragma once
#include <Arduino.h>
#define CST820_INT_PIN 16
enum GESTURE { NONE=0, SWIPE_UP=1, SWIPE_DOWN=2, SWIPE_LEFT=3, SWIPE_RIGHT=4 };
extern uint8_t Touch_interrupts;
extern struct CST820_Touch { uint8_t points; GESTURE gesture; uint16_t x; uint16_t y; } touch_data;
uint8_t Touch_Read_Data(void);
void Touch_CST820_ISR(void);
