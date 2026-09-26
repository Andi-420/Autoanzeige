#pragma once
#include "Arduino.h"
#include "I2C_Driver.h"
#include "TCA9554PWR.h"

#define CST820_ADDR                  0x15
#define CST820_INT_PIN               16
#define CST820_LCD_TOUCH_MAX_POINTS  1

enum GESTURE {
  NONE         = 0x00,
  SWIPE_UP     = 0x01,
  SWIPE_DOWN   = 0x02,
  SWIPE_LEFT   = 0x03,
  SWIPE_RIGHT  = 0x04,
  SINGLE_CLICK = 0x05,
  DOUBLE_CLICK = 0x0B,
  LONG_PRESS   = 0x0C
};

#define CST820_REG_GestureID    0x01
#define CST820_REG_Version      0x15
#define CST820_REG_ChipID       0xA7
#define CST820_REG_DisAutoSleep 0xFE

extern uint8_t Touch_interrupts;
extern struct CST820_Touch {
  uint8_t  points;
  GESTURE  gesture;
  uint16_t x;
  uint16_t y;
} touch_data;

uint8_t  Touch_Init(void);
uint8_t  CST820_Touch_Reset(void);
void     CST820_AutoSleep(bool Sleep_State);
uint16_t CST820_Read_cfg(void);
uint8_t  Touch_Read_Data(void);
String   Touch_GestureName(void);
void     IRAM_ATTR Touch_CST820_ISR(void);
