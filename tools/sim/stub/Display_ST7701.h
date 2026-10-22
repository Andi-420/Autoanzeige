#pragma once
#include <Arduino.h>
void LCD_Init(void);
void LCD_DrawBitmap(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t *color);
void Set_Backlight(uint8_t pct);
void LCD_Sleep(void);
#define LCD_BL_PIN 6
#define LCD_WIDTH   480
#define LCD_HEIGHT  480
inline void ledcDetach(int) {}
