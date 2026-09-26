#pragma once
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "TCA9554PWR.h"

// SPI Pins (ST7701 Initialisierung, nicht Framebuffer)
#define LCD_CLK_PIN   2
#define LCD_MOSI_PIN  1

// Backlight
#define LCD_BL_PIN        6
#define LCD_BL_FREQ       20000
#define LCD_BL_RESOLUTION 10      // 10-bit: 0–1023
#define LCD_BL_DEFAULT    800     // ~78%

// Display-Auflösung
#define LCD_WIDTH   480
#define LCD_HEIGHT  480

// RGB-Timing
#define LCD_PCLK_HZ  (16 * 1000 * 1000)
#define LCD_HPW  8
#define LCD_HBP  10
#define LCD_HFP  50
#define LCD_VPW  3
#define LCD_VBP  8
#define LCD_VFP  8

// RGB-Datenpins (DATA0..DATA15 = B1..B5, G0..G5, R1..R5)
#define LCD_PCLK   41
#define LCD_VSYNC  39
#define LCD_HSYNC  38
#define LCD_DE     40
#define LCD_D0      5
#define LCD_D1     45
#define LCD_D2     48
#define LCD_D3     47
#define LCD_D4     21
#define LCD_D5     14
#define LCD_D6     13
#define LCD_D7     12
#define LCD_D8     11
#define LCD_D9     10
#define LCD_D10     9
#define LCD_D11    46
#define LCD_D12     3
#define LCD_D13     8
#define LCD_D14    18
#define LCD_D15    17
#define LCD_DISP   -1

extern esp_lcd_panel_handle_t panel_handle;

void ST7701_Reset(void);
void ST7701_Init(void);
void LCD_Init(void);
void LCD_DrawBitmap(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t *color);
void Set_Backlight(uint8_t pct);   // 0–100
