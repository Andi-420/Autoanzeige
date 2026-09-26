#include "Touch_CST820.h"

struct CST820_Touch touch_data = {0};
uint8_t Touch_interrupts = 0;

static bool tp_read(uint8_t reg, uint8_t *buf, uint32_t len) {
  Wire.beginTransmission(CST820_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(true)) { printf("TP I2C read fail\r\n"); return false; }
  Wire.requestFrom(CST820_ADDR, len);
  for (uint32_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

static bool tp_write(uint8_t reg, const uint8_t *buf, uint32_t len) {
  Wire.beginTransmission(CST820_ADDR);
  Wire.write(reg);
  for (uint32_t i = 0; i < len; i++) Wire.write(buf[i]);
  if (Wire.endTransmission(true)) { printf("TP I2C write fail\r\n"); return false; }
  return true;
}

uint8_t CST820_Touch_Reset(void) {
  Set_EXIO(EXIO_PIN2, Low);
  vTaskDelay(pdMS_TO_TICKS(10));
  Set_EXIO(EXIO_PIN2, High);
  vTaskDelay(pdMS_TO_TICKS(50));
  return true;
}

uint16_t CST820_Read_cfg(void) {
  uint8_t buf[3] = {0};
  tp_read(CST820_REG_Version, buf, 1);
  printf("TP Version: 0x%02x\r\n", buf[0]);
  tp_read(CST820_REG_ChipID, buf, 3);
  printf("ChipID:0x%02x ProjID:0x%02x FwVer:0x%02x\r\n", buf[0], buf[1], buf[2]);
  return true;
}

void CST820_AutoSleep(bool Sleep_State) {
  CST820_Touch_Reset();
  uint8_t val = Sleep_State ? 0x00 : 0xFF;
  tp_write(CST820_REG_DisAutoSleep, &val, 1);
}

uint8_t Touch_Init(void) {
  pinMode(CST820_INT_PIN, INPUT_PULLUP);
  CST820_Touch_Reset();
  CST820_AutoSleep(false);
  CST820_Read_cfg();
  return true;
}

uint8_t Touch_Read_Data(void) {
  uint8_t buf[6] = {0};
  tp_read(CST820_REG_GestureID, buf, 6);
  if (buf[0] != 0x00) touch_data.gesture = (GESTURE)buf[0];
  if (buf[1] != 0x00) {
    noInterrupts();
    touch_data.points = buf[1];
    if (touch_data.points > CST820_LCD_TOUCH_MAX_POINTS)
      touch_data.points = CST820_LCD_TOUCH_MAX_POINTS;
    touch_data.x = ((buf[2] & 0x0F) << 8) | buf[3];
    touch_data.y = ((buf[4] & 0x0F) << 8) | buf[5];
    interrupts();
  }
  return true;
}

String Touch_GestureName(void) {
  switch (touch_data.gesture) {
    case SWIPE_UP:     return "SWIPE UP";
    case SWIPE_DOWN:   return "SWIPE DOWN";
    case SWIPE_LEFT:   return "SWIPE LEFT";
    case SWIPE_RIGHT:  return "SWIPE RIGHT";
    case SINGLE_CLICK: return "SINGLE CLICK";
    case DOUBLE_CLICK: return "DOUBLE CLICK";
    case LONG_PRESS:   return "LONG PRESS";
    default:           return "NONE";
  }
}

void IRAM_ATTR Touch_CST820_ISR(void) {
  Touch_interrupts = true;
}
