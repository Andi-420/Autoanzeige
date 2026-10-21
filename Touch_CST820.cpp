#include "Touch_CST820.h"

struct CST820_Touch touch_data = {0};
volatile uint8_t Touch_interrupts = 0;

static bool tp_read(uint8_t reg, uint8_t *buf, uint32_t len) {
  Wire.beginTransmission(CST820_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(true)) { printf("TP I2C read fail\r\n"); return false; }
  // Weniger Bytes als angefordert => Wire.read() liefert 0xFF (-1) und es
  // entstuenden Geister-Beruehrungen an falscher Stelle. Dann lieber verwerfen.
  if (Wire.requestFrom((uint8_t)CST820_ADDR, (uint8_t)len) != len) {
    while (Wire.available()) Wire.read();
    printf("TP I2C short read\r\n");
    return false;
  }
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
  vTaskDelay(pdMS_TO_TICKS(100));   // Chip braucht nach dem Reset etwas, bis er am I2C antwortet
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

// Sleep_State=false: Auto-Sleep AUS. Schlaeft der Chip doch ein, geht die erste
// Beruehrung nach einer Pause nur fuers Aufwecken drauf ("reagiert nicht").
// Deshalb zuruecklesen und notfalls wiederholen.
void CST820_AutoSleep(bool Sleep_State) {
  CST820_Touch_Reset();
  uint8_t val = Sleep_State ? 0x00 : 0xFF;
  for (int i = 0; i < 5; i++) {
    uint8_t rd = (uint8_t)~val;
    if (tp_write(CST820_REG_DisAutoSleep, &val, 1) &&
        tp_read(CST820_REG_DisAutoSleep, &rd, 1) && rd == val) {
      printf("TP AutoSleep %s\r\n", Sleep_State ? "an" : "aus");
      return;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  printf("TP AutoSleep setzen fehlgeschlagen\r\n");
}

uint8_t Touch_Init(void) {
  pinMode(CST820_INT_PIN, INPUT_PULLUP);
  CST820_AutoSleep(false);   // macht selbst einen Reset
  CST820_Read_cfg();
  return true;
}

uint8_t Touch_Read_Data(void) {
  uint8_t buf[6] = {0};
  if (!tp_read(CST820_REG_GestureID, buf, 6)) return false;
  uint8_t event = buf[2] >> 6;   // 0 = aufgesetzt, 1 = abgehoben, 2 = Kontakt
  uint16_t x = ((buf[2] & 0x0F) << 8) | buf[3];
  uint16_t y = ((buf[4] & 0x0F) << 8) | buf[5];
  bool xyOk = x < CST820_RES && y < CST820_RES;
  noInterrupts();
  if (buf[0] != 0x00) touch_data.gesture = (GESTURE)buf[0];
  touch_data.points = buf[1];
  if (touch_data.points > CST820_LCD_TOUCH_MAX_POINTS)
    touch_data.points = CST820_LCD_TOUCH_MAX_POINTS;
  if (touch_data.points && !xyOk) touch_data.points = 0;   // Muell verwerfen
  // Koordinaten gelten bei Beruehrung und beim Abheben (letzte Position)
  if (xyOk && (touch_data.points || event == 1)) {
    touch_data.x = x;
    touch_data.y = y;
  }
  // Finger schon wieder weg, Position aber bekannt: kurzer Tipp
  touch_data.lifted = !touch_data.points && xyOk &&
                      (event == 1 || buf[0] == SINGLE_CLICK);
  interrupts();
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
