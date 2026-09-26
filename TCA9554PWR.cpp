#include "TCA9554PWR.h"

uint8_t I2C_Read_EXIO(uint8_t REG) {
  Wire.beginTransmission(TCA9554_ADDRESS);
  Wire.write(REG);
  Wire.endTransmission();
  Wire.requestFrom(TCA9554_ADDRESS, 1);
  return Wire.available() ? Wire.read() : 0xFF;
}

uint8_t I2C_Write_EXIO(uint8_t REG, uint8_t Data) {
  Wire.beginTransmission(TCA9554_ADDRESS);
  Wire.write(REG);
  Wire.write(Data);
  uint8_t r = Wire.endTransmission();
  if (r) printf("I2C Write EXIO fail\r\n");
  return r;
}

void Mode_EXIO(uint8_t Pin, uint8_t State) {
  uint8_t bits = I2C_Read_EXIO(TCA9554_CONFIG_REG);
  I2C_Write_EXIO(TCA9554_CONFIG_REG, (0x01 << (Pin-1)) | bits);
}

void Mode_EXIOS(uint8_t PinState) {
  I2C_Write_EXIO(TCA9554_CONFIG_REG, PinState);
}

uint8_t Read_EXIO(uint8_t Pin) {
  return (I2C_Read_EXIO(TCA9554_INPUT_REG) >> (Pin-1)) & 0x01;
}

uint8_t Read_EXIOS(uint8_t REG) {
  return I2C_Read_EXIO(REG);
}

void Set_EXIO(uint8_t Pin, uint8_t State) {
  if (State > 1 || Pin < 1 || Pin > 8) { printf("Set_EXIO: bad param\r\n"); return; }
  uint8_t bits = Read_EXIOS(TCA9554_OUTPUT_REG);
  if (State) bits |=  (0x01 << (Pin-1));
  else       bits &= ~(0x01 << (Pin-1));
  I2C_Write_EXIO(TCA9554_OUTPUT_REG, bits);
}

void Set_EXIOS(uint8_t PinState) {
  I2C_Write_EXIO(TCA9554_OUTPUT_REG, PinState);
}

void Set_Toggle(uint8_t Pin) {
  Set_EXIO(Pin, (bool)!Read_EXIO(Pin));
}

void TCA9554PWR_Init(uint8_t PinState) {
  Mode_EXIOS(PinState);  // 0x00 = alle als Output
}
