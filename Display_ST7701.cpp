#include "Display_ST7701.h"
#include "Touch_CST820.h"

static spi_device_handle_t SPI_handle = NULL;
esp_lcd_panel_handle_t panel_handle   = NULL;

// ── SPI Kommando / Daten senden ──────────────────────────────────────────
static void spi_send(uint8_t is_data, uint8_t val) {
  spi_transaction_t t = {
    .cmd     = (uint32_t)is_data,
    .addr    = val,
    .length  = 0,
    .rxlength= 0,
  };
  spi_device_transmit(SPI_handle, &t);
}

#define ST7701_CMD(c)  spi_send(0, c)
#define ST7701_DAT(d)  spi_send(1, d)

// ── LCD Reset (EXIO_PIN1) ─────────────────────────────────────────────────
void ST7701_Reset(void) {
  Set_EXIO(EXIO_PIN1, Low);
  vTaskDelay(pdMS_TO_TICKS(10));
  Set_EXIO(EXIO_PIN1, High);
  vTaskDelay(pdMS_TO_TICKS(50));
}

// ── CS Helfer ─────────────────────────────────────────────────────────────
static void cs_enable()  { Set_EXIO(EXIO_PIN3, Low);  vTaskDelay(pdMS_TO_TICKS(5)); }
static void cs_disable() { Set_EXIO(EXIO_PIN3, High); vTaskDelay(pdMS_TO_TICKS(5)); }

// ── ST7701 Register-Init (exakte Waveshare-Sequenz) ──────────────────────
void ST7701_Init(void) {
  // SPI Bus
  spi_bus_config_t buscfg = {
    .mosi_io_num   = LCD_MOSI_PIN,
    .miso_io_num   = -1,
    .sclk_io_num   = LCD_CLK_PIN,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 64,
  };
  spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);

  spi_device_interface_config_t devcfg = {
    .command_bits    = 1,
    .address_bits    = 8,
    .mode            = 0,
    .clock_speed_hz  = 40000000,
    .spics_io_num    = -1,
    .queue_size      = 1,
  };
  spi_bus_add_device(SPI2_HOST, &devcfg, &SPI_handle);

  cs_enable();

  // Page 0
  ST7701_CMD(0xFF); ST7701_DAT(0x77); ST7701_DAT(0x01); ST7701_DAT(0x00); ST7701_DAT(0x00); ST7701_DAT(0x10);

  ST7701_CMD(0xC0); ST7701_DAT(0x3B); ST7701_DAT(0x00);
  ST7701_CMD(0xC1); ST7701_DAT(0x0B); ST7701_DAT(0x02);
  ST7701_CMD(0xC2); ST7701_DAT(0x07); ST7701_DAT(0x02);
  ST7701_CMD(0xCC); ST7701_DAT(0x10);
  ST7701_CMD(0xCD); ST7701_DAT(0x08);  // RGB format

  // Gamma Positiv
  ST7701_CMD(0xB0);
  ST7701_DAT(0x00); ST7701_DAT(0x11); ST7701_DAT(0x16); ST7701_DAT(0x0e);
  ST7701_DAT(0x11); ST7701_DAT(0x06); ST7701_DAT(0x05); ST7701_DAT(0x09);
  ST7701_DAT(0x08); ST7701_DAT(0x21); ST7701_DAT(0x06); ST7701_DAT(0x13);
  ST7701_DAT(0x10); ST7701_DAT(0x29); ST7701_DAT(0x31); ST7701_DAT(0x18);

  // Gamma Negativ
  ST7701_CMD(0xB1);
  ST7701_DAT(0x00); ST7701_DAT(0x11); ST7701_DAT(0x16); ST7701_DAT(0x0e);
  ST7701_DAT(0x11); ST7701_DAT(0x07); ST7701_DAT(0x05); ST7701_DAT(0x09);
  ST7701_DAT(0x09); ST7701_DAT(0x21); ST7701_DAT(0x05); ST7701_DAT(0x13);
  ST7701_DAT(0x11); ST7701_DAT(0x2a); ST7701_DAT(0x31); ST7701_DAT(0x18);

  // Page 1
  ST7701_CMD(0xFF); ST7701_DAT(0x77); ST7701_DAT(0x01); ST7701_DAT(0x00); ST7701_DAT(0x00); ST7701_DAT(0x11);

  ST7701_CMD(0xB0); ST7701_DAT(0x6d);  // VOP
  ST7701_CMD(0xB1); ST7701_DAT(0x37);  // VCOM
  ST7701_CMD(0xB2); ST7701_DAT(0x81);  // VGH 12V
  ST7701_CMD(0xB3); ST7701_DAT(0x80);
  ST7701_CMD(0xB5); ST7701_DAT(0x43);  // VGL -8.3V
  ST7701_CMD(0xB7); ST7701_DAT(0x85);
  ST7701_CMD(0xB8); ST7701_DAT(0x20);
  ST7701_CMD(0xC1); ST7701_DAT(0x78);
  ST7701_CMD(0xC2); ST7701_DAT(0x78);
  ST7701_CMD(0xD0); ST7701_DAT(0x88);

  ST7701_CMD(0xE0); ST7701_DAT(0x00); ST7701_DAT(0x00); ST7701_DAT(0x02);

  ST7701_CMD(0xE1);
  ST7701_DAT(0x03); ST7701_DAT(0xA0); ST7701_DAT(0x00); ST7701_DAT(0x00);
  ST7701_DAT(0x04); ST7701_DAT(0xA0); ST7701_DAT(0x00); ST7701_DAT(0x00);
  ST7701_DAT(0x00); ST7701_DAT(0x20); ST7701_DAT(0x20);

  ST7701_CMD(0xE2);
  for (int i = 0; i < 13; i++) ST7701_DAT(0x00);

  ST7701_CMD(0xE3); ST7701_DAT(0x00); ST7701_DAT(0x00); ST7701_DAT(0x11); ST7701_DAT(0x00);
  ST7701_CMD(0xE4); ST7701_DAT(0x22); ST7701_DAT(0x00);

  ST7701_CMD(0xE5);
  ST7701_DAT(0x05); ST7701_DAT(0xEC); ST7701_DAT(0xA0); ST7701_DAT(0xA0);
  ST7701_DAT(0x07); ST7701_DAT(0xEE); ST7701_DAT(0xA0); ST7701_DAT(0xA0);
  ST7701_DAT(0x00); ST7701_DAT(0x00); ST7701_DAT(0x00); ST7701_DAT(0x00);
  ST7701_DAT(0x00); ST7701_DAT(0x00); ST7701_DAT(0x00); ST7701_DAT(0x00);

  ST7701_CMD(0xE6); ST7701_DAT(0x00); ST7701_DAT(0x00); ST7701_DAT(0x11); ST7701_DAT(0x00);
  ST7701_CMD(0xE7); ST7701_DAT(0x22); ST7701_DAT(0x00);

  ST7701_CMD(0xE8);
  ST7701_DAT(0x06); ST7701_DAT(0xED); ST7701_DAT(0xA0); ST7701_DAT(0xA0);
  ST7701_DAT(0x08); ST7701_DAT(0xEF); ST7701_DAT(0xA0); ST7701_DAT(0xA0);
  ST7701_DAT(0x00); ST7701_DAT(0x00); ST7701_DAT(0x00); ST7701_DAT(0x00);
  ST7701_DAT(0x00); ST7701_DAT(0x00); ST7701_DAT(0x00); ST7701_DAT(0x00);

  ST7701_CMD(0xEB);
  ST7701_DAT(0x00); ST7701_DAT(0x00); ST7701_DAT(0x40); ST7701_DAT(0x40);
  ST7701_DAT(0x00); ST7701_DAT(0x00); ST7701_DAT(0x00);

  ST7701_CMD(0xED);
  ST7701_DAT(0xFF); ST7701_DAT(0xFF); ST7701_DAT(0xFF); ST7701_DAT(0xBA);
  ST7701_DAT(0x0A); ST7701_DAT(0xBF); ST7701_DAT(0x45); ST7701_DAT(0xFF);
  ST7701_DAT(0xFF); ST7701_DAT(0x54); ST7701_DAT(0xFB); ST7701_DAT(0xA0);
  ST7701_DAT(0xAB); ST7701_DAT(0xFF); ST7701_DAT(0xFF); ST7701_DAT(0xFF);

  ST7701_CMD(0xEF);
  ST7701_DAT(0x10); ST7701_DAT(0x0D); ST7701_DAT(0x04);
  ST7701_DAT(0x08); ST7701_DAT(0x3F); ST7701_DAT(0x1F);

  // Page 3
  ST7701_CMD(0xFF); ST7701_DAT(0x77); ST7701_DAT(0x01); ST7701_DAT(0x00); ST7701_DAT(0x00); ST7701_DAT(0x13);
  ST7701_CMD(0xEF); ST7701_DAT(0x08);

  // Page 0 zurück
  ST7701_CMD(0xFF); ST7701_DAT(0x77); ST7701_DAT(0x01); ST7701_DAT(0x00); ST7701_DAT(0x00); ST7701_DAT(0x00);

  ST7701_CMD(0x36); ST7701_DAT(0x00);  // MADCTL
  ST7701_CMD(0x3A); ST7701_DAT(0x66);  // COLMOD: 18-bit RGB666

  ST7701_CMD(0x11);            // Sleep Out
  vTaskDelay(pdMS_TO_TICKS(480));

  ST7701_CMD(0x20);            // Display Inversion Off
  vTaskDelay(pdMS_TO_TICKS(120));

  ST7701_CMD(0x29);            // Display On

  cs_disable();

  // ── RGB-Panel via esp_lcd ─────────────────────────────────────────────
  esp_lcd_rgb_panel_config_t rgb_cfg = {
    .clk_src = LCD_CLK_SRC_DEFAULT,
    .timings = {
      .pclk_hz           = LCD_PCLK_HZ,
      .h_res             = LCD_WIDTH,
      .v_res             = LCD_HEIGHT,
      .hsync_pulse_width = LCD_HPW,
      .hsync_back_porch  = LCD_HBP,
      .hsync_front_porch = LCD_HFP,
      .vsync_pulse_width = LCD_VPW,
      .vsync_back_porch  = LCD_VBP,
      .vsync_front_porch = LCD_VFP,
      .flags = { .pclk_active_neg = false },
    },
    .data_width       = 16,
    .bits_per_pixel   = 16,
    .num_fbs          = 2,
    .bounce_buffer_size_px = 10 * LCD_WIDTH,
    .psram_trans_align= 64,
    .hsync_gpio_num   = LCD_HSYNC,
    .vsync_gpio_num   = LCD_VSYNC,
    .de_gpio_num      = LCD_DE,
    .pclk_gpio_num    = LCD_PCLK,
    .disp_gpio_num    = LCD_DISP,
    .data_gpio_nums   = {
      LCD_D0,  LCD_D1,  LCD_D2,  LCD_D3,  LCD_D4,
      LCD_D5,  LCD_D6,  LCD_D7,  LCD_D8,  LCD_D9,  LCD_D10,
      LCD_D11, LCD_D12, LCD_D13, LCD_D14, LCD_D15
    },
    .flags = { .fb_in_psram = true, .double_fb = true },
  };
  esp_lcd_new_rgb_panel(&rgb_cfg, &panel_handle);
  esp_lcd_panel_reset(panel_handle);
  esp_lcd_panel_init(panel_handle);
}

void LCD_DrawBitmap(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t *color) {
  uint16_t x_end = x + w;
  uint16_t y_end = y + h;
  if (x_end > LCD_WIDTH)  x_end = LCD_WIDTH;
  if (y_end > LCD_HEIGHT) y_end = LCD_HEIGHT;
  esp_lcd_panel_draw_bitmap(panel_handle, x, y, x_end, y_end, color);
}

void Set_Backlight(uint8_t pct) {
  if (pct > 100) pct = 100;
  uint32_t val = pct * 10;
  if (val == 1000) val = 1023;
  ledcWrite(LCD_BL_PIN, val);
}

void LCD_Init(void) {
  ST7701_Reset();
  ST7701_Init();
  Touch_Init();
  // Backlight
  ledcAttach(LCD_BL_PIN, LCD_BL_FREQ, LCD_BL_RESOLUTION);
  Set_Backlight(80);
}
