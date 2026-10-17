/**
 * lv_conf.h — LVGL Konfiguration für Waveshare ESP32-S3-Touch-LCD-2.1
 * MUSS im selben Ordner wie der Sketch liegen (oder in Arduino/libraries/)
 * Kopiere diese Datei auch nach: Arduino/libraries/lvgl/src/lv_conf.h
 */
#if 1  /* ← auf 1 lassen! */
#ifndef LV_CONF_H
#define LV_CONF_H   /* sonst meldet LVGL bei jeder Datei "Possible failure to include lv_conf.h" */

#define LV_COLOR_DEPTH 16          /* RGB565 — passt zum ST7701 */
#define LV_COLOR_16_SWAP 0

/* Display-Auflösung */
#define LV_HOR_RES_MAX 480
#define LV_VER_RES_MAX 480

/* Memory */
#define LV_MEM_CUSTOM 0
#define LV_MEM_SIZE   (256 * 1024U)   /* 256 KB LVGL-Heap */
#define LV_MEM_ADR    0
/* Den Heap beim Start aus dem PSRAM holen statt als festes Array im internen
 * RAM: 256 KB passen dort nicht mehr neben den Rest ("DRAM segment data does
 * not fit"). PSRAM ist ohnehin Pflicht (Board-Einstellung PSRAM: OPI). */
#define LV_MEM_POOL_INCLUDE   <esp_heap_caps.h>
#define LV_MEM_POOL_ALLOC(size)  heap_caps_malloc((size), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)

/* Tick */
#define LV_TICK_CUSTOM       1
#define LV_TICK_CUSTOM_INCLUDE  <Arduino.h>
#define LV_TICK_CUSTOM_SYS_TIME_EXPR  (millis())

/* Logging (0 = aus, 1 = Serial) */
#define LV_USE_LOG  1
#define LV_LOG_LEVEL LV_LOG_LEVEL_WARN
#define LV_LOG_PRINTF 1

/* Fonts */
#define LV_FONT_MONTSERRAT_12  1
#define LV_FONT_MONTSERRAT_14  1
#define LV_FONT_MONTSERRAT_16  1
#define LV_FONT_MONTSERRAT_20  1
#define LV_FONT_MONTSERRAT_24  1
#define LV_FONT_MONTSERRAT_32  1
#define LV_FONT_MONTSERRAT_40  1
#define LV_FONT_MONTSERRAT_48  1
#define LV_FONT_DEFAULT        &lv_font_montserrat_20

/* Widgets */
#define LV_USE_ARC         1
#define LV_USE_BAR         1
#define LV_USE_BUTTON      1
#define LV_USE_LABEL       1
#define LV_USE_IMG         1
#define LV_USE_LINE        1
#define LV_USE_SCALE       1
#define LV_USE_SLIDER      1
#define LV_USE_LED         1

/* Animation */
#define LV_USE_ANIMATION   1

/* Theme */
#define LV_USE_THEME_DEFAULT    1
#define LV_THEME_DEFAULT_DARK   1   /* dunkles Theme — sieht auf rundem Display toll aus */
#define LV_THEME_DEFAULT_GROW   1

/* Draw buffer */
#define LV_DRAW_BUF_ALIGN   4

#endif /* LV_CONF_H */
#endif
