/*
 * ═══════════════════════════════════════════════════════════════════
 * OBD-II Anzeige v7 — Waveshare ESP32-S3-Touch-LCD-2.1
 * FreeRTOS-Architektur: OBD-Task auf Core 0, LVGL auf Core 1
 *
 * Swipe-Navigation (von der Hauptseite aus, Gestenerkennung des CST820):
 *   SWIPE_UP    → Helligkeit (Slider)
 *   SWIPE_DOWN  → Fehlercodes (DTC)
 *   SWIPE_LEFT  → Ladedruck / Oel-Temperatur
 *   SWIPE_RIGHT → Beschleunigung (QMI8658)
 *   Von jeder Unterseite: beliebig wischen → zurueck zur Hauptseite
 *   (Helligkeit: nur vertikal, horizontal bedient den Slider)
 *
 * G-Meter: RESET tippen = Max-Werte loeschen,
 *          RESET lang druecken (Auto steht) = Nullpunkt setzen
 *
 * ELM327: UART1, GPIO43(TX)/GPIO44(RX) + MAX232A
 * IMU:    I2C (GPIO7=SCL, GPIO15=SDA)
 *
 * Libraries:
 *  - LVGL >= 9.1
 *  - FastIMU (QMI8658)
 *
 * Board: ESP32S3 Dev Module | PSRAM: OPI | USB CDC: Enabled
 * ═══════════════════════════════════════════════════════════════════
 */

#include <Arduino.h>
#include <Wire.h>
#include <lvgl.h>
// QMI8658 via FastIMU library (Arduino Library Manager: "FastIMU" v1.2.6)
#include <FastIMU.h>
#include <Preferences.h>
#include "TCA9554PWR.h"
#include "Display_ST7701.h"
#include "Touch_CST820.h"
#include "I2C_Driver.h"

// ══════════════════════════════════════════════════════════════════
// KONFIGURATION
// ══════════════════════════════════════════════════════════════════
#define ELM_RX 44
#define ELM_TX 43
#define ELM_BAUD 38400

#define DISP_W 480
#define DISP_H 480
#define DRAW_BUF_SIZE (DISP_W * DISP_H / 10)

#define PAGE_DTC    0
#define PAGE_MAIN   1
#define PAGE_BRIGHT 2
#define PAGE_BOOST  3
#define PAGE_ACCEL  4
#define PAGE_COUNT  5

#define C_BG      0x0A0A12
#define C_SURFACE 0x12121E
#define C_RING_BG 0x1C1C2C
#define C_CYAN    0x00BFFF
#define C_GREEN   0x00E676
#define C_YELLOW  0xFFCC00
#define C_ORANGE  0xFF8C00
#define C_RED     0xFF2020
#define C_MUTED   0x555577
#define C_WHITE   0xE0E0F0
#define C_PURPLE  0xCC44FF

// ══════════════════════════════════════════════════════════════════
// GLOBALE OBJEKTE & SHARED STATE (FreeRTOS-sicher via Mutex)
// ══════════════════════════════════════════════════════════════════
HardwareSerial elmSerial(1);
QMI8658 imu;                 // FastIMU Klasse fuer QMI8658
calData calib = {0};         // FastIMU Kalibrierungsdaten (leer = unkalibriert)

static lv_color_t *draw_buf_1 = nullptr;
static lv_color_t *draw_buf_2 = nullptr;
static lv_display_t *display = nullptr;

static lv_obj_t *screens[PAGE_COUNT];
static int curPage = PAGE_MAIN;

// Mutex schützt alle gXxx-Variablen zwischen OBD-Task und LVGL-Task
static SemaphoreHandle_t dataMutex = nullptr;

// OBD-Messwerte (nur unter Mutex lesen/schreiben)
static float gBatt = 0.0f;
static int   gOilTemp = 0;
static int   gCoolant = 0;
static int   gRPM = 0;

// Ladedruck-Berechnung: MAP (Drosselklappen-/Saugrohrdruck) - Umgebungsdruck
static float gMapKPa = 0.0f;      // PID 0x0B, absoluter Druck nach Drosselklappe (kPa)
// Default = Normaldruck, falls das Auto PID 0x33 nicht unterstuetzt
// (sonst wuerde Ladedruck = MAP - 0 => im Leerlauf ca. +1 bar angezeigt)
static float gAmbientKPa = 101.3f; // PID 0x33, Umgebungs-/Barometerdruck (kPa)
static float gBoostBar = 0.0f;    // berechneter Ladedruck in bar (kann negativ sein = Unterdruck)

static bool gElmOK = false;       // ELM327-Adapter antwortet
static bool gEcuOK = false;       // Steuergeraet antwortet (Zuendung an)
static bool gOilOK = false;     // bis zur ersten gueltigen Antwort "N/A" anzeigen
static bool gIsCan = true;        // Protokoll (nur im OBD-Task benutzt), per ATDPN ermittelt
static volatile bool gFetchDTC = false;   // gesetzt von LVGL-Task, gelesen von OBD-Task

// DTC-Lesestatus fuer die Anzeige
#define DTC_IDLE    0
#define DTC_READING 1
#define DTC_DONE    2
#define DTC_ERROR   3
static int gDtcState = DTC_IDLE;  // unter Mutex

// Flag: neue Daten verfügbar → LVGL-Task soll Display updaten
static volatile bool gNewData = false;
static volatile bool gNewDTC = false;

// IMU (wird im LVGL-Task gelesen, kein Mutex nötig)
static bool gMpuOK = false;
static float gAx=0, gAy=0, gAz=0;
static float gAxMax=0, gAyMax=0; // Max-G Werte (für GTI G-Meter)
// Nullpunkt (Einbau-Schraeglage), per langem Druck auf RESET gesetzt, im NVS gespeichert
static float gOffLong=0, gOffLat=0;

// DTC
#define DTC_MAX 10
static String dtcList[DTC_MAX];
static int dtcCount = 0;

// Helligkeit (wird im NVS-Flash gespeichert und beim Start geladen)
static uint8_t gBrightness = 80;
static Preferences prefs;

static void save_brightness(void) {
  // Nur beim Loslassen/Preset speichern, nicht bei jedem Slider-Schritt (Flash-Verschleiss)
  if (prefs.getUChar("bright", 0) != gBrightness) prefs.putUChar("bright", gBrightness);
}

// Swipe: gestenbasiert via CST820-Hardware-Register (kein Koordinatenvergleich)

// ══════════════════════════════════════════════════════════════════
// FARBLOGIK OEL-TEMPERATUR (zentral, damit ueberall gleich)
// -20..60 = CYAN | 60..80 = YELLOW | 80..105 = GREEN | >105 = RED
// NO DATA / oilOk == false => CYAN
// ══════════════════════════════════════════════════════════════════
static uint32_t oil_temp_color(int oil, bool oilOk) {
  if (!oilOk) return C_CYAN;
  if (oil > 105) return C_RED;
  if (oil > 80)  return C_GREEN;
  if (oil > 60)  return C_YELLOW;
  return C_CYAN; // -20 .. 60
}

// ══════════════════════════════════════════════════════════════════
// SEITEN-NAVIGATION (vor Touch-Callback definiert)
// ══════════════════════════════════════════════════════════════════
static void refresh_main(void);
static void refresh_dtc_ui(void);
static void refresh_boost(void);
static void refresh_accel(void);

static void goto_page(int idx) {
  if (idx < 0 || idx >= PAGE_COUNT) return;
  curPage = idx;
  // Beim Betreten sofort mit aktuellen Werten fuellen (nicht erst beim naechsten OBD-Zyklus)
  switch (idx) {
    case PAGE_MAIN:  refresh_main();   break;
    case PAGE_BOOST: refresh_boost();  break;
    case PAGE_DTC:   refresh_dtc_ui(); break;
    case PAGE_ACCEL: refresh_accel();  break;
    default: break;
  }
  lv_screen_load_anim(screens[idx], LV_SCR_LOAD_ANIM_FADE_ON, 200, 0, false);
}

// ══════════════════════════════════════════════════════════════════
// LVGL CALLBACKS
// ══════════════════════════════════════════════════════════════════
void IRAM_ATTR touchISR() { Touch_CST820_ISR(); }

static void lvgl_flush_cb(lv_display_t *disp,
                           const lv_area_t *area, uint8_t *px_map) {
  LCD_DrawBitmap(area->x1, area->y1,
                 area->x2 - area->x1 + 1,
                 area->y2 - area->y1 + 1, px_map);
  lv_display_flush_ready(disp);
}

static void lvgl_touch_cb(lv_indev_t *indev, lv_indev_data_t *data) {
  // Nur lesen, wenn der Chip einen Interrupt gemeldet hat ODER der Finger
  // zuletzt noch auflag. Sonst meldete LVGL zwischen zwei Interrupts
  // "losgelassen" => Slider-Ziehen ruckelt / Buttons loesen doppelt aus.
  static bool wasPressed = false;
  if (!Touch_interrupts && !wasPressed) {
    data->state = LV_INDEV_STATE_RELEASED;
    return;
  }

  Touch_interrupts = false;
  if (!Touch_Read_Data()) touch_data.points = 0;
  wasPressed = touch_data.points > 0;

  // ── Gesten-Navigation ────────────────────────────────────────────
  // Layout:            [Helligkeit]
  //          [Ladedruck] [MAIN] [Beschleunigung]
  //                     [Fehlercodes]
  //
  // Von MAIN → Unterseiten per Swipe-Richtung
  // Von jeder UNTERSEITE → MAIN (jede Swipe-Richtung bringt zurück)
  if (touch_data.gesture != NONE) {
    if (curPage == PAGE_MAIN) {
      switch (touch_data.gesture) {
        case SWIPE_UP:    goto_page(PAGE_BRIGHT); break;
        case SWIPE_DOWN:  goto_page(PAGE_DTC);    break;
        case SWIPE_LEFT:  goto_page(PAGE_BOOST);  break;
        case SWIPE_RIGHT: goto_page(PAGE_ACCEL);  break;
        default: break;
      }
    } else if (curPage == PAGE_BRIGHT &&
               (touch_data.gesture == SWIPE_LEFT || touch_data.gesture == SWIPE_RIGHT)) {
      // Horizontales Wischen = Slider ziehen, NICHT zurueck zur Hauptseite
    } else {
      goto_page(PAGE_MAIN);
    }
    touch_data.gesture = NONE;
  }

  // ── Touch-Position für LVGL (Tap/Slider etc.) ────────────────────
  if (touch_data.points > 0) {
    data->state = LV_INDEV_STATE_PRESSED;
    data->point.x = touch_data.x;
    data->point.y = touch_data.y;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

static uint32_t lvgl_tick_cb(void) { return millis(); }

// ══════════════════════════════════════════════════════════════════
// HILFSFUNKTIONEN UI
// ══════════════════════════════════════════════════════════════════

static lv_obj_t* make_screen(void) {
  lv_obj_t *scr = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(scr, lv_color_hex(C_BG), 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
  lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t *ring = lv_arc_create(scr);
  lv_obj_set_size(ring, 474, 474);
  lv_obj_center(ring);
  lv_arc_set_bg_angles(ring, 0, 360);
  lv_arc_set_value(ring, 100);
  lv_obj_set_style_arc_color(ring, lv_color_hex(0x1A1A30), LV_PART_MAIN);
  lv_obj_set_style_arc_width(ring, 4, LV_PART_MAIN);
  lv_obj_set_style_arc_color(ring, lv_color_hex(0x1A1A30), LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(ring, 4, LV_PART_INDICATOR);
  lv_obj_set_style_bg_opa(ring, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_remove_style(ring, NULL, LV_PART_KNOB);
  lv_obj_clear_flag(ring, LV_OBJ_FLAG_CLICKABLE);

  return scr;
}

static lv_obj_t* add_title(lv_obj_t *scr, const char *txt, uint32_t col) {
  lv_obj_t *l = lv_label_create(scr);
  lv_label_set_text(l, txt);
  // Oben ist das runde Display schmal: auf Hoehe y=38 sind nur ca. 220 px
  // sichtbar. Passt der Titel in Schriftgroesse 20 nicht, wird 16 verwendet.
  const lv_font_t *f = &lv_font_montserrat_20;
  if (lv_text_get_width(txt, strlen(txt), f, 0) > 210) f = &lv_font_montserrat_16;
  lv_obj_set_style_text_font(l, f, 0);
  lv_obj_set_style_text_color(l, lv_color_hex(col), 0);
  lv_obj_align(l, LV_ALIGN_TOP_MID, 0, 38);
  return l;
}

static void add_nav_dots(lv_obj_t *scr, int active) {
  const int DOT = 8, GAP = 14;
  int total_w = PAGE_COUNT * DOT + (PAGE_COUNT-1) * GAP;
  int x0 = 240 - total_w / 2;
  for (int i = 0; i < PAGE_COUNT; i++) {
    lv_obj_t *d = lv_obj_create(scr);
    lv_obj_set_size(d, DOT, DOT);
    lv_obj_set_pos(d, x0 + i*(DOT+GAP), 452);
    lv_obj_set_style_radius(d, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(d, 0, 0);
    lv_obj_set_style_bg_color(d,
      (i == active) ? lv_color_hex(C_CYAN) : lv_color_hex(C_RING_BG), 0);
    lv_obj_set_style_bg_opa(d, LV_OPA_COVER, 0);
    lv_obj_clear_flag(d, LV_OBJ_FLAG_SCROLLABLE);
  }
}

// ══════════════════════════════════════════════════════════════════
// SEITE 1: OBD-HAUPTANZEIGE (4 Kacheln)
// ══════════════════════════════════════════════════════════════════

struct Tile {
  lv_obj_t *arc_val, *lbl_val, *lbl_unit;
  int val_min, val_max;
};
static Tile tiles[4];
static lv_obj_t *lbl_obd_status;

static void create_tile_main(lv_obj_t *scr, int idx,
                              const char *title, const char *unit,
                              int vmin, int vmax) {
  Tile &t = tiles[idx];
  t.val_min = vmin; t.val_max = vmax;

  const int S=162, G2=5;
  int x = (idx%2==0) ? (240-G2-S) : (240+G2);
  int y = (idx/2==0) ? (240-G2-S) : (240+G2);

  lv_obj_t *cont = lv_obj_create(scr);
  lv_obj_set_pos(cont, x, y);
  lv_obj_set_size(cont, S, S);
  lv_obj_set_style_bg_color(cont, lv_color_hex(C_SURFACE), 0);
  lv_obj_set_style_bg_opa(cont, LV_OPA_COVER, 0);
  lv_obj_set_style_border_color(cont, lv_color_hex(0x222233), 0);
  lv_obj_set_style_border_width(cont, 1, 0);
  lv_obj_set_style_radius(cont, 16, 0);
  lv_obj_set_style_pad_all(cont, 0, 0);
  lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t *abg = lv_arc_create(cont);
  lv_obj_set_size(abg, 126, 126); lv_obj_center(abg);
  lv_arc_set_bg_angles(abg, 135, 405); lv_arc_set_value(abg, 100);
  lv_obj_set_style_arc_color(abg, lv_color_hex(C_RING_BG), LV_PART_MAIN);
  lv_obj_set_style_arc_width(abg, 10, LV_PART_MAIN);
  lv_obj_set_style_arc_color(abg, lv_color_hex(C_RING_BG), LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(abg, 10, LV_PART_INDICATOR);
  lv_obj_set_style_bg_opa(abg, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_remove_style(abg, NULL, LV_PART_KNOB);
  lv_obj_clear_flag(abg, LV_OBJ_FLAG_CLICKABLE);

  t.arc_val = lv_arc_create(cont);
  lv_obj_set_size(t.arc_val, 126, 126); lv_obj_center(t.arc_val);
  lv_arc_set_bg_angles(t.arc_val, 135, 405);
  lv_arc_set_range(t.arc_val, 0, 100); lv_arc_set_value(t.arc_val, 0);
  lv_obj_set_style_arc_color(t.arc_val, lv_color_hex(0x111122), LV_PART_MAIN);
  lv_obj_set_style_arc_width(t.arc_val, 1, LV_PART_MAIN);
  lv_obj_set_style_arc_color(t.arc_val, lv_color_hex(C_CYAN), LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(t.arc_val, 10, LV_PART_INDICATOR);
  lv_obj_set_style_arc_rounded(t.arc_val, true, LV_PART_INDICATOR);
  lv_obj_remove_style(t.arc_val, NULL, LV_PART_KNOB);
  lv_obj_clear_flag(t.arc_val, LV_OBJ_FLAG_CLICKABLE);

  lv_obj_t *lt = lv_label_create(cont);
  lv_label_set_text(lt, title);
  lv_obj_set_style_text_font(lt, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(lt, lv_color_hex(C_MUTED), 0);
  lv_obj_align(lt, LV_ALIGN_TOP_MID, 0, 6);

  t.lbl_val = lv_label_create(cont);
  lv_label_set_text(t.lbl_val, "--");
  lv_obj_set_style_text_font(t.lbl_val, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(t.lbl_val, lv_color_hex(C_WHITE), 0);
  lv_obj_align(t.lbl_val, LV_ALIGN_CENTER, 0, -4);

  t.lbl_unit = lv_label_create(cont);
  lv_label_set_text(t.lbl_unit, unit);
  lv_obj_set_style_text_font(t.lbl_unit, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(t.lbl_unit, lv_color_hex(C_MUTED), 0);
  lv_obj_align(t.lbl_unit, LV_ALIGN_CENTER, 0, 18);
}

static void build_page_main(void) {
  lv_obj_t *scr = make_screen();
  screens[PAGE_MAIN] = scr;

  create_tile_main(scr, 0, "SPANNUNG",   "VOLT",   110, 150);
  create_tile_main(scr, 1, "OEL-TEMP",   "GRAD C", -20, 150);
  create_tile_main(scr, 2, "KUEHLWASSER","GRAD C", -20, 130);
  create_tile_main(scr, 3, "DREHZAHL",   "U/MIN",  0, 8000);

  lbl_obd_status = lv_label_create(scr);
  lv_label_set_text(lbl_obd_status, "INIT");
  lv_obj_set_style_text_font(lbl_obd_status, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(lbl_obd_status, lv_color_hex(C_MUTED), 0);
  lv_obj_align(lbl_obd_status, LV_ALIGN_CENTER, 0, 0);

  add_nav_dots(scr, PAGE_MAIN);
}

static void update_tile(int idx, const char *vs, int raw, uint32_t col) {
  Tile &t = tiles[idx];
  int pct = 0;
  if (t.val_max > t.val_min)
    pct = constrain((int)(100.0f*(raw-t.val_min)/(float)(t.val_max-t.val_min)),0,100);
  lv_arc_set_value(t.arc_val, pct);
  lv_obj_set_style_arc_color(t.arc_val, lv_color_hex(col), LV_PART_INDICATOR);
  lv_label_set_text(t.lbl_val, vs);
  lv_obj_set_style_text_color(t.lbl_val, lv_color_hex(col), 0);
  lv_obj_align(t.lbl_val, LV_ALIGN_CENTER, 0, -4);
}

static void refresh_main(void) {
  float batt;
  int oil, cool, rpm;
  bool oilOk, elmOk, ecuOk;
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  batt = gBatt; oil = gOilTemp; cool = gCoolant;
  rpm = gRPM; oilOk = gOilOK; elmOk = gElmOK; ecuOk = gEcuOK;
  xSemaphoreGive(dataMutex);

  char buf[12];
  dtostrf(batt,4,1,buf); char*p=buf; while(*p==' ')p++;
  uint32_t bc = batt<11.5f?C_RED:batt<12.5f?C_YELLOW:C_GREEN;
  update_tile(0, p, (int)(batt*10.0f), bc);

  if (oilOk) {
    snprintf(buf,sizeof(buf),"%d",oil);
  } else {
    snprintf(buf,sizeof(buf),"N/A");
  }
  update_tile(1, buf, oilOk ? oil : -20, oil_temp_color(oil, oilOk));

  snprintf(buf,sizeof(buf),"%d",cool);
  update_tile(2,buf,cool,cool>110?C_RED:cool<50?C_YELLOW:C_CYAN);

  snprintf(buf,sizeof(buf),"%d",rpm);
  update_tile(3,buf,rpm,rpm>6000?C_RED:rpm>4500?C_ORANGE:C_CYAN);

  // ELM? = Adapter antwortet nicht | ECU? = Adapter ok, Auto antwortet nicht (Zuendung aus?)
  const char *st = !elmOk ? "ELM?" : !ecuOk ? "ECU?" : "OBD OK";
  uint32_t stCol = !elmOk ? C_RED : !ecuOk ? C_YELLOW : C_GREEN;
  lv_label_set_text(lbl_obd_status, st);
  lv_obj_set_style_text_color(lbl_obd_status, lv_color_hex(stCol), 0);
  lv_obj_align(lbl_obd_status, LV_ALIGN_CENTER, 0, 0);
}

// ══════════════════════════════════════════════════════════════════
// SEITE 0: FEHLERCODES (DTC)
// ══════════════════════════════════════════════════════════════════
static lv_obj_t *dtc_list_obj = nullptr;
static lv_obj_t *lbl_dtc_count = nullptr;

static void build_page_dtc(void) {
  lv_obj_t *scr = make_screen();
  screens[PAGE_DTC] = scr;

  add_title(scr, LV_SYMBOL_WARNING " FEHLERCODES (DTC)", C_ORANGE);

  lv_obj_t *cont = lv_obj_create(scr);
  lv_obj_set_pos(cont, 90, 82);
  lv_obj_set_size(cont, 300, 230);
  lv_obj_set_style_bg_color(cont, lv_color_hex(C_SURFACE), 0);
  lv_obj_set_style_bg_opa(cont, LV_OPA_COVER, 0);
  lv_obj_set_style_border_color(cont, lv_color_hex(0x222233), 0);
  lv_obj_set_style_border_width(cont, 1, 0);
  lv_obj_set_style_radius(cont, 14, 0);
  lv_obj_set_style_pad_all(cont, 12, 0);
  lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_START,
    LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

  dtc_list_obj = cont;

  lbl_dtc_count = lv_label_create(scr);
  lv_label_set_text(lbl_dtc_count, "Tippe zum Auslesen");
  lv_obj_set_style_text_font(lbl_dtc_count, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(lbl_dtc_count, lv_color_hex(C_MUTED), 0);
  lv_obj_align(lbl_dtc_count, LV_ALIGN_CENTER, 0, 98);

  lv_obj_t *btn = lv_button_create(scr);
  lv_obj_set_size(btn, 160, 42);
  lv_obj_align(btn, LV_ALIGN_CENTER, 0, 171);
  lv_obj_set_style_bg_color(btn, lv_color_hex(0x1A1A3E), 0);
  lv_obj_set_style_bg_color(btn, lv_color_hex(0x2A2A5E), LV_STATE_PRESSED);
  lv_obj_set_style_border_color(btn, lv_color_hex(C_ORANGE), 0);
  lv_obj_set_style_border_width(btn, 1, 0);
  lv_obj_set_style_radius(btn, 21, 0);
  lv_obj_t *bl = lv_label_create(btn);
  lv_label_set_text(bl, LV_SYMBOL_REFRESH " AUSLESEN");
  lv_obj_set_style_text_font(bl, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(bl, lv_color_hex(C_ORANGE), 0);
  lv_obj_center(bl);
  lv_obj_add_event_cb(btn, [](lv_event_t*e){
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    gDtcState = DTC_READING;
    xSemaphoreGive(dataMutex);
    gFetchDTC = true;
    refresh_dtc_ui();
  }, LV_EVENT_CLICKED, nullptr);

  add_nav_dots(scr, PAGE_DTC);
}

static void refresh_dtc_ui(void) {
  lv_obj_clean(dtc_list_obj);
  char buf[32];
  int count, state;
  String codes[DTC_MAX];

  xSemaphoreTake(dataMutex, portMAX_DELAY);
  count = dtcCount;
  state = gDtcState;
  for (int i=0; i<count; i++) codes[i] = dtcList[i];
  xSemaphoreGive(dataMutex);

  if (state != DTC_DONE) {
    const char *msg = state == DTC_READING ? "Lese Fehlercodes..."
                    : state == DTC_ERROR   ? "Fehler beim Auslesen"
                    : "Tippe zum Auslesen";
    lv_label_set_text(lbl_dtc_count, msg);
    lv_obj_set_style_text_color(lbl_dtc_count,
      lv_color_hex(state == DTC_ERROR ? C_RED : C_MUTED), 0);
    lv_obj_align(lbl_dtc_count, LV_ALIGN_CENTER, 0, 98);
    return;
  }

  for (int i=0; i<count; i++) {
    lv_obj_t *l = lv_label_create(dtc_list_obj);
    lv_label_set_text(l, codes[i].c_str());
    lv_obj_set_style_text_font(l, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(l, lv_color_hex(C_RED), 0);
  }

  snprintf(buf, sizeof(buf), "%d Fehlercode(s)", count);
  lv_label_set_text(lbl_dtc_count, buf);
  lv_obj_set_style_text_color(lbl_dtc_count,
    lv_color_hex(count>0?C_RED:C_GREEN), 0);
  lv_obj_align(lbl_dtc_count, LV_ALIGN_CENTER, 0, 98);
}

// ══════════════════════════════════════════════════════════════════
// SEITE 2: HELLIGKEIT (SLIDER)
// ══════════════════════════════════════════════════════════════════
static lv_obj_t *lbl_bright_val = nullptr;
static lv_obj_t *bright_slider = nullptr;

static void build_page_brightness(void) {
  lv_obj_t *scr = make_screen();
  screens[PAGE_BRIGHT] = scr;

  add_title(scr, LV_SYMBOL_SETTINGS " HELLIGKEIT", C_YELLOW);

  lbl_bright_val = lv_label_create(scr);
  char buf[8]; snprintf(buf,sizeof(buf),"%d%%",gBrightness);
  lv_label_set_text(lbl_bright_val, buf);
  lv_obj_set_style_text_font(lbl_bright_val, &lv_font_montserrat_48, 0);
  lv_obj_set_style_text_color(lbl_bright_val, lv_color_hex(C_YELLOW), 0);
  lv_obj_align(lbl_bright_val, LV_ALIGN_CENTER, 0, -40);

  lv_obj_t *l1 = lv_label_create(scr);
  lv_label_set_text(l1, LV_SYMBOL_IMAGE);
  lv_obj_set_style_text_font(l1, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(l1, lv_color_hex(C_MUTED), 0);
  lv_obj_align(l1, LV_ALIGN_CENTER, -170, 40);

  lv_obj_t *l2 = lv_label_create(scr);
  lv_label_set_text(l2, LV_SYMBOL_IMAGE);
  lv_obj_set_style_text_font(l2, &lv_font_montserrat_32, 0);
  lv_obj_set_style_text_color(l2, lv_color_hex(C_YELLOW), 0);
  lv_obj_align(l2, LV_ALIGN_CENTER, 170, 40);

  lv_obj_t *sl = lv_slider_create(scr);
  lv_obj_set_size(sl, 300, 20);
  lv_obj_align(sl, LV_ALIGN_CENTER, 0, 40);
  lv_slider_set_range(sl, 10, 100);
  lv_slider_set_value(sl, gBrightness, LV_ANIM_OFF);
  bright_slider = sl;

  lv_obj_set_style_bg_color(sl, lv_color_hex(C_RING_BG), LV_PART_MAIN);
  lv_obj_set_style_radius(sl, 10, LV_PART_MAIN);
  lv_obj_set_style_bg_color(sl, lv_color_hex(C_YELLOW), LV_PART_INDICATOR);
  lv_obj_set_style_radius(sl, 10, LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(sl, lv_color_hex(C_WHITE), LV_PART_KNOB);
  lv_obj_set_style_radius(sl, LV_RADIUS_CIRCLE, LV_PART_KNOB);
  lv_obj_set_style_pad_all(sl, 6, LV_PART_KNOB);

  lv_obj_add_event_cb(sl, [](lv_event_t*e){
    lv_obj_t *s = (lv_obj_t*)lv_event_get_target(e);
    gBrightness = (uint8_t)lv_slider_get_value(s);
    Set_Backlight(gBrightness);
    char buf2[8]; snprintf(buf2,sizeof(buf2),"%d%%",gBrightness);
    lv_label_set_text(lbl_bright_val, buf2);
    lv_obj_align(lbl_bright_val, LV_ALIGN_CENTER, 0, -40);
  }, LV_EVENT_VALUE_CHANGED, nullptr);
  lv_obj_add_event_cb(sl, [](lv_event_t*e){ save_brightness(); },
                      LV_EVENT_RELEASED, nullptr);

  const uint8_t presets[] = {25,50,75,100};
  const char* plabels[] = {"25%","50%","75%","100%"};
  for (int i=0;i<4;i++){
    lv_obj_t *b = lv_button_create(scr);
    lv_obj_set_size(b, 68, 36);
    lv_obj_set_pos(b, 60 + i*95, 340);
    lv_obj_set_style_bg_color(b, lv_color_hex(0x1A1A3E), 0);
    lv_obj_set_style_bg_color(b, lv_color_hex(0x2A2A5E), LV_STATE_PRESSED);
    lv_obj_set_style_border_color(b, lv_color_hex(C_YELLOW), 0);
    lv_obj_set_style_border_width(b, 1, 0);
    lv_obj_set_style_radius(b, 18, 0);
    lv_obj_t *bl2 = lv_label_create(b);
    lv_label_set_text(bl2, plabels[i]);
    lv_obj_set_style_text_font(bl2, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(bl2, lv_color_hex(C_YELLOW), 0);
    lv_obj_center(bl2);
    lv_obj_set_user_data(b, (void*)(uintptr_t)presets[i]);
    lv_obj_add_event_cb(b, [](lv_event_t*e){
      uint8_t pct = (uint8_t)(uintptr_t)lv_obj_get_user_data(
        (lv_obj_t*)lv_event_get_target(e));
      gBrightness = pct;
      Set_Backlight(pct);
      save_brightness();
      if (bright_slider) lv_slider_set_value(bright_slider, pct, LV_ANIM_ON);
      char buf3[8]; snprintf(buf3,sizeof(buf3),"%d%%",pct);
      lv_label_set_text(lbl_bright_val, buf3);
      lv_obj_align(lbl_bright_val, LV_ALIGN_CENTER, 0, -40);
    }, LV_EVENT_CLICKED, nullptr);
  }

  add_nav_dots(scr, PAGE_BRIGHT);
}

// ══════════════════════════════════════════════════════════════════
// SEITE 3: LADEDRUCK + OEL-TEMPERATUR (zwei parallele Arcs)
// ══════════════════════════════════════════════════════════════════
static lv_obj_t *boost_arc = nullptr;
static lv_obj_t *oil_arc = nullptr;
static lv_obj_t *lbl_boost_val = nullptr;
static lv_obj_t *lbl_oil_val = nullptr;
static lv_obj_t *lbl_boost_status = nullptr;

// Skalierung: bar -> Arc-Wert in "Centibar" (int), damit lv_arc mit 0.01-bar-Aufloesung arbeiten kann
#define BOOST_ARC_MIN (-50)   // -0.5 bar
#define BOOST_ARC_MAX  150    // +1.5 bar
#define OIL_ARC_MIN   (-20)   // Grad C
#define OIL_ARC_MAX    150    // Grad C

static void build_page_boost(void) {
  lv_obj_t *scr = make_screen();
  screens[PAGE_BOOST] = scr;

  add_title(scr, "LADEDRUCK / OEL-TEMP", C_PURPLE);

  // ── Aeusserer Arc: LADEDRUCK (lila) ─────────────────────────
  lv_obj_t *abg_boost = lv_arc_create(scr);
  lv_obj_set_size(abg_boost, 300, 300); lv_obj_center(abg_boost);
  lv_arc_set_bg_angles(abg_boost, 135, 405); lv_arc_set_value(abg_boost, 100);
  lv_obj_set_style_arc_color(abg_boost, lv_color_hex(C_RING_BG), LV_PART_MAIN);
  lv_obj_set_style_arc_width(abg_boost, 18, LV_PART_MAIN);
  lv_obj_set_style_arc_color(abg_boost, lv_color_hex(C_RING_BG), LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(abg_boost, 18, LV_PART_INDICATOR);
  lv_obj_set_style_bg_opa(abg_boost, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_remove_style(abg_boost, NULL, LV_PART_KNOB);
  lv_obj_clear_flag(abg_boost, LV_OBJ_FLAG_CLICKABLE);

  boost_arc = lv_arc_create(scr);
  lv_obj_set_size(boost_arc, 300, 300); lv_obj_center(boost_arc);
  lv_arc_set_bg_angles(boost_arc, 135, 405);
  lv_arc_set_range(boost_arc, BOOST_ARC_MIN, BOOST_ARC_MAX);
  lv_arc_set_value(boost_arc, 0);
  lv_obj_set_style_arc_color(boost_arc, lv_color_hex(0x111122), LV_PART_MAIN);
  lv_obj_set_style_arc_width(boost_arc, 1, LV_PART_MAIN);
  lv_obj_set_style_arc_color(boost_arc, lv_color_hex(C_PURPLE), LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(boost_arc, 18, LV_PART_INDICATOR);
  lv_obj_set_style_arc_rounded(boost_arc, true, LV_PART_INDICATOR);
  lv_obj_remove_style(boost_arc, NULL, LV_PART_KNOB);
  lv_obj_clear_flag(boost_arc, LV_OBJ_FLAG_CLICKABLE);

  // ── Innerer, paralleler Arc: OEL-TEMPERATUR ─────────────────
  lv_obj_t *abg_oil = lv_arc_create(scr);
  lv_obj_set_size(abg_oil, 226, 226); lv_obj_center(abg_oil);
  lv_arc_set_bg_angles(abg_oil, 135, 405); lv_arc_set_value(abg_oil, 100);
  lv_obj_set_style_arc_color(abg_oil, lv_color_hex(C_RING_BG), LV_PART_MAIN);
  lv_obj_set_style_arc_width(abg_oil, 16, LV_PART_MAIN);
  lv_obj_set_style_arc_color(abg_oil, lv_color_hex(C_RING_BG), LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(abg_oil, 16, LV_PART_INDICATOR);
  lv_obj_set_style_bg_opa(abg_oil, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_remove_style(abg_oil, NULL, LV_PART_KNOB);
  lv_obj_clear_flag(abg_oil, LV_OBJ_FLAG_CLICKABLE);

  oil_arc = lv_arc_create(scr);
  lv_obj_set_size(oil_arc, 226, 226); lv_obj_center(oil_arc);
  lv_arc_set_bg_angles(oil_arc, 135, 405);
  lv_arc_set_range(oil_arc, OIL_ARC_MIN, OIL_ARC_MAX);
  lv_arc_set_value(oil_arc, 0);
  lv_obj_set_style_arc_color(oil_arc, lv_color_hex(0x111122), LV_PART_MAIN);
  lv_obj_set_style_arc_width(oil_arc, 1, LV_PART_MAIN);
  lv_obj_set_style_arc_color(oil_arc, lv_color_hex(C_CYAN), LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(oil_arc, 16, LV_PART_INDICATOR);
  lv_obj_set_style_arc_rounded(oil_arc, true, LV_PART_INDICATOR);
  lv_obj_remove_style(oil_arc, NULL, LV_PART_KNOB);
  lv_obj_clear_flag(oil_arc, LV_OBJ_FLAG_CLICKABLE);

  // ── Ladedruck-Anzeige ────────────────────────────────────────
  lbl_boost_val = lv_label_create(scr);
  lv_label_set_text(lbl_boost_val, "--");
  lv_obj_set_style_text_font(lbl_boost_val, &lv_font_montserrat_40, 0);
  lv_obj_set_style_text_color(lbl_boost_val, lv_color_hex(C_PURPLE), 0);
  lv_obj_align(lbl_boost_val, LV_ALIGN_CENTER, 0, -34);

  lv_obj_t *lu = lv_label_create(scr);
  lv_label_set_text(lu, "bar (Ladedruck)");
  lv_obj_set_style_text_font(lu, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(lu, lv_color_hex(C_MUTED), 0);
  lv_obj_align(lu, LV_ALIGN_CENTER, 0, 4);

  // ── Oel-Temperatur-Anzeige ───────────────────────────────────
  lbl_oil_val = lv_label_create(scr);
  lv_label_set_text(lbl_oil_val, "--");
  lv_obj_set_style_text_font(lbl_oil_val, &lv_font_montserrat_32, 0);
  lv_obj_set_style_text_color(lbl_oil_val, lv_color_hex(C_CYAN), 0);
  lv_obj_align(lbl_oil_val, LV_ALIGN_CENTER, 0, 44);

  lv_obj_t *lu2 = lv_label_create(scr);
  lv_label_set_text(lu2, "Grad C (Oel)");
  lv_obj_set_style_text_font(lu2, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(lu2, lv_color_hex(C_MUTED), 0);
  lv_obj_align(lu2, LV_ALIGN_CENTER, 0, 76);

  lbl_boost_status = lv_label_create(scr);
  lv_label_set_text(lbl_boost_status, "MAP - Umgebungsdruck");
  lv_obj_set_style_text_font(lbl_boost_status, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(lbl_boost_status, lv_color_hex(C_MUTED), 0);
  lv_obj_align(lbl_boost_status, LV_ALIGN_CENTER, 0, 100);

  add_nav_dots(scr, PAGE_BOOST);
}

static void refresh_boost(void) {
  float boostBar;
  int oil;
  bool oilOk;
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  boostBar = gBoostBar;
  oil = gOilTemp;
  oilOk = gOilOK;
  xSemaphoreGive(dataMutex);

  // ── Ladedruck-Arc + Label ───────────────────────────────────
  char buf[16];
  snprintf(buf, sizeof(buf), "%+.2f", boostBar);
  lv_label_set_text(lbl_boost_val, buf);
  lv_obj_align(lbl_boost_val, LV_ALIGN_CENTER, 0, -34);

  int boostArcVal = (int)roundf(boostBar * 100.0f); // bar -> centibar
  boostArcVal = constrain(boostArcVal, BOOST_ARC_MIN, BOOST_ARC_MAX);
  lv_arc_set_value(boost_arc, boostArcVal);

  uint32_t boostCol = boostBar > 1.0f ? C_RED
                     : boostBar > 0.5f ? C_ORANGE
                     : boostBar < 0.0f ? C_CYAN   // Unterdruck = cyan
                     : C_PURPLE;
  lv_obj_set_style_arc_color(boost_arc, lv_color_hex(boostCol), LV_PART_INDICATOR);
  lv_obj_set_style_text_color(lbl_boost_val, lv_color_hex(boostCol), 0);

  // ── Oel-Temperatur-Arc + Label ───────────────────────────────
  // Farblogik: -20..60=CYAN | 60..80=YELLOW | 80..105=GREEN | >105=RED
  // Bei NO DATA (oilOk==false) => CYAN
  if (oilOk) {
    snprintf(buf, sizeof(buf), "%d", oil);
  } else {
    snprintf(buf, sizeof(buf), "N/A");
  }
  lv_label_set_text(lbl_oil_val, buf);
  lv_obj_align(lbl_oil_val, LV_ALIGN_CENTER, 0, 44);

  int oilArcVal = oilOk ? constrain(oil, OIL_ARC_MIN, OIL_ARC_MAX) : OIL_ARC_MIN;
  lv_arc_set_value(oil_arc, oilArcVal);

  uint32_t oilCol = oil_temp_color(oil, oilOk);
  lv_obj_set_style_arc_color(oil_arc, lv_color_hex(oilCol), LV_PART_INDICATOR);
  lv_obj_set_style_text_color(lbl_oil_val, lv_color_hex(oilCol), 0);
}

// ══════════════════════════════════════════════════════════════════
// SEITE 4: BESCHLEUNIGUNGSSENSOR (IMU / G-Meter)
// ══════════════════════════════════════════════════════════════════
static lv_obj_t *gm_dot_cur = nullptr;
static lv_obj_t *gm_dot_max = nullptr;
static lv_obj_t *gm_lbl_gx = nullptr;
static lv_obj_t *gm_lbl_gy = nullptr;
static lv_obj_t *gm_lbl_gtot = nullptr;
static lv_obj_t *gm_lbl_gmax = nullptr;

static void build_page_accel(void) {
  lv_obj_t *scr = make_screen();
  screens[PAGE_ACCEL] = scr;
  add_title(scr, LV_SYMBOL_CHARGE " G-FORCE METER", C_GREEN);

  if (!gMpuOK) {
    lv_obj_t *l = lv_label_create(scr);
    lv_label_set_text(l, LV_SYMBOL_WARNING " QMI8658 nicht gefunden");
    lv_obj_set_style_text_font(l, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(l, lv_color_hex(C_RED), 0);
    lv_obj_align(l, LV_ALIGN_CENTER, 0, 0);
    add_nav_dots(scr, PAGE_ACCEL);
    return;
  }

  lv_obj_t *c_outer = lv_obj_create(scr);
  lv_obj_set_size(c_outer, 260, 260);
  lv_obj_set_style_radius(c_outer, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(c_outer, lv_color_hex(0x1A1A2E), 0);
  lv_obj_set_style_border_color(c_outer, lv_color_hex(0x444466), 0);
  lv_obj_set_style_border_width(c_outer, 2, 0);
  lv_obj_set_style_pad_all(c_outer, 0, 0);
  lv_obj_align(c_outer, LV_ALIGN_CENTER, 0, -20);

  lv_obj_t *c_mid = lv_obj_create(scr);
  lv_obj_set_size(c_mid, 130, 130);
  lv_obj_set_style_radius(c_mid, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(c_mid, lv_color_hex(0x0F0F1E), 0);
  lv_obj_set_style_border_color(c_mid, lv_color_hex(0x333355), 0);
  lv_obj_set_style_border_width(c_mid, 1, 0);
  lv_obj_set_style_pad_all(c_mid, 0, 0);
  lv_obj_align(c_mid, LV_ALIGN_CENTER, 0, -20);

  lv_obj_t *cross_h = lv_obj_create(scr);
  lv_obj_set_size(cross_h, 254, 1);
  lv_obj_set_style_bg_color(cross_h, lv_color_hex(0x333355), 0);
  lv_obj_set_style_border_width(cross_h, 0, 0);
  lv_obj_align(cross_h, LV_ALIGN_CENTER, 0, -20);

  lv_obj_t *cross_v = lv_obj_create(scr);
  lv_obj_set_size(cross_v, 1, 254);
  lv_obj_set_style_bg_color(cross_v, lv_color_hex(0x333355), 0);
  lv_obj_set_style_border_width(cross_v, 0, 0);
  lv_obj_align(cross_v, LV_ALIGN_CENTER, 0, -20);

  const char *lbl_texts[] = {"BREMSEN", "GAS", "RECHTS", "LINKS"};
  const int lbl_dx[] = { 0, 0, 160, -160};
  const int lbl_dy[] = {-140, 100, -20, -20};
  for (int i=0; i<4; i++) {
    lv_obj_t *ll = lv_label_create(scr);
    lv_label_set_text(ll, lbl_texts[i]);
    lv_obj_set_style_text_font(ll, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(ll, lv_color_hex(0x555577), 0);
    lv_obj_align(ll, LV_ALIGN_CENTER, lbl_dx[i], lbl_dy[i]);
  }

  lv_obj_t *lg1 = lv_label_create(scr);
  lv_label_set_text(lg1, "1G");
  lv_obj_set_style_text_font(lg1, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(lg1, lv_color_hex(0x444466), 0);
  lv_obj_align(lg1, LV_ALIGN_CENTER, 93, -20);

  lv_obj_t *lg05 = lv_label_create(scr);
  lv_label_set_text(lg05, "0.5G");
  lv_obj_set_style_text_font(lg05, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(lg05, lv_color_hex(0x333355), 0);
  lv_obj_align(lg05, LV_ALIGN_CENTER, 43, -20);

  gm_dot_max = lv_obj_create(scr);
  lv_obj_set_size(gm_dot_max, 14, 14);
  lv_obj_set_style_radius(gm_dot_max, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(gm_dot_max, lv_color_hex(0xFFCC00), 0);
  lv_obj_set_style_bg_opa(gm_dot_max, LV_OPA_80, 0);
  lv_obj_set_style_border_width(gm_dot_max, 0, 0);
  lv_obj_align(gm_dot_max, LV_ALIGN_CENTER, 0, -20);

  gm_dot_cur = lv_obj_create(scr);
  lv_obj_set_size(gm_dot_cur, 20, 20);
  lv_obj_set_style_radius(gm_dot_cur, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(gm_dot_cur, lv_color_hex(0xFF3300), 0);
  lv_obj_set_style_border_color(gm_dot_cur, lv_color_hex(0xFF8866), 0);
  lv_obj_set_style_border_width(gm_dot_cur, 2, 0);
  lv_obj_set_style_shadow_width(gm_dot_cur, 12, 0);
  lv_obj_set_style_shadow_color(gm_dot_cur, lv_color_hex(0xFF3300), 0);
  lv_obj_align(gm_dot_cur, LV_ALIGN_CENTER, 0, -20);

  lv_obj_t *lx = lv_label_create(scr);
  lv_label_set_text(lx, "V/Z:");
  lv_obj_set_style_text_font(lx, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(lx, lv_color_hex(C_MUTED), 0);
  lv_obj_align(lx, LV_ALIGN_CENTER, -120, 127);

  gm_lbl_gx = lv_label_create(scr);
  lv_label_set_text(gm_lbl_gx, "+0.00G");
  lv_obj_set_style_text_font(gm_lbl_gx, &lv_font_montserrat_16, 0);
  lv_obj_set_style_text_color(gm_lbl_gx, lv_color_hex(0x00CCFF), 0);
  lv_obj_align(gm_lbl_gx, LV_ALIGN_CENTER, -65, 127);

  lv_obj_t *ly = lv_label_create(scr);
  lv_label_set_text(ly, "L/R:");
  lv_obj_set_style_text_font(ly, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(ly, lv_color_hex(C_MUTED), 0);
  lv_obj_align(ly, LV_ALIGN_CENTER, 50, 127);

  gm_lbl_gy = lv_label_create(scr);
  lv_label_set_text(gm_lbl_gy, "+0.00G");
  lv_obj_set_style_text_font(gm_lbl_gy, &lv_font_montserrat_16, 0);
  lv_obj_set_style_text_color(gm_lbl_gy, lv_color_hex(0x00FF88), 0);
  lv_obj_align(gm_lbl_gy, LV_ALIGN_CENTER, 105, 127);

  gm_lbl_gtot = lv_label_create(scr);
  lv_label_set_text(gm_lbl_gtot, "0.00G");
  lv_obj_set_style_text_font(gm_lbl_gtot, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(gm_lbl_gtot, lv_color_hex(C_WHITE), 0);
  lv_obj_align(gm_lbl_gtot, LV_ALIGN_CENTER, -50, 154);

  gm_lbl_gmax = lv_label_create(scr);
  lv_label_set_text(gm_lbl_gmax, LV_SYMBOL_UP " MAX: 0.00G");
  lv_obj_set_style_text_font(gm_lbl_gmax, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(gm_lbl_gmax, lv_color_hex(0xFFCC00), 0);
  lv_obj_align(gm_lbl_gmax, LV_ALIGN_CENTER, 0, 185);

  lv_obj_t *btn = lv_btn_create(scr);
  lv_obj_set_size(btn, 90, 28);
  lv_obj_set_style_bg_color(btn, lv_color_hex(0x222244), 0);
  lv_obj_set_style_radius(btn, 6, 0);
  lv_obj_align(btn, LV_ALIGN_CENTER, 60, 154);
  lv_obj_add_event_cb(btn, [](lv_event_t *e){
    gAxMax = 0; gAyMax = 0;
    if (gm_lbl_gmax) lv_label_set_text(gm_lbl_gmax, LV_SYMBOL_UP " MAX: 0.00G");
    if (gm_dot_max) lv_obj_align(gm_dot_max, LV_ALIGN_CENTER, 0, -20);
  }, LV_EVENT_CLICKED, nullptr);
  // Lang druecken (im stehenden Auto): aktuelle Lage als Nullpunkt uebernehmen
  lv_obj_add_event_cb(btn, [](lv_event_t *e){
    gOffLong = gAz; gOffLat = gAy;
    prefs.putFloat("offLong", gOffLong);
    prefs.putFloat("offLat", gOffLat);
    gAxMax = 0; gAyMax = 0;
    if (gm_lbl_gmax) lv_label_set_text(gm_lbl_gmax, "NULLPUNKT GESETZT");
  }, LV_EVENT_LONG_PRESSED, nullptr);
  lv_obj_t *btn_lbl = lv_label_create(btn);
  lv_label_set_text(btn_lbl, LV_SYMBOL_REFRESH " RESET");
  lv_obj_set_style_text_font(btn_lbl, &lv_font_montserrat_12, 0);
  lv_obj_center(btn_lbl);

  add_nav_dots(scr, PAGE_ACCEL);
}

static void refresh_accel(void) {
  if (!gMpuOK) return;
  if (!gm_dot_cur || !gm_lbl_gx) return;

  // FIX 1: FastIMU liefert AccelData bereits in "g" (nicht m/s^2) -
  // die vorherige Division durch 9.81f hat die Werte zusaetzlich
  // verkleinert -> Anzeige war um den Faktor ~9.81 (~10) zu klein.
  // FIX 2: Die Laengsachse (Bremsen/Gas) nutzt jetzt den Z-Sensor
  // (gAz) statt X (gAx), weil das Board vermutlich um 90 Grad um die
  // Links/Rechts-Achse (Y, war schon korrekt) gedreht verbaut ist -
  // die reale Vor/Zurueck-Beschleunigung erscheint dadurch auf der
  // Z-Achse des Sensors statt auf X.
  float ax_g = gAz - gOffLong;  // Laengsachse (Bremsen/Gas) - vorher: gAx / 9.81f
  float ay_g = gAy - gOffLat;   // Querachse  (Links/Rechts) - unveraendert, war schon korrekt

  float ax_abs = fabsf(ax_g);
  float ay_abs = fabsf(ay_g);
  bool new_max = false;
  if (ax_abs > fabsf(gAxMax)) { gAxMax = ax_g; new_max = true; }
  if (ay_abs > fabsf(gAyMax)) { gAyMax = ay_g; new_max = true; }

  const int center_x = 0;
  const int center_y = -20;
  const float scale = 130.0f;

  float mag = sqrtf(ax_g*ax_g + ay_g*ay_g);
  float cx = ay_g, cy = -ax_g;
  if (mag > 1.0f) { cx /= mag; cy /= mag; }
  int px = center_x + (int)(cx * scale);
  int py = center_y + (int)(cy * scale);
  lv_obj_align(gm_dot_cur, LV_ALIGN_CENTER, px, py);

  uint32_t dot_color;
  if (mag < 0.3f) dot_color = 0x00CC44;
  else if (mag < 0.6f) dot_color = 0xFF8800;
  else dot_color = 0xFF2200;
  lv_obj_set_style_bg_color(gm_dot_cur, lv_color_hex(dot_color), 0);
  lv_obj_set_style_shadow_color(gm_dot_cur, lv_color_hex(dot_color), 0);

  float mx = gAyMax, my = -gAxMax;  // keine /9.81f mehr, siehe FIX 1 oben
  float mmag = sqrtf(mx*mx + my*my);
  if (mmag > 1.0f) { mx /= mmag; my /= mmag; }
  lv_obj_align(gm_dot_max, LV_ALIGN_CENTER,
    center_x + (int)(mx * scale),
    center_y + (int)(my * scale));

  char buf[20];
  snprintf(buf, sizeof(buf), "%+.2fG", ax_g);
  lv_label_set_text(gm_lbl_gx, buf);

  snprintf(buf, sizeof(buf), "%+.2fG", ay_g);
  lv_label_set_text(gm_lbl_gy, buf);

  float total_g = sqrtf(ax_g*ax_g + ay_g*ay_g);
  snprintf(buf, sizeof(buf), "%.2fG", total_g);
  lv_label_set_text(gm_lbl_gtot, buf);

  if (new_max) {
    float max_total = sqrtf(gAxMax*gAxMax + gAyMax*gAyMax); // keine /9.81f mehr
    snprintf(buf, sizeof(buf), LV_SYMBOL_UP " MAX: %.2fG", max_total);
    lv_label_set_text(gm_lbl_gmax, buf);
  }
}

// ══════════════════════════════════════════════════════════════════
// OBD-TASK (läuft auf Core 0 — getrennt von LVGL auf Core 1)
// Blockierende UART-Kommunikation hier ist völlig ok,
// da LVGL auf Core 1 unabhängig weiterläuft.
// ══════════════════════════════════════════════════════════════════
static String elmSendCmd(const char *cmd, uint32_t ms = 2000) {
  while (elmSerial.available()) elmSerial.read();
  elmSerial.print(cmd);
  elmSerial.print('\r');
  String r = "";
  uint32_t t = millis();
  while (millis() - t < ms) {
    if (elmSerial.available()) {
      char c = elmSerial.read();
      if (c == '>') break;
      if (c == '\r') c = '\n';   // Zeilengrenzen behalten (fuer Multi-Frame-DTC)
      r += c;
    } else {
      vTaskDelay(1);  // CPU abgeben statt Busy-Wait (Idle-Task/Watchdog auf Core 0)
    }
  }
  r.trim();
  return r;
}

// Antwort des ELM327 selbst ist leer => Adapter antwortet nicht (abgezogen/tot)
// Antworten wie NO DATA / UNABLE TO CONNECT => ELM ok, aber Steuergeraet schweigt
static bool elmIsEcuError(const String &r) {
  return r.indexOf("NO DATA") >= 0 || r.indexOf("UNABLE") >= 0 ||
         r.indexOf("ERROR") >= 0   || r.indexOf("STOPPED") >= 0 ||
         r.indexOf("?") >= 0;
}

static bool elmParseOBD(const String &resp, const char *tag,
                         uint8_t &A, uint8_t &B) {
  for (int p = 0; p < 2; p++) {
    String n = String(tag);
    if (p == 1) n = n.substring(0,2) + " " + n.substring(2,4);
    int idx = resp.indexOf(n);
    if (idx < 0) continue;
    String d = resp.substring(idx + n.length());
    d.trim(); d.replace(" ",""); d.replace("\n","");
    if (d.length() < 2 || !isHexadecimalDigit(d[0]) || !isHexadecimalDigit(d[1])) continue;
    if (d.length() < 2) continue;
    A = (uint8_t)strtol(d.substring(0,2).c_str(), nullptr, 16);
    B = (d.length() >= 4)
        ? (uint8_t)strtol(d.substring(2,4).c_str(), nullptr, 16) : 0;
    return true;
  }
  return false;
}

static void elmInit(void) {
  String ver = elmSendCmd("ATZ", 3000);
  if (!ver.length()) {
    Serial.println("[ELM] keine Antwort");
    return;
  }

  Serial.print("[ELM] "); Serial.println(ver);
  elmSendCmd("ATE0"); elmSendCmd("ATL0"); elmSendCmd("ATS0");
  elmSendCmd("ATH0"); elmSendCmd("ATSP0"); elmSendCmd("ATAT1");

  // Erste Anfrage loest die automatische Protokollsuche aus (kann dauern),
  // danach das erkannte Protokoll abfragen: 6..9 = CAN (A..C ebenfalls CAN).
  elmSendCmd("0100", 8000);
  String dp = elmSendCmd("ATDPN", 1000);
  dp.replace("A", "");   // "A6" = automatisch gewaehlt, Protokoll 6
  if (dp.length() > 0) {
    char p = dp[dp.length() - 1];
    gIsCan = (p >= '6' && p <= '9') || (p >= 'A' && p <= 'C');
  }
  Serial.printf("[ELM] Protokoll %s (%s)\n", dp.c_str(), gIsCan ? "CAN" : "kein CAN");

  xSemaphoreTake(dataMutex, portMAX_DELAY);
  gElmOK = true;
  xSemaphoreGive(dataMutex);
  Serial.println("[ELM] OK");
}

// ── DTC-Auswertung (Mode 03) ────────────────────────────────────────
// Reine C-Funktion ohne Arduino-Abhaengigkeiten (auf dem PC testbar).
// resp: ELM-Antwort, Zeilen durch '\n' getrennt, mit/ohne Leerzeichen.
//   CAN, 1 Frame:      "43 02 01 23 04 56"          (Byte nach 43 = Anzahl)
//   CAN, Multi-Frame:  "00A\n0: 43 04 01 23 04 56\n1: 07 89 0A BC 00 00"
//   K-Line/J1850:      "43 01 23 04 56 00 00"       (je Zeile 3 Codes, 0000 = leer)
// Liefert Anzahl Codes, oder -1 wenn keine gueltige Antwort.
static void dtc_format(uint8_t b1, uint8_t b2, char *out) {
  static const char pfx[] = {'P', 'C', 'B', 'U'};
  // Format: Buchstabe + 4 Hex-Stellen, z.B. 0x01 0x23 -> "P0123"
  snprintf(out, 6, "%c%02X%02X", pfx[(b1 >> 6) & 0x03], b1 & 0x3F, b2);
}

static int hexval(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

static int dtc_parse(const char *resp, bool isCan, char out[][6], int maxOut) {
  // 1) In Nachrichten zerlegen: je Nachricht nur Hex-Zeichen sammeln
  const int MAXMSG = 8, MAXHEX = 128;
  char msgs[MAXMSG][MAXHEX + 1];
  int nMsg = 0;
  bool any = false;

  const char *p = resp;
  while (*p) {
    const char *eol = strchr(p, '\n');
    if (!eol) eol = p + strlen(p);

    char line[MAXHEX + 1]; int ll = 0;
    for (const char *q = p; q < eol && ll < MAXHEX; q++)
      if (*q != ' ') line[ll++] = *q;
    line[ll] = 0;
    p = (*eol) ? eol + 1 : eol;

    if (ll == 0) continue;
    if (strstr(line, "NODATA")) { any = true; continue; }  // keine Codes gespeichert

    bool cont = false;
    const char *data = line;
    if (ll >= 2 && line[1] == ':' && hexval(line[0]) >= 0) {   // "N:" Multi-Frame
      cont = (line[0] != '0');
      data = line + 2;
    } else {
      bool allHex = true;
      for (int i = 0; i < ll; i++) if (hexval(line[i]) < 0) { allHex = false; break; }
      if (!allHex) continue;                 // z.B. "SEARCHING..."
      if (ll <= 3) continue;                 // Multi-Frame-Laengenzeile "00A"
    }

    if (!cont) {
      if (strncmp(data, "43", 2) != 0 || nMsg >= MAXMSG) continue;
      msgs[nMsg][0] = 0;
      nMsg++;
    } else if (nMsg == 0) {
      continue;
    }
    strncat(msgs[nMsg - 1], data, MAXHEX - strlen(msgs[nMsg - 1]));
  }

  // 2) Nachrichten dekodieren
  int count = 0;
  for (int m = 0; m < nMsg; m++) {
    const char *h = msgs[m] + 2;   // hinter "43"
    int len = strlen(h);
    int num;
    if (isCan) {
      if (len < 2) continue;
      num = hexval(h[0]) * 16 + hexval(h[1]);
      h += 2; len -= 2;
    } else {
      num = len / 4;
    }
    any = true;
    for (int i = 0; i < num && len >= 4 && count < maxOut; i++, h += 4, len -= 4) {
      int a1 = hexval(h[0]), a2 = hexval(h[1]), a3 = hexval(h[2]), a4 = hexval(h[3]);
      if (a1 < 0 || a2 < 0 || a3 < 0 || a4 < 0) break;
      uint8_t b1 = a1 * 16 + a2, b2 = a3 * 16 + a4;
      if (b1 == 0 && b2 == 0) continue;
      char code[6];
      dtc_format(b1, b2, code);
      bool dup = false;   // mehrere Steuergeraete koennen denselben Code melden
      for (int k = 0; k < count; k++) if (!strcmp(out[k], code)) { dup = true; break; }
      if (!dup) strcpy(out[count++], code);
    }
  }
  return any ? count : -1;
}

static void elmFetchDTC(void) {
  String resp = elmSendCmd("03", 5000);
  char codes[DTC_MAX][6];
  int n = resp.length() ? dtc_parse(resp.c_str(), gIsCan, codes, DTC_MAX) : -1;
  Serial.printf("[DTC] Antwort '%s' -> %d\n", resp.c_str(), n);

  xSemaphoreTake(dataMutex, portMAX_DELAY);
  if (n >= 0) {
    dtcCount = n;
    for (int i = 0; i < n; i++) dtcList[i] = String(codes[i]);
    gDtcState = DTC_DONE;
  } else {
    gDtcState = DTC_ERROR;
  }
  gNewDTC = true;
  xSemaphoreGive(dataMutex);
}

// Fehlerzaehler (nur im OBD-Task benutzt)
static uint8_t sElmSilent = 0;  // aufeinanderfolgende Kommandos ganz ohne Antwort
static uint8_t sEcuFail = 0;    // aufeinanderfolgende PID-Abfragen ohne gueltige Daten

// PID abfragen + Verbindungsstatistik fuehren
static bool obdQuery(const char *cmd, const char *tag, uint8_t &a, uint8_t &b) {
  String r = elmSendCmd(cmd, 2500);
  if (r.length() == 0) {
    if (sElmSilent < 255) sElmSilent++;
    return false;
  }
  sElmSilent = 0;
  if (elmParseOBD(r, tag, a, b)) {
    sEcuFail = 0;
    return true;
  }
  if (elmIsEcuError(r) && sEcuFail < 255) sEcuFail++;
  return false;
}

#define SLOW_MS       1000UL   // Batterie, Kuehlwasser, Oel
#define AMB_MS       10000UL   // Umgebungsdruck
#define OIL_MAX_FAIL  5
#define OIL_RETRY_MS 30000UL

static void obdTask(void *param) {
  elmSerial.begin(ELM_BAUD, SERIAL_8N1, ELM_RX, ELM_TX);
  delay(1000);
  elmInit();

  uint32_t tSlow = 0, tAmb = 0, tOilRetry = 0;
  uint8_t oilFail = 0;

  for (;;) {
    if (gFetchDTC) {
      gFetchDTC = false;
      elmFetchDTC();
    }

    bool ok;
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    ok = gElmOK;
    xSemaphoreGive(dataMutex);

    if (!ok) {
      delay(5000);
      elmInit();
      sElmSilent = 0;
      sEcuFail = 0;
      continue;
    }

    uint8_t a=0, b=0;
    float batt = gBatt;
    int rpm = gRPM;
    int cool = gCoolant;
    int oil = gOilTemp;
    bool oilOk = gOilOK;

    float mapKPa = gMapKPa;
    float ambKPa = gAmbientKPa;
    float boostBar = gBoostBar;

    uint32_t now = millis();

    // ── Schnelle Werte: JEDEN Zyklus (Drehzahl, Ladedruck) ─────────
    if (obdQuery("010C", "410C", a, b))
      rpm = ((int)a * 256 + b) / 4;

    // Ladedruck: MAP (Drosselklappendruck, PID 0x0B) minus Umgebungsdruck (PID 0x33)
    if (obdQuery("010B", "410B", a, b))
      mapKPa = (float)a; // PID 0x0B liefert Druck direkt in kPa (0-255)

    // ── Langsame Werte: ca. 1x pro Sekunde ─────────────────────────
    if (now - tSlow >= SLOW_MS) {
      tSlow = now;

      String v = elmSendCmd("ATRV", 2000);
      if (v.length() == 0) { if (sElmSilent < 255) sElmSilent++; }
      else {
        sElmSilent = 0;
        float f = v.toFloat();   // "12.6V" -> 12.6
        if (f > 1.0f) batt = f;
      }

      if (obdQuery("0105", "4105", a, b))
        cool = (int)a - 40;

      // Oel-Temperatur (PID 0x5C): viele Autos kennen diese PID nicht und
      // antworten jedes Mal mit NO DATA. Nach OIL_MAX_FAIL Fehlversuchen
      // nur noch alle OIL_RETRY_MS erneut probieren (keine Dauersperre).
      bool oilDue = (oilFail < OIL_MAX_FAIL) || (now - tOilRetry >= OIL_RETRY_MS);
      if (oilDue) {
        if (obdQuery("015C", "415C", a, b)) {
          oil = (int)a - 40;
          oilOk = true;
          oilFail = 0;
        } else {
          oilOk = false;
          if (oilFail < 255) oilFail++;
          tOilRetry = now;
        }
      }
    }

    // ── Umgebungsdruck aendert sich kaum: alle 10 s ────────────────
    if (tAmb == 0 || now - tAmb >= AMB_MS) {
      tAmb = now;
      if (obdQuery("0133", "4133", a, b) && a > 50)
        ambKPa = (float)a; // PID 0x33 = absoluter Umgebungs-/Barometerdruck in kPa
    }

    boostBar = (mapKPa - ambKPa) / 100.0f; // kPa -> bar
    if (boostBar < -0.5f) boostBar = -0.5f;
    if (boostBar > 1.5f)  boostBar = 1.5f;

    // ── Verbindungsstatus ─────────────────────────────────────────
    bool elmOk = sElmSilent < 3;          // 3x gar keine Antwort => Adapter weg
    bool ecuOk = elmOk && sEcuFail < 4;   // ELM da, aber Steuergeraet schweigt
    if (!elmOk) Serial.println("[ELM] keine Antwort mehr -> Neuinitialisierung");

    xSemaphoreTake(dataMutex, portMAX_DELAY);
    gBatt = batt;
    gRPM = rpm;
    gCoolant = cool;
    gOilTemp = oil;
    gMapKPa = mapKPa;
    gAmbientKPa = ambKPa;
    gBoostBar = boostBar;
    gOilOK = oilOk;
    gElmOK = elmOk;
    gEcuOK = ecuOk;
    gNewData = true;
    xSemaphoreGive(dataMutex);

    delay(20); // kurze Pause, dann nächster Zyklus
  }
}

// ══════════════════════════════════════════════════════════════════
// SETUP
// ══════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000);
  delay(200);
  Serial.println("\n=== OBD-II v7 ===");

  if (!psramFound()) {
    Serial.println("PSRAM fehlt!"); while (1) delay(1000);
  }

  dataMutex = xSemaphoreCreateMutex();

  // ── 1. I2C starten ────────────────────────────────────────
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000); // 100kHz, stabiler als 400kHz
  delay(50);

  // ── 2. TCA9554 ZUERST — setzt Touch-Reset-Pin HIGH ────────
  TCA9554PWR_Init(0x00);
  Set_EXIOS(0xFF);
  delay(50);
  Set_EXIO(EXIO_PIN8, Low); // Buzzer aus

  // ── 3. I2C Scanner ────────────────────────────────────────
  printf("[I2C] Scan...\n");
  Serial.println("[I2C] Scan...");
  Serial.flush();
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.printf(" -> 0x%02X gefunden\n", addr);
      printf(" -> 0x%02X gefunden\n", addr);
      found++;
    }
  }
  Serial.printf("[I2C] %d Geraet(e)\n", found);
  Serial.flush();
  printf("[I2C] %d Geraet(e)\n", found);

  // ── 4. QMI8658 via FastIMU ────────────────────────────────
  gMpuOK = false;
  int imuErr = imu.init(calib, 0x6B);
  if (imuErr == 0) {
    gMpuOK = true;
    Serial.println("[IMU] QMI8658 OK auf 0x6B");
    printf("[IMU] QMI8658 OK\n");
  } else {
    imuErr = imu.init(calib, 0x6A);
    if (imuErr == 0) {
      gMpuOK = true;
      Serial.println("[IMU] QMI8658 OK auf 0x6A");
      printf("[IMU] QMI8658 OK (0x6A)\n");
    } else {
      Serial.printf("[IMU] NICHT GEFUNDEN (err=%d)\n", imuErr);
      printf("[IMU] NICHT GEFUNDEN\n");
    }
  }
  Serial.flush();

  // ── 5. Display initialisieren ─────────────────────────────
  LCD_Init();
  prefs.begin("obd", false);
  gBrightness = constrain(prefs.getUChar("bright", 80), 10, 100);
  Set_Backlight(gBrightness);
  gOffLong = prefs.getFloat("offLong", 0.0f);
  gOffLat  = prefs.getFloat("offLat", 0.0f);
  Serial.println("[LCD] OK");

  pinMode(CST820_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CST820_INT_PIN), touchISR, FALLING);

  lv_init();
  lv_tick_set_cb(lvgl_tick_cb);

  draw_buf_1 = (lv_color_t*)heap_caps_malloc(
    DRAW_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  draw_buf_2 = (lv_color_t*)heap_caps_malloc(
    DRAW_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

  display = lv_display_create(DISP_W, DISP_H);
  lv_display_set_flush_cb(display, lvgl_flush_cb);
  lv_display_set_buffers(display, draw_buf_1, draw_buf_2,
    DRAW_BUF_SIZE * sizeof(lv_color_t),
    LV_DISPLAY_RENDER_MODE_PARTIAL);

  lv_indev_t *indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, lvgl_touch_cb);

  build_page_dtc();
  build_page_main();
  build_page_brightness();
  build_page_boost();
  build_page_accel();

  lv_screen_load(screens[PAGE_MAIN]);
  lv_task_handler();

  xTaskCreatePinnedToCore(
    obdTask,
    "OBD",
    8192,
    nullptr,
    1,
    nullptr,
    0
  );

  Serial.println("[SETUP] fertig");
}

// ══════════════════════════════════════════════════════════════════
// LOOP — läuft auf Core 1, NUR LVGL + Display-Updates
// ══════════════════════════════════════════════════════════════════
static uint32_t lastMPU = 0;
static uint32_t lastAccelUi = 0;

void loop() {
  lv_task_handler();

  bool newData = false;
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  if (gNewData) { newData = true; gNewData = false; }
  xSemaphoreGive(dataMutex);

  if (newData) {
    if (curPage == PAGE_MAIN) refresh_main();
    if (curPage == PAGE_BOOST) refresh_boost();
  }

  bool newDTC = false;
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  if (gNewDTC) { newDTC = true; gNewDTC = false; }
  xSemaphoreGive(dataMutex);
  if (newDTC && curPage == PAGE_DTC) refresh_dtc_ui();

  // IMU mit 50 Hz lesen und glaetten (Motor-/Strassenvibrationen),
  // Anzeige aber nur mit 10 Hz aktualisieren
  if (gMpuOK && millis() - lastMPU >= 20) {
    AccelData accelData;
    imu.update();
    imu.getAccel(&accelData);
    const float k = 0.2f;   // Glaettungsfaktor (kleiner = ruhiger, traeger)
    gAx += k * (accelData.accelX - gAx);
    gAy += k * (accelData.accelY - gAy);
    gAz += k * (accelData.accelZ - gAz);
    lastMPU = millis();
  }
  if (gMpuOK && curPage == PAGE_ACCEL && millis() - lastAccelUi >= 100) {
    refresh_accel();
    lastAccelUi = millis();
  }

  delay(5); // 5ms -> ~200 LVGL-Ticks/s, Touch reagiert fluessig
}
