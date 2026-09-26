# Autoanzeige

Auslesen von OBD-II-Werten im Auto und Anzeige auf einem runden 480×480-Touchdisplay.

Gedacht für einen **Skoda Fabia 3 mit 1.0 TSI** (CAN-Bus).

## Hardware

| Teil | Details |
|---|---|
| Board | Waveshare **ESP32-S3-Touch-LCD-2.1** (ST7701-Display, CST820-Touch, TCA9554-IO-Expander) |
| OBD-Adapter | ELM327 an UART1: GPIO43 (TX) / GPIO44 (RX), 38400 Baud, über MAX232A |
| Beschleunigungssensor | QMI8658 (auf dem Board), I2C: GPIO7 (SCL) / GPIO15 (SDA), Adresse 0x6B bzw. 0x6A |

## Software / Build

- Arduino IDE mit ESP32-Core 3.x
- Board-Einstellungen: **ESP32S3 Dev Module**, **PSRAM: OPI**, **USB CDC On Boot: Enabled**
- Bibliotheken:
  - **LVGL** ≥ 9.1
  - **FastIMU** (1.2.6, für den QMI8658)
- `lv_conf.h` liegt im Sketch-Ordner (siehe Kommentar in der Datei)

Architektur: Der OBD-Task läuft auf Core 0 (blockierende UART-Kommunikation), LVGL und die Anzeige auf Core 1. Die Messwerte werden über einen Mutex ausgetauscht.

## Seiten und Bedienung

```
                 [Helligkeit]
                      ↑
[Ladedruck/Öl]  ←  [HAUPTSEITE]  →  [G-Meter]
                      ↓
                [Fehlercodes]
```

Von der Hauptseite aus in die gewünschte Richtung wischen. Von jeder Unterseite aus bringt beliebiges Wischen zurück zur Hauptseite. Auf der Helligkeitsseite geht das nur senkrecht, weil waagerechtes Wischen den Slider bedient.

| Seite | Inhalt |
|---|---|
| **Hauptseite** | Batteriespannung, Öltemperatur, Kühlwasser, Drehzahl. In der Mitte der Status: `OBD OK` (alles gut), `ECU?` (Adapter ok, Auto antwortet nicht, z. B. Zündung aus), `ELM?` (Adapter antwortet nicht) |
| **Ladedruck / Öl** | Ladedruck = Saugrohrdruck (PID 0x0B) − Umgebungsdruck (PID 0x33), Anzeige −0,5 … +1,5 bar. Innerer Bogen: Öltemperatur |
| **Fehlercodes** | „AUSLESEN“ tippen, dann werden die gespeicherten Fehlercodes (Mode 03) angezeigt, z. B. `P0123` |
| **Helligkeit** | Slider und Presets 25/50/75/100 %. Die Einstellung bleibt nach einem Neustart erhalten |
| **G-Meter** | Längs- und Querbeschleunigung mit Max-Punkt. **RESET tippen** löscht die Max-Werte. **RESET lange drücken** (Auto steht still) setzt den Nullpunkt, um einen schrägen Einbau auszugleichen. Der Nullpunkt wird gespeichert |

## Abgefragte Werte

| Wert | Befehl / PID | Intervall |
|---|---|---|
| Drehzahl | 010C | jeder Zyklus |
| Saugrohrdruck (MAP) | 010B | jeder Zyklus |
| Batteriespannung | ATRV (misst der ELM327 selbst) | 1 s |
| Kühlwasser | 0105 | 1 s |
| Öltemperatur | 015C | 1 s; nach 5 Fehlversuchen nur noch alle 30 s |
| Umgebungsdruck | 0133 | 10 s (Startwert 101,3 kPa, bis der erste Messwert da ist) |

### Hinweise zum Fabia 3 / 1.0 TSI

- Skoda/VW liefert die **Öltemperatur** über die Standard-PID 0x5C meist nicht. Dann zeigt die Kachel `N/A`, und die Abfrage wird automatisch seltener wiederholt, damit Drehzahl und Ladedruck schneller aktualisiert werden.
- Das Protokoll wird beim Start automatisch erkannt (`ATSP0`, dann `ATDPN`). Beim Fabia sollte CAN erkannt werden. Die Fehlercode-Auswertung kann auch Antworten verarbeiten, die über mehrere CAN-Frames verteilt kommen.
- Wenn der Adapter nicht mehr antwortet (3 leere Antworten hintereinander), wird er automatisch neu initialisiert.

## Dateien

| Datei | Inhalt |
|---|---|
| `OBD_LVGL_v7.ino` | Hauptprogramm: Oberfläche, OBD-Task, IMU |
| `Display_ST7701.*` | Display-Initialisierung (SPI + RGB-Panel) und Hintergrundbeleuchtung |
| `Touch_CST820.*` | Touch-Controller inkl. Hardware-Gestenerkennung |
| `TCA9554PWR.*` | IO-Expander (LCD-/Touch-Reset, LCD-CS, Summer) |
| `I2C_Driver.*` | I2C-Hilfsfunktionen |
| `lv_conf.h` | LVGL-Konfiguration |
| `tools/sim/` | UI-Simulator für den PC (siehe unten) |

## UI-Simulator (am PC testen)

Unter `tools/sim/` liegt ein Simulator, der die Seiten mit LVGL am PC rendert und prüft,
ob Texte oder Buttons über den runden Displayrand hinausragen:

```bash
tools/sim/build.sh
```

Details siehe [tools/sim/README.md](tools/sim/README.md).

## Offene Ideen

- Fehlercodes löschen (Mode 04), am besten mit Sicherheitsabfrage
- VAG-spezifische Öltemperatur über Mode 22 (UDS) statt über die Standard-PID
- `lv_conf.h` auf das LVGL-9-Format umstellen (einige Einträge stammen noch aus LVGL 8 und werden ignoriert)
