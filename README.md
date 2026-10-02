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
  - **LVGL** 9.x (verwendet: 9.5; getestet im Simulator mit 9.0 – 9.5)
  - **FastIMU** (1.2.6, für den QMI8658)
- `lv_conf.h` liegt im Sketch-Ordner (siehe Kommentar in der Datei)
- Die Arduino IDE verlangt, dass der Ordner genauso heißt wie der Sketch: `OBD.ino` muss in einem Ordner `OBD` liegen

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
| **Ladedruck / Öl** | Ladedruck = Saugrohrdruck (PID 0x0B) − Umgebungsdruck (PID 0x33), Anzeige −0,5 … +1,5 bar. Innerer Bogen: Öltemperatur. Knopf **OEL-SUCHE** öffnet die Öl-Suche (siehe unten) |
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
| Öltemperatur | gespeicherte VAG-DID per `22xxxx` (aus der Öl-Suche), sonst 015C | 1 s; 015C nach 5 Fehlversuchen nur noch alle 30 s |
| Umgebungsdruck | 0133 | 10 s (Startwert 101,3 kPa, bis der erste Messwert da ist) |

### Hinweise zum Fabia 3 / 1.0 TSI

- Skoda/VW liefert die **Öltemperatur** über die Standard-PID 0x5C nicht (beim Fabia bestätigt: auch ScanMaster findet sie nicht). Ohne gespeicherte DID zeigt die Kachel `N/A`, und die Abfrage wird automatisch seltener wiederholt. Abhilfe: die **Öl-Suche**.
- Laut ScanMaster: ISO 15765-4 CAN 11 Bit / 500 kbit, Motorsteuergerät antwortet auf `7E8` (angefragt über `7E0`).
- Das Protokoll wird beim Start automatisch erkannt (`ATSP0`, dann `ATDPN`). Beim Fabia sollte CAN erkannt werden. Die Fehlercode-Auswertung kann auch Antworten verarbeiten, die über mehrere CAN-Frames verteilt kommen.
- Wenn der Adapter nicht mehr antwortet (3 leere Antworten hintereinander), wird er automatisch neu initialisiert.

## Öl-Suche (herstellerspezifische Öltemperatur)

Das Motorsteuergerät kennt die Öltemperatur, aber nur unter einer herstellerspezifischen
Kennnummer (DID), abfragbar mit UDS Mode 22 (`22 xxxx`). Eine öffentliche Liste gibt es nicht,
deshalb sucht das Programm sie selbst. **Mode 22 liest nur**, am Steuergerät wird nichts verändert.

1. **Kalt-Scan:** Motor kalt starten, Auto steht. Ladedruckseite → **OEL-SUCHE** → **KALT-SCAN**.
   Fragt die DIDs `1000–2FFF` ab (ca. 5–7 min) und merkt sich alle, die antworten.
   **Lang drücken** startet die Vollsuche `0000–FFFF` (ca. 45 min).
   Die normale Anzeige ist währenddessen pausiert. Die Treffer werden gespeichert, Zündung aus ist danach kein Problem.
2. **Warm fahren**, anhalten, Öl-Suche öffnen → **WARM-VERGL.**
   Liest alle Treffer erneut und rechnet sie mit den gängigen Temperaturformeln um
   (`A-40`, `A*0.75-48`, Kelvin/10, °C/10). Kandidaten: kalt −30…70 °C, warm 60…150 °C, mindestens 20 K wärmer.
3. **Kandidaten prüfen:** Die Liste zeigt `DID Formel kalt>warm jetzt …` mit Live-Wert.
   `=KW` heißt: verhält sich genau wie das Kühlwasser, ist also wahrscheinlich nicht das Öl.
   Öl wird langsamer warm und liegt im Fahrbetrieb meist etwas über dem Kühlwasser.
4. **Kandidat lang drücken** = speichern. Ab dann kommt die Öltemperatur von dieser DID
   (mit `*` markiert). Ist eine DID gespeichert und keine Kandidatenliste da, kann man sie dort
   per langem Druck wieder löschen.

Technisch: Für Mode 22 wird das Motorsteuergerät direkt adressiert (`ATSH7E0`). Ist eine DID
gespeichert, bleibt das für alle Live-Werte so; nur zum Fehlercode-Lesen wird kurz auf Rundruf
(`ATSH7DF`) umgeschaltet. Die Serial-Ausgabe (`[SCAN] ...`) listet alle Treffer und Kandidaten mit.

## Dateien

| Datei | Inhalt |
|---|---|
| `OBD.ino` | Hauptprogramm: Oberfläche, OBD-Task, IMU |
| `OilScan.h` | Logik der Öl-Suche (UDS-Antworten, Formeln, Kandidaten), ohne Arduino-Abhängigkeiten |
| `Display_ST7701.*` | Display-Initialisierung (SPI + RGB-Panel) und Hintergrundbeleuchtung |
| `Touch_CST820.*` | Touch-Controller inkl. Hardware-Gestenerkennung |
| `TCA9554PWR.*` | IO-Expander (LCD-/Touch-Reset, LCD-CS, Summer) |
| `I2C_Driver.*` | I2C-Hilfsfunktionen |
| `lv_conf.h` | LVGL-Konfiguration |
| `tools/sim/` | UI-Simulator für den PC (siehe unten) |
| `tools/tests/` | PC-Test der Öl-Such-Logik (Befehl steht oben in der Datei) |

## UI-Simulator (am PC testen)

Unter `tools/sim/` liegt ein Simulator, der die Seiten mit LVGL 9.5 am PC rendert und prüft,
ob Texte oder Buttons über den runden Displayrand hinausragen:

```bash
tools/sim/build.sh
```

Details siehe [tools/sim/README.md](tools/sim/README.md).

## Offene Ideen

- Fehlercodes löschen (Mode 04), am besten mit Sicherheitsabfrage
- `lv_conf.h` auf das LVGL-9-Format umstellen (einige Einträge stammen noch aus LVGL 8 und werden ignoriert)
