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
- **Alle Dateien aus dem Repo** gehören in diesen Ordner (nicht nur `OBD.ino`): `OilScan.h`, `font_boost_*.h`, die Treiber `*.cpp/*.h`. Am einfachsten: Repo als ZIP herunterladen (GitHub → Code → Download ZIP) und den Inhalt nach `OBD\` entpacken

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
| **Hauptseite** | Batteriespannung, Öltemperatur, Katalysatortemperatur (cyan < 300 °C, grün, orange > 850 °C, rot > 950 °C), Zündwinkel (° vor OT; orange wenn negativ = nach OT). Drehzahl und Kühlwasser zeigt schon das Kombiinstrument; sie werden intern weiter gelesen (Schaltblitz, Öl-Suche). In der Mitte der Status: `OBD OK` (alles gut), `ECU?` (Adapter ok, Auto antwortet nicht, z. B. Zündung aus), `ELM?` (Adapter antwortet nicht) |
| **Ladedruck / Öl** | Großer Ladedruck-Ring außen, Öl-Ring innen, in der Mitte **BOOST** mit Wert in fetter Schrift. Ladedruck = Saugrohrdruck (PID 0x87, sonst 0x0B) − Umgebungsdruck (PID 0x33), Anzeige −0,6 … +2,0 bar. Innerer Bogen: Öltemperatur. Unter dem Ladedruck klein nebeneinander: **Öl** (links) und **Ansaugluft** (rechts; cyan, gelb ab 50 °C, rot ab 65 °C). **Schaltblitz:** ab 3500 U/min blinkt ein roter Ring am Rand (5× pro Sekunde; Schwelle `SHIFT_RPM` in `OBD.ino`). Knopf **OEL-SUCHE** öffnet die Öl-Suche (siehe unten) |
| **Fehlercodes** | „AUSLESEN“ tippen, dann werden die gespeicherten Fehlercodes (Mode 03) angezeigt, z. B. `P0123` |
| **Helligkeit** | Slider und Presets 25/50/75/100 %. Die Einstellung bleibt nach einem Neustart erhalten |
| **G-Meter** | Längs- und Querbeschleunigung mit Max-Punkt. **RESET tippen** löscht die Max-Werte. **RESET lange drücken** (Auto steht still) setzt den Nullpunkt, um einen schrägen Einbau auszugleichen. Der Nullpunkt wird gespeichert |

## Abgefragte Werte

| Wert | Befehl / PID | Intervall |
|---|---|---|
| Drehzahl | 010C (für den Schaltblitz) | jeder Zyklus |
| Saugrohrdruck (MAP) | 0187 (1/32 kPa, bis 2048 kPa), falls das Auto es kennt; sonst 010B (1 Byte, max. 255 kPa ≈ +1,55 bar Ladedruck) | jeder Zyklus |
| Batteriespannung | ATRV (misst der ELM327 selbst) | 1 s |
| Kühlwasser | 0105 (für die Öl-Suche) | 1 s |
| Zündwinkel | 010E: A/2 − 64 ° vor OT | 250 ms |
| Ansauglufttemperatur | 010F: A − 40 °C | 1 s |
| Katalysatortemperatur | 013C (Bank 1, Sensor 1): (A·256+B)/10 − 40 °C | 1 s |
| Öltemperatur | `2211BD` an das Motorsteuergerät (7E0), Formel (A·256+B)/10 − 273,1 (Kelvin/10). Fest eingebaut; eine per Öl-Suche gespeicherte andere DID hat Vorrang | 1 s |
| Umgebungsdruck | 0133 | 10 s (Startwert 101,3 kPa, bis der erste Messwert da ist) |

Liefert das Auto einen Wert nicht (Antwort `NO DATA`), zeigt die Kachel `N/A`; Kat-Temperatur, Zündwinkel und Ansaugluft werden dann nach 5 Fehlversuchen nur noch alle 30 s erneut versucht.

### Hinweise zum Fabia 3 / 1.0 TSI

- Skoda/VW liefert die **Öltemperatur** nicht über die Standard-PID 0x5C. Beim Fabia steckt sie im Motorsteuergerät unter **DID `11BD`** (Kelvin/10), per Öl-Suche gefunden. Dieser Wert ist fest eingebaut.
- Laut ScanMaster: ISO 15765-4 CAN 11 Bit / 500 kbit, Motorsteuergerät antwortet auf `7E8` (angefragt über `7E0`).
- Das Protokoll wird automatisch erkannt (`ATSP0`, dann `ATDPN`). Startet die Anzeige vor der Zündung, ist es zunächst unbekannt und wird nachgeholt, sobald das Auto antwortet. Erst dann wird auf `7E0` umgeschaltet (vorher blieb die Öltemperatur in diesem Fall bis zum Neustart auf `N/A`). Die Fehlercode-Auswertung kann auch Antworten verarbeiten, die über mehrere CAN-Frames verteilt kommen.
- Wenn der Adapter nicht mehr antwortet (3 leere Antworten hintereinander), wird er automatisch neu initialisiert.

## Schlafmodus (dauerhaft am OBD-Stecker)

Damit die Anzeige am OBD-Stecker bleiben kann, ohne die Batterie leerzuziehen:

- **Einschlafen**, wenn alle Bedingungen erfüllt sind:
  - das Steuergerät antwortet **3 Minuten** lang nicht (Zündung aus), Start-Stopp ist also kein Problem,
  - die Spannung liegt unter **13,0 V** (Lichtmaschine lädt nicht),
  - das Display wurde **2 Minuten** nicht angetippt,
  - keine Öl-Suche läuft.

  In den letzten 30 Sekunden steht auf der Hauptseite `SCHLAF 25s`; Antippen verschiebt den Schlaf.
  Dann: Hintergrundlicht und Panel aus, Touch stromsparend, ELM327 per `AT LP` in den Stromsparmodus, ESP32 in den Deep Sleep.
- **Aufwachen:**
  - alle **30 s** kurze Prüfung ohne Display (ca. 1,5–2,5 s): Spannung **≥ 13,2 V** oder **Drehzahl > 0** → normaler Start;
  - **Antippen** des Displays weckt sofort;
  - unter **12,0 V** wird nur noch alle **10 min** geprüft (Batterieschutz).
- Alle Schwellen stehen oben in `OBD.ino` (`SLEEP_AFTER_MS`, `WAKE_VOLT`, `WAKE_CHECK_S` …).
- Im seriellen Monitor: `[SLEEP] …` beim Einschlafen, `[WAKE] Grund …, Prüfungen im Schlaf: …` beim Start.
- **Grenzen:** Der ELM327 hängt direkt an 12 V. Nachbauten setzen `AT LP` oft nicht um und ziehen weiter Strom. Auch der 12→5-V-Wandler hat einen Ruhestrom.
  Die tatsächliche Entladung also am besten einmal messen. Die Spannungsprüfung nutzt `ATRV` des ELM; misst der falsch (zu hoch), schläft die Anzeige nicht ein.

## Öl-Suche (herstellerspezifische Öltemperatur)

Das Motorsteuergerät kennt die Öltemperatur, aber nur unter einer herstellerspezifischen
Kennnummer (DID), abfragbar mit UDS Mode 22 (`22 xxxx`). Eine öffentliche Liste gibt es nicht,
deshalb kann das Programm sie selbst suchen. **Für den Fabia ist das schon erledigt: `11BD` ist fest eingebaut.**
Die Suche ist nur noch für andere Autos oder zur Kontrolle nötig. **Mode 22 liest nur**, am Steuergerät wird nichts verändert.

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
   (mit `*` markiert) statt von `11BD`. Ohne Kandidatenliste zeigt die Seite die aktive DID;
   eine abweichende lässt sich dort per langem Druck wieder auf `11BD` zurücksetzen.

Technisch: Für Mode 22 wird das Motorsteuergerät direkt adressiert (`ATSH7E0`). Das gilt für alle Live-Werte; nur zum Fehlercode-Lesen wird kurz auf Rundruf
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
| `font_boost_*.h` | Fette Schrift (Montserrat ExtraBold, OFL-Lizenz) für die Ladedruckseite, nur die benötigten Zeichen; erzeugt mit `lv_font_conv`, per `#include` in `OBD.ino` eingebunden |
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
