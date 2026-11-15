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
  - **LVGL** 9.x (verwendet: 9.6; getestet im Simulator mit 9.0 – 9.5, auf dem ESP32-Compiler mit 9.6)
  - **FastIMU** (1.2.6, für den QMI8658)
- `lv_conf.h` liegt im Sketch-Ordner und zusätzlich unter `Arduino/libraries/lvgl/src/` (nach jedem LVGL-Update neu dorthin kopieren). Der LVGL-Heap (256 KB) liegt im PSRAM, im internen RAM ist dafür kein Platz
- Die Arduino IDE verlangt, dass der Ordner genauso heißt wie der Sketch: `OBD.ino` muss in einem Ordner `OBD` liegen
- **Alle Dateien aus dem Repo** gehören in diesen Ordner (nicht nur `OBD.ino`): `OilScan.h`, `font_boost_*.c`, die Treiber `*.cpp/*.h`. Am einfachsten: Repo als ZIP herunterladen (GitHub → Code → Download ZIP) und den Inhalt nach `OBD\` entpacken

Architektur: Der OBD-Task läuft auf Core 0 (blockierende UART-Kommunikation), LVGL und die Anzeige auf Core 1. Die Messwerte werden über einen Mutex ausgetauscht.

## Seiten und Bedienung

```
                [Fehlercodes]
                      ↑
[Ladedruck/Öl]  ←  [HAUPTSEITE]  →  [G-Meter]
                      ↓       ⤷ Mitte antippen: [Fahrt-Statistik]
                 [Helligkeit]
```

Von der Hauptseite aus in die gewünschte Richtung wischen. Von jeder Unterseite aus bringt beliebiges Wischen zurück zur Hauptseite. Auf der Helligkeitsseite geht das nur senkrecht, weil waagerechtes Wischen den Slider bedient. Die zuletzt angezeigte Seite wird gespeichert, nach dem Einschalten oder Aufwachen startet die Anzeige wieder dort (aus der Öl-Suche heraus auf der Ladedruckseite).

| Seite | Inhalt |
|---|---|
| **Hauptseite** | Batteriespannung, Öltemperatur, Katalysatortemperatur (cyan < 300 °C, grün, orange > 850 °C, rot > 950 °C), Zündwinkel (° vor OT; orange wenn negativ = nach OT). Drehzahl und Kühlwasser zeigt schon das Kombiinstrument; sie werden intern weiter gelesen (Schaltblitz, Öl-Suche). In der Mitte der Status: `OBD OK` (alles gut), `ECU?` (Adapter ok, Auto antwortet nicht, z. B. Zündung aus), `ELM?` (Adapter antwortet nicht). **Mitte antippen** öffnet die Fahrt-Statistik |
| **Ladedruck / Öl** | Großer Ladedruck-Ring außen, Öl-Ring innen, in der Mitte **BOOST** mit Wert in fetter Schrift. Ladedruck = Saugrohrdruck (PID 0x87, sonst 0x0B) − Umgebungsdruck (PID 0x33), Anzeige −0,6 … +2,0 bar. Ein **roter Strich** quer über den Ladedruck-Ring markiert das bisher erreichte Maximum dieser Fahrt (erscheint ab dem ersten Überdruck, wird auch erfasst, während eine andere Seite angezeigt wird, und beim Einschlafen/Aufwachen zurückgesetzt). **Spitzenwert-Anzeige wie beim Verstärker:** zwei kleine rote Striche außen und innen am Ring bleiben 2 s beim höchsten Ladedruck stehen und sinken dann mit 0,6 bar/s auf den aktuellen Wert zurück (nur im Überdruck; `PEAK_HOLD_MS`, `PEAK_FALL_CB_S`). Innerer Bogen: Öltemperatur (cyan bis 60 °C, türkisgrün bis 80 °C, grün bis 105 °C, darüber rot). Unter dem Ladedruck klein nebeneinander: **Öl** (links) und **Ansaugluft** (rechts; cyan, gelb ab 50 °C, rot ab 65 °C). **Schaltblitz:** ab 3500 U/min blinkt ein roter Ring am Rand (5× pro Sekunde; Schwelle `SHIFT_RPM` in `OBD.ino`). Knopf **OEL-SUCHE** öffnet die Öl-Suche (siehe unten) |
| **Fehlercodes** | „AUSLESEN“ tippen, dann werden die gespeicherten Fehlercodes (Mode 03, rot, z. B. `P0123`) und darunter die **anstehenden** (Mode 07, orange, mit dem Zusatz „anstehend“) angezeigt. Anstehend heißt: Das Steuergerät hat den Fehler bemerkt, aber noch nicht fest gespeichert (die Motorkontrollleuchte ist meist noch aus). Die Seite **liest nur**, gelöscht wird nichts. Die minütliche Hintergrundprüfung liest nur die gespeicherten Codes; die anstehenden stammen vom letzten Antippen |
| **Helligkeit** | Slider und Presets 25/50/75/100 %. Die Einstellung bleibt nach einem Neustart erhalten. Knopf **WARNTEST** löst einen Probealarm aus (Hinweisfeld + Summer), Antippen des Hinweisfelds beendet ihn |
| **Fahrt-Statistik** | Fahrzeit (Zündung an), Strecke, Tempo mittel und max, Ladedruck max, Drehzahl max, Öl/Wasser max, G-Werte (Bremsen/Gas, Kurve; nur ab 10 km/h, damit Türzuschlagen nicht zählt). Erreichbar über die Mitte der Hauptseite; nach einer Fahrt (mind. 1 min) erscheint sie **automatisch bei Zündung aus**, darunter der Schlaf-Countdown. Springt der Motor wieder an, geht es zurück zur vorherigen Seite. Eine „Fahrt“ läuft vom Aufwachen bis zum Einschlafen, kurzes Abstellen gehört also dazu. Beim Einschlafen wird sie gespeichert; bis die neue Fahrt 1 min alt ist, zeigt die Seite die **letzte Fahrt**. Wischen = zurück zur Hauptseite |
| **G-Meter** | Längs- und Querbeschleunigung mit Max-Punkt. **RESET tippen** (Auto steht still, möglichst eben) übernimmt die momentane Lage des Geräts als neue Nullausrichtung und löscht die Max-Werte. Dabei wird die Schwerkraftrichtung gemessen und die Anzeige auf die waagerechte Längs- und Querachse umgerechnet, so dass auch ein stark schräger Einbau sauber ausgeglichen wird. Die Ausrichtung wird gespeichert und bleibt nach Neustart und Schlafmodus erhalten |

## Abgefragte Werte

| Wert | Befehl / PID | Intervall |
|---|---|---|
| Drehzahl | 010C (für den Schaltblitz) | jeder Zyklus |
| Saugrohrdruck (MAP) | 0187 (1/32 kPa, bis 2048 kPa), falls das Auto es kennt; sonst 010B (1 Byte, max. 255 kPa ≈ +1,55 bar Ladedruck) | jeder Zyklus |
| Batteriespannung | ATRV (misst der ELM327 selbst) | 1 s |
| Kühlwasser | 0105 (für die Öl-Suche) | 1 s |
| Zündwinkel | 010E: A/2 − 64 ° vor OT | 250 ms |
| Ansauglufttemperatur | 010F: A − 40 °C | 1 s |
| Geschwindigkeit | 010D: A km/h (Fahrt-Statistik) | 1 s |
| Katalysatortemperatur | 013C (Bank 1, Sensor 1): (A·256+B)/10 − 40 °C | 1 s |
| Öltemperatur | `2211BD` an das Motorsteuergerät (7E0), Formel (A·256+B)/10 − 273,1 (Kelvin/10). Fest eingebaut; eine per Öl-Suche gespeicherte andere DID hat Vorrang | 1 s |
| Umgebungsdruck | 0133 | 10 s (Startwert 101,3 kPa, bis der erste Messwert da ist) |
| Fehlerspeicher | 03 (Rundruf an alle Steuergeräte) | 60 s im Hintergrund, erste Prüfung ca. 15 s nach dem Start (siehe Warnungen) |

**Schnellabfrage:** An jede Abfrage wird eine `1` angehängt (`010C1`). Der ELM327 liefert die Antwort dann sofort, statt jedes Mal abzuwarten, ob noch ein weiteres Steuergerät antwortet; Drehzahl und Ladedruck kommen dadurch häufiger. Weil Nachbauten das nicht immer können, wird es nach jedem Start des Adapters zuerst an der Drehzahl ausprobiert und nur bei Erfolg benutzt. Im seriellen Monitor steht `[ELM] Schnellabfrage …` und alle 30 s die erreichte Rate (`[OBD] … Zyklen/s`). Abschalten: `FAST_QUERY 0` in `OBD.ino`.

Liefert das Auto einen Wert nicht (Antwort `NO DATA`), zeigt die Kachel `N/A`; die Öltemperatur hält bei einzelnen Aussetzern den letzten Wert (erst nach 3 Lesefehlern hintereinander `N/A`). Antwortet das Steuergerät gar nicht mehr (`ECU?`, Zündung aus), zeigen Öl, Kat, Zündwinkel, Ansaugluft und Ladedruck `N/A` statt des letzten Werts. Die Batteriespannung misst der Adapter selbst, sie bleibt sichtbar. Kat-Temperatur, Zündwinkel, Ansaugluft und Geschwindigkeit werden dann nach 5 Fehlversuchen nur noch alle 30 s erneut versucht.

### Hinweise zum Fabia 3 / 1.0 TSI

- Skoda/VW liefert die **Öltemperatur** nicht über die Standard-PID 0x5C. Beim Fabia steckt sie im Motorsteuergerät unter **DID `11BD`** (Kelvin/10), per Öl-Suche gefunden. Dieser Wert ist fest eingebaut.
- Laut ScanMaster: ISO 15765-4 CAN 11 Bit / 500 kbit, Motorsteuergerät antwortet auf `7E8` (angefragt über `7E0`).
- Das Protokoll wird automatisch erkannt (`ATSP0`, dann `ATDPN`). Startet die Anzeige vor der Zündung, ist es zunächst unbekannt und wird nachgeholt, sobald das Auto antwortet. Erst dann wird auf `7E0` umgeschaltet (vorher blieb die Öltemperatur in diesem Fall bis zum Neustart auf `N/A`). Die Fehlercode-Auswertung kann auch Antworten verarbeiten, die über mehrere CAN-Frames verteilt kommen.
- Wenn der Adapter nicht mehr antwortet (3 leere Antworten hintereinander), wird er automatisch neu initialisiert.

## Warnungen (Summer)

Der Summer auf dem Board (am IO-Expander, EXIO8) piept, und über jeder Seite erscheint ein rotes Hinweisfeld:

| Warnung | Auslöser | Ton | Antippen des Hinweisfelds |
|---|---|---|---|
| **Kühlwasser** | über **105 °C** | 3× kurz, jede Minute wiederholt | quittiert: still, bis die Temperatur wieder unter 100 °C war |
| **Öl zu heiß** | über **120 °C** | 3× kurz, jede Minute wiederholt | quittiert: still, bis die Temperatur wieder unter 115 °C war |
| **Neuer Fehlercode** | ein Code, der beim letzten Auslesen noch nicht da war | 2× lang, einmalig | öffnet die Fehlercode-Seite |

- Temperaturwarnungen gelten nur, solange das Steuergerät antwortet. Ein einzelner Lesefehler beendet sie nicht; Zündung aus schon.
- **Fehlerspeicher-Prüfung:** Der OBD-Task liest jede Minute im Hintergrund Mode 03. Das dauert normalerweise 0,1–0,3 s; nur in dieser Zeit stehen die OBD-Werte (Ladedruck, Drehzahl …) kurz still. Anzeige, Touch und G-Meter laufen auf dem anderen Prozessorkern ungebremst weiter. Ab `SHIFT_RPM` (3500 U/min) wird die Prüfung verschoben, damit beim Ausdrehen kein Ladedruck-Spitzenwert fehlt. Die Dauer jeder Prüfung steht im seriellen Monitor (`[DTC] Pruefung: … (120 ms)`).
- Die zuletzt bekannte Fehlerliste wird gespeichert (NVS). Ein Code, der dauerhaft im Speicher steht, meldet sich daher nur **einmal**, nicht nach jedem Aufwachen. Nach dem Löschen gilt ein wieder auftauchender Code wieder als neu. Beim allerersten Start mit der neuen Version gelten vorhandene Codes als neu.
- Das Ergebnis der Hintergrundprüfung erscheint auch auf der Fehlercode-Seite; wer dort gerade ist, bekommt keinen Alarm.
- Schwellen und Intervalle oben in `OBD.ino`: `WARN_OIL_C`, `WARN_COOL_C`, `WARN_HYST_C`, `WARN_REPEAT_MS`, `DTC_CHECK_MS`.

## Schlafmodus (dauerhaft am OBD-Stecker)

Damit die Anzeige am OBD-Stecker bleiben kann, ohne die Batterie leerzuziehen:

- **Einschlafen**, wenn alle Bedingungen erfüllt sind:
  - das Steuergerät antwortet **3 Minuten** lang nicht (Zündung aus), Start-Stopp ist also kein Problem,
  - die Spannung liegt unter **13,0 V** (Lichtmaschine lädt nicht),
  - das Display wurde **2 Minuten** nicht angetippt,
  - keine Öl-Suche läuft (eine offen gelassene Live-Anzeige der Öl-Suche hält das Gerät nicht wach, sobald das Auto nicht mehr antwortet).

  In den letzten 30 Sekunden steht auf der Hauptseite `SCHLAF 25s`; Antippen verschiebt den Schlaf.
  Dann: Hintergrundlicht und Panel aus, Touch stromsparend, ELM327 per `AT LP` in den Stromsparmodus, ESP32 in den Deep Sleep.
- **Aufwachen:**
  - alle **30 s** kurze Prüfung ohne Display (ca. 1,5–2,5 s): Spannung **≥ 13,2 V** oder **Drehzahl > 0** → normaler Start;
  - **Antippen** des Displays weckt sofort;
  - unter **11,0 V** wird nur noch alle **3 Stunden** geprüft (Batterieschutz). Ein Motorstart wird dann erst bei der nächsten Prüfung erkannt, also bis zu 3 h später; Antippen weckt weiterhin sofort.
    Komplett abschalten kann sich das Gerät nicht: ESP32, 12→5-V-Wandler und ELM327 hängen ohne schaltbare Versorgung an Dauerplus. Dafür bräuchte es Zusatzhardware (z. B. Relais oder MOSFET mit Selbsthaltung).
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
| `font_boost_*.c` | Fette Schrift (Montserrat ExtraBold, OFL-Lizenz) für die Ladedruckseite, nur die benötigten Zeichen; erzeugt mit `lv_font_conv`. Müssen als C (nicht C++) übersetzt werden |
| `tools/sim/` | UI-Simulator für den PC (siehe unten) |
| `tools/tests/` | PC-Test der Öl-Such-Logik (Befehl steht oben in der Datei) |
| `laptop-tool/` | Eigenständiges OBD-Programm für den Laptop (Python, Windows), siehe unten. Gehört nicht in den Sketch-Ordner |

## OBD-Tool für den Laptop

Unter `laptop-tool/` liegt ein kleines Windows-Programm (Python mit Fensteroberfläche), das über einen
ELM327 am Seriell-USB-Wandler Fehlercodes liest und löscht, Live-Werte anzeigt und als CSV mitschreibt.
Es nutzt dieselben Abfragen wie die Anzeige, also auch die Öltemperatur über `22 11BD`.
Start mit `start.bat`, Details siehe [laptop-tool/README.md](laptop-tool/README.md).

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
