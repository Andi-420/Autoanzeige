# UI-Simulator

Rendert die Seiten von `OBD_LVGL_v7.ino` am PC als Bilder, ohne Board und ohne Flashen.
Der echte Sketch wird gegen LVGL 9.2.2 kompiliert; Arduino, ESP32, FreeRTOS, Display,
Touch, IMU und NVS werden durch einfache Stubs in `stub/` ersetzt.

Zusätzlich prüft der Simulator jede Beschriftung und jeden Button darauf, ob er
**außerhalb des runden Displays** liegt (weiter als 230 px von der Mitte entfernt).

## Voraussetzungen

Linux, macOS oder Windows mit WSL, sowie:

- `git`, `gcc`/`g++`, `python3`
- optional `Pillow` (`pip install pillow`) für PNG-Ausgabe, sonst bleiben die Bilder `.ppm`

## Benutzung

```bash
tools/sim/build.sh
```

Beim ersten Start wird LVGL nach `tools/sim/build/lvgl` geklont und einmalig gebaut
(dauert ein bis zwei Minuten). Danach geht jeder Durchlauf in wenigen Sekunden.

Ausgabe:

```
Seite dtc:
Seite main:
Seite bright:
Seite boost:
Seite accel:
  [accel] AUSSERHALB r=295  (10,410)-(32,424)  'V/Z:'     <- Beispiel für einen Fehler
Gesamt ausserhalb: 0
Bilder: tools/sim/build/out_*.png
```

Die Bilder liegen in `tools/sim/build/` (`out_main.png`, `out_boost.png`, ...).
Der Bereich außerhalb des runden Displays ist dunkelrot eingefärbt.
Das Skript endet mit Fehlercode 1, wenn etwas außerhalb liegt.

## Beispieldaten ändern

Die angezeigten Werte (Drehzahl, Ladedruck, Fehlercodes, G-Werte, ...) stehen in
`sim_main.cpp` im Block „Beispieldaten“. Am besten Werte wählen, die möglichst lange
Texte erzeugen, z. B. negative Zahlen oder vierstellige Drehzahlen.

## Hinweise

- Wird im Sketch eine neue Arduino- oder ESP32-Funktion benutzt, muss sie ggf. in
  `stub/Arduino.h` (oder dem passenden Stub) ergänzt werden, sonst schlägt der Build fehl.
- Der Simulator prüft nur die Oberfläche. OBD-Kommunikation, Touch und Sensoren
  laufen nicht (die Stubs liefern feste Werte).
- Andere LVGL-Version: `LVGL_VER=v9.3.0 tools/sim/build.sh` (vorher `tools/sim/build/` löschen).
