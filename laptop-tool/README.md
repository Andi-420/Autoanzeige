# OBD-Tool für den Laptop

Kleines Windows-Programm (Python), das über den ELM327 am Seriell-USB-Wandler
Fehlercodes und Live-Werte liest. Es nutzt dieselben Abfragen wie die Anzeige im
Auto (`OBD.ino`), also auch die Öltemperatur des Fabia 3 (`22 11BD` am Motorsteuergerät).

| Reiter | Inhalt |
|---|---|
| **Live-Werte** | Drehzahl, Ladedruck (mit Maximum), Öl, Kühlwasser, Ansaugluft, Katalysator, Zündwinkel, Batterie, Geschwindigkeit, Motorlast, Drosselklappe, Saugrohrdruck. Auf Wunsch als CSV-Datei mitschreiben |
| **Fehlercodes** | gespeicherte (Mode 03), anstehende (Mode 07) und permanente (Mode 0A) Codes aller abgasrelevanten Steuergeräte, Zustand der Motorkontrollleuchte. Dazu der Fehlerspeicher des Motorsteuergeräts direkt (UDS), mit herstellerspezifischen Codes und Fehlerart. Löschen (Mode 04) mit Sicherheitsabfrage |
| **Steuergerät** | direkter Zugriff aufs Motorsteuergerät (nur lesend): Teilenummer und Softwareversion, beliebige Kennnummern lesen (Mode 22), einen Bereich durchsuchen wie die Öl-Suche am Display, gefundene Werte als eigene Live-Kacheln speichern |
| **Fahrzeug** | Fahrgestellnummer, Adapter, Protokoll, Spannung und die Liste der Werte, die das Auto über OBD liefert |
| **Terminal** | eigene Befehle an den ELM327 schicken (z. B. `ATRV`, `010C`) und die komplette Kommunikation mitlesen |

## Installation (Windows 11)

1. **Python** von <https://www.python.org/downloads/> installieren (Version 3.9 oder neuer).
   Im ersten Fenster des Installers **„Add python.exe to PATH“** anhaken.
   Tkinter (die Fensteroberfläche) ist dabei automatisch mit installiert.
2. Den Ordner `laptop-tool` aus dem Repo irgendwohin kopieren, z. B. nach `Dokumente\OBD-Tool`.
   Er gehört **nicht** in den Arduino-Sketch-Ordner (stört dort aber auch nicht).
3. **`start.bat` doppelklicken.** Beim ersten Start wird automatisch `pyserial` installiert
   (braucht einmal Internet). Danach öffnet sich das Fenster.

Alternativ in der Eingabeaufforderung: `py -m pip install pyserial` und `py obd_tool.py`.

## Benutzung

1. ELM327 ans Auto stecken (er bekommt seinen Strom aus dem OBD-Stecker), Seriell-USB-Wandler an den Laptop.
2. **Zündung an** (für Live-Werte Motor an).
3. Oben bei **Anschluss** den COM-Port des Wandlers wählen. Welcher es ist, steht im
   Geräte-Manager unter „Anschlüsse (COM & LPT)“. Nach dem Einstecken **↻** drücken.
4. **Baud: Auto** probiert 38400, 9600, 115200, 57600 und 230400 durch.
   Die Anzeige im Auto nutzt 38400; wenn du die Rate kennst, geht das Verbinden mit fester Rate schneller.
5. **Verbinden.** Unten rechts stehen dann Adapter, Protokoll (beim Fabia: ISO 15765-4 CAN 11 Bit, 500 kbit/s) und Spannung.

Port und Baudrate merkt sich das Programm für den nächsten Start (`%USERPROFILE%\.obd_tool.json`).

**Ohne Auto ausprobieren:** als Anschluss **„Demo (ohne Auto)“** wählen. Dann antwortet ein
simulierter ELM327 mit einem Motor, der Leerlauf, Beschleunigen und Ausrollen im Wechsel fährt, und ein paar Fehlercodes.

### Fehlercodes löschen

Löscht den Fehlerspeicher aller abgasrelevanten Steuergeräte. Dabei gehen auch die
Freeze-Frame-Daten und die Bereitschaftstests (Readiness) verloren; bis die nach einigen Fahrten
wieder durchgelaufen sind, fällt das Auto bei der Abgasuntersuchung durch. Am besten bei
**Motor aus, Zündung an** löschen. Permanente Codes lassen sich grundsätzlich nicht löschen, sie
verschwinden erst, wenn das Steuergerät den Fehler bei späteren Fahrten nicht mehr feststellt.

### Steuergerät: Kennnummern suchen und als Live-Wert speichern

Das Motorsteuergerät kennt viel mehr Messwerte als die genormten OBD-Werte, aber nur unter
herstellerspezifischen Kennnummern (DIDs), die nirgends öffentlich aufgelistet sind. So findet man sie:

1. Motor **kalt**, Zündung an. Im Reiter **Steuergerät** einen Bereich eintragen (Vorschlag `1000`–`2FFF`,
   dauert etwa 5–10 Minuten) und **Suche starten**. Die Live-Werte sind währenddessen pausiert.
   Alle Kennnummern, die antworten, landen in der Liste, mit den gängigen Umrechnungen
   (A−40, A·0,75−48, Kelvin/10, °C/10).
2. Nach einer Fahrt **Treffer neu lesen**. In der Spalte „vorher“ steht dann der alte Wert, so sieht man,
   was sich wie eine Temperatur verändert hat.
3. Zeile auswählen (oder doppelklicken) → **Als Live-Wert hinzufügen …**: Name, Formel und Einheit
   wählen. Der Wert erscheint dann als eigene Kachel bei den Live-Werten und in der CSV-Aufzeichnung.
   Die Liste bleibt gespeichert; unter „Eigene Live-Werte“ lässt sich ein Wert wieder entfernen.

Es wird nur gelesen (UDS-Dienste 22 und 19), am Steuergerät wird nichts verändert oder codiert.

### CSV-Aufzeichnung

**● CSV-Aufzeichnung** fragt nach einem Dateinamen und startet die Live-Werte. Die Datei nutzt
Semikolon und Dezimalkomma, öffnet sich also im deutschen Excel direkt richtig. Eine Zeile pro
Abfragezyklus (am Auto einige pro Sekunde), mit Uhrzeit und Sekunden seit dem Start.

## Wenn es nicht klappt

| Meldung / Problem | Ursache und Abhilfe |
|---|---|
| Kein COM-Port in der Liste | Wandler nicht erkannt. Im Geräte-Manager nachsehen: Steht er mit gelbem Ausrufezeichen da, fehlt der Treiber oder er ist gesperrt (siehe nächste Zeile) |
| Geräte-Manager zeigt **Code 10** oder „PL2303HXA PHASED OUT SINCE 2012“ | Häufigster Grund, warum ein altes OBD-Programm plötzlich nicht mehr läuft: Windows Update hat für einen nachgebauten **Prolific-PL2303**-Chip einen neuen Treiber installiert, der ihn ablehnt. Abhilfe: im Geräte-Manager einen älteren Prolific-Treiber auswählen („Treiber aktualisieren → Auf meinem Computer suchen → Aus einer Liste auswählen“), oder einen Wandler mit FTDI- oder CH340-Chip nehmen (ca. 10 €) |
| „COMx ist belegt“ | Ein anderes Programm (altes OBD-Programm, Arduino IDE, serieller Monitor) hat den Port offen. Schließen und neu verbinden |
| „Auf COMx antwortet kein ELM327“ | Falscher Port, Adapter nicht am Auto (kein Strom), oder eine ungewöhnliche Baudrate |
| Verbunden, aber „Auto antwortet nicht“ | Zündung aus. Sobald das Auto antwortet, wird das Protokoll automatisch erkannt |
| Ein Wert bleibt bei „–“ | Das Auto liefert ihn nicht über OBD. Er wird dann nur noch alle 30 s erneut versucht. Im Reiter **Fahrzeug** steht, welche Werte das Auto kennt |

Im Reiter **Terminal** steht die komplette Kommunikation mit dem Adapter. Mit **Speichern …**
lässt sie sich als Textdatei ablegen, z. B. um sie hier ins Projekt zu stellen.

## Grenzen

- Gelesen wird nur, was über **OBD-II** geht, plus Fehlerspeicher und Kennnummern des **Motorsteuergeräts** (UDS).
  Fehlerspeicher von Airbag, ABS, Komfortelektronik usw. und Codierungen/Anpassungen brauchen
  herstellerspezifische Diagnose (bei VW/Skoda z. B. VCDS oder OBDeleven).
- Die Beschreibungen der Fehlercodes gibt es nur für häufige genormte Codes und ohne Gewähr.
  Bei anderen Codes steht der Bereich (z. B. „Antrieb: Zündung, Zündaussetzer“ oder „herstellerspezifisch“).

## Technik

Ablauf wie in `OBD.ino`: `ATZ`, `ATE0`, `ATL0`, `ATS0`, `ATH0`, `ATSP0`, `ATAT1`, dann `0100`
(startet die automatische Protokollsuche) und `ATDPN`. Live-Werte gehen per `ATSH7E0` direkt an das
Motorsteuergerät, Fehlercodes per `ATSH7DF` als Rundruf an alle. Formeln:

| Wert | Abfrage | Formel |
|---|---|---|
| Drehzahl | 010C | (A·256+B)/4 |
| Ladedruck | 0187 (falls vorhanden) bzw. 010B, minus 0133 | Saugrohrdruck − Umgebungsdruck, in bar |
| Öltemperatur | 22 11BD an 7E0, sonst 015C | (A·256+B)/10 − 273,1 bzw. A − 40 |
| Kühlwasser / Ansaugluft | 0105 / 010F | A − 40 |
| Katalysator | 013C | (A·256+B)/10 − 40 |
| Zündwinkel | 010E | A/2 − 64 |
| Batterie | ATRV | misst der ELM327 selbst |
| Geschwindigkeit / Last / Drosselklappe | 010D / 0104 / 0111 | A bzw. A·100/255 |

| Datei | Inhalt |
|---|---|
| `obd_tool.py` | Fenster (Tkinter) und Hintergrund-Thread für die Kommunikation |
| `elm327.py` | COM-Port, Baudratensuche, ELM327-Befehle, Fehlercodes, Fahrzeugdaten |
| `live.py` | Live-Werte mit Abfrageintervallen und Formeln |
| `obd_decode.py` | Auswertung der Antworten (auch Multi-Frame), Fehlercode- und PID-Texte |
| `simulator.py` | Simulierter ELM327 für Demo-Modus und Tests |
| `tests/test_obd.py` | Tests ohne Auto: `py -m unittest discover tests` |
| `start.bat` | Startet das Programm und installiert beim ersten Mal `pyserial` |
