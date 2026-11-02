"""Auswertung der ELM327-Antworten.

Reine Python-Funktionen ohne Abhängigkeiten, damit sie ohne Adapter testbar sind.
Die Logik entspricht der im ESP32-Sketch (OBD.ino / OilScan.h).

Der Adapter wird mit ATS0 (ohne Leerzeichen) und ATH0 (ohne CAN-Header) betrieben.
Die Funktionen kommen aber auch mit Leerzeichen klar.
"""

import re

HEX = set("0123456789ABCDEF")


def _clean_lines(resp):
    """Antwort in Zeilen zerlegen, Leerzeichen entfernen, Grossbuchstaben."""
    out = []
    for line in resp.replace("\r", "\n").split("\n"):
        line = line.replace(" ", "").strip().upper()
        if line:
            out.append(line)
    return out


def _is_hex(s):
    return bool(s) and all(c in HEX for c in s)


def messages(resp):
    """Antwort in einzelne Nachrichten (Hex-Strings ohne Leerzeichen) zerlegen.

    CAN-Antworten über mehrere Frames sehen beim ELM327 so aus:
        00A                 <- Länge in Bytes (hex)
        0: 43 04 01 23 04 56
        1: 07 89 0A BC 00 00
    Diese Teile werden zu einer Nachricht zusammengesetzt und auf die
    angegebene Länge gekürzt (Füllbytes fallen weg).
    Zeilen wie "SEARCHING...", "NO DATA" oder "?" werden ignoriert.
    """
    msgs = []
    lengths = []
    cur = None          # Index der Multi-Frame-Nachricht, an die angehängt wird
    pending_len = None  # Längenzeile, die vor "0:" kam
    for line in _clean_lines(resp):
        if len(line) >= 2 and line[1] == ":" and line[0] in HEX:
            data = line[2:]
            if not _is_hex(data):
                continue
            if line[0] == "0" or cur is None:
                msgs.append(data)
                lengths.append(pending_len)
                cur = len(msgs) - 1
                pending_len = None
            else:
                msgs[cur] += data
            continue
        if not _is_hex(line):
            continue
        if len(line) == 3:          # Längenzeile einer Multi-Frame-Antwort
            pending_len = int(line, 16)
            continue
        msgs.append(line)
        lengths.append(None)
        cur = None
    out = []
    for m, n in zip(msgs, lengths):
        if n is not None:
            m = m[: 2 * n]
        out.append(m[: len(m) - (len(m) % 2)])
    return out


def hex_bytes(s):
    """'1AF8' -> [0x1A, 0xF8]"""
    return [int(s[i:i + 2], 16) for i in range(0, len(s) - 1, 2)]


def has_no_data(resp):
    return "NODATA" in resp.replace(" ", "").upper()


def parse_obd_all(resp, tag):
    """Datenbytes hinter 'tag' (z.B. '410C') aus allen Nachrichten.

    Bei einer Rundruf-Anfrage können mehrere Steuergeräte antworten,
    deshalb eine Liste (eine Byte-Liste pro Antwort).
    """
    tag = tag.upper()
    out = []
    for m in messages(resp):
        if m.startswith(tag):
            out.append(hex_bytes(m[len(tag):]))
    return out


def parse_obd(resp, tag):
    """Datenbytes der ersten passenden Antwort, oder None."""
    all_ = parse_obd_all(resp, tag)
    return all_[0] if all_ else None


# ── Fehlercodes ──────────────────────────────────────────────────────

def dtc_format(b1, b2):
    """Zwei Bytes in einen Fehlercode umwandeln, z.B. 0x01 0x23 -> 'P0123'."""
    letter = "PCBU"[(b1 >> 6) & 0x03]
    return "%s%d%X%02X" % (letter, (b1 >> 4) & 0x03, b1 & 0x0F, b2)


def parse_dtcs(resp, is_can=True, service="43"):
    """Fehlercodes aus der Antwort auf Mode 03 (43), 07 (47) oder 0A (4A).

    CAN:      "43 02 01 23 04 56"   (Byte nach 43 = Anzahl der Codes)
    K-Line:   "43 01 23 04 56 00 00" (je Zeile bis zu 3 Codes, 0000 = leer)
    Liefert eine Liste (leer = keine Codes) oder None bei ungültiger Antwort.
    """
    msgs = [m for m in messages(resp) if m.startswith(service)]
    if not msgs:
        return [] if has_no_data(resp) else None
    codes = []
    for m in msgs:
        h = m[2:]
        if is_can:
            if len(h) < 2:
                continue
            count = int(h[:2], 16)
            h = h[2:]
        else:
            count = len(h) // 4
        for i in range(count):
            chunk = h[4 * i: 4 * i + 4]
            if len(chunk) < 4:
                break
            b1, b2 = int(chunk[:2], 16), int(chunk[2:], 16)
            if b1 == 0 and b2 == 0:
                continue
            code = dtc_format(b1, b2)
            if code not in codes:      # mehrere Steuergeräte melden evtl. denselben Code
                codes.append(code)
    return codes


# ── UDS Mode 22 (herstellerspezifische Messwerte) ────────────────────

UDS_OK, UDS_NEG, UDS_NONE = "ok", "neg", "none"


def parse_uds(resp, did):
    """Antwort auf '22 DDDD' auswerten.

    Positiv: 62 DD DD <Daten>  -> ('ok', [Daten])
    Negativ: 7F 22 <NRC>       -> ('neg', NRC)    (0x31 = DID unbekannt)
    Sonst                      -> ('none', None)
    """
    want = "62%04X" % did
    for m in messages(resp):
        i = m.find(want)
        if i >= 0 and i % 2 == 0:
            return UDS_OK, hex_bytes(m[i + 6:])
    for m in messages(resp):
        i = m.find("7F22")
        if i >= 0 and i % 2 == 0 and len(m) >= i + 6:
            return UDS_NEG, int(m[i + 4:i + 6], 16)
    return UDS_NONE, None


# ── Fahrgestellnummer (Mode 09 PID 02) ───────────────────────────────

def parse_vin(resp):
    """Fahrgestellnummer aus der Antwort auf 0902, oder None.

    CAN:    eine Nachricht "49 02 01" + 17 Zeichen (über mehrere Frames)
    K-Line: fünf Nachrichten "49 02 NN" + je 4 Bytes
    """
    msgs = [m for m in messages(resp) if m.startswith("4902") and len(m) >= 6]
    if not msgs:
        return None
    msgs.sort(key=lambda m: int(m[4:6], 16))
    data = []
    for m in msgs:
        data += hex_bytes(m[6:])
    while data and data[0] == 0:
        data.pop(0)
    text = "".join(chr(b) for b in data[:17] if 0x20 < b < 0x7F)
    return text or None


# ── Unterstützte PIDs ───────────────────────────────────────────────

def parse_supported(data, base):
    """4 Bitmasken-Bytes der Antwort auf 01<base> -> Menge der PIDs base+1 .. base+32."""
    if not data or len(data) < 4:
        return set()
    bits = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]
    return {base + i + 1 for i in range(32) if bits & (1 << (31 - i))}


def parse_voltage(resp):
    """'12.4V' -> 12.4, sonst None."""
    m = re.search(r"(\d{1,2}[.,]\d)\s*V", resp.upper())
    return float(m.group(1).replace(",", ".")) if m else None


# ── Protokolle (ATDPN) ───────────────────────────────────────────────

PROTOCOLS = {
    "1": "SAE J1850 PWM",
    "2": "SAE J1850 VPW",
    "3": "ISO 9141-2 (K-Line)",
    "4": "ISO 14230-4 KWP (5-Baud-Init)",
    "5": "ISO 14230-4 KWP (Fast-Init)",
    "6": "ISO 15765-4 CAN (11 Bit, 500 kbit/s)",
    "7": "ISO 15765-4 CAN (29 Bit, 500 kbit/s)",
    "8": "ISO 15765-4 CAN (11 Bit, 250 kbit/s)",
    "9": "ISO 15765-4 CAN (29 Bit, 250 kbit/s)",
    "A": "SAE J1939 CAN",
    "B": "USER1 CAN",
    "C": "USER2 CAN",
}


def protocol_is_can(p):
    return p in "6789ABC"


# ── Namen der Mode-01-PIDs (für die Liste "unterstützte Werte") ────

PID_NAMES = {
    0x01: "Status Motorkontrollleuchte und Anzahl Fehlercodes",
    0x02: "Fehlercode, der den Freeze-Frame ausgelöst hat",
    0x03: "Status Kraftstoffsystem",
    0x04: "Motorlast (berechnet)",
    0x05: "Kühlmitteltemperatur",
    0x06: "Gemischanpassung kurz, Bank 1",
    0x07: "Gemischanpassung lang, Bank 1",
    0x08: "Gemischanpassung kurz, Bank 2",
    0x09: "Gemischanpassung lang, Bank 2",
    0x0A: "Kraftstoffdruck",
    0x0B: "Saugrohrdruck (absolut)",
    0x0C: "Drehzahl",
    0x0D: "Geschwindigkeit",
    0x0E: "Zündzeitpunkt",
    0x0F: "Ansauglufttemperatur",
    0x10: "Luftmasse (MAF)",
    0x11: "Drosselklappenstellung",
    0x12: "Sekundärluft",
    0x13: "Vorhandene Lambdasonden",
    0x1C: "OBD-Norm des Fahrzeugs",
    0x1D: "Vorhandene Lambdasonden (4 Bänke)",
    0x1E: "Status Nebenabtrieb",
    0x1F: "Laufzeit seit Motorstart",
    0x20: "Unterstützte PIDs 21-40",
    0x21: "Strecke mit Motorkontrollleuchte an",
    0x22: "Kraftstoff-Raildruck (relativ)",
    0x23: "Kraftstoff-Raildruck",
    0x2C: "Abgasrückführung Sollwert",
    0x2D: "Abgasrückführung Abweichung",
    0x2E: "Tankentlüftung Sollwert",
    0x2F: "Tankfüllstand",
    0x30: "Warmläufe seit Fehlerspeicher-Löschen",
    0x31: "Strecke seit Fehlerspeicher-Löschen",
    0x32: "Dampfdruck Tanksystem",
    0x33: "Umgebungsluftdruck",
    0x3C: "Katalysatortemperatur Bank 1 Sensor 1",
    0x3D: "Katalysatortemperatur Bank 2 Sensor 1",
    0x3E: "Katalysatortemperatur Bank 1 Sensor 2",
    0x3F: "Katalysatortemperatur Bank 2 Sensor 2",
    0x40: "Unterstützte PIDs 41-60",
    0x41: "Status der Überwachungen in diesem Fahrzyklus",
    0x42: "Spannung Steuergerät",
    0x43: "Absolute Last",
    0x44: "Lambda Sollwert",
    0x45: "Relative Drosselklappenstellung",
    0x46: "Umgebungstemperatur",
    0x47: "Drosselklappenstellung B",
    0x48: "Drosselklappenstellung C",
    0x49: "Gaspedalstellung D",
    0x4A: "Gaspedalstellung E",
    0x4B: "Gaspedalstellung F",
    0x4C: "Drosselklappe Sollwert",
    0x4D: "Laufzeit mit Motorkontrollleuchte an",
    0x4E: "Zeit seit Fehlerspeicher-Löschen",
    0x4F: "Maximalwerte Lambda, Spannung, Strom, Saugrohrdruck",
    0x50: "Maximalwert Luftmasse",
    0x51: "Kraftstoffart",
    0x52: "Ethanolanteil",
    0x53: "Dampfdruck Tanksystem (absolut)",
    0x54: "Dampfdruck Tanksystem",
    0x59: "Kraftstoff-Raildruck (absolut)",
    0x5A: "Relative Gaspedalstellung",
    0x5B: "Restlebensdauer Hybridbatterie",
    0x5C: "Motoröltemperatur",
    0x5D: "Einspritzzeitpunkt",
    0x5E: "Kraftstoffverbrauch",
    0x5F: "Abgasnorm",
    0x60: "Unterstützte PIDs 61-80",
    0x61: "Fahrerwunsch-Drehmoment",
    0x62: "Ist-Drehmoment",
    0x63: "Referenz-Drehmoment",
    0x64: "Drehmoment-Kennpunkte",
    0x80: "Unterstützte PIDs 81-A0",
    0x87: "Saugrohrdruck (erweitert, bis 2048 kPa)",
    0xA0: "Unterstützte PIDs A1-C0",
    0xA6: "Kilometerstand",
    0xC0: "Unterstützte PIDs C1-E0",
}
for _i in range(8):
    PID_NAMES.setdefault(0x14 + _i, "Lambdasonde %d (Spannung)" % (_i + 1))
    PID_NAMES.setdefault(0x24 + _i, "Lambdasonde %d (Lambda, Spannung)" % (_i + 1))
    PID_NAMES.setdefault(0x34 + _i, "Lambdasonde %d (Lambda, Strom)" % (_i + 1))
del _i


def pid_name(pid):
    return PID_NAMES.get(pid, "PID %02X" % pid)


# ── Beschreibung der Fehlercodes ─────────────────────────────────────
# Nur genormte Codes (SAE J2012), deutsche Kurzfassung, ohne Gewähr.
# Herstellerspezifische Codes (z.B. P1xxx) bitte nachschlagen.

DTC_TEXT = {
    "P0016": "Kurbelwelle/Nockenwelle Zuordnung (Bank 1, Sensor A)",
    "P0087": "Kraftstoffdruck zu niedrig",
    "P0088": "Kraftstoffdruck zu hoch",
    "P0100": "Luftmassenmesser Stromkreis",
    "P0101": "Luftmassenmesser Bereich/Funktion",
    "P0105": "Saugrohrdrucksensor Stromkreis",
    "P0106": "Saugrohrdrucksensor Bereich/Funktion",
    "P0115": "Kühlmitteltemperatursensor Stromkreis",
    "P0116": "Kühlmitteltemperatursensor Bereich/Funktion",
    "P0128": "Kühlmitteltemperatur unter Thermostat-Regeltemperatur",
    "P0130": "Lambdasonde Stromkreis (Bank 1, Sensor 1)",
    "P0133": "Lambdasonde reagiert langsam (Bank 1, Sensor 1)",
    "P0171": "Gemisch zu mager (Bank 1)",
    "P0172": "Gemisch zu fett (Bank 1)",
    "P0234": "Ladedruck zu hoch (Overboost)",
    "P0299": "Ladedruck zu niedrig (Underboost)",
    "P0300": "Zündaussetzer, mehrere/zufällige Zylinder",
    "P0301": "Zündaussetzer Zylinder 1",
    "P0302": "Zündaussetzer Zylinder 2",
    "P0303": "Zündaussetzer Zylinder 3",
    "P0304": "Zündaussetzer Zylinder 4",
    "P0325": "Klopfsensor 1 Stromkreis (Bank 1)",
    "P0335": "Kurbelwellensensor Stromkreis",
    "P0340": "Nockenwellensensor Stromkreis (Bank 1)",
    "P0420": "Katalysator Wirkungsgrad zu gering (Bank 1)",
    "P0441": "Tankentlüftung falscher Durchfluss",
    "P0455": "Tankentlüftung großes Leck",
    "P0500": "Geschwindigkeitssensor",
    "P0505": "Leerlaufregelung",
    "P0562": "Bordnetzspannung zu niedrig",
    "P0563": "Bordnetzspannung zu hoch",
    "P0571": "Bremslichtschalter Stromkreis A",
    "P0606": "Steuergerät Prozessorfehler",
    "P2187": "Gemisch zu mager im Leerlauf (Bank 1)",
    "P2188": "Gemisch zu fett im Leerlauf (Bank 1)",
    "U0100": "Keine Kommunikation mit Motorsteuergerät",
    "U0101": "Keine Kommunikation mit Getriebesteuergerät",
    "U0121": "Keine Kommunikation mit ABS-Steuergerät",
    "U0155": "Keine Kommunikation mit Kombiinstrument",
}

_SYSTEM = {"P": "Antrieb", "C": "Fahrwerk", "B": "Karosserie", "U": "Netzwerk"}
_P_AREA = {
    "0": "Kraftstoff-/Luftzumessung, Zusatzemission",
    "1": "Kraftstoff-/Luftzumessung",
    "2": "Kraftstoff-/Luftzumessung (Einspritzung)",
    "3": "Zündung, Zündaussetzer",
    "4": "Abgasüberwachung",
    "5": "Geschwindigkeit, Leerlauf, Eingänge",
    "6": "Steuergerät, Ausgänge",
    "7": "Getriebe",
    "8": "Getriebe",
    "9": "Getriebe",
    "A": "Hybridantrieb",
}


def dtc_description(code):
    """Kurzbeschreibung bzw. Bereich eines Fehlercodes."""
    if code in DTC_TEXT:
        return DTC_TEXT[code]
    if len(code) != 5:
        return ""
    system = _SYSTEM.get(code[0], "")
    if code[1] == "1" or (code[0] == "P" and code[1] == "3" and code[2] in "0123"):
        return "%s, herstellerspezifisch" % system
    if code[0] == "P" and code[1] in "02":
        return "Antrieb: %s" % _P_AREA.get(code[2], "genormt")
    return "%s, genormt" % system
