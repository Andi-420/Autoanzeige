"""Live-Werte zyklisch abfragen (gleiche Werte und Formeln wie auf dem Display).

Schnelle Werte (Drehzahl, Ladedruck, Geschwindigkeit) kommen in jedem Zyklus,
langsame nur alle 0,25 / 1 / 10 s. Liefert das Auto einen Wert nicht, wird er
nach 5 Fehlversuchen nur noch alle 30 s erneut probiert.
"""

import time

import obd_decode as dec
from elm327 import ElmError

OIL_DID = 0x11BD        # Öltemperatur im Motorsteuergerät (Fabia 3, Kelvin/10)

# (Schlüssel, Bezeichnung, Einheit, Nachkommastellen) – Reihenfolge = Anzeige und CSV
FIELDS = [
    ("rpm", "Drehzahl", "U/min", 0),
    ("boost", "Ladedruck", "bar", 2),
    ("oil", "Öltemperatur", "°C", 0),
    ("coolant", "Kühlwasser", "°C", 0),
    ("iat", "Ansaugluft", "°C", 0),
    ("cat", "Katalysator", "°C", 0),
    ("timing", "Zündwinkel", "° v. OT", 1),
    ("batt", "Batterie", "V", 1),
    ("speed", "Geschwindigkeit", "km/h", 0),
    ("load", "Motorlast", "%", 0),
    ("throttle", "Drosselklappe", "%", 0),
    ("map", "Saugrohrdruck", "kPa", 0),
    ("baro", "Umgebungsdruck", "kPa", 0),
]

MAX_FAIL = 5
RETRY_S = 30.0


class _Item:
    def __init__(self, key, interval, fn):
        self.key, self.interval, self.fn = key, interval, fn
        self.due = 0.0
        self.fail = 0


class LiveReader:
    def __init__(self, elm):
        self.elm = elm
        self.values = {k: None for k, *_ in FIELDS}
        self.boost_max = None
        self.oil_source = "22 11BD" if elm.is_can else "015C"
        self._map87 = None      # None = noch nicht geprüft
        self._items = [
            _Item("rpm", 0, self._rpm),
            _Item("speed", 0, lambda: self._pid1("0D", lambda d: d[0])),
            _Item("map", 0, self._map),
            _Item("timing", 0.25, lambda: self._pid1("0E", lambda d: d[0] / 2.0 - 64)),
            _Item("load", 0.5, lambda: self._pid1("04", lambda d: d[0] * 100 / 255.0)),
            _Item("throttle", 0.5, lambda: self._pid1("11", lambda d: d[0] * 100 / 255.0)),
            _Item("coolant", 1, lambda: self._pid1("05", lambda d: d[0] - 40)),
            _Item("iat", 1, lambda: self._pid1("0F", lambda d: d[0] - 40)),
            _Item("cat", 1, lambda: self._pid1("3C", lambda d: ((d[0] << 8) | d[1]) / 10.0 - 40, 2)),
            _Item("oil", 1, self._oil),
            _Item("batt", 1, self.elm.voltage),
            _Item("baro", 10, self._baro),
        ]

    # ── einzelne Werte ───────────────────────────────────────────────

    def _pid1(self, pid, fn, need=1, timeout=2.5):
        d = self.elm.query("01" + pid, "41" + pid, timeout)
        if d is None or len(d) < need:
            return None
        return fn(d)

    def _rpm(self, timeout=2.5):
        return self._pid1("0C", lambda d: ((d[0] << 8) | d[1]) / 4.0, 2, timeout)

    def _map(self):
        # PID 0x0B hat nur 1 Byte (max. 255 kPa, also ca. +1,55 bar Ladedruck).
        # PID 0x87 liefert 1/32 kPa bis 2048 kPa, falls das Auto ihn kennt.
        if self._map87 is None and self.elm.protocol:
            d = self.elm.query("0180", "4180")
            self._map87 = bool(d) and bool(d[0] & 0x02)
        if self._map87:
            d = self.elm.query("0187", "4187")
            if d and len(d) >= 3 and d[0] & 0x01:
                return ((d[1] << 8) | d[2]) / 32.0
        return self._pid1("0B", lambda d: float(d[0]))

    def _baro(self):
        v = self._pid1("33", lambda d: float(d[0]))
        return v if v is not None and v > 50 else None

    def _oil(self):
        if self.oil_source == "22 11BD":
            st, val = self.elm.uds_read(OIL_DID)
            if st == dec.UDS_OK and len(val) >= 2:
                return ((val[0] << 8) | val[1]) / 10.0 - 273.1
            if st == dec.UDS_NEG or not self.elm.is_can:
                self.oil_source = "015C"    # Steuergerät kennt die DID nicht
            return None
        return self._pid1("5C", lambda d: d[0] - 40)

    # ── Zyklus ───────────────────────────────────────────────────────

    def cycle(self):
        """Ein Durchlauf: fällige Werte abfragen. Liefert eine Kopie der Werte."""
        if not self.elm.protocol:
            # Auto hat noch nie geantwortet (Zündung aus): nur Drehzahl und Spannung
            # probieren, mit langer Wartezeit für die automatische Protokollsuche
            self.values["rpm"] = self._rpm(timeout=8.0)
            self.values["batt"] = self.elm.voltage()
            if self.elm.silent >= 3:
                raise ElmError("Der Adapter antwortet nicht mehr. Kabel prüfen und neu verbinden.")
            if not self.elm.protocol:
                return self._result(ecu_ok=False)
        if self.elm.is_can:
            self.elm.set_header(True)   # Live-Werte direkt vom Motorsteuergerät
        for it in self._items:
            now = time.monotonic()
            if now < it.due:
                continue
            v = it.fn()
            self.values[it.key] = v
            if v is None:
                it.fail += 1
            else:
                it.fail = 0
            it.due = now + (RETRY_S if it.fail >= MAX_FAIL else it.interval)
            if self.elm.silent >= 3:
                raise ElmError("Der Adapter antwortet nicht mehr. Kabel prüfen und neu verbinden.")

        baro = self.values["baro"] or 101.3     # bis zum ersten Messwert Normaldruck
        mp = self.values["map"]
        if mp is not None:
            boost = (mp - baro) / 100.0
            self.values["boost"] = boost
            if self.boost_max is None or boost > self.boost_max:
                self.boost_max = boost
        else:
            self.values["boost"] = None

        return self._result(self.elm.ecu_fail < 4)

    def _result(self, ecu_ok):
        out = dict(self.values)
        out["_boost_max"] = self.boost_max
        out["_oil_source"] = self.oil_source
        out["_ecu_ok"] = ecu_ok
        return out

    def reset_max(self):
        self.boost_max = None
