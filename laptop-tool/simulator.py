"""Simulierter ELM327 für den Demo-Modus und die Tests.

Verhält sich nach außen wie ein pyserial-Port (write/read/in_waiting/...)
und antwortet wie ein ELM327 an einem Fabia 3 mit CAN-Bus: Motor läuft in
einer 20-Sekunden-Schleife Leerlauf, Beschleunigen, Schalten, Ausrollen,
Öl und Kühlwasser werden langsam warm. Ein paar Fehlercodes sind gespeichert.
"""

import math
import random
import threading
import time

VIN = "DEMOFAHRZEUG12345"

# Unterstützte Mode-01-PIDs (Bitmasken für 0100/0120/0140)
SUPPORTED = {0x01, 0x03, 0x04, 0x05, 0x06, 0x07, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x11,
             0x13, 0x15, 0x1C, 0x1F, 0x20, 0x21, 0x2E, 0x2F, 0x30, 0x31, 0x33, 0x3C,
             0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x49, 0x4A, 0x4C}


def _code_bytes(code):
    b1 = ("PCBU".index(code[0]) << 6) | (int(code[1]) << 4) | int(code[2], 16)
    return [b1, int(code[3:], 16)]


class SimulatedELM:
    def __init__(self, speed=1.0, ignition=True):
        self.speed = speed          # Zeitraffer für Tests
        self.ignition = ignition    # False = Zündung aus, Auto antwortet nicht
        self.t0 = time.monotonic()
        self.timeout = 0.05
        self.is_open = True
        self._out = bytearray()
        self._inbuf = ""
        self._lock = threading.Lock()
        self.stored = ["P0299", "P0420", "P0016"]
        self.pending = ["P0301"]
        self.permanent = ["P0420"]
        self._reset()

    def _reset(self):
        self.echo = True
        self.spaces = True
        self.headers = False
        self.protocol = None        # erst nach der ersten Anfrage gefunden
        self.header = "7DF"

    # ── pyserial-Schnittstelle ───────────────────────────────────────

    @property
    def in_waiting(self):
        with self._lock:
            return len(self._out)

    def reset_input_buffer(self):
        with self._lock:
            self._out.clear()

    def close(self):
        self.is_open = False

    def write(self, data):
        self._inbuf += data.decode("ascii", "replace")
        while "\r" in self._inbuf:
            cmd, self._inbuf = self._inbuf.split("\r", 1)
            resp = self._handle(cmd.strip())
            time.sleep(0.015)                  # Antwortzeit wie beim echten Adapter
            text = (cmd + "\r" if self.echo else "") + resp + "\r\r>"
            with self._lock:
                self._out += text.encode("ascii")
        return len(data)

    def read(self, size=1):
        end = time.monotonic() + (self.timeout or 0)
        while True:
            with self._lock:
                if self._out:
                    chunk = bytes(self._out[:size])
                    del self._out[:size]
                    return chunk
            if time.monotonic() >= end:
                return b""
            time.sleep(0.005)

    # ── Antworten ────────────────────────────────────────────────────

    def _fmt(self, data):
        h = ["%02X" % b for b in data]
        return " ".join(h) if self.spaces else "".join(h)

    def _frames(self, data):
        """Wie der ELM327 eine CAN-Antwort ausgibt (ein oder mehrere Frames)."""
        if len(data) <= 7:
            return self._fmt(data)
        lines = ["%03X" % len(data)]
        lines.append("0:" + (" " if self.spaces else "") + self._fmt(data[:6]))
        rest, n = data[6:], 1
        while rest:
            part = (rest[:7] + [0] * 7)[:7]
            lines.append("%X:" % (n & 0xF) + (" " if self.spaces else "") + self._fmt(part))
            rest, n = rest[7:], n + 1
        return "\r".join(lines)

    def _handle(self, cmd):
        c = cmd.replace(" ", "").upper()
        if not c:
            return ""
        if c.startswith("AT"):
            return self._at(c[2:])
        if not all(ch in "0123456789ABCDEF" for ch in c) or len(c) % 2:
            return "?"
        prefix = ""
        if not self.ignition:
            return "NO DATA" if self.protocol else "SEARCHING...\rUNABLE TO CONNECT"
        if self.protocol is None:
            self.protocol = "6"
            prefix = "SEARCHING...\r"
        return prefix + self._obd(c)

    def _at(self, a):
        if a == "Z":
            self._reset()
            return "\r\rELM327 v1.5"
        if a in ("I", "WS"):
            return "ELM327 v1.5"
        if a == "RV":
            return "%.1fV" % self._state()["batt"]
        if a == "DPN":
            return ("A" + self.protocol) if self.protocol else "A0"
        if a == "DP":
            return "AUTO, ISO 15765-4 (CAN 11/500)" if self.protocol else "AUTO"
        if a in ("E0", "E1"):
            self.echo = a == "E1"
        elif a in ("S0", "S1"):
            self.spaces = a == "S1"
        elif a in ("H0", "H1"):
            self.headers = a == "H1"
        elif a.startswith("SH"):
            self.header = a[2:]
        elif not (a[:1] in ("L", "D") or a[:2] in ("SP", "TP", "AT", "ST", "LP", "CA")):
            return "?"
        return "OK"

    def _state(self):
        t = (time.monotonic() - self.t0) * self.speed
        warm = min(1.0, t / 300.0)
        p = t % 20.0
        if p < 4:                       # Leerlauf
            rpm, acc = 820.0, 0.0
        elif p < 12:                    # Beschleunigen
            rpm, acc = 1500 + (p - 4) / 8 * 3300, (p - 4) / 8
        elif p < 14:                    # geschaltet
            rpm, acc = 2600.0, 0.3
        else:                           # Ausrollen
            rpm, acc = 2600 - (p - 14) / 6 * 1780, 0.0
        rpm += random.uniform(-15, 15)
        return {
            "rpm": rpm,
            "speed": int(rpm / 38) if p >= 4 else 0,
            "load": 18 + 70 * acc,
            "throttle": 4 + 75 * acc,
            "map": 33 + 200 * acc ** 1.5,
            "baro": 99,
            "coolant": 20 + 70 * warm,
            "oil": 18 + 84 * min(1.0, t / 420.0),
            "iat": 24 + 14 * acc,
            "cat": 300 + 420 * warm + 90 * acc,
            "timing": 8 + 14 * math.sin(p / 3.2) - 6 * acc,
            "batt": 14.2 + random.uniform(-0.05, 0.05),
        }

    def _obd(self, c):
        mode, rest = c[:2], c[2:]
        s = self._state()
        tcm = "\r" + self._fmt([int(mode, 16) + 0x40, 0]) if self.header == "7DF" else ""

        if mode == "01" and len(rest) == 2:
            pid = int(rest, 16)
            if pid % 0x20 == 0:
                if pid != 0 and pid not in SUPPORTED:
                    return "NO DATA"
                bits = 0
                for i in range(32):
                    if pid + i + 1 in SUPPORTED:
                        bits |= 1 << (31 - i)
                return self._fmt([0x41, pid] + list(bits.to_bytes(4, "big")))
            if pid not in SUPPORTED:
                return "NO DATA"
            data = {
                0x01: [0x80 | len(self.stored), 0x07, 0x65, 0x00] if self.stored else [0, 0x07, 0x65, 0],
                0x04: [int(s["load"] * 255 / 100)],
                0x05: [int(s["coolant"] + 40)],
                0x0B: [min(255, int(s["map"]))],
                0x0C: list(int(s["rpm"] * 4).to_bytes(2, "big")),
                0x0D: [s["speed"]],
                0x0E: [max(0, min(255, int((s["timing"] + 64) * 2)))],
                0x0F: [int(s["iat"] + 40)],
                0x11: [int(s["throttle"] * 255 / 100)],
                0x33: [s["baro"]],
                0x3C: list(int((s["cat"] + 40) * 10).to_bytes(2, "big")),
            }.get(pid, [0, 0])
            return self._fmt([0x41, pid] + data)

        if mode in ("03", "07", "0A") and not rest:
            codes = {"03": self.stored, "07": self.pending, "0A": self.permanent}[mode]
            data = [int(mode, 16) + 0x40, len(codes)]
            for code in codes:
                data += _code_bytes(code)
            return self._frames(data) + tcm

        if mode == "04" and not rest:
            self.stored, self.pending = [], []      # permanente Codes bleiben stehen
            return "44" + ("\r44" if self.header == "7DF" else "")

        if c == "0902":
            return self._frames([0x49, 0x02, 0x01] + [ord(ch) for ch in VIN])

        if mode == "22" and len(rest) == 4:
            if self.header != "7E0":
                return "NO DATA"
            if rest == "11BD":
                k10 = int((s["oil"] + 273.1) * 10)
                return self._fmt([0x62, 0x11, 0xBD] + list(k10.to_bytes(2, "big")))
            return self._fmt([0x7F, 0x22, 0x31])

        return "NO DATA"
