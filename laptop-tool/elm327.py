"""Kommunikation mit dem ELM327 über einen (virtuellen) COM-Port.

Ablauf wie im ESP32-Sketch:
  ATZ, ATE0, ATL0, ATS0, ATH0, ATSP0, ATAT1, dann 0100 (löst die automatische
  Protokollsuche aus) und ATDPN (welches Protokoll wurde gefunden).
  Live-Werte gehen per ATSH7E0 direkt an das Motorsteuergerät (nötig für die
  Öltemperatur über Mode 22), Fehlercodes per ATSH7DF an alle Steuergeräte.
"""

import re
import time

import obd_decode as dec

try:
    import serial
    from serial.tools import list_ports
except ImportError:          # pyserial fehlt: Demo-Modus geht trotzdem
    serial = None
    list_ports = None

BAUDRATES = (38400, 9600, 115200, 57600, 230400)

# Antworten, bei denen der Adapter zwar da ist, das Auto aber nicht antwortet
ECU_ERRORS = ("NODATA", "UNABLE", "ERROR", "STOPPED", "?", "BUFFERFULL", "BUSBUSY")


class ElmError(Exception):
    """Fehler mit verständlicher Meldung für die Oberfläche."""


def list_serial_ports():
    """[(Gerät, Beschreibung)], z.B. [('COM3', 'Prolific USB-to-Serial Comm Port (COM3)')]"""
    if list_ports is None:
        return []
    ports = sorted(list_ports.comports(), key=lambda p: (len(p.device), p.device))
    return [(p.device, p.description or "") for p in ports]


def _open_error(port, exc):
    text = str(exc)
    if "PermissionError" in text or "Zugriff verweigert" in text or "Access is denied" in text:
        return ElmError("%s ist belegt. Läuft noch ein anderes OBD-Programm? "
                        "Das bitte schließen und nochmal verbinden." % port)
    if "FileNotFoundError" in text or "could not open port" in text:
        return ElmError("%s gibt es nicht (mehr). Ist der USB-Wandler eingesteckt? "
                        "Liste mit ↻ aktualisieren." % port)
    return ElmError("%s lässt sich nicht öffnen: %s" % (port, text))


def _probe(ser):
    """Prüfen, ob bei der eingestellten Baudrate ein ELM327 antwortet."""
    try:
        ser.reset_input_buffer()
        ser.write(b"\r")
        time.sleep(0.3)
        ser.reset_input_buffer()
        ser.write(b"ATI\r")
        buf = bytearray()
        end = time.monotonic() + 1.5
        while time.monotonic() < end and b">" not in buf:
            buf += ser.read(ser.in_waiting or 1)
    except Exception:
        return False
    text = buf.decode("ascii", "replace").upper()
    return "ELM" in text or "OBD" in text or "STN" in text


def open_adapter(port, baud="auto", log=None, progress=None):
    """COM-Port öffnen und Baudrate suchen. Liefert ein ELM327-Objekt (noch nicht initialisiert)."""
    if serial is None:
        raise ElmError("pyserial ist nicht installiert. Bitte start.bat benutzen "
                       "oder 'py -m pip install pyserial' ausführen.")
    bauds = BAUDRATES if str(baud).lower() == "auto" else (int(baud),)
    for b in bauds:
        if progress:
            progress("Prüfe %s mit %d Baud ..." % (port, b))
        try:
            ser = serial.Serial(port, b, timeout=0.05, write_timeout=2)
        except (serial.SerialException, OSError, ValueError) as e:
            raise _open_error(port, e)
        if _probe(ser):
            elm = ELM327(ser, log=log)
            elm.baud = b
            return elm
        ser.close()
    raise ElmError("Auf %s antwortet kein ELM327 (probiert: %s Baud). Richtiger Port? "
                   "Adapter am Auto eingesteckt (er bekommt seinen Strom vom OBD-Stecker)?"
                   % (port, ", ".join(str(b) for b in bauds)))


class ELM327:
    def __init__(self, port, log=None):
        self.port = port            # pyserial-Objekt oder Simulator
        self.log = log              # Funktion(text) für das Terminal-Fenster
        self.baud = None
        self.version = ""
        self.protocol = None        # z.B. '6', None = noch unbekannt
        self.is_can = True
        self.header_phys = None     # True = 7E0, False = 7DF, None = unbekannt
        self.silent = 0             # Befehle hintereinander ganz ohne Antwort
        self.ecu_fail = 0           # Abfragen hintereinander ohne gültige Daten

    # ── Grundfunktionen ──────────────────────────────────────────────

    def close(self):
        try:
            self.port.close()
        except Exception:
            pass

    def send(self, cmd, timeout=2.5):
        """Befehl schicken und Antwort bis zum Prompt '>' lesen."""
        try:
            self.port.reset_input_buffer()
            self.port.write((cmd + "\r").encode("ascii"))
            buf = bytearray()
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                chunk = self.port.read(self.port.in_waiting or 1)
                if chunk:
                    buf += chunk
                    if b">" in chunk:
                        break
        except Exception as e:      # USB-Wandler abgezogen o.ä.
            raise ElmError("Verbindung zum Adapter verloren: %s" % e)
        got_prompt = b">" in buf
        text = buf.split(b">")[0].decode("ascii", "replace").replace("\r", "\n")
        lines = [l.strip() for l in text.split("\n") if l.strip()]
        if lines and lines[0].replace(" ", "").upper() == cmd.replace(" ", "").upper():
            lines = lines[1:]       # Echo (falls ATE0 nicht gegriffen hat)
        resp = "\n".join(lines)
        if resp or got_prompt:
            self.silent = 0
        else:
            self.silent += 1
        if self.log:
            self.log("> %s\n%s" % (cmd, resp if resp else "(keine Antwort)"))
        return resp

    def init(self):
        """Adapter zurücksetzen und einstellen. Liefert ein Info-Dict."""
        self.header_phys = None
        self.protocol = None
        ver = self.send("ATZ", 3.0)
        if not ver:
            raise ElmError("Adapter antwortet nicht auf ATZ.")
        m = re.search(r"(ELM327|STN\w*|OBD\w*)[^\n]*", ver, re.I)
        self.version = m.group(0).strip() if m else ver.split("\n")[-1]
        for c in ("ATE0", "ATL0", "ATS0", "ATH0", "ATSP0", "ATAT1"):
            self.send(c, 1.5)
        # erste Anfrage startet die automatische Protokollsuche (kann einige Sekunden dauern)
        first = self.send("0100", 8.0)
        self.detect_protocol()
        return {
            "version": self.version,
            "baud": self.baud,
            "protocol": self.protocol,
            "protocol_name": dec.PROTOCOLS.get(self.protocol, "noch unbekannt"),
            "ecu_ok": dec.parse_obd(first, "4100") is not None,
            "voltage": self.voltage(),
        }

    def detect_protocol(self):
        dp = self.send("ATDPN", 1.0).strip().upper()
        if dp.startswith("A"):
            dp = dp[1:]                 # "A6" = automatisch gefunden, Protokoll 6
        if len(dp) == 1 and dp in dec.PROTOCOLS:
            self.protocol = dp
            self.is_can = dec.protocol_is_can(dp)
        return self.protocol

    def set_header(self, phys):
        """CAN-Adressierung: 7E0 = Motorsteuergerät direkt, 7DF = Rundruf an alle."""
        if not self.protocol or not self.is_can or phys == self.header_phys:
            return
        r = self.send("ATSH7E0" if phys else "ATSH7DF", 1.0)
        if "OK" in r.upper():
            self.header_phys = phys

    def query(self, cmd, tag, timeout=2.5):
        """OBD-Abfrage, liefert die Datenbytes hinter 'tag' oder None."""
        r = self.send(cmd, timeout)
        data = dec.parse_obd(r, tag)
        self._count(r, data is not None)
        return data

    def query_all(self, cmd, tag, timeout=2.5):
        r = self.send(cmd, timeout)
        data = dec.parse_obd_all(r, tag)
        self._count(r, bool(data))
        return data

    def _count(self, resp, ok):
        if ok:
            self.ecu_fail = 0
            if not self.protocol:       # Auto antwortet jetzt -> Protokoll nachholen
                self.detect_protocol()
        elif resp and any(e in resp.replace(" ", "").upper() for e in ECU_ERRORS):
            self.ecu_fail += 1

    def uds_read(self, did, timeout=1.5):
        """Mode 22 an das Motorsteuergerät. Liefert (Status, Daten/NRC)."""
        self.set_header(True)
        r = self.send("22%04X" % did, timeout)
        st, val = dec.parse_uds(r, did)
        self._count(r, st == dec.UDS_OK)
        return st, val

    def voltage(self):
        return dec.parse_voltage(self.send("ATRV", 1.5))

    # ── Fehlercodes ──────────────────────────────────────────────────

    def read_dtcs(self):
        """Gespeicherte, anstehende und permanente Fehlercodes aller Steuergeräte."""
        was = self.header_phys
        self.set_header(False)
        try:
            status = self.query_all("0101", "4101", 3.0)
            if not status:
                raise ElmError("Das Auto antwortet nicht. Zündung an?")
            mil = any(s and s[0] & 0x80 for s in status)
            count = sum(s[0] & 0x7F for s in status if s)
            res = {"mil": mil, "count": count}
            for mode, svc, key in (("03", "43", "stored"), ("07", "47", "pending"),
                                   ("0A", "4A", "permanent")):
                res[key] = dec.parse_dtcs(self.send(mode, 5.0), self.is_can, svc)
            if res["stored"] is None:
                raise ElmError("Ungültige Antwort beim Lesen der Fehlercodes (siehe Terminal).")
            return res
        finally:
            if was is not None:
                self.set_header(was)

    def clear_dtcs(self):
        """Mode 04: Fehlerspeicher löschen (alle abgasrelevanten Steuergeräte)."""
        was = self.header_phys
        self.set_header(False)
        try:
            r = self.send("04", 5.0)
        finally:
            if was is not None:
                self.set_header(was)
        msgs = dec.messages(r)
        if any(m.startswith("44") for m in msgs):
            return True
        if any("7F04" in m for m in msgs):
            raise ElmError("Das Steuergerät hat das Löschen abgelehnt. "
                           "Motor aus, Zündung an und nochmal versuchen.")
        raise ElmError("Keine Bestätigung vom Auto (Antwort: %s)." % (r or "keine"))

    # ── Fahrzeugdaten ────────────────────────────────────────────────

    def read_vin(self):
        return dec.parse_vin(self.send("0902", 5.0))

    def read_supported(self):
        """Alle unterstützten Mode-01-PIDs (über 0100, 0120, 0140, ...)."""
        pids = set()
        for base in range(0x00, 0xE0, 0x20):
            answers = self.query_all("01%02X" % base, "41%02X" % base, 3.0)
            if not answers:
                break
            for a in answers:
                pids |= dec.parse_supported(a, base)
            if (base + 0x20) not in pids:
                break
        return pids

    # ── Direkter Zugriff aufs Motorsteuergerät (UDS, nur lesend) ────

    def need_uds(self):
        if not self.protocol:
            raise ElmError("Das Auto antwortet noch nicht. Zündung an und kurz warten.")
        if not self.is_can:
            raise ElmError("Direkter Zugriff aufs Steuergerät geht nur bei CAN-Fahrzeugen.")

    def read_did(self, did):
        """Eine Kennnummer lesen. Liefert (Status, Daten/NRC) wie uds_read."""
        self.need_uds()
        return self.uds_read(did, 2.0)

    def read_ecu_info(self):
        """Teilenummer, Softwareversion usw. des Motorsteuergeräts."""
        self.need_uds()
        info = []
        for did, label in dec.ECU_INFO_DIDS:
            st, val = self.uds_read(did, 2.0)
            if st == dec.UDS_OK:
                text = dec.ascii_text(val) or " ".join("%02X" % b for b in val)
                info.append((label, text))
        return info

    def read_uds_dtcs(self):
        """Fehlerspeicher des Motorsteuergeräts (UDS 19 02, alle Statusbits)."""
        self.need_uds()
        self.set_header(True)
        r = self.send("1902FF", 6.0)
        st, val = dec.parse_uds_dtcs(r)
        if st == dec.UDS_OK:
            return val
        if st == dec.UDS_NEG:
            raise ElmError("Das Motorsteuergerät liefert seinen Fehlerspeicher nicht "
                           "(%s)." % dec.nrc_text(val))
        raise ElmError("Keine Antwort vom Motorsteuergerät. Zündung an?")

    def scan_dids(self, lo, hi):
        """Kennnummern lo..hi durchprobieren (wie die Öl-Suche am Display).

        Generator: liefert regelmäßig ('progress', did, Anteil) und für jede
        Kennnummer, die antwortet, ('hit', did, daten). Mode 22 liest nur.
        """
        self.need_uds()
        self.set_header(True)
        silent = nrc11 = hits = 0
        total = hi - lo + 1
        for did in range(lo, hi + 1):
            st, val = self.uds_read(did, 1.0)
            if st == dec.UDS_OK:
                silent = 0
                hits += 1
                yield ("hit", did, val)
            elif st == dec.UDS_NEG:
                silent = 0
                if val == 0x11:
                    nrc11 += 1
            else:
                silent += 1
            if silent >= 20:
                raise ElmError("Das Motorsteuergerät antwortet nicht mehr. Zündung an?")
            if nrc11 >= 20 and hits == 0:
                raise ElmError("Das Motorsteuergerät unterstützt das Lesen von Kennnummern nicht.")
            if (did - lo) % 16 == 15 or did == hi:
                yield ("progress", did, (did - lo + 1) / float(total))
