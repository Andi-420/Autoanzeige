#!/usr/bin/env python3
"""OBD-Tool für den Laptop: Fehlercodes und Live-Werte über einen ELM327.

Start unter Windows: start.bat doppelklicken (oder: py obd_tool.py)
Ohne Auto ausprobieren: Anschluss "Demo (ohne Auto)" wählen, oder py obd_tool.py --demo
"""

import csv
import datetime
import json
import os
import queue
import sys
import threading
import time
import traceback
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from tkinter import font as tkfont

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import obd_decode as dec                                          # noqa: E402
from elm327 import BAUDRATES, ELM327, ElmError, list_serial_ports, open_adapter  # noqa: E402
from live import FIELDS, LiveReader                               # noqa: E402
from simulator import SimulatedELM                                # noqa: E402

APP_TITLE = "OBD-Tool (ELM327)"
DEMO_PORT = "Demo (ohne Auto)"
SETTINGS = os.path.join(os.path.expanduser("~"), ".obd_tool.json")
TILE_HIDDEN = {"baro"}      # steht klein unter dem Ladedruck (in der CSV-Datei als eigene Spalte)

C_TEXT, C_DIM = "#1d232b", "#6b7380"
C_BLUE, C_TEAL, C_GREEN = "#1f6fd1", "#11898a", "#1c8c3a"
C_ORANGE, C_RED = "#d27400", "#c62828"
C_BG, C_TILE, C_BORDER = "#eef0f3", "#ffffff", "#d3d7dd"


def fmt(v, nd, sign=False):
    """Zahl mit deutschem Komma, '–' wenn kein Wert."""
    if v is None:
        return "–"
    s = ("%+.*f" if sign else "%.*f") % (nd, v)
    return s.replace(".", ",")


def value_color(key, v):
    """Farben wie auf dem Display."""
    if v is None:
        return C_DIM
    if key == "oil":
        return C_BLUE if v < 60 else C_TEAL if v < 80 else C_GREEN if v <= 105 else C_RED
    if key == "cat":
        return C_BLUE if v < 300 else C_GREEN if v <= 850 else C_ORANGE if v <= 950 else C_RED
    if key == "iat":
        return C_BLUE if v < 50 else C_ORANGE if v < 65 else C_RED
    if key == "timing" and v < 0:
        return C_ORANGE
    if key == "batt" and v < 12.0:
        return C_ORANGE
    return C_TEXT


# ══════════════════════════════════════════════════════════════════════
# Hintergrund-Thread: die ganze Kommunikation mit dem Adapter läuft hier,
# damit das Fenster nicht einfriert. Aufträge kommen über eine Queue,
# Ergebnisse gehen über eine zweite Queue zurück an das Fenster.
# ══════════════════════════════════════════════════════════════════════
class Worker(threading.Thread):
    def __init__(self, events):
        super().__init__(daemon=True)
        self.events = events
        self.jobs = queue.Queue()
        self.elm = None
        self.live = None
        self.log_all = False        # auch die Live-Abfragen im Terminal zeigen
        self._quiet = False

    def post(self, kind, data=None):
        self.events.put((kind, data))

    def submit(self, name, **kw):
        self.jobs.put((name, kw))

    def _log(self, text):
        if self.log_all or not self._quiet:
            self.post("log", text)

    def run(self):
        while True:
            try:
                name, kw = self.jobs.get_nowait() if self.live else self.jobs.get(timeout=0.2)
            except queue.Empty:
                name = None
            if name == "quit":
                self._close()
                return
            if name:
                try:
                    getattr(self, "do_" + name)(**kw)
                except ElmError as e:
                    self.post("error", str(e))
                except Exception as e:
                    self.post("log", traceback.format_exc())
                    self.post("error", "Unerwarteter Fehler: %s" % e)
                self.post("idle")
            if self.live and self.elm:
                self._live_cycle()

    def _live_cycle(self):
        self._quiet = True
        t = time.monotonic()
        try:
            vals = self.live.cycle()
        except ElmError as e:
            self.live = None
            self.post("live_state", False)
            self.post("error", str(e))
            return
        finally:
            self._quiet = False
        vals["_cycle"] = time.monotonic() - t
        self.post("live", vals)

    def _need(self):
        if not self.elm:
            raise ElmError("Nicht verbunden.")

    def _close(self):
        self.live = None
        if self.elm:
            self.elm.close()
            self.elm = None

    # ── Aufträge ─────────────────────────────────────────────────────

    def do_connect(self, port, baud):
        self._close()
        try:
            if port == DEMO_PORT:
                elm = ELM327(SimulatedELM(), log=self._log)
                elm.baud = "Demo"
            else:
                elm = open_adapter(port, baud, log=self._log,
                                   progress=lambda t: self.post("busy", t))
            self.post("busy", "Adapter gefunden, stelle ihn ein. Die Protokollsuche "
                              "kann bis zu 10 Sekunden dauern ...")
            try:
                info = elm.init()
            except Exception:
                elm.close()
                raise
        except Exception:
            self.post("disconnected", None)
            raise
        self.elm = elm
        info["port"] = port
        self.post("connected", info)

    def do_disconnect(self):
        self._close()
        self.post("live_state", False)
        self.post("disconnected", "Getrennt.")

    def do_live(self, on):
        self._need()
        self.live = LiveReader(self.elm) if on else None
        self.post("live_state", bool(on))

    def do_reset_max(self):
        if self.live:
            self.live.reset_max()

    def do_read_dtcs(self):
        self._need()
        self.post("busy", "Lese Fehlercodes ...")
        self.post("dtcs", self.elm.read_dtcs())

    def do_clear_dtcs(self):
        self._need()
        self.post("busy", "Lösche Fehlercodes ...")
        self.elm.clear_dtcs()
        self.post("cleared")
        time.sleep(1.0)             # Steuergerät kurz Zeit lassen
        self.do_read_dtcs()

    def do_vehicle(self):
        self._need()
        self.post("busy", "Lese Fahrzeugdaten ...")
        info = {
            "vin": self.elm.read_vin(),
            "pids": sorted(self.elm.read_supported()),
            "voltage": self.elm.voltage(),
            "version": self.elm.version,
            "baud": self.elm.baud,
            "protocol_name": dec.PROTOCOLS.get(self.elm.protocol, "noch unbekannt"),
        }
        self.post("vehicle", info)

    def do_raw(self, cmd):
        self._need()
        self.elm.send(cmd, 5.0)     # Antwort landet über _log im Terminal
        up = cmd.replace(" ", "").upper()
        if up.startswith(("ATZ", "ATD", "ATWS", "ATSH")):
            self.elm.header_phys = None     # Header-Stand unbekannt -> neu setzen
        if up.startswith(("ATZ", "ATD", "ATWS", "ATH1", "ATE1")):
            self.post("log", "Hinweis: Dieser Befehl verstellt den Adapter. "
                             "Am besten danach neu verbinden.")


# ══════════════════════════════════════════════════════════════════════
# Fenster
# ══════════════════════════════════════════════════════════════════════
class App:
    def __init__(self, root, autostart_demo=False):
        self.root = root
        self.events = queue.Queue()
        self.worker = Worker(self.events)
        self.worker.start()
        self.connected = False
        self.live_on = False
        self.rec = None             # (Datei, csv.writer, Startzeit, Zeilen)
        self.history, self.hist_pos = [], 0
        self.settings = self._load_settings()

        root.title(APP_TITLE)
        root.geometry("1000x680")
        root.minsize(820, 560)
        root.protocol("WM_DELETE_WINDOW", self.on_close)

        base = tkfont.nametofont("TkDefaultFont").actual("family")
        self.f_small = tkfont.Font(family=base, size=9)
        self.f_title = tkfont.Font(family=base, size=10, weight="bold")
        self.f_value = tkfont.Font(family=base, size=26, weight="bold")
        self.f_mono = tkfont.nametofont("TkFixedFont").copy()
        self.f_mono.configure(size=10)

        self._build_top()
        nb = ttk.Notebook(root)
        nb.pack(fill="both", expand=True, padx=8, pady=(0, 4))
        self.nb = nb
        self._build_live(nb)
        self._build_dtc(nb)
        self._build_vehicle(nb)
        self._build_terminal(nb)
        bottom = ttk.Frame(root)
        bottom.pack(fill="x", side="bottom")
        self.conn_lbl = ttk.Label(bottom, text="", foreground=C_DIM, padding=(10, 2))
        self.conn_lbl.pack(side="right")
        self.status = ttk.Label(bottom, text="Bereit. Anschluss wählen und verbinden.",
                                anchor="w", padding=(10, 2))
        self.status.pack(side="left", fill="x", expand=True)

        self.refresh_ports()
        self._set_connected(False)
        root.after(50, self._poll)
        if autostart_demo:
            self.port_var.set(DEMO_PORT)
            root.after(200, self.toggle_connect)

    # ── Aufbau ───────────────────────────────────────────────────────

    def _build_top(self):
        top = ttk.Frame(self.root, padding=(8, 8, 8, 4))
        top.pack(fill="x")
        ttk.Label(top, text="Anschluss:").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(top, textvariable=self.port_var, width=48, state="readonly")
        self.port_cb.pack(side="left", padx=(4, 2))
        ttk.Button(top, text="↻", width=3, command=self.refresh_ports).pack(side="left")
        ttk.Label(top, text="Baud:").pack(side="left", padx=(12, 0))
        self.baud_var = tk.StringVar(value=self.settings.get("baud", "Auto"))
        ttk.Combobox(top, textvariable=self.baud_var, width=8, state="readonly",
                     values=["Auto"] + [str(b) for b in BAUDRATES]).pack(side="left", padx=4)
        self.conn_btn = ttk.Button(top, text="Verbinden", width=12, command=self.toggle_connect)
        self.conn_btn.pack(side="left", padx=(12, 0))

    def _build_live(self, nb):
        tab = ttk.Frame(nb, padding=8)
        nb.add(tab, text="  Live-Werte  ")
        bar = ttk.Frame(tab)
        bar.pack(fill="x", pady=(0, 8))
        self.live_btn = ttk.Button(bar, text="▶  Live starten", width=16, command=self.toggle_live)
        self.live_btn.pack(side="left")
        self.max_btn = ttk.Button(bar, text="Max zurücksetzen",
                                  command=lambda: self.worker.submit("reset_max"))
        self.max_btn.pack(side="left", padx=6)
        self.rec_btn = ttk.Button(bar, text="●  CSV-Aufzeichnung", command=self.toggle_rec)
        self.rec_btn.pack(side="left")
        self.rec_lbl = ttk.Label(bar, text="", foreground=C_DIM)
        self.rec_lbl.pack(side="left", padx=8)
        self.rate_lbl = ttk.Label(bar, text="", foreground=C_DIM)
        self.rate_lbl.pack(side="right")

        grid = tk.Frame(tab, bg=C_BG, padx=6, pady=6)
        grid.pack(fill="both", expand=True)
        self.tiles = {}
        cols = 4
        shown = [f for f in FIELDS if f[0] not in TILE_HIDDEN]
        for i, (key, name, unit, nd) in enumerate(shown):
            r, c = divmod(i, cols)
            t = tk.Frame(grid, bg=C_TILE, highlightthickness=1, highlightbackground=C_BORDER,
                         padx=10, pady=6)
            t.grid(row=r, column=c, sticky="nsew", padx=4, pady=4)
            tk.Label(t, text=name, bg=C_TILE, fg=C_DIM, font=self.f_title).pack(anchor="w")
            row = tk.Frame(t, bg=C_TILE)
            row.pack(anchor="w")
            val = tk.Label(row, text="–", bg=C_TILE, fg=C_DIM, font=self.f_value)
            val.pack(side="left")
            tk.Label(row, text=unit, bg=C_TILE, fg=C_DIM, font=self.f_title).pack(
                side="left", anchor="s", pady=(0, 8), padx=(4, 0))
            sub = tk.Label(t, text="", bg=C_TILE, fg=C_DIM, font=self.f_small, justify="left")
            sub.pack(anchor="w")
            self.tiles[key] = (val, sub, nd)
        for c in range(cols):
            grid.columnconfigure(c, weight=1, uniform="tile")
        for r in range((len(shown) + cols - 1) // cols):
            grid.rowconfigure(r, weight=1)
        ttk.Label(tab, foreground=C_DIM, wraplength=900, justify="left", text=(
            "Ladedruck = Saugrohrdruck − Umgebungsdruck. Öltemperatur kommt beim Fabia vom "
            "Motorsteuergerät (22 11BD), sonst über die Standard-PID 5C. Werte, die das Auto "
            "nicht liefert, zeigen „–“ und werden nur noch alle 30 s erneut versucht.")
        ).pack(fill="x", pady=(6, 0))

    def _build_dtc(self, nb):
        tab = ttk.Frame(nb, padding=8)
        nb.add(tab, text="  Fehlercodes  ")
        bar = ttk.Frame(tab)
        bar.pack(fill="x", pady=(0, 8))
        self.dtc_btn = ttk.Button(bar, text="Fehlercodes auslesen", command=self.read_dtcs)
        self.dtc_btn.pack(side="left")
        self.clr_btn = ttk.Button(bar, text="Fehlercodes löschen …", command=self.clear_dtcs)
        self.clr_btn.pack(side="left", padx=6)
        self.mil_lbl = ttk.Label(bar, text="", font=self.f_title)
        self.mil_lbl.pack(side="left", padx=12)

        frame = ttk.Frame(tab)
        frame.pack(fill="both", expand=True)
        self.dtc_tree = ttk.Treeview(frame, columns=("code", "art", "text"), show="headings")
        for col, txt, w in (("code", "Code", 90), ("art", "Art", 120), ("text", "Beschreibung", 600)):
            self.dtc_tree.heading(col, text=txt, anchor="w")
            self.dtc_tree.column(col, width=w, anchor="w", stretch=(col == "text"))
        sb = ttk.Scrollbar(frame, orient="vertical", command=self.dtc_tree.yview)
        self.dtc_tree.configure(yscrollcommand=sb.set)
        self.dtc_tree.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")
        self.dtc_info = ttk.Label(tab, text="", foreground=C_DIM, wraplength=900, justify="left")
        self.dtc_info.pack(fill="x", pady=(6, 0))
        ttk.Label(tab, foreground=C_DIM, wraplength=900, justify="left", text=(
            "Gespeichert = bestätigter Fehler (Mode 03), anstehend = im aktuellen oder letzten "
            "Fahrzyklus erkannt (Mode 07), permanent = lässt sich nicht löschen und verschwindet "
            "erst, wenn das Steuergerät den Fehler nicht mehr sieht (Mode 0A). Beschreibungen nur "
            "für genormte Codes und ohne Gewähr.")
        ).pack(fill="x", pady=(4, 0))

    def _build_vehicle(self, nb):
        tab = ttk.Frame(nb, padding=8)
        nb.add(tab, text="  Fahrzeug  ")
        bar = ttk.Frame(tab)
        bar.pack(fill="x", pady=(0, 8))
        self.veh_btn = ttk.Button(bar, text="Fahrzeugdaten lesen", command=self.read_vehicle)
        self.veh_btn.pack(side="left")

        info = ttk.Frame(tab)
        info.pack(fill="x", pady=(0, 8))
        self.veh_vals = {}
        for r, (key, txt) in enumerate((("vin", "Fahrgestellnummer"), ("version", "Adapter"),
                                         ("baud", "Baudrate"), ("protocol_name", "Protokoll"),
                                         ("voltage", "Batteriespannung"))):
            ttk.Label(info, text=txt + ":", foreground=C_DIM).grid(row=r, column=0, sticky="w", padx=(0, 12))
            v = ttk.Label(info, text="–", font=self.f_title)
            v.grid(row=r, column=1, sticky="w")
            self.veh_vals[key] = v

        ttk.Label(tab, text="Werte, die das Auto über OBD liefert (Mode 01):").pack(anchor="w")
        frame = ttk.Frame(tab)
        frame.pack(fill="both", expand=True)
        self.pid_tree = ttk.Treeview(frame, columns=("pid", "name"), show="headings")
        self.pid_tree.heading("pid", text="PID", anchor="w")
        self.pid_tree.heading("name", text="Bezeichnung", anchor="w")
        self.pid_tree.column("pid", width=70, stretch=False)
        self.pid_tree.column("name", width=600)
        sb = ttk.Scrollbar(frame, orient="vertical", command=self.pid_tree.yview)
        self.pid_tree.configure(yscrollcommand=sb.set)
        self.pid_tree.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")

    def _build_terminal(self, nb):
        tab = ttk.Frame(nb, padding=8)
        nb.add(tab, text="  Terminal  ")
        frame = ttk.Frame(tab)
        frame.pack(fill="both", expand=True)
        self.term = tk.Text(frame, font=self.f_mono, wrap="word", state="disabled",
                            bg="#15181d", fg="#d7dde5", insertbackground="#d7dde5")
        sb = ttk.Scrollbar(frame, orient="vertical", command=self.term.yview)
        self.term.configure(yscrollcommand=sb.set)
        self.term.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")
        self.term.tag_configure("cmd", foreground="#7fc4ff")

        bar = ttk.Frame(tab)
        bar.pack(fill="x", pady=(6, 0))
        ttk.Label(bar, text="Befehl:").pack(side="left")
        self.cmd_var = tk.StringVar()
        self.cmd_entry = ttk.Entry(bar, textvariable=self.cmd_var, width=30, font=self.f_mono)
        self.cmd_entry.pack(side="left", padx=4)
        self.cmd_entry.bind("<Return>", lambda e: self.send_raw())
        self.cmd_entry.bind("<Up>", lambda e: self._hist(-1))
        self.cmd_entry.bind("<Down>", lambda e: self._hist(1))
        self.send_btn = ttk.Button(bar, text="Senden", command=self.send_raw)
        self.send_btn.pack(side="left")
        ttk.Button(bar, text="Leeren", command=self._term_clear).pack(side="left", padx=(12, 0))
        ttk.Button(bar, text="Speichern …", command=self._term_save).pack(side="left", padx=4)
        self.logall_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(bar, text="auch Live-Abfragen anzeigen", variable=self.logall_var,
                        command=self._logall).pack(side="left", padx=12)
        ttk.Label(tab, foreground=C_DIM, wraplength=900, justify="left", text=(
            "Beispiele: ATRV (Spannung), 010C (Drehzahl), 0105 (Kühlwasser), 03 (Fehlercodes), "
            "ATSH7E0 und dann 2211BD (Öltemperatur vom Motorsteuergerät). Befehle wie ATZ oder "
            "ATH1 verstellen den Adapter, danach am besten neu verbinden.")
        ).pack(fill="x", pady=(4, 0))

    # ── Anschluss ────────────────────────────────────────────────────

    def refresh_ports(self):
        ports = list_serial_ports()
        values = ["%s – %s" % (dev, desc) if desc and desc != "n/a" else dev for dev, desc in ports]
        values.append(DEMO_PORT)
        self.port_cb["values"] = values
        cur = self.port_var.get() or self.settings.get("port", "")
        for v in values:
            if v.split(" – ")[0] == cur.split(" – ")[0]:
                self.port_var.set(v)
                break
        else:
            self.port_var.set(values[0])
        if not ports:
            self._status("Kein COM-Port gefunden. USB-Wandler eingesteckt? Treiber im "
                         "Geräte-Manager prüfen (siehe README). Der Demo-Modus geht trotzdem.")

    def toggle_connect(self):
        if self.connected:
            self.worker.submit("disconnect")
            return
        sel = self.port_var.get()
        port = DEMO_PORT if sel == DEMO_PORT else sel.split(" – ")[0].strip()
        if not port:
            return
        self.settings.update(port=port, baud=self.baud_var.get())
        self._save_settings()
        self.conn_btn.configure(state="disabled")
        self._status("Verbinde mit %s ..." % port)
        self.worker.submit("connect", port=port, baud=self.baud_var.get())

    def _set_connected(self, on, info=None):
        self.connected = on
        st = "normal" if on else "disabled"
        for b in (self.live_btn, self.max_btn, self.dtc_btn, self.clr_btn, self.veh_btn, self.send_btn):
            b.configure(state=st)
        self.port_cb.configure(state="disabled" if on else "readonly")
        self.conn_btn.configure(text="Trennen" if on else "Verbinden", state="normal")
        if on and info:
            parts = [info["version"], "%s Baud" % info["baud"] if info["baud"] != "Demo" else "Demo",
                     info["protocol_name"]]
            if info.get("voltage"):
                parts.append(fmt(info["voltage"], 1) + " V")
            self.conn_lbl.configure(text="  ·  ".join(parts), foreground=C_GREEN)
        elif not on:
            self.conn_lbl.configure(text="nicht verbunden", foreground=C_DIM)
            self._set_live(False)

    def _set_live(self, on):
        self.live_on = on
        self.live_btn.configure(text="■  Live stoppen" if on else "▶  Live starten")
        if not on:
            self.rate_lbl.configure(text="")

    # ── Aktionen ─────────────────────────────────────────────────────

    def toggle_live(self):
        self.worker.submit("live", on=not self.live_on)

    def read_dtcs(self):
        self.worker.submit("read_dtcs")

    def clear_dtcs(self):
        ok = messagebox.askyesno(
            "Fehlercodes löschen",
            "Fehlercodes wirklich löschen?\n\n"
            "Dabei werden auch die Freeze-Frame-Daten und die Bereitschaftstests "
            "(Readiness) zurückgesetzt. Das lässt sich nicht rückgängig machen. "
            "Ohne Bereitschaftstests fällt das Auto bei der Abgasuntersuchung durch, "
            "bis es wieder einige Zeit gefahren wurde.\n\n"
            "Am besten Motor aus, Zündung an.",
            icon="warning", default="no")
        if ok:
            self.worker.submit("clear_dtcs")

    def read_vehicle(self):
        self.worker.submit("vehicle")

    def send_raw(self):
        cmd = self.cmd_var.get().strip()
        if not cmd or not self.connected:
            return
        self.history.append(cmd)
        self.hist_pos = len(self.history)
        self.cmd_var.set("")
        self.worker.submit("raw", cmd=cmd)

    def _hist(self, d):
        if not self.history:
            return "break"
        self.hist_pos = max(0, min(len(self.history), self.hist_pos + d))
        self.cmd_var.set(self.history[self.hist_pos] if self.hist_pos < len(self.history) else "")
        self.cmd_entry.icursor("end")
        return "break"

    def _logall(self):
        self.worker.log_all = self.logall_var.get()

    def toggle_rec(self):
        if self.rec:
            self._stop_rec()
            return
        name = datetime.datetime.now().strftime("obd_log_%Y-%m-%d_%H-%M-%S.csv")
        path = filedialog.asksaveasfilename(
            title="Aufzeichnung speichern unter", initialfile=name, defaultextension=".csv",
            initialdir=self.settings.get("logdir") or os.path.expanduser("~"),
            filetypes=[("CSV-Datei", "*.csv")])
        if not path:
            return
        try:
            f = open(path, "w", newline="", encoding="utf-8-sig")
        except OSError as e:
            messagebox.showerror(APP_TITLE, "Datei lässt sich nicht anlegen:\n%s" % e)
            return
        self.settings["logdir"] = os.path.dirname(path)
        self._save_settings()
        w = csv.writer(f, delimiter=";")       # ; und Komma: so öffnet das deutsche Excel es direkt
        w.writerow(["Zeit", "t [s]"] + ["%s [%s]" % (name, unit) for _, name, unit, _ in FIELDS])
        self.rec = [f, w, time.monotonic(), 0, os.path.basename(path)]
        self.rec_btn.configure(text="■  Aufzeichnung stoppen")
        self.rec_lbl.configure(text="Aufzeichnung: %s" % self.rec[4], foreground=C_RED)
        if not self.live_on and self.connected:
            self.toggle_live()

    def _stop_rec(self):
        f, _, _, n, name = self.rec
        f.close()
        self.rec = None
        self.rec_btn.configure(text="●  CSV-Aufzeichnung")
        self.rec_lbl.configure(text="gespeichert: %s (%d Zeilen)" % (name, n), foreground=C_DIM)

    # ── Ereignisse vom Hintergrund-Thread ────────────────────────────

    def _poll(self):
        try:
            while True:
                kind, data = self.events.get_nowait()
                getattr(self, "on_" + kind)(data)
        except queue.Empty:
            pass
        self.root.after(50, self._poll)

    def on_busy(self, text):
        self._status(text)

    def on_idle(self, _):
        if self.status.cget("text").endswith("..."):
            self._status("Bereit.")

    def on_error(self, text):
        self._status(text)
        messagebox.showerror(APP_TITLE, text)

    def on_connected(self, info):
        self._set_connected(True, info)
        if info["ecu_ok"]:
            self._status("Verbunden. Das Auto antwortet.")
        else:
            self._status("Adapter verbunden, aber das Auto antwortet (noch) nicht. Zündung an? "
                         "Das Protokoll wird erkannt, sobald es antwortet.")
        for k in self.veh_vals:
            self.veh_vals[k].configure(text="–")
        self.veh_vals["version"].configure(text=info["version"])
        self.veh_vals["baud"].configure(text=str(info["baud"]))
        self.veh_vals["protocol_name"].configure(text=info["protocol_name"])

    def on_disconnected(self, text):
        self._set_connected(False)
        if self.rec:
            self._stop_rec()
        if text:
            self._status(text)

    def on_live_state(self, on):
        self._set_live(on)
        if not on and self.rec:
            self._stop_rec()

    def on_live(self, v):
        for key, (lbl, sub, nd) in self.tiles.items():
            x = v.get(key)
            lbl.configure(text=fmt(x, nd, sign=(key == "boost")), fg=value_color(key, x))
        bmax = v.get("_boost_max")
        sub = []
        if bmax is not None:
            sub.append("Max %s bar" % fmt(bmax, 2, True))
        if v.get("baro"):
            sub.append("Umgebung %s kPa" % fmt(v["baro"], 0))
        self.tiles["boost"][1].configure(text="\n".join(sub))
        self.tiles["oil"][1].configure(text="Quelle: %s" % v.get("_oil_source", ""))
        ecu = "" if v.get("_ecu_ok") else "  ·  Auto antwortet nicht (Zündung aus?)"
        self.rate_lbl.configure(text="Zyklus %s s%s" % (fmt(v["_cycle"], 2), ecu),
                                foreground=C_DIM if not ecu else C_ORANGE)
        if self.rec:
            f, w, t0, n, _ = self.rec
            now = datetime.datetime.now()
            row = [now.strftime("%d.%m.%Y %H:%M:%S,") + "%03d" % (now.microsecond // 1000),
                   fmt(time.monotonic() - t0, 2)]
            row += ["" if v.get(k) is None else fmt(v[k], nd) for k, _, _, nd in FIELDS]
            w.writerow(row)
            self.rec[3] = n + 1
            if self.rec[3] % 20 == 0:
                f.flush()
                self.rec_lbl.configure(text="Aufzeichnung: %s (%d Zeilen)" % (self.rec[4], self.rec[3]))

    def on_dtcs(self, r):
        self.dtc_tree.delete(*self.dtc_tree.get_children())
        n = 0
        for key, art in (("stored", "gespeichert"), ("pending", "anstehend"), ("permanent", "permanent")):
            for code in r.get(key) or []:
                self.dtc_tree.insert("", "end", values=(code, art, dec.dtc_description(code)))
                n += 1
        mil = "AN" if r["mil"] else "aus"
        self.mil_lbl.configure(text="Motorkontrollleuchte: %s  ·  gemeldete Codes: %d" % (mil, r["count"]),
                               foreground=C_RED if r["mil"] else C_GREEN)
        notes = []
        if n == 0:
            notes.append("Keine Fehlercodes gespeichert.")
        for key, art in (("pending", "Anstehende"), ("permanent", "Permanente")):
            if r.get(key) is None:
                notes.append("%s Codes: vom Auto nicht unterstützt." % art)
        notes.append("Gelesen um %s." % datetime.datetime.now().strftime("%H:%M:%S"))
        self.dtc_info.configure(text="  ".join(notes))
        self._status("Fehlercodes gelesen: %d." % n)

    def on_cleared(self, _):
        self._status("Fehlercodes gelöscht. Lese zur Kontrolle neu ...")

    def on_vehicle(self, info):
        for k, lbl in self.veh_vals.items():
            v = info.get(k)
            if k == "voltage":
                v = fmt(v, 1) + " V" if v else None
            lbl.configure(text=str(v) if v else "–")
        if not info.get("vin"):
            self.veh_vals["vin"].configure(text="– (liefert das Auto nicht)")
        self.pid_tree.delete(*self.pid_tree.get_children())
        for pid in info["pids"]:
            if pid % 0x20:      # die "unterstützte PIDs"-Einträge selbst weglassen
                self.pid_tree.insert("", "end", values=("%02X" % pid, dec.pid_name(pid)))
        self._status("Fahrzeugdaten gelesen.")

    def on_log(self, text):
        self.term.configure(state="normal")
        first, _, rest = text.partition("\n")
        if first.startswith("> "):
            self.term.insert("end", first + "\n", "cmd")
            self.term.insert("end", rest + "\n")
        else:
            self.term.insert("end", text + "\n")
        lines = int(self.term.index("end-1c").split(".")[0])
        if lines > 5000:
            self.term.delete("1.0", "%d.0" % (lines - 4000))
        self.term.see("end")
        self.term.configure(state="disabled")

    # ── Hilfsfunktionen ──────────────────────────────────────────────

    def _status(self, text):
        self.status.configure(text=text)

    def _term_clear(self):
        self.term.configure(state="normal")
        self.term.delete("1.0", "end")
        self.term.configure(state="disabled")

    def _term_save(self):
        path = filedialog.asksaveasfilename(
            title="Terminal-Log speichern", defaultextension=".txt",
            initialfile=datetime.datetime.now().strftime("obd_terminal_%Y-%m-%d_%H-%M.txt"),
            filetypes=[("Textdatei", "*.txt")])
        if path:
            with open(path, "w", encoding="utf-8") as f:
                f.write(self.term.get("1.0", "end"))

    def _load_settings(self):
        try:
            with open(SETTINGS, encoding="utf-8") as f:
                return json.load(f)
        except (OSError, ValueError):
            return {}

    def _save_settings(self):
        try:
            with open(SETTINGS, "w", encoding="utf-8") as f:
                json.dump(self.settings, f)
        except OSError:
            pass

    def on_close(self):
        if self.rec:
            self._stop_rec()
        self.worker.submit("quit")
        self.root.destroy()


def main():
    if sys.platform == "win32":
        try:                        # scharfe Schrift bei Windows-Skalierung (125 %, 150 % ...)
            import ctypes
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except Exception:
            pass
    root = tk.Tk()
    app = App(root, autostart_demo="--demo" in sys.argv)
    root.mainloop()
    app.worker.join(timeout=2)


if __name__ == "__main__":
    main()
