full updated code 
:::::
#!/usr/bin/env python3
# =============================================================
#  Dual Motor Controller — Raspberry Pi MASTER  [v4.0]
#  Changes from v3:
#    • Terminal log panel removed entirely
#    • Enable Driver defaults to ON for both motors
#    • Mode panel is context-aware:
#        – Random  → shows only random config (runs INFINITELY until STOP)
#        – Sweep   → shows only sweep config
#        – Manual  → shows shared RPM slider + entry + per-motor direction
#    • Random mode: infinite steps until user hits STOP
#    • Manual mode: one shared slider + entry box moves both motors together;
#                   each motor still has its own direction radio
#
#  Serial protocol (Pi → Teensy):
#    C,<rpm1>,<dir1>,<en1>,<rpm2>,<dir2>,<en2>\n
#    STOP\n  |  RESET\n  |  STATUS\n
#  Serial protocol (Teensy → Pi):
#    E,<count1>,<count2>,<rpm1_actual>,<rpm2_actual>\n
#    READY\n  |  WATCHDOG\n  |  RESET_OK\n
# =============================================================

import tkinter as tk
from tkinter import ttk
import threading
import serial
import serial.tools.list_ports
import time
import queue
import random
import io
import sys
import os

try:
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    MATPLOTLIB_OK = True
except ImportError:
    MATPLOTLIB_OK = False

# ─── SERIAL CONFIG ───────────────────────────────────────────
TEENSY_BAUD = 115200
TEENSY_PORT = "/dev/ttyACM0"

# ─── MOTOR LIMITS ────────────────────────────────────────────
MIN_RPM = 0
MAX_RPM = 100.0

# ─── PID / CONTROL CONSTANTS ─────────────────────────────────
PID_INTERVAL  = 0.1
FILTER_ALPHA  = 0.3
RAMP_UP       = 1.0
RAMP_DOWN     = 1.5
DEADBAND_RPM  = 0.8
DEADBAND_HOLD = 3
I_LIMIT       = 30.0

# ─── ZN GAIN PRESETS ─────────────────────────────────────────
ZN_GAINS = {
    "classic":      {"Kp": 0.240, "Ki": 2.400, "Kd": 0.006},
    "moderate":     {"Kp": 0.132, "Ki": 1.320, "Kd": 0.009},
    "no_overshoot": {"Kp": 0.080, "Ki": 0.800, "Kd": 0.005},
}

Kp = Ki = Kd = 0.0
MODE = "classic"
_run_mode_for_graph = "manual"

running = False

# ─── LOG QUEUE (kept for internal use, not shown in GUI) ─────
log_queue = queue.Queue()

def qprint(*args, **kwargs):
    buf = io.StringIO()
    print(*args, file=buf, **kwargs)
    text = buf.getvalue()
    sys.stdout.write(text)
    sys.stdout.flush()
    # still put in queue in case anything reads it, but GUI won't display

# ─── SERIAL STATE ────────────────────────────────────────────
ser = None
ser_lock = threading.Lock()

def find_teensy_port():
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        if "teensy" in desc or "usbmodem" in desc or "acm" in p.device:
            return p.device
    return TEENSY_PORT

def serial_connect():
    global ser
    port = find_teensy_port()
    try:
        with ser_lock:
            if ser and ser.is_open:
                ser.close()
            ser = serial.Serial(port, TEENSY_BAUD, timeout=0.1)
        time.sleep(0.3)
        qprint(f"[SERIAL] Connected → {port}")
        return True
    except Exception as e:
        qprint(f"[SERIAL] Connect failed: {e}")
        return False

def serial_write(cmd: str):
    global ser
    if ser and ser.is_open:
        try:
            with ser_lock:
                ser.write((cmd + "\n").encode())
        except Exception as e:
            qprint(f"[SERIAL] Write error: {e}")

# ─── PER-MOTOR STATE ─────────────────────────────────────────
class MotorState:
    def __init__(self, label):
        self.label = label
        self.enc_count    = 0
        self.actual_rpm   = 0.0
        self.filtered_rpm = 0.0
        self.target_rpm   = 0.0
        self.command_rpm  = 0.0
        self.integral     = 0.0
        self.last_error   = 0.0
        self.freq_locked      = False
        self.locked_command   = 0.0
        self.deadband_counter = 0
        self.direction    = 1      # 1=CW, 0=CCW
        self.pending_dir  = None
        self.pending_rpm  = None
        self.enabled      = True   # DEFAULT ON
        self.current_rpm  = 0.0
        self.rpm_log      = []
        self.filtered_log = []
        self.target_log   = []
        self.time_log     = []
        self.start_time   = 0.0
        self.log_file     = None
        self.log_filename = ""

    def reset(self):
        self.enc_count = 0
        self.actual_rpm = 0.0
        self.filtered_rpm = 0.0
        self.target_rpm = 0.0
        self.command_rpm = 0.0
        self.integral = 0.0
        self.last_error = 0.0
        self.freq_locked = False
        self.locked_command = 0.0
        self.deadband_counter = 0
        self.pending_dir = None
        self.pending_rpm = None
        self.current_rpm = 0.0
        self.rpm_log.clear()
        self.filtered_log.clear()
        self.target_log.clear()
        self.time_log.clear()
        self.start_time = time.time()

M1 = MotorState("M1")
M2 = MotorState("M2")

# ─── DIRECTION-SAFE TARGET REQUEST ───────────────────────────
def request_target(ms: MotorState, rpm: float, direction: int):
    rpm = max(0.0, min(float(MAX_RPM), float(rpm)))
    if direction != ms.direction and ms.current_rpm > 0.5:
        ms.pending_dir      = direction
        ms.pending_rpm      = rpm
        ms.target_rpm       = 0.0
        ms.freq_locked      = False
        ms.deadband_counter = 0
        return True
    else:
        ms.direction        = direction
        ms.target_rpm       = rpm
        ms.freq_locked      = False
        ms.deadband_counter = 0
        return False

# ─── TELEMETRY READER THREAD ─────────────────────────────────
def telemetry_thread():
    global ser
    while True:
        try:
            if not (ser and ser.is_open):
                time.sleep(1.0)
                serial_connect()
                continue
            line = ser.readline().decode(errors="ignore").strip()
            if line.startswith("E,"):
                parts = line.split(",")
                if len(parts) >= 5:
                    M1.enc_count = int(parts[1])
                    M2.enc_count = int(parts[2])
                    rpm1 = abs(float(parts[3]))
                    rpm2 = abs(float(parts[4]))
                    M1.filtered_rpm = (1.0 - FILTER_ALPHA) * M1.filtered_rpm + FILTER_ALPHA * rpm1
                    M2.filtered_rpm = (1.0 - FILTER_ALPHA) * M2.filtered_rpm + FILTER_ALPHA * rpm2
                    M1.actual_rpm = rpm1
                    M2.actual_rpm = rpm2
            elif line == "WATCHDOG":
                qprint("[TEENSY] Watchdog triggered — motors stopped")
            elif line == "READY":
                qprint("[TEENSY] READY")
            elif line == "RESET_OK":
                qprint("[TEENSY] RESET_OK")
        except serial.SerialException:
            qprint("[SERIAL] Connection lost — retrying...")
            time.sleep(2.0)
            serial_connect()
        except Exception:
            pass
        time.sleep(0.005)

# ─── PID THREAD ──────────────────────────────────────────────
def pid_thread():
    global running
    last_flush  = time.time()
    last_print  = [time.time(), time.time()]

    while True:
        if not running:
            time.sleep(0.05)
            continue

        t0 = time.time()

        for idx, ms in enumerate((M1, M2)):
            # Release queued direction flip once stopped
            if (ms.pending_dir is not None
                    and ms.current_rpm <= 0.05
                    and ms.target_rpm == 0.0):
                ms.direction  = ms.pending_dir
                ms.target_rpm = ms.pending_rpm
                ms.pending_dir = None
                ms.pending_rpm = None

            actual_rpm = ms.actual_rpm
            error      = ms.target_rpm - ms.filtered_rpm
            deadband   = (max(0.3, 0.12 * ms.target_rpm)
                          if ms.target_rpm > 0 else DEADBAND_RPM)

            if ms.target_rpm == 0:
                ms.command_rpm      = 0.0
                ms.integral         = 0.0
                ms.filtered_rpm     = 0.0
                ms.freq_locked      = False
                ms.locked_command   = 0.0
                ms.deadband_counter = 0
                ms.last_error       = 0.0
                ms.current_rpm      = 0.0

            elif abs(error) <= deadband:
                ms.deadband_counter += 1
                if ms.deadband_counter >= DEADBAND_HOLD:
                    if not ms.freq_locked:
                        ms.locked_command = ms.command_rpm
                        ms.freq_locked    = True
                    ms.command_rpm = ms.locked_command
            else:
                if ms.freq_locked:
                    ms.freq_locked      = False
                    ms.deadband_counter = 0
                else:
                    ms.deadband_counter = max(0, ms.deadband_counter - 1)

                derivative    = (error - ms.last_error) / PID_INTERVAL
                ms.last_error = error
                i_term        = max(-I_LIMIT, min(I_LIMIT, Ki * ms.integral))
                pid_out       = (Kp * error) + i_term + (Kd * derivative)
                new_cmd       = max(0.0, min(float(MAX_RPM), ms.target_rpm + pid_out))

                saturated = ((new_cmd <= 0.0 and error < 0) or
                             (new_cmd >= MAX_RPM and error > 0))
                if not saturated:
                    ms.integral = max(-80.0, min(80.0,
                                      ms.integral + error * PID_INTERVAL))
                ms.command_rpm = new_cmd

            ms.current_rpm = ms.filtered_rpm

            t = time.time() - ms.start_time
            ms.time_log.append(t)
            ms.rpm_log.append(actual_rpm)
            ms.filtered_log.append(ms.filtered_rpm)
            ms.target_log.append(ms.target_rpm)

            if ms.log_file:
                ms.log_file.write(
                    f"{t:.2f},{ms.target_rpm:.2f},{actual_rpm:.2f},"
                    f"{ms.filtered_rpm:.2f},{ms.command_rpm:.2f},"
                    f"{ms.enc_count},{int(ms.freq_locked)}\n"
                )

        # Periodic CSV flush
        if time.time() - last_flush >= 5.0:
            for ms in (M1, M2):
                if ms.log_file:
                    ms.log_file.flush()
            last_flush = time.time()

        # Send command to Teensy
        cmd = (
            f"C,"
            f"{M1.command_rpm:.2f},{M1.direction},{int(M1.enabled)},"
            f"{M2.command_rpm:.2f},{M2.direction},{int(M2.enabled)}"
        )
        serial_write(cmd)

        elapsed = time.time() - t0
        time.sleep(max(0.0, PID_INTERVAL - elapsed))

# ─── SWEEP THREAD ────────────────────────────────────────────
def sweep_thread(sweep_rpms, sweep_dwell):
    global running
    qprint(f"[SWEEP] Starting — {len(sweep_rpms)} steps, {sweep_dwell}s each")
    time.sleep(3.0)

    for idx, rpm in enumerate(sweep_rpms):
        if not running:
            break
        for ms in (M1, M2):
            ms.target_rpm       = float(rpm)
            ms.enabled          = True
            ms.freq_locked      = False
            ms.deadband_counter = 0
        elapsed = 0.0
        while elapsed < sweep_dwell and running:
            time.sleep(0.5)
            elapsed += 0.5

    for ms in (M1, M2):
        ms.target_rpm  = 0.0
        ms.command_rpm = 0.0
        ms.integral    = 0.0
        ms.pending_dir = None
        ms.pending_rpm = None
    time.sleep(2.0)
    stop_run()

# ─── RANDOM THREAD — INFINITE until stop_run() called ────────
def random_thread(rmin, rmax, tmin, tmax):
    global running
    qprint(f"[RANDOM] Starting infinite random — RPM {rmin}-{rmax}, time {tmin}-{tmax}s")
    time.sleep(3.0)

    step = 0
    while running:
        step += 1
        for ms in (M1, M2):
            rpm       = random.randint(rmin, rmax)
            direction = random.choice([0, 1])
            ms.enabled = True
            request_target(ms, rpm, direction)

        dwell = random.randint(tmin, tmax)
        elapsed = 0.0
        while elapsed < dwell and running:
            time.sleep(0.5)
            elapsed += 0.5

    qprint("[RANDOM] Stopped.")
    for ms in (M1, M2):
        ms.target_rpm  = 0.0
        ms.command_rpm = 0.0
        ms.integral    = 0.0
        ms.pending_dir = None
        ms.pending_rpm = None

# ─── MANUAL THREAD ───────────────────────────────────────────
def manual_thread():
    global running
    while running:
        time.sleep(0.2)

# ─── STOP / CLEANUP ──────────────────────────────────────────
def stop_run():
    global running
    running = False
    time.sleep(0.3)
    serial_write("STOP")
    for ms in (M1, M2):
        ms.target_rpm  = 0.0
        ms.command_rpm = 0.0
        ms.enabled     = True   # keep enabled=True so driver stays armed
        if ms.log_file:
            try:
                ms.log_file.flush()
                ms.log_file.close()
            except Exception:
                pass
            ms.log_file = None

# ─── GRAPH ───────────────────────────────────────────────────
def plot_graphs():
    if not MATPLOTLIB_OK:
        return
    if not M1.time_log and not M2.time_log:
        return

    fig, axes = plt.subplots(2, 2, figsize=(16, 8))
    fig.suptitle(
        f"Dual Motor PID  [{MODE}]  Kp={Kp} Ki={Ki} Kd={Kd}  "
        f"[{_run_mode_for_graph.upper()} MODE]",
        fontsize=12
    )
    colors = {"M1": ("#1565C0", "#0D9DDB"), "M2": ("#0D9DDB", "#1565C0")}
    for col, ms in enumerate((M1, M2)):
        c_act, c_tgt = colors[ms.label]
        ax1 = axes[0][col]
        ax2 = axes[1][col]
        ax1.plot(ms.time_log, ms.target_log, "--", label="Target RPM",
                 linewidth=1.5, color=c_tgt)
        ax1.plot(ms.time_log, ms.rpm_log, drawstyle="steps-mid",
                 label="Actual RPM", linewidth=1.5, color=c_act)
        ax1.set_ylabel("RPM"); ax1.set_title(f"{ms.label} — Speed Tracking")
        ax1.legend(); ax1.grid(True, alpha=0.3)
        errors = [t - f for t, f in zip(ms.target_log, ms.filtered_log)]
        ax2.plot(ms.time_log, errors, drawstyle="steps-mid",
                 color="#1565C0", linewidth=1.0, label="Error")
        ax2.axhline(0, color="gray", linewidth=0.8, linestyle="--")
        ax2.set_xlabel("Time (s)"); ax2.set_ylabel("Error (RPM)")
        ax2.legend(); ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    ts = time.strftime("%Y%m%d_%H%M%S")
    fname = f"dual_graph_{ts}.png"
    plt.savefig(fname, dpi=150)
    plt.show()

# ─── COLOUR PALETTE ──────────────────────────────────────────
C = {
    "BG":   "#F2F7FD",
    "PNL":  "#FFFFFF",
    "PNL2": "#EAF3FC",
    "ACC":  "#1565C0",
    "ACC2": "#0D47A1",
    "GRN":  "#2E8B57",
    "RED":  "#D32F2F",
    "ORG":  "#EF6C00",
    "FG":   "#11263F",
    "SUB":  "#5C7A99",
    "SEP":  "#BBD6EF",
    "BTN":  "#DCEAFB",
    "BTNF": "#C3DCF5",
    "M1":   "#1565C0",
    "M2":   "#0D9DDB",
}

# ═══════════════════════════════════════════════════════════════
# ── GUI
# ═══════════════════════════════════════════════════════════════
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Dual Motor PID Controller — Pi Master / Teensy Slave")
        self.geometry("820x700")
        self.resizable(True, True)
        self.configure(bg=C["BG"])
        self.minsize(700, 580)
        self._build_styles()
        self._build_ui()
        self._refresh_telemetry()
        serial_connect()
        threading.Thread(target=telemetry_thread, daemon=True).start()
        threading.Thread(target=pid_thread,       daemon=True).start()

    # ── ttk styles ──────────────────────────────────────────
    def _build_styles(self):
        s = ttk.Style(self)
        s.theme_use("clam")
        s.configure(".", background=C["BG"], foreground=C["FG"],
                    font=("Courier New", 10))
        s.configure("TFrame",     background=C["BG"])
        s.configure("TLabel",     background=C["BG"], foreground=C["FG"])
        s.configure("TEntry",
                    fieldbackground=C["BTN"], foreground=C["FG"],
                    insertcolor=C["ACC"], relief="flat", borderwidth=0)
        s.configure("TCombobox",
                    fieldbackground=C["BTN"], foreground=C["FG"],
                    selectbackground=C["BTN"], selectforeground=C["ACC"])
        s.map("TCombobox", fieldbackground=[("readonly", C["BTN"])])

        for name, bg, abg in [
            ("Run",   "#2E8B57", "#256E45"),
            ("Stop",  "#D32F2F", "#B71C1C"),
            ("Graph", C["BTN"],  C["BTNF"]),
            ("Sec",   C["BTN"],  C["BTNF"]),
            ("M1",    C["M1"],   C["ACC2"]),
            ("M2",    C["M2"],   "#0B86BC"),
        ]:
            fg   = "#FFFFFF" if name in ("Run", "Stop", "M1", "M2") else C["ACC"]
            bold = "bold"    if name in ("Run", "Stop") else "normal"
            sz   = 12        if name in ("Run", "Stop") else 10
            s.configure(f"{name}.TButton",
                        background=bg, foreground=fg,
                        font=("Courier New", sz, bold),
                        relief="flat", padding=8 if name in ("Run","Stop") else 5)
            s.map(f"{name}.TButton", background=[("active", abg)])

    # ── Main layout ─────────────────────────────────────────
    def _build_ui(self):
        # Header
        hdr = tk.Frame(self, bg=C["PNL"], height=60)
        hdr.pack(fill="x")
        hdr.pack_propagate(False)
        tk.Label(hdr, text="⚙  DUAL MOTOR PID CONTROLLER",
                 bg=C["PNL"], fg=C["ACC"],
                 font=("Courier New", 14, "bold")).pack(side="left", padx=18, pady=14)

        badge = tk.Frame(hdr, bg=C["PNL"])
        badge.pack(side="right", padx=18)
        self._serial_var = tk.StringVar(value="✗ Not connected")
        tk.Label(badge, textvariable=self._serial_var,
                 bg=C["PNL"], fg=C["SUB"],
                 font=("Courier New", 9, "bold")).pack(anchor="e")
        self._status_lbl = tk.Label(badge, text="● IDLE",
                                     bg=C["PNL"], fg=C["SUB"],
                                     font=("Courier New", 11, "bold"))
        self._status_lbl.pack(anchor="e")

        tk.Frame(self, bg=C["ACC"], height=2).pack(fill="x")

        body = tk.Frame(self, bg=C["BG"])
        body.pack(fill="both", expand=True, padx=14, pady=10)
        body.columnconfigure(0, weight=1)
        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=1)

        left = tk.Frame(body, bg=C["BG"])
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 8))

        right = tk.Frame(body, bg=C["BG"])
        right.grid(row=0, column=1, sticky="nsew")

        # Left column: ZN tuning + Mode + mode-specific content + actions
        self._card_zn(left)
        self._card_mode(left)
        self._card_actions(left)
        self._card_serial(left)

        # Right column: live telemetry for both motors
        self._card_telemetry(right)

    # ── CARD: ZN presets ────────────────────────────────────
    def _card_zn(self, parent):
        f = self._card(parent, "ZN TUNING MODE")
        self._zn_var = tk.StringVar(value="classic")
        for val, lbl in [
            ("classic",      "Classic       Kp=0.240  Ki=2.400  Kd=0.006"),
            ("moderate",     "Moderate      Kp=0.132  Ki=1.320  Kd=0.009"),
            ("no_overshoot", "No-Overshoot  Kp=0.080  Ki=0.800  Kd=0.005"),
        ]:
            tk.Radiobutton(
                f, text=lbl, variable=self._zn_var, value=val,
                bg=C["PNL2"], fg=C["FG"], selectcolor=C["PNL2"],
                activebackground=C["PNL2"], activeforeground=C["ACC"],
                font=("Courier New", 9), indicatoron=True
            ).pack(anchor="w", padx=10, pady=2)

    # ── CARD: Run mode ───────────────────────────────────────
    def _card_mode(self, parent):
        f = self._card(parent, "RUN MODE")

        self._run_mode_var = tk.StringVar(value="manual")
        rf = tk.Frame(f, bg=C["PNL2"])
        rf.pack(fill="x", padx=8, pady=6)
        for val, lbl in [("sweep","Sweep"), ("random","Random"), ("manual","Manual")]:
            tk.Radiobutton(
                rf, text=lbl, variable=self._run_mode_var, value=val,
                command=self._on_mode_change,
                bg=C["PNL2"], fg=C["FG"], selectcolor=C["PNL2"],
                activebackground=C["PNL2"], activeforeground=C["ACC"],
                font=("Courier New", 10)
            ).pack(side="left", padx=8)

        # ── Sweep config ──────────────────────────────────────
        self._sweep_frame = tk.LabelFrame(
            f, text=" Sweep Config ",
            bg=C["PNL2"], fg=C["ACC"], font=("Courier New", 9),
            bd=1, relief="groove"
        )
        g = self._sweep_frame
        tk.Label(g, text="RPM list (comma-sep):",
                 bg=C["PNL2"], fg=C["FG"], font=("Courier New", 9)
                 ).grid(row=0, column=0, sticky="w", padx=6, pady=2)
        self._sweep_rpms_var = tk.StringVar(value="5,10,20,30,40,50,40,30,20,10,5")
        ttk.Entry(g, textvariable=self._sweep_rpms_var, width=26
                  ).grid(row=0, column=1, padx=6, pady=2)
        tk.Label(g, text="Dwell (sec):",
                 bg=C["PNL2"], fg=C["FG"], font=("Courier New", 9)
                 ).grid(row=1, column=0, sticky="w", padx=6)
        self._sweep_dwell_var = tk.StringVar(value="10")
        ttk.Entry(g, textvariable=self._sweep_dwell_var, width=8
                  ).grid(row=1, column=1, sticky="w", padx=6)

        # ── Random config ─────────────────────────────────────
        self._random_frame = tk.LabelFrame(
            f, text=" Random Config  [runs until STOP] ",
            bg=C["PNL2"], fg=C["ACC"], font=("Courier New", 9),
            bd=1, relief="groove"
        )
        g = self._random_frame
        tk.Label(g, text="⚠ Each motor: independent RPM + direction",
                 bg=C["PNL2"], fg=C["ORG"],
                 font=("Courier New", 8, "italic")
                 ).grid(row=0, column=0, columnspan=2, pady=(4, 2), padx=6)
        tk.Label(g, text="⟳ Runs INFINITELY until you press STOP",
                 bg=C["PNL2"], fg=C["GRN"],
                 font=("Courier New", 8, "italic")
                 ).grid(row=1, column=0, columnspan=2, pady=(0, 4), padx=6)
        rnd_fields = [
            ("Min RPM",      "3",  "_rnd_rmin"),
            ("Max RPM",     "50",  "_rnd_rmax"),
            ("Min Time (s)", "5",  "_rnd_tmin"),
            ("Max Time (s)","15",  "_rnd_tmax"),
        ]
        for row, (lbl, dflt, attr) in enumerate(rnd_fields, start=2):
            tk.Label(g, text=lbl + ":", bg=C["PNL2"], fg=C["FG"],
                     font=("Courier New", 9)
                     ).grid(row=row, column=0, sticky="w", padx=6, pady=2)
            var = tk.StringVar(value=dflt)
            setattr(self, attr, var)
            ttk.Entry(g, textvariable=var, width=8
                      ).grid(row=row, column=1, sticky="w", padx=6)

        # ── Manual config: shared slider + entry ──────────────
        self._manual_frame = tk.LabelFrame(
            f, text=" Manual Config ",
            bg=C["PNL2"], fg=C["ACC"], font=("Courier New", 9),
            bd=1, relief="groove"
        )
        g = self._manual_frame

        tk.Label(g, text="Both motors — shared RPM:",
                 bg=C["PNL2"], fg=C["FG"],
                 font=("Courier New", 9, "bold")
                 ).grid(row=0, column=0, columnspan=3, sticky="w", padx=6, pady=(6, 2))

        self._manual_rpm_var = tk.DoubleVar(value=20.0)

        # Slider
        self._manual_slider = tk.Scale(
            g, from_=MIN_RPM, to=MAX_RPM,
            orient="horizontal", variable=self._manual_rpm_var,
            resolution=0.5, length=220,
            bg=C["PNL2"], fg=C["FG"],
            highlightthickness=0, sliderrelief="flat",
            troughcolor=C["BTN"], activebackground=C["ACC"],
            command=self._on_slider_change
        )
        self._manual_slider.grid(row=1, column=0, columnspan=3,
                                  padx=6, pady=2, sticky="ew")

        # Entry box
        entry_row = tk.Frame(g, bg=C["PNL2"])
        entry_row.grid(row=2, column=0, columnspan=3, padx=6, pady=4, sticky="w")
        tk.Label(entry_row, text="RPM:", bg=C["PNL2"], fg=C["SUB"],
                 font=("Courier New", 9)).pack(side="left")
        self._manual_entry_var = tk.StringVar(value="20.0")
        self._manual_entry = ttk.Entry(entry_row,
                                        textvariable=self._manual_entry_var, width=7)
        self._manual_entry.pack(side="left", padx=4)
        self._manual_entry.bind("<Return>", self._on_entry_commit)
        self._manual_entry.bind("<FocusOut>", self._on_entry_commit)
        ttk.Button(entry_row, text="SET", style="Sec.TButton",
                   command=self._manual_set_both).pack(side="left", padx=3)

        # Per-motor direction (M1)
        tk.Frame(g, bg=C["SEP"], height=1).grid(
            row=3, column=0, columnspan=3, sticky="ew", padx=6, pady=4)
        tk.Label(g, text="M1 Direction:", bg=C["PNL2"], fg=C["M1"],
                 font=("Courier New", 9, "bold")
                 ).grid(row=4, column=0, sticky="w", padx=6)
        self._m1_dir_var = tk.StringVar(value="CW")
        dir_f1 = tk.Frame(g, bg=C["PNL2"])
        dir_f1.grid(row=4, column=1, columnspan=2, sticky="w", padx=4)
        for d in ("CW", "CCW"):
            tk.Radiobutton(dir_f1, text=d, variable=self._m1_dir_var, value=d,
                           bg=C["PNL2"], fg=C["FG"], selectcolor=C["PNL2"],
                           activebackground=C["PNL2"], activeforeground=C["M1"],
                           font=("Courier New", 9),
                           command=self._manual_set_both
                           ).pack(side="left", padx=4)

        # Per-motor direction (M2)
        tk.Label(g, text="M2 Direction:", bg=C["PNL2"], fg=C["M2"],
                 font=("Courier New", 9, "bold")
                 ).grid(row=5, column=0, sticky="w", padx=6, pady=(2, 6))
        self._m2_dir_var = tk.StringVar(value="CW")
        dir_f2 = tk.Frame(g, bg=C["PNL2"])
        dir_f2.grid(row=5, column=1, columnspan=2, sticky="w", padx=4)
        for d in ("CW", "CCW"):
            tk.Radiobutton(dir_f2, text=d, variable=self._m2_dir_var, value=d,
                           bg=C["PNL2"], fg=C["FG"], selectcolor=C["PNL2"],
                           activebackground=C["PNL2"], activeforeground=C["M2"],
                           font=("Courier New", 9),
                           command=self._manual_set_both
                           ).pack(side="left", padx=4)

        self._on_mode_change()

    def _on_mode_change(self):
        self._sweep_frame.pack_forget()
        self._random_frame.pack_forget()
        self._manual_frame.pack_forget()
        mode = self._run_mode_var.get()
        if mode == "sweep":
            self._sweep_frame.pack(fill="x", padx=8, pady=(4, 6))
        elif mode == "random":
            self._random_frame.pack(fill="x", padx=8, pady=(4, 6))
        else:  # manual
            self._manual_frame.pack(fill="x", padx=8, pady=(4, 6))

    # ── Slider / entry sync ─────────────────────────────────
    def _on_slider_change(self, val):
        self._manual_entry_var.set(f"{float(val):.1f}")
        if running:
            self._manual_set_both()

    def _on_entry_commit(self, event=None):
        try:
            val = float(self._manual_entry_var.get())
            val = max(float(MIN_RPM), min(float(MAX_RPM), val))
            self._manual_rpm_var.set(val)
            self._manual_entry_var.set(f"{val:.1f}")
            if running:
                self._manual_set_both()
        except ValueError:
            pass

    def _manual_set_both(self):
        """Apply shared RPM + per-motor direction to both motors."""
        if not running:
            return
        try:
            val = float(self._manual_entry_var.get())
        except ValueError:
            return
        val = max(0.0, min(float(MAX_RPM), val))
        d1 = 1 if self._m1_dir_var.get() == "CW" else 0
        d2 = 1 if self._m2_dir_var.get() == "CW" else 0
        M1.enabled = True
        M2.enabled = True
        request_target(M1, val, d1)
        request_target(M2, val, d2)

    # ── CARD: Live telemetry ─────────────────────────────────
    def _card_telemetry(self, parent):
        for ms, accent, btn_s in [
            (M1, C["M1"], "M1.TButton"),
            (M2, C["M2"], "M2.TButton"),
        ]:
            f = self._card(parent, f"{ms.label} — MOTOR {'1' if ms.label=='M1' else '2'}",
                           accent=accent)

            # Big live RPM readout
            disp = tk.Label(f, text="0.00 RPM", bg=C["PNL2"], fg=accent,
                            font=("Courier New", 20, "bold"), width=12, relief="flat")
            disp.pack(pady=(8, 4), padx=10)
            setattr(self, f"_{ms.label.lower()}_rpm_disp", disp)

            # Encoder count
            enc_lbl = tk.Label(f, text="Enc: 0", bg=C["PNL2"], fg=C["SUB"],
                               font=("Courier New", 9))
            enc_lbl.pack()
            setattr(self, f"_{ms.label.lower()}_enc_lbl", enc_lbl)

            # Target / Command live readout
            info_row = tk.Frame(f, bg=C["PNL2"])
            info_row.pack(fill="x", padx=10, pady=4)
            tgt_lbl = tk.Label(info_row, text="Tgt: 0.0", bg=C["PNL2"],
                               fg=C["FG"], font=("Courier New", 9))
            tgt_lbl.pack(side="left", expand=True)
            cmd_lbl = tk.Label(info_row, text="Cmd: 0.0", bg=C["PNL2"],
                               fg=C["FG"], font=("Courier New", 9))
            cmd_lbl.pack(side="left", expand=True)
            setattr(self, f"_{ms.label.lower()}_tgt_lbl", tgt_lbl)
            setattr(self, f"_{ms.label.lower()}_cmd_lbl", cmd_lbl)

            # Lock status
            lock_lbl = tk.Label(f, text="──", bg=C["PNL2"],
                                fg=C["SUB"], font=("Courier New", 9, "bold"))
            lock_lbl.pack(pady=(0, 4))
            setattr(self, f"_{ms.label.lower()}_lock_lbl", lock_lbl)

            # Quick stop button
            stop_row = tk.Frame(f, bg=C["PNL2"])
            stop_row.pack(fill="x", padx=10, pady=(0, 8))
            ttk.Button(stop_row, text=f"STOP {ms.label}", style="Sec.TButton",
                       command=lambda m=ms: self._motor_stop(m)
                       ).pack(fill="x")

    def _motor_stop(self, ms: MotorState):
        ms.target_rpm  = 0.0
        ms.command_rpm = 0.0
        ms.integral    = 0.0
        ms.pending_dir = None
        ms.pending_rpm = None

    # ── CARD: Actions ───────────────────────────────────────
    def _card_actions(self, parent):
        f = self._card(parent, "CONTROL")
        row1 = tk.Frame(f, bg=C["PNL2"])
        row1.pack(pady=(6, 4), padx=10, fill="x")
        ttk.Button(row1, text="▶  START", style="Run.TButton",
                   command=self._start
                   ).pack(side="left", expand=True, fill="x", padx=(0, 5))
        ttk.Button(row1, text="■  STOP", style="Stop.TButton",
                   command=self._stop
                   ).pack(side="left", expand=True, fill="x")

        row2 = tk.Frame(f, bg=C["PNL2"])
        row2.pack(pady=(0, 8), padx=10, fill="x")
        ttk.Button(row2, text="📊 SHOW GRAPH", style="Graph.TButton",
                   command=lambda: threading.Thread(
                       target=plot_graphs, daemon=True).start()
                   ).pack(fill="x")

    # ── CARD: Serial status ──────────────────────────────────
    def _card_serial(self, parent):
        f = self._card(parent, "SERIAL")
        row = tk.Frame(f, bg=C["PNL2"])
        row.pack(fill="x", padx=10, pady=6)
        ttk.Button(row, text="Re-connect", style="Sec.TButton",
                   command=self._reconnect
                   ).pack(side="left", padx=(0, 6))
        ttk.Button(row, text="RESET Teensy", style="Sec.TButton",
                   command=lambda: serial_write("RESET")
                   ).pack(side="left")

    def _reconnect(self):
        if serial_connect():
            self._serial_var.set(f"● Connected → {ser.port if ser else '?'}")
        else:
            self._serial_var.set("✗ Connection failed")

    # ── Card helper ─────────────────────────────────────────
    def _card(self, parent, title, accent=None):
        acc   = accent or C["ACC"]
        outer = tk.Frame(parent, bg=C["PNL2"], bd=0,
                          highlightthickness=1,
                          highlightbackground=C["SEP"])
        outer.pack(fill="x", pady=5)
        hdr = tk.Frame(outer, bg=C["PNL2"])
        hdr.pack(fill="x")
        tk.Label(hdr, text=title, bg=C["PNL2"], fg=acc,
                 font=("Courier New", 9, "bold")).pack(anchor="w", padx=10, pady=(6, 2))
        tk.Frame(outer, bg=acc, height=1).pack(fill="x", padx=10, pady=(0, 4))
        return outer

    def _set_status(self, text, color=None):
        self._status_lbl.config(text=text, fg=color or C["SUB"])

    # ── Start ────────────────────────────────────────────────
    def _start(self):
        global running, Kp, Ki, Kd, MODE, _run_mode_for_graph
        if running:
            return

        MODE = self._zn_var.get()
        Kp   = ZN_GAINS[MODE]["Kp"]
        Ki   = ZN_GAINS[MODE]["Ki"]
        Kd   = ZN_GAINS[MODE]["Kd"]

        M1.reset(); M2.reset()
        M1.enabled = True
        M2.enabled = True

        ts = time.strftime("%Y%m%d_%H%M%S")
        for ms, tag in ((M1, "m1"), (M2, "m2")):
            ms.log_filename = f"motor_{tag}_{ts}.csv"
            ms.log_file     = open(ms.log_filename, "w")
            ms.log_file.write(
                "Time,TargetRPM,ActualRPM,FilteredRPM,"
                "CommandRPM,EncoderCount,Locked\n"
            )

        running = True
        self._set_status("● RUNNING", C["GRN"])
        run_mode = self._run_mode_var.get()
        _run_mode_for_graph = run_mode

        if run_mode == "sweep":
            try:
                rpms  = [int(x.strip()) for x in
                         self._sweep_rpms_var.get().split(",")]
                dwell = float(self._sweep_dwell_var.get())
            except ValueError:
                running = False; return
            t4 = threading.Thread(target=sweep_thread,
                                  args=(rpms, dwell), daemon=False)

        elif run_mode == "random":
            try:
                rmin = int(self._rnd_rmin.get())
                rmax = int(self._rnd_rmax.get())
                tmin = int(self._rnd_tmin.get())
                tmax = int(self._rnd_tmax.get())
            except ValueError:
                running = False; return
            # daemon=True so it dies when user hits stop_run()
            t4 = threading.Thread(target=random_thread,
                                  args=(rmin, rmax, tmin, tmax), daemon=True)

        else:  # manual
            t4 = threading.Thread(target=manual_thread, daemon=True)

        t4.start()
        self.after(500, lambda: self._watch_run(t4))

    def _watch_run(self, t4):
        if not t4.is_alive():
            self._set_status("● IDLE", C["SUB"])
        else:
            self.after(500, lambda: self._watch_run(t4))

    def _stop(self):
        global running
        if running:
            threading.Thread(target=stop_run, daemon=True).start()
            self._set_status("● STOPPING", C["ORG"])
            self.after(1200, lambda: self._set_status("● IDLE", C["SUB"]))

    # ── Telemetry display refresh (100ms) ────────────────────
    def _refresh_telemetry(self):
        for ms in (M1, M2):
            disp = getattr(self, f"_{ms.label.lower()}_rpm_disp")
            disp.config(text=f"{ms.actual_rpm:.2f} RPM")

            enc = getattr(self, f"_{ms.label.lower()}_enc_lbl")
            enc.config(text=f"Enc: {ms.enc_count}")

            tgt = getattr(self, f"_{ms.label.lower()}_tgt_lbl")
            tgt.config(text=f"Tgt: {ms.target_rpm:.1f}")

            cmd = getattr(self, f"_{ms.label.lower()}_cmd_lbl")
            cmd.config(text=f"Cmd: {ms.command_rpm:.1f}")

            lck = getattr(self, f"_{ms.label.lower()}_lock_lbl")
            if ms.freq_locked:
                lck.config(text="🔒 LOCKED", fg=C["ORG"])
            elif running and ms.target_rpm > 0:
                lck.config(text="⟳ PID active", fg=C["GRN"])
            else:
                lck.config(text="──", fg=C["SUB"])

        if ser and ser.is_open:
            self._serial_var.set(f"● Connected → {ser.port}")
        else:
            self._serial_var.set("✗ Disconnected")

        self.after(100, self._refresh_telemetry)

    # ── Window close ─────────────────────────────────────────
    def on_close(self):
        global running
        running = False
        time.sleep(0.2)
        stop_run()
        self.destroy()


# ═══════════════════════════════════════════════════════════════
if __name__ == "__main__":
    app = App()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()
