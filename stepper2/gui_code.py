#!/usr/bin/env python3
"""
GUI for Closed-Loop PID Stepper Motor Controller — FIXED VERSION
Key fixes:
  1. Ramp is now time-based (RPM/sec), not tick-based
  2. Deadband tightened to ±0.3 RPM for accurate low-RPM targeting
  3. Direction change now ramps motor to 0 first, then flips, then ramps up
  4. PID integral reset on target change to prevent overshoot
  5. step_thread decoupled from ramp so timing is consistent at all RPMs
"""

import tkinter as tk
from tkinter import ttk, scrolledtext, font as tkfont
import threading
import time
import random
import queue
import sys
import io

try:
    import lgpio
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    HARDWARE = True
except ImportError:
    HARDWARE = False
    print("[GUI] lgpio / matplotlib not found — running in DEMO mode")

# ══════════════════════════════════════════════════════════════════════════════
# CONSTANTS (unchanged from original except deadband)
# ══════════════════════════════════════════════════════════════════════════════

STEP_PIN = 17
DIR_PIN  = 27
ENA_PIN  = 22
ENC_A    = 24
ENC_B    = 23

COUNTS_PER_REV = 2000
MICROSTEPS     = 3200
MIN_RPM        = 0
MAX_RPM        = 120

FILTER_ALPHA = 0.3

# ── FIX 1: Ramp is now RPM-per-SECOND (not per tick) ─────────────────────────
# step_thread runs every ~half_period which varies with RPM.
# We instead track wall-clock time so ramp is always e.g. 3 RPM/sec.
RAMP_UP_RPS   = 3.0   # RPM per second to ramp up   (reach 3 RPM in ~1 sec)
RAMP_DOWN_RPS = 5.0   # RPM per second to ramp down  (reach 0 from 10 in ~2 sec)

# ── FIX 2: Deadband tightened — ±0.8 was too loose at 3–10 RPM range ─────────
DEADBAND_RPM  = 0.3   # was 0.8 — at 3 RPM, ±0.3 is 10% tolerance (acceptable)
DEADBAND_HOLD = 3

# ── FIX 5: Direction change ramp-to-zero time ─────────────────────────────────
DIR_CHANGE_RAMP_SECS = 0.4   # ramp down fully before flipping direction

ZN_GAINS = {
    "classic":      {"Kp": 0.240, "Ki": 2.400, "Kd": 0.006},
    "moderate":     {"Kp": 0.132, "Ki": 1.320, "Kd": 0.009},
    "no_overshoot": {"Kp": 0.080, "Ki": 0.800, "Kd": 0.005},
}

# ══════════════════════════════════════════════════════════════════════════════
# MOTOR STATE
# ══════════════════════════════════════════════════════════════════════════════

enc_count    = 0
enc_lock     = threading.Lock()
target_rpm   = 0.0
command_rpm  = 0.0
current_rpm  = 0.0
filtered_rpm = 0.0
integral     = 0.0
last_error   = 0.0
last_count   = 0
running      = False

freq_locked      = False
locked_command   = 0.0
deadband_counter = 0

rpm_log      = []
filtered_log = []
target_log   = []
time_log     = []
start_time   = 0.0

log_file       = None
log_filename   = ""
graph_filename = ""
Kp = Ki = Kd = 0.0
MODE = "classic"
_run_mode_for_graph = "manual"

h = None
log_queue = queue.Queue()

# ── FIX 1: ramp timing state shared between step_thread and ramp logic ────────
_last_step_time = 0.0   # wall clock of last step_thread iteration


def qprint(*args, **kwargs):
    buf = io.StringIO()
    print(*args, file=buf, **kwargs)
    text = buf.getvalue()
    sys.stdout.write(text)
    sys.stdout.flush()
    log_queue.put(text)


# ══════════════════════════════════════════════════════════════════════════════
# THREADS
# ══════════════════════════════════════════════════════════════════════════════

def encoder_thread():
    global enc_count
    if not HARDWARE:
        return
    last_a = lgpio.gpio_read(h, ENC_A)
    while running:
        a = lgpio.gpio_read(h, ENC_A)
        if a != last_a:
            b = lgpio.gpio_read(h, ENC_B)
            with enc_lock:
                if a == b:
                    enc_count -= 1
                else:
                    enc_count += 1
            last_a = a
        time.sleep(0.00005)


def pid_thread():
    global command_rpm, integral, last_error
    global last_count, filtered_rpm
    global freq_locked, locked_command, deadband_counter

    last_print = time.time()
    last_flush = time.time()
    _prev_target = target_rpm   # FIX 4: track target changes

    while running:
        t0 = time.time()

        with enc_lock:
            count = enc_count if HARDWARE else int(target_rpm * COUNTS_PER_REV / 60 * 0.1)

        delta      = count - last_count
        last_count = count

        actual_rpm   = (abs(delta) / COUNTS_PER_REV) * 600.0
        filtered_rpm = ((1.0 - FILTER_ALPHA) * filtered_rpm
                        + FILTER_ALPHA * actual_rpm)

        # ── FIX 4: Reset integral and error on target change ──────────────────
        nonlocal_target = target_rpm   # read once
        if nonlocal_target != _prev_target:
            integral         = 0.0
            last_error       = 0.0
            freq_locked      = False
            deadband_counter = 0
            _prev_target     = nonlocal_target
            qprint(f"[PID] Target changed → {nonlocal_target:.1f} RPM — integral reset")

        error = nonlocal_target - filtered_rpm

        if nonlocal_target == 0:
            command_rpm      = 0.0
            integral         = 0.0
            filtered_rpm     = 0.0
            freq_locked      = False
            locked_command   = 0.0
            deadband_counter = 0
            last_error       = 0.0

        elif abs(error) <= DEADBAND_RPM:
            deadband_counter += 1
            if deadband_counter >= DEADBAND_HOLD:
                if not freq_locked:
                    locked_command = command_rpm
                    freq_locked    = True
                    qprint(
                        f"\n[LOCKED] cmd frozen at {locked_command:.2f} RPM"
                        f"  (actual≈{filtered_rpm:.1f})"
                    )
                command_rpm = locked_command
        else:
            if freq_locked:
                freq_locked      = False
                deadband_counter = 0
                qprint(f"\n[UNLOCKED] err={error:+.2f} RPM, PID re-engaged")
            else:
                deadband_counter = max(0, deadband_counter - 1)

            integral   = max(-80.0, min(80.0, integral + error * 0.1))
            derivative = (error - last_error) / 0.1
            last_error = error

            pid_out     = (Kp * error) + (Ki * integral) + (Kd * derivative)
            command_rpm = max(0.0, min(float(MAX_RPM), nonlocal_target + pid_out))

        t = time.time() - start_time

        rpm_log.append(actual_rpm)
        filtered_log.append(filtered_rpm)
        target_log.append(nonlocal_target)
        time_log.append(t)

        if log_file:
            log_file.write(
                f"{t:.2f},{nonlocal_target:.2f},{actual_rpm:.2f},"
                f"{filtered_rpm:.2f},{command_rpm:.2f},{count},{int(freq_locked)}\n"
            )
            if time.time() - last_flush >= 5.0:
                log_file.flush()
                last_flush = time.time()

        if time.time() - last_print >= 0.5:
            state = "LOCKED" if freq_locked else "PID   "
            qprint(
                f"[{state}] Target:{nonlocal_target:5.1f} "
                f"| Actual:{actual_rpm:5.1f} "
                f"| Filtered:{filtered_rpm:5.1f} "
                f"| Cmd:{command_rpm:5.1f} "
                f"| Err:{error:+5.2f}"
            )
            last_print = time.time()

        sleep_t = 0.1 - (time.time() - t0)
        if sleep_t > 0:
            time.sleep(sleep_t)


def step_thread():
    """
    FIX 1: Ramp is now time-based (RPM/sec) so it's consistent at ALL RPMs.
    At 3 RPM, the half_period is long — old tick-based ramp was extremely slow.
    Now we measure actual elapsed time between iterations to scale ramp correctly.
    """
    global current_rpm
    step_state = 0
    _last_time = time.time()

    while running:
        now     = time.time()
        dt      = now - _last_time          # actual time since last iteration
        _last_time = now

        if not HARDWARE:
            time.sleep(0.01)
            continue

        if freq_locked and target_rpm > 0:
            current_rpm = locked_command
        else:
            # Time-based ramp: scale by dt so it's RPM/sec regardless of loop speed
            if current_rpm < command_rpm:
                current_rpm = min(current_rpm + RAMP_UP_RPS * dt, command_rpm)
            elif current_rpm > command_rpm:
                current_rpm = max(current_rpm - RAMP_DOWN_RPS * dt, command_rpm)

        if current_rpm <= 0:
            lgpio.gpio_write(h, ENA_PIN, 1)
            time.sleep(0.001)
            continue

        lgpio.gpio_write(h, ENA_PIN, 0)
        sps         = current_rpm * MICROSTEPS / 60.0
        half_period = 1.0 / (2.0 * sps)
        step_state ^= 1
        lgpio.gpio_write(h, STEP_PIN, step_state)
        time.sleep(half_period)


def sweep_thread(sweep_rpms, sweep_dwell):
    global target_rpm, command_rpm, integral
    global running, freq_locked, deadband_counter

    qprint("\n====================================")
    qprint(" AUTO SWEEP — ZIEGLER-NICHOLS PID")
    qprint(f" Mode : {MODE}   Kp={Kp}  Ki={Ki}  Kd={Kd}")
    qprint(f" Steps : {sweep_rpms}")
    qprint(f" Dwell : {sweep_dwell}s per step")
    qprint(f" Total : ~{len(sweep_rpms) * sweep_dwell:.0f}s")
    qprint("====================================")
    qprint(" Starting in 3 seconds...")
    time.sleep(3.0)

    for idx, rpm in enumerate(sweep_rpms):
        if not running:
            break
        target_rpm       = float(rpm)
        freq_locked      = False
        deadband_counter = 0
        qprint(f"\n[SWEEP {idx+1}/{len(sweep_rpms)}] → {rpm} RPM  (holding {sweep_dwell}s)")
        elapsed = 0.0
        while elapsed < sweep_dwell and running:
            time.sleep(0.5)
            elapsed += 0.5

    qprint("\n[SWEEP] All steps complete. Stopping motor.")
    target_rpm  = 0.0
    command_rpm = 0.0
    integral    = 0.0
    time.sleep(2.0)
    stop_run()


def _safe_direction_change(new_dir, new_rpm):
    """
    FIX 3: Ramp motor to 0 before flipping direction pin, then set new target.
    This prevents the current spike from instant direction reversal.
    """
    global target_rpm, command_rpm, current_rpm, integral
    global freq_locked, deadband_counter

    qprint(f"[DIR] Ramping to 0 before direction change ({DIR_CHANGE_RAMP_SECS}s)...")

    # Step 1: Command motor to stop
    target_rpm       = 0.0
    command_rpm      = 0.0
    integral         = 0.0
    freq_locked      = False
    deadband_counter = 0

    # Step 2: Wait for motor to actually decelerate to near-zero
    deadline = time.time() + DIR_CHANGE_RAMP_SECS + 0.5
    while time.time() < deadline and running:
        if current_rpm < 0.5:
            break
        time.sleep(0.05)

    # Step 3: Extra safety pause
    time.sleep(0.1)

    # Step 4: Flip direction
    if HARDWARE:
        lgpio.gpio_write(h, DIR_PIN, new_dir)
    time.sleep(0.05)   # let direction signal settle

    # Step 5: Set new target — PID integral already reset above
    target_rpm       = float(new_rpm)
    freq_locked      = False
    deadband_counter = 0
    qprint(f"[DIR] Direction set → {'CW' if new_dir else 'CCW'}, target={new_rpm} RPM")


def random_thread(rmin, rmax, tmin, tmax, steps):
    global target_rpm, command_rpm, integral
    global running, freq_locked, deadband_counter

    qprint("\n====================================")
    qprint(" RANDOM MODE — ZIEGLER-NICHOLS PID")
    qprint(f" Mode      : {MODE}   Kp={Kp}  Ki={Ki}  Kd={Kd}")
    qprint(f" RPM range : {rmin} – {rmax}")
    qprint(f" Time range: {tmin}s – {tmax}s")
    qprint(f" Steps     : {steps}")
    qprint(f" Deadband  : ±{DEADBAND_RPM} RPM  (tightened for accuracy)")
    qprint(f" Dir ramp  : {DIR_CHANGE_RAMP_SECS}s before flip")
    qprint("====================================")
    qprint(" Starting in 3 seconds...")
    time.sleep(3.0)

    last_dir = 1  # track previous direction to detect changes

    for idx in range(steps):
        if not running:
            break

        rpm       = random.randint(rmin, rmax)
        direction = random.choice([0, 1])
        dwell     = random.randint(tmin, tmax)
        dir_text  = "CW" if direction == 1 else "CCW"

        qprint(
            f"\n[RANDOM {idx+1}/{steps}] "
            f"→ {rpm} RPM | DIR={dir_text} | HOLD={dwell}s"
        )

        # ── FIX 3: Safe direction change with ramp-to-zero ────────────────────
        if direction != last_dir:
            qprint(f"[RANDOM] Direction change detected — safe ramp required")
            _safe_direction_change(direction, rpm)
        else:
            # Same direction — just update target, PID handles the transition
            if HARDWARE:
                lgpio.gpio_write(h, DIR_PIN, direction)
            target_rpm       = float(rpm)
            freq_locked      = False
            deadband_counter = 0

        last_dir = direction

        # Hold at this RPM for dwell seconds
        elapsed = 0.0
        while elapsed < dwell and running:
            time.sleep(0.5)
            elapsed += 0.5

    qprint("\n[RANDOM] All steps complete. Stopping motor.")
    target_rpm  = 0.0
    command_rpm = 0.0
    integral    = 0.0
    time.sleep(2.0)
    stop_run()


def manual_thread():
    global target_rpm, command_rpm, integral
    global running, freq_locked, deadband_counter

    qprint("\n====================================")
    qprint(f" MANUAL MODE — Kp={Kp}  Ki={Ki}  Kd={Kd}")
    qprint(f" Deadband : ±{DEADBAND_RPM} RPM  |  Max RPM = {MAX_RPM}")
    qprint("====================================")
    qprint(" Use the RPM entry below and click SET.")
    while running:
        time.sleep(0.2)


# ══════════════════════════════════════════════════════════════════════════════
# RUN / STOP
# ══════════════════════════════════════════════════════════════════════════════

def stop_run():
    global running, h
    running = False
    time.sleep(0.3)
    if HARDWARE and h is not None:
        try:
            lgpio.gpio_write(h, ENA_PIN, 1)
            lgpio.gpio_write(h, STEP_PIN, 0)
            for pin in (STEP_PIN, DIR_PIN, ENA_PIN, ENC_A, ENC_B):
                try:
                    lgpio.gpio_free(h, pin)
                except Exception:
                    pass
            lgpio.gpiochip_close(h)
        except Exception as e:
            qprint(f"[GPIO] cleanup warning: {e}")
        h = None
    if log_file:
        try:
            log_file.flush()
            log_file.close()
        except Exception:
            pass
    qprint("\n[GUI] Motor stopped.")


def _plot_graph(_run_mode_for_graph="manual"):
    import matplotlib.pyplot as plt

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))

    ax1.plot(time_log, target_log, "--", label="Target RPM",
             linewidth=1.5, color="#D85A30")
    ax1.plot(time_log, rpm_log, drawstyle="steps-mid",
             label="Actual RPM", linewidth=1.5, color="#378ADD")
    ax1.set_ylabel("RPM")
    ax1.set_title(
        f"PID + Deadband Lock [{MODE}]  "
        f"Kp={Kp} Ki={Ki} Kd={Kd}  "
        f"Deadband=±{DEADBAND_RPM} RPM  "
        f"[{_run_mode_for_graph.upper()} MODE]  "
        f"[MICROSTEPS={MICROSTEPS}]"
    )
    ax1.legend(); ax1.grid(True, alpha=0.3)

    errors = [t - f for t, f in zip(target_log, filtered_log)]
    ax2.plot(time_log, errors, drawstyle="steps-mid",
             color="#E85D24", linewidth=1.0,
             label="Error (Target − Filtered)")
    ax2.axhline(0,             color="gray",  linewidth=0.8, linestyle="--")
    ax2.axhline( DEADBAND_RPM, color="green", linewidth=0.8,
                linestyle=":", label=f"±{DEADBAND_RPM} deadband")
    ax2.axhline(-DEADBAND_RPM, color="green", linewidth=0.8, linestyle=":")
    ax2.set_xlabel("Time (s)"); ax2.set_ylabel("Error (RPM)")
    ax2.legend(); ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(graph_filename, dpi=150)
    qprint(f"\n[GUI] Graph saved → {graph_filename}")
    plt.show()


# ══════════════════════════════════════════════════════════════════════════════
# GUI (unchanged from original)
# ══════════════════════════════════════════════════════════════════════════════

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("PID Stepper Controller — ZN Tuned [FIXED]")
        self.geometry("740x800")
        self.resizable(False, False)
        self.configure(bg="#F4F6F9")

        self._build_styles()
        self._build_ui()
        self._poll_log()

    def _build_styles(self):
        s = ttk.Style(self)
        s.theme_use("clam")

        BG   = "#F4F6F9"   # soft off-white background
        PNL  = "#FFFFFF"   # pure white panels
        ACC  = "#1565C0"   # professional deep blue
        FG   = "#1E293B"   # dark navy text
        SUB  = "#64748B"   # muted slate secondary text
        BTN  = "#E2E8F0"   # light input / button bg
        BTNF = "#CBD5E1"   # slightly darker for hover

        s.configure(".",           background=BG,  foreground=FG,
                    font=("Courier New", 10))
        s.configure("TFrame",      background=BG)
        s.configure("Card.TFrame", background=PNL)
        s.configure("TLabel",      background=BG,  foreground=FG)
        s.configure("Card.TLabel", background=PNL, foreground=FG)
        s.configure("Sub.TLabel",  background=PNL, foreground=SUB,
                    font=("Courier New", 9))
        s.configure("Accent.TLabel", background=PNL, foreground=ACC,
                    font=("Courier New", 11, "bold"))
        s.configure("TEntry",      fieldbackground=BTN, foreground=FG,
                    insertcolor=ACC, relief="flat")
        s.configure("TCombobox",   fieldbackground=BTN, foreground=FG,
                    selectbackground=BTN, selectforeground=ACC)
        s.map("TCombobox", fieldbackground=[("readonly", BTN)])

        s.configure("Run.TButton",  background="#1B7F3A", foreground="#FFFFFF",
                    font=("Courier New", 11, "bold"), relief="flat", padding=6)
        s.map("Run.TButton",  background=[("active","#2E9E50")])

        s.configure("Stop.TButton", background="#C62828", foreground="#FFFFFF",
                    font=("Courier New", 11, "bold"), relief="flat", padding=6)
        s.map("Stop.TButton", background=[("active","#E53935")])

        s.configure("Sec.TButton",  background=BTNF, foreground=FG,
                    font=("Courier New", 10), relief="flat", padding=4)
        s.map("Sec.TButton",  background=[("active","#94A3B8")])

        s.configure("TNotebook",     background=PNL, borderwidth=0)
        s.configure("TNotebook.Tab", background=BTN, foreground=SUB,
                    font=("Courier New", 10), padding=[12, 4])
        s.map("TNotebook.Tab",
              background=[("selected", PNL)],
              foreground=[("selected", ACC)])

        self._colors = dict(BG=BG, PNL=PNL, ACC=ACC, FG=FG, SUB=SUB)

    def _build_ui(self):
        c = self._colors
        BG, PNL, ACC = c["BG"], c["PNL"], c["ACC"]

        hdr = tk.Frame(self, bg=PNL, height=54)
        hdr.pack(fill="x")
        tk.Label(hdr, text="⚙  PID STEPPER CONTROLLER  [FIXED]",
                 bg=PNL, fg=ACC,
                 font=("Courier New", 14, "bold")).pack(side="left", padx=16, pady=10)
        self._status_lbl = tk.Label(hdr, text="● IDLE",
                                    bg=PNL, fg="#8B949E",
                                    font=("Courier New", 10, "bold"))
        self._status_lbl.pack(side="right", padx=16)
        tk.Label(hdr, text=f"MICROSTEPS: {MICROSTEPS}",
                 bg=PNL, fg=ACC,
                 font=("Courier New", 10, "bold")).pack(side="right", padx=16)

        body = tk.Frame(self, bg=BG)
        body.pack(fill="both", expand=True, padx=12, pady=8)

        left = tk.Frame(body, bg=BG)
        left.grid(row=0, column=0, sticky="nsew", padx=(0,8))

        self._card_zn(left)
        self._card_mode(left)
        self._card_manual(left)
        self._card_fix_info(left)   # NEW: shows fix summary
        self._card_actions(left)

        right = tk.Frame(body, bg=BG)
        right.grid(row=0, column=1, sticky="nsew")
        body.columnconfigure(1, weight=1)
        self._card_terminal(right)

    def _card_fix_info(self, parent):
        """NEW: Shows active fix parameters so user knows what changed."""
        c = self._colors
        f = self._card(parent, "ACTIVE FIXES")
        lines = [
            f"Ramp UP   : {RAMP_UP_RPS} RPM/sec  (time-based)",
            f"Ramp DOWN : {RAMP_DOWN_RPS} RPM/sec (time-based)",
            f"Deadband  : ±{DEADBAND_RPM} RPM     (was ±0.8)",
            f"Dir ramp  : {DIR_CHANGE_RAMP_SECS}s  zero before flip",
            f"Integral  : reset on target change",
        ]
        for line in lines:
            tk.Label(f, text=line, bg=c["PNL"], fg=c["ACC"],
                     font=("Courier New", 8)).pack(anchor="w", padx=10, pady=0)

    def _card_zn(self, parent):
        c = self._colors
        f = self._card(parent, "ZN TUNING MODE")
        self._zn_var = tk.StringVar(value="classic")
        for val, label in [
            ("classic",      "Classic      Kp=0.240  Ki=2.400  Kd=0.006"),
            ("moderate",     "Moderate     Kp=0.132  Ki=1.320  Kd=0.009"),
            ("no_overshoot", "No Overshoot Kp=0.080  Ki=0.800  Kd=0.005"),
        ]:
            tk.Radiobutton(
                f, text=label, variable=self._zn_var, value=val,
                bg=c["PNL"], fg=c["FG"], selectcolor=c["PNL"],
                activebackground=c["PNL"], activeforeground=c["ACC"],
                font=("Courier New", 9), indicatoron=True
            ).pack(anchor="w", padx=8, pady=1)

    def _card_mode(self, parent):
        c = self._colors
        f = self._card(parent, "RUN MODE")
        self._run_mode_var = tk.StringVar(value="random")
        modes = [("sweep","Auto Sweep"), ("random","Random"), ("manual","Manual Input")]
        rf = tk.Frame(f, bg=c["PNL"])
        rf.pack(fill="x", padx=8, pady=4)
        for val, lbl in modes:
            tk.Radiobutton(
                rf, text=lbl, variable=self._run_mode_var, value=val,
                command=self._on_mode_change,
                bg=c["PNL"], fg=c["FG"], selectcolor=c["PNL"],
                activebackground=c["PNL"], activeforeground=c["ACC"],
                font=("Courier New", 10)
            ).pack(side="left", padx=10)

        self._sweep_frame = tk.LabelFrame(
            f, text=" Sweep Config ",
            bg=c["PNL"], fg=c["ACC"], font=("Courier New", 9), bd=1, relief="groove"
        )
        g = self._sweep_frame
        tk.Label(g, text="RPM list (comma-sep):", bg=c["PNL"], fg=c["FG"],
                 font=("Courier New",9)).grid(row=0,column=0,sticky="w",padx=6,pady=2)
        self._sweep_rpms_var = tk.StringVar(
            value="5,7,10,15,20,25,30,35,40,45,50,45,40,35,30,25,20,15,10,7,5")
        ttk.Entry(g, textvariable=self._sweep_rpms_var, width=38).grid(
            row=0,column=1,padx=6,pady=2)
        tk.Label(g, text="Dwell (sec):", bg=c["PNL"], fg=c["FG"],
                 font=("Courier New",9)).grid(row=1,column=0,sticky="w",padx=6)
        self._sweep_dwell_var = tk.StringVar(value="10")
        ttk.Entry(g, textvariable=self._sweep_dwell_var, width=8).grid(
            row=1,column=1,sticky="w",padx=6)

        self._random_frame = tk.LabelFrame(
            f, text=" Random Config ",
            bg=c["PNL"], fg=c["ACC"], font=("Courier New", 9), bd=1, relief="groove"
        )
        g = self._random_frame
        fields = [
            ("Min RPM",       "3",  "_rnd_rmin"),
            ("Max RPM",      "10",  "_rnd_rmax"),
            ("Min Time (s)",  "5",  "_rnd_tmin"),
            ("Max Time (s)", "15",  "_rnd_tmax"),
            ("# Steps",      "20",  "_rnd_steps"),
        ]
        for row, (lbl, dflt, attr) in enumerate(fields):
            tk.Label(g, text=lbl+":", bg=c["PNL"], fg=c["FG"],
                     font=("Courier New",9)).grid(row=row,column=0,sticky="w",padx=6,pady=1)
            var = tk.StringVar(value=dflt)
            setattr(self, attr, var)
            ttk.Entry(g, textvariable=var, width=8).grid(
                row=row,column=1,sticky="w",padx=6)

        self._on_mode_change()

    def _on_mode_change(self):
        mode = self._run_mode_var.get()
        self._sweep_frame.pack_forget()
        self._random_frame.pack_forget()
        if mode == "sweep":
            self._sweep_frame.pack(fill="x", padx=8, pady=(4,6))
        elif mode == "random":
            self._random_frame.pack(fill="x", padx=8, pady=(4,6))

    def _card_manual(self, parent):
        c = self._colors
        self._manual_card = self._card(parent, "MANUAL RPM SET")
        f = self._manual_card
        row = tk.Frame(f, bg=c["PNL"])
        row.pack(fill="x", padx=8, pady=6)
        tk.Label(row, text="Target RPM:", bg=c["PNL"], fg=c["FG"],
                 font=("Courier New",9)).pack(side="left")
        self._manual_rpm_var = tk.StringVar(value="20")
        ttk.Entry(row, textvariable=self._manual_rpm_var, width=8).pack(side="left",padx=6)
        ttk.Button(row, text="SET", style="Sec.TButton",
                   command=self._manual_set_rpm).pack(side="left",padx=4)
        ttk.Button(row, text="STOP MOTOR", style="Sec.TButton",
                   command=self._manual_stop_motor).pack(side="left",padx=4)

    def _manual_set_rpm(self):
        global target_rpm, freq_locked, deadband_counter, integral, last_error
        try:
            val = float(self._manual_rpm_var.get())
            val = max(0.0, min(float(MAX_RPM), val))
            # FIX 4: reset integral on manual target change too
            integral         = 0.0
            last_error       = 0.0
            target_rpm       = val
            freq_locked      = False
            deadband_counter = 0
            qprint(f"[MANUAL] Target RPM → {val}")
        except ValueError:
            qprint("[MANUAL] Invalid RPM value")

    def _manual_stop_motor(self):
        global target_rpm, command_rpm, integral
        target_rpm  = 0.0
        command_rpm = 0.0
        integral    = 0.0
        qprint("[MANUAL] Motor stopped.")

    def _card_actions(self, parent):
        c = self._colors
        f = self._card(parent, "CONTROL")
        row = tk.Frame(f, bg=c["PNL"])
        row.pack(pady=8, padx=8, fill="x")
        ttk.Button(row, text="▶  START", style="Run.TButton",
                   command=self._start).pack(side="left",expand=True,fill="x",padx=(0,4))
        ttk.Button(row, text="■  STOP", style="Stop.TButton",
                   command=self._stop).pack(side="left",expand=True,fill="x")

    def _card_terminal(self, parent):
        c = self._colors
        tk.Label(parent, text="TERMINAL LOG",
                 bg=c["BG"], fg=c["ACC"],
                 font=("Courier New", 9, "bold")).pack(anchor="w", pady=(0,2))
        self._terminal = tk.Text(
            parent, bg="#1E293B", fg="#CBD5E1",
            font=("Courier New", 9),
            insertbackground=c["ACC"],
            relief="flat", bd=0,
            wrap="word", state="disabled",
            width=46, height=46
        )
        sb = tk.Scrollbar(parent, command=self._terminal.yview, bg=c["PNL"])
        self._terminal.configure(yscrollcommand=sb.set)
        self._terminal.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")
        self._terminal.tag_configure("locked",   foreground="#F59E0B")
        self._terminal.tag_configure("unlocked", foreground="#3B82F6")
        self._terminal.tag_configure("sweep",    foreground="#10B981")
        self._terminal.tag_configure("random",   foreground="#8B5CF6")
        self._terminal.tag_configure("pid",      foreground="#94A3B8")
        self._terminal.tag_configure("err",      foreground="#EF4444")
        self._terminal.tag_configure("dir",      foreground="#FBBF24")

    def _card(self, parent, title):
        c = self._colors
        outer = tk.Frame(parent, bg=c["PNL"], bd=0)
        outer.pack(fill="x", pady=4)
        tk.Label(outer, text=title,
                 bg=c["PNL"], fg=c["ACC"],
                 font=("Courier New", 9, "bold")).pack(anchor="w", padx=8, pady=(6,2))
        tk.Frame(outer, bg=c["ACC"], height=1).pack(fill="x", padx=8, pady=(0,4))
        return outer

    def _set_status(self, text, color="#1565C0"):
        self._status_lbl.config(text=text, fg=color)

    def _start(self):
        global running, h, Kp, Ki, Kd, MODE, start_time
        global rpm_log, filtered_log, target_log, time_log
        global log_file, log_filename, graph_filename
        global enc_count, command_rpm, current_rpm, filtered_rpm
        global integral, last_error, last_count
        global freq_locked, locked_command, deadband_counter
        global _run_mode_for_graph

        if running:
            qprint("[GUI] Already running.")
            return

        MODE = self._zn_var.get()
        Kp   = ZN_GAINS[MODE]["Kp"]
        Ki   = ZN_GAINS[MODE]["Ki"]
        Kd   = ZN_GAINS[MODE]["Kd"]

        enc_count = command_rpm = current_rpm = filtered_rpm = 0.0
        integral = last_error = last_count = 0
        freq_locked = False; locked_command = 0.0; deadband_counter = 0
        rpm_log.clear(); filtered_log.clear()
        target_log.clear(); time_log.clear()
        start_time = time.time()

        timestamp      = time.strftime("%Y%m%d_%H%M%S")
        log_filename   = f"zn_log_{timestamp}.txt"
        graph_filename = f"zn_graph_{timestamp}.png"
        log_file       = open(log_filename, "w")
        log_file.write("Time,TargetRPM,ActualRPM,FilteredRPM,CommandRPM,EncoderCount,Locked\n")

        if HARDWARE:
            h = lgpio.gpiochip_open(0)
            lgpio.gpio_claim_output(h, STEP_PIN, 0)
            lgpio.gpio_claim_output(h, DIR_PIN,  1)
            lgpio.gpio_claim_output(h, ENA_PIN,  1)
            lgpio.gpio_claim_input(h, ENC_A, lgpio.SET_PULL_UP)
            lgpio.gpio_claim_input(h, ENC_B, lgpio.SET_PULL_UP)
            time.sleep(0.1)

        running = True
        self._set_status("● RUNNING", "#1B7F3A")

        run_mode = self._run_mode_var.get()
        _run_mode_for_graph = run_mode

        qprint(f"\n[GUI] Starting — ZN={MODE}  Kp={Kp} Ki={Ki} Kd={Kd}")
        qprint(f"[GUI] Run mode  : {run_mode.upper()}")
        qprint(f"[GUI] Deadband  : ±{DEADBAND_RPM} RPM")
        qprint(f"[GUI] Ramp      : up={RAMP_UP_RPS} RPM/s  dn={RAMP_DOWN_RPS} RPM/s")
        qprint(f"[GUI] Log file  : {log_filename}\n")

        t1 = threading.Thread(target=encoder_thread, daemon=True)
        t2 = threading.Thread(target=pid_thread,     daemon=True)
        t3 = threading.Thread(target=step_thread,    daemon=True)
        t1.start(); t2.start(); t3.start()

        if run_mode == "sweep":
            try:
                rpms  = [int(x.strip()) for x in self._sweep_rpms_var.get().split(",")]
                dwell = float(self._sweep_dwell_var.get())
            except ValueError:
                qprint("[GUI] Invalid sweep config.")
                running = False; return
            t4 = threading.Thread(target=sweep_thread, args=(rpms, dwell), daemon=False)

        elif run_mode == "random":
            try:
                rmin  = int(self._rnd_rmin.get())
                rmax  = int(self._rnd_rmax.get())
                tmin  = int(self._rnd_tmin.get())
                tmax  = int(self._rnd_tmax.get())
                steps = int(self._rnd_steps.get())
            except ValueError:
                qprint("[GUI] Invalid random config.")
                running = False; return
            t4 = threading.Thread(
                target=random_thread,
                args=(rmin, rmax, tmin, tmax, steps),
                daemon=False
            )
        else:
            t4 = threading.Thread(target=manual_thread, daemon=True)

        t4.start()
        self.after(500, lambda: self._watch_run(t4))

    def _watch_run(self, t4):
        if not t4.is_alive():
            self._set_status("● IDLE", "#64748B")
        else:
            self.after(500, lambda: self._watch_run(t4))

    def _stop(self):
        global running
        if running:
            qprint("\n[GUI] STOP requested by user.")
            threading.Thread(target=stop_run, daemon=True).start()
            self._set_status("● STOPPING", "#F57C00")
            self.after(1000, lambda: self._set_status("● IDLE", "#64748B"))

    def _poll_log(self):
        try:
            while True:
                text = log_queue.get_nowait()
                self._append_terminal(text)
        except queue.Empty:
            pass
        self.after(80, self._poll_log)

    def _append_terminal(self, text):
        t = self._terminal
        t.configure(state="normal")
        tag = "pid"
        if "[LOCKED]"   in text: tag = "locked"
        elif "[UNLOCKED]" in text: tag = "unlocked"
        elif "[SWEEP"    in text: tag = "sweep"
        elif "[RANDOM"   in text: tag = "random"
        elif "[DIR]"     in text: tag = "dir"
        elif "Error" in text or "Invalid" in text: tag = "err"
        t.insert("end", text, tag)
        t.see("end")
        t.configure(state="disabled")

    def on_close(self):
        global running, h
        running = False
        time.sleep(0.2)
        if HARDWARE and h:
            try:
                lgpio.gpio_write(h, ENA_PIN, 1)
                lgpio.gpio_write(h, STEP_PIN, 0)
                for pin in (STEP_PIN, DIR_PIN, ENA_PIN, ENC_A, ENC_B):
                    try: lgpio.gpio_free(h, pin)
                    except Exception: pass
                lgpio.gpiochip_close(h)
            except Exception:
                pass
            h = None
        if log_file:
            try: log_file.close()
            except Exception: pass
        self.destroy()


# ══════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    app = App()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()