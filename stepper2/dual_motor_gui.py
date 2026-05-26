#!/usr/bin/env python3
"""
Dual Motor GUI — Two Independent PID Stepper Controllers
Click START BOTH → both run Random mode with independently random RPM sequences.

Motor 1 GPIO : STEP=17  DIR=27  ENA=22  ENC_A=24  ENC_B=23
Motor 2 GPIO : STEP=5   DIR=6   ENA=13  ENC_A=19  ENC_B=26

Based on gui_code.py (ZN-tuned PID, tick-based ramp, deadband lock).
"""

import tkinter as tk
from tkinter import ttk
import threading
import time
import random
import queue
import sys
import io

try:
    import lgpio
    HARDWARE = True
except ImportError:
    HARDWARE = False
    print("[GUI] lgpio not found — running in DEMO mode")

# ══════════════════════════════════════════════════════════════════════════════
# CONSTANTS
# ══════════════════════════════════════════════════════════════════════════════

COUNTS_PER_REV = 2000
MICROSTEPS     = 3200
MAX_RPM        = 120
FILTER_ALPHA   = 0.3
RAMP_UP        = 1.0
RAMP_DOWN      = 1.5
DEADBAND_RPM   = 0.8
DEADBAND_HOLD  = 3

ZN_GAINS = {
    "classic":      {"Kp": 0.240, "Ki": 2.400, "Kd": 0.006},
    "moderate":     {"Kp": 0.132, "Ki": 1.320, "Kd": 0.009},
    "no_overshoot": {"Kp": 0.080, "Ki": 0.800, "Kd": 0.005},
}

# ══════════════════════════════════════════════════════════════════════════════
# MOTOR CLASS  — all state and threads for one motor, fully independent
# ══════════════════════════════════════════════════════════════════════════════

class Motor:
    def __init__(self, name, step_pin, dir_pin, ena_pin, enc_a, enc_b,
                 zn_mode="classic",
                 rnd_min=3, rnd_max=30, rnd_tmin=5, rnd_tmax=15, rnd_steps=20):
        self.name     = name
        self.STEP_PIN = step_pin
        self.DIR_PIN  = dir_pin
        self.ENA_PIN  = ena_pin
        self.ENC_A    = enc_a
        self.ENC_B    = enc_b

        gains    = ZN_GAINS[zn_mode]
        self.Kp  = gains["Kp"]
        self.Ki  = gains["Ki"]
        self.Kd  = gains["Kd"]

        self.rnd_min   = rnd_min
        self.rnd_max   = rnd_max
        self.rnd_tmin  = rnd_tmin
        self.rnd_tmax  = rnd_tmax
        self.rnd_steps = rnd_steps

        # ── runtime state ────────────────────────────────────────────────────
        self.enc_count        = 0
        self.enc_lock         = threading.Lock()
        self.target_rpm       = 0.0
        self.command_rpm      = 0.0
        self.current_rpm      = 0.0
        self.filtered_rpm     = 0.0
        self.integral         = 0.0
        self.last_error       = 0.0
        self.last_count       = 0
        self.running          = False
        self.freq_locked      = False
        self.locked_command   = 0.0
        self.deadband_counter = 0
        self.h                = None
        self.log_file         = None
        self.log_queue        = queue.Queue()

        # set by GUI after widget build
        self._terminal = None
        self._gui_vars = {}
        self._zn_var   = None

    # ── helpers ──────────────────────────────────────────────────────────────

    def qprint(self, *args, **kwargs):
        buf = io.StringIO()
        print(*args, file=buf, **kwargs)
        self.log_queue.put(buf.getvalue())

    def _reset(self):
        self.enc_count = 0
        self.command_rpm = self.current_rpm = self.filtered_rpm = 0.0
        self.integral = self.last_error = 0.0
        self.last_count = 0
        self.freq_locked = False
        self.locked_command = 0.0
        self.deadband_counter = 0
        self.target_rpm = 0.0

    # ── threads ──────────────────────────────────────────────────────────────

    def _encoder_thread(self):
        if not HARDWARE:
            return
        last_a = lgpio.gpio_read(self.h, self.ENC_A)
        while self.running:
            a = lgpio.gpio_read(self.h, self.ENC_A)
            if a != last_a:
                b = lgpio.gpio_read(self.h, self.ENC_B)
                with self.enc_lock:
                    if a == b:
                        self.enc_count -= 1
                    else:
                        self.enc_count += 1
                last_a = a
            time.sleep(0.00005)

    def _pid_thread(self):
        last_flush = time.time()

        while self.running:
            t0 = time.time()

            with self.enc_lock:
                count = (self.enc_count if HARDWARE
                         else int(self.target_rpm * COUNTS_PER_REV / 60 * 0.1))

            delta           = count - self.last_count
            self.last_count = count

            actual_rpm        = (abs(delta) / COUNTS_PER_REV) * 600.0
            self.filtered_rpm = ((1.0 - FILTER_ALPHA) * self.filtered_rpm
                                 + FILTER_ALPHA * actual_rpm)

            error = self.target_rpm - self.filtered_rpm

            if self.target_rpm == 0:
                self.command_rpm      = 0.0
                self.integral         = 0.0
                self.filtered_rpm     = 0.0
                self.freq_locked      = False
                self.locked_command   = 0.0
                self.deadband_counter = 0
                self.last_error       = 0.0

            elif abs(error) <= DEADBAND_RPM:
                self.deadband_counter += 1
                if self.deadband_counter >= DEADBAND_HOLD:
                    if not self.freq_locked:
                        self.locked_command = self.command_rpm
                        self.freq_locked    = True
                        self.qprint(f"[{self.name}] LOCKED at {self.locked_command:.2f} RPM")
                    self.command_rpm = self.locked_command
            else:
                if self.freq_locked:
                    self.freq_locked      = False
                    self.deadband_counter = 0
                else:
                    self.deadband_counter = max(0, self.deadband_counter - 1)

                self.integral   = max(-80.0, min(80.0, self.integral + error * 0.1))
                derivative      = (error - self.last_error) / 0.1
                self.last_error = error
                pid_out          = (self.Kp * error
                                    + self.Ki * self.integral
                                    + self.Kd * derivative)
                self.command_rpm = max(0.0, min(float(MAX_RPM),
                                                self.target_rpm + pid_out))

            if self.log_file:
                t = time.time()
                self.log_file.write(
                    f"{t:.2f},{self.target_rpm:.2f},{actual_rpm:.2f},"
                    f"{self.filtered_rpm:.2f},{self.command_rpm:.2f}\n"
                )
                if time.time() - last_flush >= 5.0:
                    self.log_file.flush()
                    last_flush = time.time()

            sleep_t = 0.1 - (time.time() - t0)
            if sleep_t > 0:
                time.sleep(sleep_t)

    def _step_thread(self):
        step_state = 0

        while self.running:
            if not HARDWARE:
                time.sleep(0.01)
                continue

            if self.freq_locked and self.target_rpm > 0:
                self.current_rpm = self.locked_command
            else:
                if self.current_rpm < self.command_rpm:
                    self.current_rpm = min(
                        self.current_rpm + RAMP_UP, self.command_rpm)
                elif self.current_rpm > self.command_rpm:
                    self.current_rpm = max(
                        self.current_rpm - RAMP_DOWN, self.command_rpm)

            if self.current_rpm <= 0:
                lgpio.gpio_write(self.h, self.ENA_PIN, 1)
                time.sleep(0.001)
                continue

            lgpio.gpio_write(self.h, self.ENA_PIN, 0)
            sps         = self.current_rpm * MICROSTEPS / 60.0
            half_period = 1.0 / (2.0 * sps)
            step_state ^= 1
            lgpio.gpio_write(self.h, self.STEP_PIN, step_state)
            time.sleep(half_period)

    def _random_thread(self):
        self.qprint(f"\n[{self.name}] === RANDOM MODE STARTED ===")
        self.qprint(f"[{self.name}] RPM range  : {self.rnd_min} – {self.rnd_max}")
        self.qprint(f"[{self.name}] Time range : {self.rnd_tmin}s – {self.rnd_tmax}s")
        self.qprint(f"[{self.name}] Steps      : {self.rnd_steps}")
        self.qprint(f"[{self.name}] Starting in 3 seconds...")
        time.sleep(3.0)

        for idx in range(self.rnd_steps):
            if not self.running:
                break

            rpm       = random.randint(self.rnd_min, self.rnd_max)
            direction = random.choice([0, 1])
            dwell     = random.randint(self.rnd_tmin, self.rnd_tmax)
            dir_text  = "CW" if direction == 1 else "CCW"

            self.qprint(
                f"[{self.name}] Step {idx+1}/{self.rnd_steps} "
                f"→ {rpm} RPM  {dir_text}  hold {dwell}s"
            )

            if HARDWARE:
                lgpio.gpio_write(self.h, self.DIR_PIN, direction)
            self.target_rpm       = float(rpm)
            self.freq_locked      = False
            self.deadband_counter = 0

            elapsed = 0.0
            while elapsed < dwell and self.running:
                time.sleep(0.5)
                elapsed += 0.5

        self.qprint(f"\n[{self.name}] All steps complete — stopping.")
        self.target_rpm  = 0.0
        self.command_rpm = 0.0
        self.integral    = 0.0
        time.sleep(2.0)
        self.stop()

    # ── start / stop ─────────────────────────────────────────────────────────

    def start(self):
        self._reset()

        timestamp     = time.strftime("%Y%m%d_%H%M%S")
        log_fname     = f"{self.name}_log_{timestamp}.txt"
        self.log_file = open(log_fname, "w")
        self.log_file.write("Time,TargetRPM,ActualRPM,FilteredRPM,CommandRPM\n")

        if HARDWARE:
            self.h = lgpio.gpiochip_open(0)
            lgpio.gpio_claim_output(self.h, self.STEP_PIN, 0)
            lgpio.gpio_claim_output(self.h, self.DIR_PIN,  1)
            lgpio.gpio_claim_output(self.h, self.ENA_PIN,  1)
            lgpio.gpio_claim_input(self.h,  self.ENC_A, lgpio.SET_PULL_UP)
            lgpio.gpio_claim_input(self.h,  self.ENC_B, lgpio.SET_PULL_UP)
            time.sleep(0.1)

        self.running = True

        threading.Thread(target=self._encoder_thread, daemon=True).start()
        threading.Thread(target=self._pid_thread,     daemon=True).start()
        threading.Thread(target=self._step_thread,    daemon=True).start()

        t = threading.Thread(target=self._random_thread, daemon=False)
        t.start()
        return t

    def stop(self):
        self.running = False
        time.sleep(0.3)

        if HARDWARE and self.h is not None:
            try:
                lgpio.gpio_write(self.h, self.ENA_PIN,  1)
                lgpio.gpio_write(self.h, self.STEP_PIN, 0)
                for pin in (self.STEP_PIN, self.DIR_PIN,
                            self.ENA_PIN, self.ENC_A, self.ENC_B):
                    try:
                        lgpio.gpio_free(self.h, pin)
                    except Exception:
                        pass
                lgpio.gpiochip_close(self.h)
            except Exception as e:
                self.qprint(f"[{self.name}] GPIO cleanup warning: {e}")
            self.h = None

        if self.log_file:
            try:
                self.log_file.flush()
                self.log_file.close()
            except Exception:
                pass
        self.log_file = None


# ══════════════════════════════════════════════════════════════════════════════
# MOTOR INSTANCES
# Motor 1 — original stepper1 pins,  classic ZN,  low RPM range
# Motor 2 — second stepper pins,     moderate ZN, wider RPM range
# ══════════════════════════════════════════════════════════════════════════════

MOTOR1 = Motor(
    name="M1",
    step_pin=17, dir_pin=27, ena_pin=22, enc_a=24, enc_b=23,
    zn_mode="classic",
    rnd_min=3,  rnd_max=30, rnd_tmin=5, rnd_tmax=15, rnd_steps=20,
)

MOTOR2 = Motor(
    name="M2",
    step_pin=5,  dir_pin=6,  ena_pin=13, enc_a=19, enc_b=26,
    zn_mode="moderate",
    rnd_min=5,  rnd_max=50, rnd_tmin=4, rnd_tmax=12, rnd_steps=20,
)

# ══════════════════════════════════════════════════════════════════════════════
# GUI
# ══════════════════════════════════════════════════════════════════════════════

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Dual PID Stepper Controller")
        self.geometry("1140x820")
        self.resizable(False, False)
        self.configure(bg="#F4F6F9")

        self._build_styles()
        self._build_ui()
        self._poll_logs()

    # ── styles ───────────────────────────────────────────────────────────────

    def _build_styles(self):
        s = ttk.Style(self)
        s.theme_use("clam")

        BG   = "#F4F6F9"
        PNL  = "#FFFFFF"
        ACC  = "#1565C0"
        FG   = "#1E293B"
        SUB  = "#64748B"
        BTN  = "#E2E8F0"
        BTNF = "#CBD5E1"

        s.configure(".",            background=BG,  foreground=FG,
                    font=("Courier New", 10))
        s.configure("TFrame",       background=BG)
        s.configure("TLabel",       background=BG,  foreground=FG)
        s.configure("PNL.TLabel",   background=PNL, foreground=FG)
        s.configure("TEntry",       fieldbackground=BTN, foreground=FG,
                    insertcolor=ACC, relief="flat")
        s.configure("TCombobox",    fieldbackground=BTN, foreground=FG,
                    selectbackground=BTN, selectforeground=ACC)
        s.map("TCombobox",          fieldbackground=[("readonly", BTN)])

        s.configure("Run.TButton",  background="#1B7F3A", foreground="#FFFFFF",
                    font=("Courier New", 12, "bold"), relief="flat", padding=8)
        s.map("Run.TButton",        background=[("active", "#2E9E50")])

        s.configure("Stop.TButton", background="#C62828", foreground="#FFFFFF",
                    font=("Courier New", 12, "bold"), relief="flat", padding=8)
        s.map("Stop.TButton",       background=[("active", "#E53935")])

        s.configure("Sec.TButton",  background=BTNF, foreground=FG,
                    font=("Courier New", 10), relief="flat", padding=4)
        s.map("Sec.TButton",        background=[("active", "#94A3B8")])

        self._c = dict(BG=BG, PNL=PNL, ACC=ACC, FG=FG, SUB=SUB, BTN=BTN)

    # ── layout ───────────────────────────────────────────────────────────────

    def _build_ui(self):
        c = self._c

        # Header
        hdr = tk.Frame(self, bg=c["PNL"], height=58)
        hdr.pack(fill="x")
        tk.Label(
            hdr, text="⚙  DUAL PID STEPPER CONTROLLER",
            bg=c["PNL"], fg=c["ACC"],
            font=("Courier New", 15, "bold")
        ).pack(side="left", padx=18, pady=14)

        self._status_lbl = tk.Label(
            hdr, text="● IDLE",
            bg=c["PNL"], fg=c["SUB"],
            font=("Courier New", 11, "bold")
        )
        self._status_lbl.pack(side="right", padx=18)

        demo_tag = "" if HARDWARE else "  [DEMO MODE]"
        tk.Label(
            hdr, text=f"MICROSTEPS: {MICROSTEPS}{demo_tag}",
            bg=c["PNL"], fg=c["ACC"],
            font=("Courier New", 10, "bold")
        ).pack(side="right", padx=12)

        # Divider + control bar
        tk.Frame(self, bg="#D1D9E0", height=1).pack(fill="x")
        ctrl = tk.Frame(self, bg=c["PNL"])
        ctrl.pack(fill="x")
        btn_row = tk.Frame(ctrl, bg=c["PNL"])
        btn_row.pack(pady=10, padx=20)
        ttk.Button(
            btn_row, text="▶  START BOTH", style="Run.TButton",
            command=self._start_both
        ).pack(side="left", padx=8)
        ttk.Button(
            btn_row, text="■  STOP BOTH", style="Stop.TButton",
            command=self._stop_both
        ).pack(side="left", padx=8)
        tk.Frame(self, bg="#D1D9E0", height=1).pack(fill="x")

        # Two motor panels
        body = tk.Frame(self, bg=c["BG"])
        body.pack(fill="both", expand=True, padx=14, pady=10)
        body.columnconfigure(0, weight=1)
        body.columnconfigure(1, weight=1)

        self._build_motor_panel(body, MOTOR1, col=0)
        self._build_motor_panel(body, MOTOR2, col=1)

    def _build_motor_panel(self, parent, motor, col):
        c = self._c
        PNL, ACC, FG, SUB = c["PNL"], c["ACC"], c["FG"], c["SUB"]

        pad_right = (0, 7) if col == 0 else (7, 0)
        outer = tk.Frame(parent, bg=PNL, bd=0,
                         highlightbackground="#D1D9E0", highlightthickness=1)
        outer.grid(row=0, column=col, sticky="nsew", padx=pad_right)

        # Coloured card header (M1=blue, M2=teal-dark)
        hdr_color = "#1565C0" if col == 0 else "#0D6B52"
        card_hdr = tk.Frame(outer, bg=hdr_color, height=38)
        card_hdr.pack(fill="x")
        tk.Label(
            card_hdr,
            text=(f"  MOTOR {motor.name[-1]}  —  "
                  f"STEP={motor.STEP_PIN}  DIR={motor.DIR_PIN}  "
                  f"ENA={motor.ENA_PIN}  ENC_A={motor.ENC_A}  ENC_B={motor.ENC_B}"),
            bg=hdr_color, fg="#FFFFFF",
            font=("Courier New", 9, "bold")
        ).pack(side="left", padx=10, pady=9)

        # Config fields
        cfg = tk.Frame(outer, bg=PNL)
        cfg.pack(fill="x", padx=14, pady=(10, 4))

        fields = [
            ("Min RPM",     "rnd_min"),
            ("Max RPM",     "rnd_max"),
            ("Min Time (s)","rnd_tmin"),
            ("Max Time (s)","rnd_tmax"),
            ("Steps",       "rnd_steps"),
        ]
        gui_vars = {}
        for row, (lbl, attr) in enumerate(fields):
            tk.Label(cfg, text=lbl + ":", bg=PNL, fg=FG,
                     font=("Courier New", 9)).grid(
                row=row, column=0, sticky="w", pady=2)
            var = tk.StringVar(value=str(getattr(motor, attr)))
            ttk.Entry(cfg, textvariable=var, width=10).grid(
                row=row, column=1, sticky="w", padx=10)
            gui_vars[attr] = var
        motor._gui_vars = gui_vars

        # ZN mode selector
        tk.Label(cfg, text="ZN Mode:", bg=PNL, fg=FG,
                 font=("Courier New", 9)).grid(
            row=len(fields), column=0, sticky="w", pady=2)
        zn_var = tk.StringVar(
            value="classic" if col == 0 else "moderate")
        motor._zn_var = zn_var
        ttk.Combobox(cfg, textvariable=zn_var,
                     values=["classic", "moderate", "no_overshoot"],
                     width=16, state="readonly").grid(
            row=len(fields), column=1, sticky="w", padx=10, pady=2)

        # Divider
        tk.Frame(outer, bg="#E2E8F0", height=1).pack(
            fill="x", padx=14, pady=(6, 4))

        # Terminal label
        tk.Label(outer, text="TERMINAL LOG",
                 bg=PNL, fg=ACC,
                 font=("Courier New", 8, "bold")).pack(anchor="w", padx=14)

        # Terminal + scrollbar
        term_frame = tk.Frame(outer, bg=PNL)
        term_frame.pack(fill="both", expand=True, padx=14, pady=(2, 12))

        term = tk.Text(
            term_frame,
            bg="#1E293B", fg="#CBD5E1",
            font=("Courier New", 8),
            relief="flat", bd=0,
            wrap="word", state="disabled",
            width=52, height=30
        )
        sb = tk.Scrollbar(term_frame, command=term.yview, bg=PNL)
        term.configure(yscrollcommand=sb.set)
        term.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")

        term.tag_configure("info",   foreground="#94A3B8")
        term.tag_configure("step",   foreground="#10B981")
        term.tag_configure("dir",    foreground="#FBBF24")
        term.tag_configure("locked", foreground="#F59E0B")
        term.tag_configure("err",    foreground="#EF4444")
        term.tag_configure("done",   foreground="#3B82F6")

        motor._terminal = term

    # ── control ──────────────────────────────────────────────────────────────

    def _set_status(self, text, color="#1565C0"):
        self._status_lbl.config(text=text, fg=color)

    def _start_both(self):
        any_started = False
        for motor in (MOTOR1, MOTOR2):
            if motor.running:
                motor.qprint(f"[{motor.name}] Already running — skipped.")
                continue

            # Apply GUI values to motor before starting
            try:
                motor.rnd_min   = int(motor._gui_vars["rnd_min"].get())
                motor.rnd_max   = int(motor._gui_vars["rnd_max"].get())
                motor.rnd_tmin  = int(motor._gui_vars["rnd_tmin"].get())
                motor.rnd_tmax  = int(motor._gui_vars["rnd_tmax"].get())
                motor.rnd_steps = int(motor._gui_vars["rnd_steps"].get())
                gains = ZN_GAINS[motor._zn_var.get()]
                motor.Kp, motor.Ki, motor.Kd = gains["Kp"], gains["Ki"], gains["Kd"]
            except (ValueError, KeyError) as e:
                motor.qprint(f"[{motor.name}] Config error ({e}) — using defaults.")

            t = motor.start()
            self.after(500, lambda m=motor, th=t: self._watch(m, th))
            any_started = True

        if any_started:
            self._set_status("● RUNNING", "#1B7F3A")

    def _stop_both(self):
        for motor in (MOTOR1, MOTOR2):
            if motor.running:
                threading.Thread(target=motor.stop, daemon=True).start()
        self._set_status("● STOPPING", "#F57C00")
        self.after(1500, lambda: self._set_status("● IDLE", "#64748B"))

    def _watch(self, motor, thread):
        if thread.is_alive():
            self.after(500, lambda: self._watch(motor, thread))
        else:
            # Both done?
            if not MOTOR1.running and not MOTOR2.running:
                self._set_status("● IDLE", "#64748B")

    # ── log polling ──────────────────────────────────────────────────────────

    def _poll_logs(self):
        for motor in (MOTOR1, MOTOR2):
            if motor._terminal is None:
                continue
            try:
                while True:
                    text = motor.log_queue.get_nowait()
                    self._append(motor._terminal, text)
            except queue.Empty:
                pass
        self.after(80, self._poll_logs)

    def _append(self, term, text):
        term.configure(state="normal")
        tl = text.lower()
        if   "step "    in text:  tag = "step"
        elif "dir"      in tl:    tag = "dir"
        elif "locked"   in tl:    tag = "locked"
        elif "error"    in tl or "invalid" in tl: tag = "err"
        elif "complete" in tl or "stopping" in tl: tag = "done"
        else:                     tag = "info"
        term.insert("end", text, tag)
        term.see("end")
        term.configure(state="disabled")

    # ── close ─────────────────────────────────────────────────────────────────

    def on_close(self):
        for motor in (MOTOR1, MOTOR2):
            motor.running = False
            if HARDWARE and motor.h:
                try:
                    lgpio.gpio_write(motor.h, motor.ENA_PIN,  1)
                    lgpio.gpio_write(motor.h, motor.STEP_PIN, 0)
                    for pin in (motor.STEP_PIN, motor.DIR_PIN,
                                motor.ENA_PIN, motor.ENC_A, motor.ENC_B):
                        try:
                            lgpio.gpio_free(motor.h, pin)
                        except Exception:
                            pass
                    lgpio.gpiochip_close(motor.h)
                except Exception:
                    pass
            if motor.log_file:
                try:
                    motor.log_file.close()
                except Exception:
                    pass
        self.destroy()


# ══════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    app = App()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()
