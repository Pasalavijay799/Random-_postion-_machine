#!/usr/bin/env python3

# =========================================================
# CLOSED LOOP PID SPEED CONTROL
# Tuned using Ziegler-Nichols Method
#
# Motor  : PR42H34-1334BED (NEMA17 with encoder)
# Driver : TB6600
# Board  : Raspberry Pi 4
# Encoder: 500 PPR (2000 counts/rev)
#
# TB6600 DIP SWITCHES:
#   S1=ON  S2=ON  S3=OFF  → 200 pulse/rev (full step)
#   S4=ON  S5=ON  S6=OFF  → 1.5A current
#
# WIRING (BCM):
#   GPIO17 → PUL+   GPIO27 → DIR+   GPIO22 → ENA+
#   GPIO24 ← Enc A+ (orange)
#   GPIO23 ← Enc B+ (blue)
#   3.3V   → Enc VCC (red)   ⚠ NOT 5V
#   GND    → Enc GND (black) + TB6600 PUL- DIR- ENA- GND
#   24V    → TB6600 VCC+
#
# RUN: sudo python3 stepper_zn_pid_random.py
#
# =========================================================
# DEADBAND LOCK EXPLAINED
# =========================================================
# Problem: When actual RPM ≈ target RPM, PID still adjusts
#          command_rpm slightly → step thread changes
#          half_period → frequency wobbles HIGH/LOW.
#
# Fix:
#   1. FILTER_ALPHA = 0.3  (smooth noisy encoder reading)
#   2. DEADBAND_RPM = 0.8  (±0.8 RPM tolerance zone)
#   3. DEADBAND_HOLD = 3   (must stay inside for 3×100ms)
#   4. When locked:
#        - command_rpm frozen at locked_command
#        - integral frozen (no windup)
#        - step thread snaps current_rpm directly (no ramp)
#   5. New target → lock cleared, PID re-engages
# =========================================================
# ZIEGLER-NICHOLS TUNING THEORY
# =========================================================
# Step 1: Set Ki=0, Kd=0. Increase Kp until oscillation.
#         That Kp = Ultimate Gain (Ku)
# Step 2: Measure oscillation period = Ultimate Period (Tu)
# Step 3: Apply ZN formulas:
#         Classic PID:
#           Kp = 0.6 × Ku        Ki = 2×Kp/Tu    Kd = Kp×Tu/8
#         No-overshoot:
#           Kp = 0.2 × Ku        Ki = 2×Kp/Tu    Kd = Kp×Tu/3
#
# YOUR SYSTEM VALUES:
#   Ku = 0.40   Tu = 0.20s
#
# CALCULATED GAINS:
#   Classic      : Kp=0.240  Ki=2.400  Kd=0.006
#   Moderate     : Kp=0.132  Ki=1.320  Kd=0.009
#   No-overshoot : Kp=0.080  Ki=0.800  Kd=0.005
# =========================================================
# RUN MODE CONFIG
# =========================================================
#   RUN_MODE = "sweep"   → auto sweep through SWEEP_RPMS
#   RUN_MODE = "manual"  → type RPM values manually
#   RUN_MODE = "random"  → random RPM + random dwell time
# =========================================================

import lgpio
import time
import threading
import random
import matplotlib.pyplot as plt

# ── Run Mode ──────────────────────────────────
RUN_MODE = "random"

# ── Auto Sweep Config ─────────────────────────
SWEEP_RPMS  = [5, 7, 10, 15, 20, 25, 30, 35, 40, 45, 50,
               45, 40, 35, 30, 25, 20, 15, 10, 7, 5]
SWEEP_DWELL = 15.0

# ── Random Mode Config ────────────────────────
RANDOM_MIN_RPM  = 5
RANDOM_MAX_RPM  = 50
RANDOM_MIN_TIME = 5.0
RANDOM_MAX_TIME = 15.0
RANDOM_STEPS    = 15

# ── GPIO Pins ─────────────────────────────────
STEP_PIN = 17
DIR_PIN  = 27
ENA_PIN  = 22
ENC_A    = 24
ENC_B    = 23

# ── Motor Settings ────────────────────────────
COUNTS_PER_REV = 2000
MICROSTEPS     = 3200
MIN_RPM        = 0
MAX_RPM        = 120

# ── ZN Tuning Mode ────────────────────────────
MODE = "no_overshoot"

ZN_GAINS = {
    "classic":      {"Kp": 0.240, "Ki": 2.400, "Kd": 0.006},
    "moderate":     {"Kp": 0.132, "Ki": 1.320, "Kd": 0.009},
    "no_overshoot": {"Kp": 0.080, "Ki": 0.800, "Kd": 0.005},
}

Kp = ZN_GAINS[MODE]["Kp"]
Ki = ZN_GAINS[MODE]["Ki"]
Kd = ZN_GAINS[MODE]["Kd"]

# ── Low-Pass Filter ───────────────────────────
# 0.3 = smooth (was 1.0 = no filter, caused noisy PID input)
FILTER_ALPHA = 0.3

# ── Ramp Limiter ──────────────────────────────
RAMP_UP   = 1.0
RAMP_DOWN = 1.5

# ── Deadband Lock Config ───────────────────────
# When |error| < DEADBAND_RPM for DEADBAND_HOLD consecutive
# PID cycles → freeze command_rpm and integrator.
# Tune DEADBAND_RPM: raise if still hunting, lower if
# steady-state is too far from target.
DEADBAND_RPM  = 0.8   # ± RPM tolerance (try 0.5 – 1.5)
DEADBAND_HOLD = 3     # cycles before locking (3 × 100ms = 0.3s)

# ── Shared State ──────────────────────────────
enc_count    = 0
enc_lock     = threading.Lock()
target_rpm   = 0.0
command_rpm  = 0.0
current_rpm  = 0.0
filtered_rpm = 0.0
integral     = 0.0
last_error   = 0.0
last_count   = 0
running      = True

# ── Deadband Lock State ───────────────────────
freq_locked      = False   # True = steady-state hold active
locked_command   = 0.0     # command_rpm frozen at lock moment
deadband_counter = 0       # counts consecutive in-band cycles

# ── Logging ───────────────────────────────────
rpm_log      = []
filtered_log = []
target_log   = []
time_log     = []
start_time   = time.time()

timestamp      = time.strftime("%Y%m%d_%H%M%S")
log_filename   = f"zn_log_{timestamp}.txt"
graph_filename = f"zn_graph_{timestamp}.png"

log_file = open(log_filename, "w")
log_file.write("Time,TargetRPM,ActualRPM,FilteredRPM,CommandRPM,EncoderCount,Locked\n")

# ── GPIO Init ─────────────────────────────────
h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(h, STEP_PIN, 0)
lgpio.gpio_claim_output(h, DIR_PIN,  1)
lgpio.gpio_claim_output(h, ENA_PIN,  1)
lgpio.gpio_claim_input(h, ENC_A, lgpio.SET_PULL_UP)
lgpio.gpio_claim_input(h, ENC_B, lgpio.SET_PULL_UP)
time.sleep(0.1)

print("GPIO Initialized.")
print(f"ZN Mode      : {MODE}")
print(f"Kp={Kp}  Ki={Ki}  Kd={Kd}")
print(f"Deadband     : ±{DEADBAND_RPM} RPM  (lock after {DEADBAND_HOLD} cycles)")
print(f"Filter alpha : {FILTER_ALPHA}")


# ──────────────────────────────────────────────
# Encoder Thread
# ──────────────────────────────────────────────
def encoder_thread():
    global enc_count
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


# ──────────────────────────────────────────────
# PID Thread — 100ms loop
# ──────────────────────────────────────────────
def pid_thread():
    global command_rpm, integral, last_error
    global last_count, filtered_rpm
    global freq_locked, locked_command, deadband_counter

    last_print = time.time()

    while running:
        t0 = time.time()

        with enc_lock:
            count = enc_count

        delta      = count - last_count
        last_count = count

        actual_rpm   = (abs(delta) / COUNTS_PER_REV) * 600.0
        filtered_rpm = ((1.0 - FILTER_ALPHA) * filtered_rpm
                        + FILTER_ALPHA * actual_rpm)

        error = target_rpm - filtered_rpm

        # ── CASE 1: Motor stopped ──────────────
        if target_rpm == 0:
            command_rpm      = 0.0
            integral         = 0.0
            filtered_rpm     = 0.0
            freq_locked      = False
            locked_command   = 0.0
            deadband_counter = 0
            last_error       = 0.0

        # ── CASE 2: Inside deadband ────────────
        elif abs(error) <= DEADBAND_RPM:
            deadband_counter += 1

            if deadband_counter >= DEADBAND_HOLD:
                if not freq_locked:
                    # First time hitting the lock threshold:
                    # snapshot the current command and freeze it
                    locked_command = command_rpm
                    freq_locked    = True
                    print(
                        f"\n[LOCKED] cmd frozen at {locked_command:.2f} RPM"
                        f"  (actual≈{filtered_rpm:.1f})"
                    )

                # Hold exactly the frozen command — no PID math
                command_rpm = locked_command
                # Do NOT touch integral or last_error

        # ── CASE 3: Outside deadband — run PID ─
        else:
            if freq_locked:
                # Drifted outside band — unlock and let PID correct
                freq_locked      = False
                deadband_counter = 0
                print(f"\n[UNLOCKED] err={error:+.2f} RPM, PID re-engaged")
            else:
                # Decay counter so a brief excursion doesn't reset progress
                deadband_counter = max(0, deadband_counter - 1)

            integral   = max(-80.0, min(80.0, integral + error * 0.1))
            derivative = (error - last_error) / 0.1
            last_error = error

            pid_out     = (Kp * error) + (Ki * integral) + (Kd * derivative)
            command_rpm = max(0.0, min(float(MAX_RPM), target_rpm + pid_out))

        # ── Logging ────────────────────────────
        t = time.time() - start_time
        rpm_log.append(actual_rpm)
        filtered_log.append(filtered_rpm)
        target_log.append(target_rpm)
        time_log.append(t)

        log_file.write(
            f"{t:.2f},{target_rpm:.2f},{actual_rpm:.2f},"
            f"{filtered_rpm:.2f},{command_rpm:.2f},{count},{int(freq_locked)}\n"
        )
        log_file.flush()

        if time.time() - last_print >= 0.5:
            state = "LOCKED" if freq_locked else "PID   "
            print(
                f"\r[{state}] Target:{target_rpm:5.1f} "
                f"| Actual:{actual_rpm:5.1f} "
                f"| Filtered:{filtered_rpm:5.1f} "
                f"| Cmd:{command_rpm:5.1f} "
                f"| Err:{error:+5.2f}    ",
                end=""
            )
            last_print = time.time()

        sleep_t = 0.1 - (time.time() - t0)
        if sleep_t > 0:
            time.sleep(sleep_t)


# ──────────────────────────────────────────────
# Step Pulse Thread
# ──────────────────────────────────────────────
def step_thread():
    global current_rpm
    step_state = 0

    while running:
        if freq_locked and target_rpm > 0:
            # Frequency locked: snap directly — zero ramp jitter
            current_rpm = locked_command
        else:
            # Normal ramp toward command_rpm
            if current_rpm < command_rpm:
                current_rpm = min(current_rpm + RAMP_UP, command_rpm)
            elif current_rpm > command_rpm:
                current_rpm = max(current_rpm - RAMP_DOWN, command_rpm)

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


# ──────────────────────────────────────────────
# Auto Sweep Thread
# ──────────────────────────────────────────────
def sweep_thread():
    global target_rpm, command_rpm, integral
    global running, freq_locked, deadband_counter

    print("\n====================================")
    print(" AUTO SWEEP — ZIEGLER-NICHOLS PID")
    print(f" Mode : {MODE}")
    print(f" Kp={Kp}  Ki={Ki}  Kd={Kd}")
    print(f" Steps : {SWEEP_RPMS}")
    print(f" Dwell : {SWEEP_DWELL}s per step")
    print(f" Total : ~{len(SWEEP_RPMS) * SWEEP_DWELL:.0f}s")
    print("====================================")
    print(" Starting in 3 seconds...")
    print(" Press Ctrl+C to stop at any time")
    print("====================================\n")
    time.sleep(3.0)

    for idx, rpm in enumerate(SWEEP_RPMS):
        if not running:
            break

        target_rpm       = float(rpm)
        freq_locked      = False       # clear lock on new setpoint
        deadband_counter = 0
        print(f"\n[SWEEP {idx+1}/{len(SWEEP_RPMS)}] → {rpm} RPM  "
              f"(holding {SWEEP_DWELL}s)")

        elapsed = 0.0
        while elapsed < SWEEP_DWELL and running:
            time.sleep(0.5)
            elapsed += 0.5

    print("\n[SWEEP] All steps complete. Stopping motor.")
    target_rpm  = 0.0
    command_rpm = 0.0
    integral    = 0.0
    time.sleep(2.0)
    running = False


# ──────────────────────────────────────────────
# Random Mode Thread
# ──────────────────────────────────────────────
def random_thread():
    global target_rpm, command_rpm, integral
    global running, freq_locked, deadband_counter

    print("\n====================================")
    print(" RANDOM MODE — ZIEGLER-NICHOLS PID")
    print(f" Mode      : {MODE}")
    print(f" Kp={Kp}  Ki={Ki}  Kd={Kd}")
    print(f" RPM range : {RANDOM_MIN_RPM} – {RANDOM_MAX_RPM}")
    print(f" Time range: {RANDOM_MIN_TIME}s – {RANDOM_MAX_TIME}s")
    print(f" Steps     : {RANDOM_STEPS}")
    print("====================================")
    print(" Starting in 3 seconds...")
    print(" Press Ctrl+C to stop at any time")
    print("====================================\n")
    time.sleep(3.0)

    for idx in range(RANDOM_STEPS):
        if not running:
            break

        rpm              = random.randint(RANDOM_MIN_RPM, RANDOM_MAX_RPM)
        dwell            = 15
        target_rpm       = float(rpm)
        freq_locked      = False       # clear lock on every new setpoint
        deadband_counter = 0
        print(f"\n[RANDOM {idx+1}/{RANDOM_STEPS}] → {rpm} RPM  "
              f"(holding {dwell}s)")

        elapsed = 0.0
        while elapsed < dwell and running:
            time.sleep(0.5)
            elapsed += 0.5

    print("\n[RANDOM] All steps complete. Stopping motor.")
    target_rpm  = 0.0
    command_rpm = 0.0
    integral    = 0.0
    time.sleep(2.0)
    running = False


# ──────────────────────────────────────────────
# Manual Input Thread
# ──────────────────────────────────────────────
def input_thread():
    global target_rpm, command_rpm, integral
    global running, freq_locked, deadband_counter

    print("\n====================================")
    print(" CLOSED LOOP PID — ZIEGLER-NICHOLS")
    print(f" Mode : {MODE}")
    print(f" Kp={Kp}  Ki={Ki}  Kd={Kd}")
    print(f" Deadband : ±{DEADBAND_RPM} RPM")
    print(f" Max RPM = {MAX_RPM}")
    print("====================================")
    print(" Enter RPM  |  0 = Stop  |  Q = Quit")
    print(" M = show current mode")
    print("====================================\n")

    while running:
        try:
            s = input("RPM> ").strip()

            if s.upper() == "Q":
                running = False
                break

            elif s.upper() == "M":
                print(f"Mode: {MODE}  |  Kp={Kp}  Ki={Ki}  Kd={Kd}")
                print("Restart script to change mode.")

            else:
                val = float(s)
                val = max(0.0, min(float(MAX_RPM), val))
                target_rpm       = val
                freq_locked      = False   # clear lock on new setpoint
                deadband_counter = 0
                if val == 0:
                    command_rpm = 0.0
                    integral    = 0.0
                    print("Motor stopped.")
                else:
                    print(f"Target RPM = {val}")

        except ValueError:
            print("Invalid. Enter a number or Q.")
        except EOFError:
            running = False
            break


# ──────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────
if __name__ == "__main__":

    if RUN_MODE == "sweep":
        control_fn = sweep_thread
    elif RUN_MODE == "random":
        control_fn = random_thread
    else:
        control_fn = input_thread

    try:
        t1 = threading.Thread(target=encoder_thread, daemon=True)
        t2 = threading.Thread(target=pid_thread,     daemon=True)
        t3 = threading.Thread(target=step_thread,    daemon=True)
        t4 = threading.Thread(target=control_fn,     daemon=False)

        t1.start(); t2.start(); t3.start(); t4.start()
        t4.join()

    except KeyboardInterrupt:
        print("\nInterrupted.")

    finally:
        print("\nShutting down...")
        running = False
        time.sleep(0.2)

        lgpio.gpio_write(h, ENA_PIN,  1)
        lgpio.gpio_write(h, STEP_PIN, 0)
        lgpio.gpiochip_close(h)
        log_file.close()

        if len(rpm_log) > 0:
            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))

            ax1.plot(time_log, target_log, "--", label="Target RPM",
                     linewidth=1.5, color="#D85A30")
            ax1.plot(time_log, rpm_log, drawstyle="steps-mid",
                     label="Actual RPM", linewidth=1.5, color="#378ADD")
            ax1.set_ylabel("RPM")
            ax1.set_title(
                f"PID + Deadband Lock [{MODE}]  "
                f"Kp={Kp} Ki={Ki} Kd={Kd}  "
                f"Deadband=±{DEADBAND_RPM} RPM  [{RUN_MODE.upper()} MODE]"
            )
            ax1.legend()
            ax1.grid(True, alpha=0.3)

            errors = [t - f for t, f in zip(target_log, filtered_log)]
            ax2.plot(time_log, errors, drawstyle="steps-mid",
                     color="#E85D24", linewidth=1.0,
                     label="Error (Target − Filtered)")
            ax2.axhline(0,              color="gray",  linewidth=0.8, linestyle="--")
            ax2.axhline( DEADBAND_RPM,  color="green", linewidth=0.8,
                        linestyle=":", label=f"±{DEADBAND_RPM} deadband")
            ax2.axhline(-DEADBAND_RPM,  color="green", linewidth=0.8, linestyle=":")
            ax2.set_xlabel("Time (s)")
            ax2.set_ylabel("Error (RPM)")
            ax2.legend()
            ax2.grid(True, alpha=0.3)

            plt.tight_layout()
            plt.savefig(graph_filename, dpi=150)
            plt.show()

        print(f"\nSaved:")
        print(f"  Log   → {log_filename}")
        print(f"  Graph → {graph_filename}")
        print("Done.")
