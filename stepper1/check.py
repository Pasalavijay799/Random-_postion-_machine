#!/usr/bin/env python3
# Quick test — checks if lgpio callback works for encoder
# RUN: sudo python3 test_encoder_callback.py

import lgpio
import time

ENC_A = 24
ENC_B = 23

count     = 0
callbacks = 0

h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_input(h, ENC_A, lgpio.SET_PULL_UP)
lgpio.gpio_claim_input(h, ENC_B, lgpio.SET_PULL_UP)
time.sleep(0.1)

def on_enc(chip, gpio, level, tick):
    global count, callbacks
    callbacks += 1
    b = lgpio.gpio_read(h, ENC_B)
    if level == b:
        count -= 1
    else:
        count += 1

cb = lgpio.callback(h, ENC_A, lgpio.BOTH_EDGES, on_enc)

print("=== Encoder Callback Test ===")
print("Rotate shaft by hand — watch count change")
print("Ctrl+C to stop\n")

prev = 0
try:
    while True:
        time.sleep(0.2)
        if count != prev:
            print(f"Count={count:6d}  Callbacks={callbacks:6d}  "
                  f"A={lgpio.gpio_read(h,ENC_A)}  B={lgpio.gpio_read(h,ENC_B)}")
            prev = count
        else:
            print(f"No change — Count={count}  "
                  f"A={lgpio.gpio_read(h,ENC_A)}  "
                  f"B={lgpio.gpio_read(h,ENC_B)}        ", end="\r")
except KeyboardInterrupt:
    pass

print(f"\n\nFinal count     = {count}")
print(f"Total callbacks = {callbacks}")
if callbacks == 0:
    print("\n✗ PROBLEM: callback never fired")
    print("  Possible causes:")
    print("  1. Encoder not connected to GPIO24/GPIO23")
    print("  2. Encoder VCC on 5V instead of 3.3V")
    print("  3. lgpio version does not support callbacks")
    print("\n  Try polling test instead:")
    print("  sudo python3 encoder_test.py")
else:
    print(f"\n✓ Callback working — {callbacks} edges detected")

cb.cancel()
lgpio.gpiochip_close(h)
