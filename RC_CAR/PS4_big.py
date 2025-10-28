#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, struct, time
from adafruit_servokit import ServoKit

# PCA9685 setup
kit = ServoKit(channels=16)

# Channel mapping
STEER_CH1 = 1      # steering servo 1
STEER_CH2 = 2      # steering servo 2
THROTTLE_CH = 3    # throttle
B1_CH = 4          # button 0
B2_CH = 5          # button 1
B3_CH = 6          # button 2

# Configure pulse ranges
for ch in [STEER_CH1, STEER_CH2, THROTTLE_CH, B1_CH, B2_CH, B3_CH]:
    kit.servo[ch].set_pulse_width_range(1000, 2000)

def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

def axis_to_us(val, full_range=True, deadzone=2000, steer=False):
    # val is -32767..32767
    if abs(val) < deadzone:
        return 1500
    norm = clamp(val, -32767, 32767) / 32767.0  # -1..1
    if steer:
        return int(1500 + norm * 500)       # steering always full range
    else:
        if full_range:
            return int(1500 + norm * 500)   # 1000..2000
        else:
            return int(1500 + norm * 250)   # 1250..1750

def set_us(ch, us):
    us = clamp(us, 1000, 2000)
    angle = int((us - 1000) * 180 / 1000)
    kit.servo[ch].angle = angle

# Joystick device
JS_DEV = "/dev/input/js0"

# js_event struct
JS_EVENT_FORMAT = "IhBB"
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FORMAT)
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS   = 0x02
JS_EVENT_INIT   = 0x80

# Axes: left stick Y=1, right stick X=3, L2=2, R2=5
axes = {1:0, 2:0, 3:0, 5:0}

# Button toggle states
button_states = {0:1000, 1:1000, 2:1000}

# init neutral
for ch in [STEER_CH1, STEER_CH2, THROTTLE_CH]:
    set_us(ch, 1500)
for ch in [B1_CH, B2_CH, B3_CH]:
    set_us(ch, 1000)

def open_joystick():
    """Wait until joystick is available, then return fd"""
    while True:
        try:
            fd = os.open(JS_DEV, os.O_RDONLY | os.O_NONBLOCK)
            print(f"[INFO] Connected to {JS_DEV}")
            return fd
        except FileNotFoundError:
            print(f"[WAIT] Joystick {JS_DEV} not found, retrying...")
            time.sleep(2)
        except Exception as e:
            print(f"[ERROR] Failed to open joystick: {e}")
            time.sleep(2)

# ==== MAIN LOOP ====
while True:
    fd = open_joystick()
    try:
        while True:
            try:
                data = os.read(fd, JS_EVENT_SIZE)
            except BlockingIOError:
                time.sleep(0.01)
                continue

            if len(data) != JS_EVENT_SIZE:
                continue

            time_ms, value, etype, number = struct.unpack(JS_EVENT_FORMAT, data)
            base_type = etype & ~JS_EVENT_INIT

            if base_type == JS_EVENT_AXIS:
                axes[number] = value

                # check multiplier: L2 or R2 pressed?
                l2_val = axes.get(2, -32767)
                r2_val = axes.get(5, -32767)
                full_range = (l2_val > -10000) or (r2_val > -10000)

                if number == 3:  # right stick X -> steering
                    us = axis_to_us(-axes[3], full_range=True, steer=True)  # הפוך כיוון
                    set_us(STEER_CH1, us)
                    set_us(STEER_CH2, us)

                elif number == 1:  # left stick Y -> throttle
                    us = axis_to_us(-axes[1], full_range=full_range, steer=False)
                    set_us(THROTTLE_CH, us)

            elif base_type == JS_EVENT_BUTTON:
                if value == 1:  # button press
                    if number in button_states:
                        # toggle 1000 <-> 2000
                        button_states[number] = 2000 if button_states[number] == 1000 else 1000
                        if number == 0:
                            set_us(B1_CH, button_states[number])
                        elif number == 1:
                            set_us(B2_CH, button_states[number])
                        elif number == 2:
                            set_us(B3_CH, button_states[number])

            # debug print
            l2_val = axes.get(2, -32767)
            r2_val = axes.get(5, -32767)
            full_range = (l2_val > -10000) or (r2_val > -10000)

            steer_us   = axis_to_us(-axes.get(3, 0), full_range=True, steer=True)
            throttle_us= axis_to_us(-axes.get(1, 0), full_range=full_range, steer=False)
            b1_us = button_states[0]
            b2_us = button_states[1]
            b3_us = button_states[2]

            print("Steer={} Throttle={} L2={} R2={} (full_range={}) B1={} B2={} B3={}".format(
                steer_us, throttle_us, l2_val, r2_val, full_range, b1_us, b2_us, b3_us
            ))

    except OSError:
        print("[WARN] Joystick disconnected, waiting to reconnect...")
        time.sleep(2)
        # חוזר להתחלה ופותח שוב את ה־joystick
