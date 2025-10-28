#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Fake RC Sticks GUI + PS4 + Optional YOLO/ByteTrack
- GUI sliders + PS4 controller
- Attitude row: Roll, Pitch, Yaw (deg) in one line
- Telemetry row: Alt, GroundSpeed, Dist to WP, 3D Speed, DistToHome
- Autopilot altitude control:
    * ALT_HOLD helper (RC pulses)
    * GUIDED: Hold here + +/- 0.1 m steps (no RC pulses)
    * Quick modes: LOITER / POSHOLD / BRAKE
- Flight Modes panel: GUIDED and STABILIZE (quick switch)
- Optional: YOLOv8 + ByteTrack for Yaw tracking of a person (CH4)
All labels/comments are ASCII-only.
"""

import math, time, threading, tkinter as tk
from tkinter import ttk, messagebox
from pymavlink import mavutil
import pygame

# Optional: YOLO / camera
YOLO_OK = True
try:
    import cv2
    from ultralytics import YOLO as _YOLO
except Exception as _e:
    YOLO_OK = False
    _YOLO = None
    cv2 = None
    print("Warning: YOLO/Camera unavailable:", _e)

# Optional: ByteTrack & deps
BT_OK = True
try:
    import numpy as np
    # Compat aliases for NumPy 2.x when older libs expect these
    if not hasattr(np, "float"): np.float = float
    if not hasattr(np, "int"):   np.int   = int
    if not hasattr(np, "bool"):  np.bool  = bool
    import torch
    from types import SimpleNamespace
    from yolox.tracker.byte_tracker import BYTETracker
except Exception as _e:
    BT_OK = False
    print("Warning: ByteTrack unavailable:", _e)

# Config
DEVICE = "COM40"  # e.g. "COM14" (Windows) or "udp:127.0.0.1:14550" (SITL)
BAUD   = 115200

RC_MIN, RC_MAX, RC_MID = 1000, 2000, 1500
SEND_HZ   = 20.0
DEADZONE  = 0.10

# YOLO + ByteTrack tuning
YOLO_DB_PIX      = 40
YOLO_YAW_GAIN    = 0.6
YOLO_THR_MIN     = 1150
YOLO_SHOW_WINDOW = True
YOLO_CAM_INDEX   = 0
YOLO_MODEL_NAME  = "yolov8n.pt"
YOLO_PERSON_CLS  = 0
YOLO_CONF        = 0.20
YOLO_IOU         = 0.50
DRAW_CONF_MIN    = 0.80
FORCE_FALLBACK_TO_DET = True

# ByteTrack
BT_TRACK_THRESH  = 0.15
BT_MATCH_THRESH  = 0.80
BT_TRACK_BUFFER  = 30

# Altitude handling
ALT_SOURCE_HOLD_S = 1.5

# ALT_HOLD helper (RC pulses)
ALT_BUMP_DEFAULT = 0.1
ALT_BUMP_FRAC = 0.30
ALT_BUMP_MIN_T = 0.08
ALT_BUMP_MAX_T = 1.00
FALLBACK_SPEED_UP = 1.5
FALLBACK_SPEED_DN = 1.0

# Baro Alt-Hold controller
ALT_CTRL_TOL_M        = 0.05
ALT_CTRL_KP_US_PER_M  = 1200.0
ALT_CTRL_MAX_US       = 300
ALT_CTRL_DT           = 0.05
ALT_CTRL_PRINT_PERIOD = 0.20

# GUIDED keepalive
GUIDED_KEEPALIVE_HZ = 5.0


def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v
def apply_deadzone(val, dz=DEADZONE): return 0.0 if abs(val) < dz else val

def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    a1, b1 = math.radians(lat1), math.radians(lon1)
    a2, b2 = math.radians(lat2), math.radians(lon2)
    da, db = a2-a1, b2-b1
    h = math.sin(da/2)**2 + math.cos(a1)*math.cos(a2)*math.sin(db/2)**2
    return 2*R*math.asin(math.sqrt(h))

def _param_id_to_str(pid):
    if isinstance(pid, (bytes, bytearray)):
        try:
            return pid.decode('ascii', errors='ignore').rstrip('\x00')
        except Exception:
            return str(pid).rstrip('\x00')
    return str(pid).rstrip('\x00')

def _id_color(track_id: int):
    r = (37 * track_id) % 255
    g = (17 * track_id) % 255
    b = (29 * track_id) % 255
    r = 60 if r < 60 else r
    g = 60 if g < 60 else g
    b = 60 if b < 60 else b
    return int(b), int(g), int(r)  # OpenCV BGR

def _rad2deg_wrap(rad: float) -> float:
    deg = math.degrees(rad)
    while deg > 180.0:  deg -= 360.0
    while deg <= -180.0: deg += 360.0
    return deg


class FakeSticks:
    def __init__(self, root):
        self.root = root
        self.root.title(f"Fake RC Sticks + PS4 ({DEVICE} @{BAUD})")

        # MAVLink connect
        try:
            self.m = mavutil.mavlink_connection(DEVICE, baud=BAUD)
            self.m.wait_heartbeat(timeout=5)
            ttk.Label(root, text=f"Connected: sys {self.m.target_system}, comp {self.m.target_component}").pack()
            try:
                self.m.mav.command_long_send(
                    self.m.target_system, self.m.target_component,
                    mavutil.mavlink.MAV_CMD_GET_HOME_POSITION, 0, 0,0,0,0,0,0,0
                )
            except Exception:
                pass
            self._set_msg_rate(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,            10)
            self._set_msg_rate(mavutil.mavlink.MAVLINK_MSG_ID_ALTITUDE,            10)
            self._set_msg_rate(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10)
            self._set_msg_rate(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,              5)
            self._set_msg_rate(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,           1)
        except Exception as e:
            messagebox.showerror("MAVLink", f"Connect failed: {e}")
            root.destroy(); return

        # PS4 joystick
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            self.js = pygame.joystick.Joystick(0); self.js.init()
            print(f"[PS4] Connected: {self.js.get_name()}")
        else:
            self.js = None; print("[PS4] No joystick detected")

        # RC channels
        self.roll  = tk.IntVar(value=RC_MID)
        self.pitch = tk.IntVar(value=RC_MID)
        self.thr   = tk.IntVar(value=RC_MIN)
        self.yaw   = tk.IntVar(value=RC_MID)

        # Servos
        self.servo5 = tk.IntVar(value=1500)
        self.servo7 = tk.IntVar(value=1500)

        # Battery
        self.batt_v   = tk.StringVar(value="--.- V")
        self.batt_a   = tk.StringVar(value="--.- A")
        self.batt_pct = tk.StringVar(value="-- %")

        # Core telemetry
        self.t_alt      = tk.StringVar(value="--")
        self.t_gspeed   = tk.StringVar(value="--")
        self.t_wpdist   = tk.StringVar(value="--")
        self.t_disthome = tk.StringVar(value="--")
        self.t_speed3d  = tk.StringVar(value="--")

        # Attitude (deg)
        self.t_roll  = tk.StringVar(value="--")
        self.t_pitch = tk.StringVar(value="--")
        self.t_yaw   = tk.StringVar(value="--")
        self._att_ts = 0.0

        # Position / altitude buffers
        self._cur_latlon  = None
        self._home_latlon = None

        self._alt_src  = None
        self._alt_ts   = 0.0
        self._alt_val  = None
        self._priority = {'VFR':1, 'GPI':2, 'ALTITUDE':3}

        # Alt-Hold controller state
        self.alt_hold_enabled = False
        self.alt_hold_thread  = None
        self.alt_tgt = tk.DoubleVar(value=0.0)  # reused by GUIDED
        self.alt_err = tk.DoubleVar(value=0.0)

        # GUIDED keepalive state
        self._guided_active = False
        self._guided_target = None  # (lat, lon, alt_rel)
        self._guided_lock   = threading.Lock()

        # ByteTrack debug throttling
        self._id_last_print = {}
        self._print_cooldown = 1.0

        # GUI
        lf = ttk.LabelFrame(root, text="RC Sticks", padding=8); lf.pack(fill="x", padx=8, pady=6)
        self._mk_slider(lf, "Roll (CH1)",     self.roll,  RC_MID)
        self._mk_slider(lf, "Pitch (CH2)",    self.pitch, RC_MID)
        self._mk_slider(lf, "Throttle (CH3)", self.thr,   RC_MIN)
        self._mk_slider(lf, "Yaw (CH4)",      self.yaw,   RC_MID)

        ls = ttk.LabelFrame(root, text="Servos (CH5 & CH7)", padding=8); ls.pack(fill="x", padx=8, pady=6)
        self._mk_slider(ls,"Servo CH5", self.servo5, 1500)
        self._mk_slider(ls,"Servo CH7", self.servo7, 1500)

        lb = ttk.LabelFrame(root, text="Battery", padding=8); lb.pack(fill="x", padx=8, pady=6)
        ttk.Label(lb, text="Voltage:").grid(row=0,column=0,sticky="w"); ttk.Label(lb, textvariable=self.batt_v).grid(row=0,column=1,sticky="w")
        ttk.Label(lb, text="Current:").grid(row=1,column=0,sticky="w"); ttk.Label(lb, textvariable=self.batt_a).grid(row=1,column=1,sticky="w")
        ttk.Label(lb, text="Remaining:").grid(row=2,column=0,sticky="w"); ttk.Label(lb, textvariable=self.batt_pct).grid(row=2,column=1,sticky="w")

        att = ttk.LabelFrame(root, text="Attitude (deg)", padding=8); att.pack(fill="x", padx=8, pady=6)
        self._mk_metric(att, "Roll (deg)",  self.t_roll)
        self._mk_metric(att, "Pitch (deg)", self.t_pitch)
        self._mk_metric(att, "Yaw (deg)",   self.t_yaw)

        tele = ttk.LabelFrame(root, text="Telemetry", padding=8); tele.pack(fill="x", padx=8, pady=6)
        self._mk_metric(tele, "Altitude (m)",      self.t_alt)
        self._mk_metric(tele, "GroundSpeed (m/s)", self.t_gspeed)
        self._mk_metric(tele, "Dist to WP (m)",    self.t_wpdist)
        self._mk_metric(tele, "3D Speed (m/s)",    self.t_speed3d)
        self._mk_metric(tele, "DistToHome (m)",    self.t_disthome)

        ly = ttk.LabelFrame(root, text="YOLO + ByteTrack (Yaw tracking)", padding=8); ly.pack(fill="x", padx=8, pady=6)
        self.yolo_enabled = False
        ready_txt = "YOLO: ready"
        if not YOLO_OK: ready_txt = "YOLO: unavailable"
        if not BT_OK:   ready_txt += " (ByteTrack unavailable)"
        self.yolo_status = tk.StringVar(value=ready_txt)
        ttk.Label(ly, textvariable=self.yolo_status).pack(side="left", padx=4)
        ttk.Button(ly, text="Start", command=self.start_yolo).pack(side="left", padx=4)
        ttk.Button(ly, text="Stop",  command=self.stop_yolo).pack(side="left", padx=4)

        lm = ttk.LabelFrame(root, text="ALT_HOLD (RC helper)", padding=8); lm.pack(fill="x", padx=8, pady=6)
        ttk.Button(lm, text="ALT_HOLD: Hover", command=self.hold_alt).pack(side="left", expand=True, fill="x", padx=4)
        ttk.Button(lm, text="+0.1 m (pulse)", command=lambda: self.bump_alt(+ALT_BUMP_DEFAULT)).pack(side="left", expand=True, fill="x", padx=4)
        ttk.Button(lm, text="-0.1 m (pulse)", command=lambda: self.bump_alt(-ALT_BUMP_DEFAULT)).pack(side="left", expand=True, fill="x", padx=4)

        lc = ttk.LabelFrame(root, text="Altitude Hold (Controller, baro)", padding=8); lc.pack(fill="x", padx=8, pady=6)
        ttk.Button(lc, text="Start CTRL", command=self.start_alt_hold_ctrl).pack(side="left", expand=True, fill="x", padx=4)
        ttk.Button(lc, text="+0.1 m", command=lambda: self.bump_target(+0.1)).pack(side="left", expand=True, fill="x", padx=4)
        ttk.Button(lc, text="-0.1 m", command=lambda: self.bump_target(-0.1)).pack(side="left", expand=True, fill="x", padx=4)
        ttk.Button(lc, text="Stop CTRL", command=self.stop_alt_hold_ctrl).pack(side="left", expand=True, fill="x", padx=4)
        row = ttk.Frame(lc, padding=6); row.pack(fill="x", padx=4)
        ttk.Label(row, text="Target (m):").pack(side="left")
        ttk.Label(row, textvariable=self.alt_tgt, width=8).pack(side="left", padx=(0,12))
        ttk.Label(row, text="Error (m):").pack(side="left")
        ttk.Label(row, textvariable=self.alt_err, width=8).pack(side="left")

        lg = ttk.LabelFrame(root, text="Autopilot Altitude (no RC pulses)", padding=8)
        lg.pack(fill="x", padx=8, pady=6)
        ttk.Button(lg, text="GUIDED: Hold here", command=self.hold_alt_guided_here).pack(side="left", expand=True, fill="x", padx=4)
        ttk.Button(lg, text="+0.1 m (GUIDED)", command=lambda: self.bump_alt_guided(+0.1)).pack(side="left", expand=True, fill="x", padx=4)
        ttk.Button(lg, text="-0.1 m (GUIDED)", command=lambda: self.bump_alt_guided(-0.1)).pack(side="left", expand=True, fill="x", padx=4)
        ttk.Button(lg, text="Stop GUIDED", command=self.stop_guided_hold).pack(side="left", padx=4)
        ttk.Button(lg, text="LOITER",   command=lambda: self.set_mode("LOITER")).pack(side="left", padx=4)
        ttk.Button(lg, text="POSHOLD",  command=lambda: self.set_mode("POSHOLD")).pack(side="left", padx=4)
        ttk.Button(lg, text="BRAKE",    command=lambda: self.set_mode("BRAKE")).pack(side="left", padx=4)

        # NEW: Flight modes (quick switch)
        mf = ttk.LabelFrame(root, text="Flight Modes", padding=8)
        mf.pack(fill="x", padx=8, pady=6)
        ttk.Button(mf, text="GUIDED",
                   command=lambda: self.set_mode("GUIDED")).pack(
                       side="left", expand=True, fill="x", padx=4)
        ttk.Button(mf, text="STABILIZE",
                   command=lambda: self.set_mode("STABILIZE")).pack(
                       side="left", expand=True, fill="x", padx=4)

        btns = ttk.Frame(root, padding=8); btns.pack(fill="x")
        ttk.Button(btns, text="Force ARM", command=self.force_arm).pack(side="left", expand=True, fill="x", padx=4)
        ttk.Button(btns, text="DISARM",    command=self.disarm).pack(side="left", expand=True, fill="x", padx=4)
        ttk.Button(btns, text="Reset All", command=self.reset_all).pack(side="left", expand=True, fill="x", padx=4)
        ttk.Button(btns, text="Exit",      command=self.on_close).pack(side="left", expand=True, fill="x", padx=4)

        # Threads
        self.running = True
        threading.Thread(target=self._send_loop,             daemon=True).start()
        threading.Thread(target=self._mav_telemetry,         daemon=True).start()
        threading.Thread(target=self._guided_keepalive_loop, daemon=True).start()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        try:
            self.root.state('zoomed')
        except Exception:
            self.root.geometry("1200x900")

    # GUI helpers
    def _mk_slider(self, parent, label, var, reset_val):
        row = ttk.Frame(parent); row.pack(fill="x", pady=4)
        ttk.Label(row, text=label, width=14).pack(side="left")
        s = ttk.Scale(row, from_=RC_MIN, to=RC_MAX, orient="horizontal", variable=var)
        s.pack(side="left", fill="x", expand=True, padx=8)
        ttk.Label(row, textvariable=var, width=5).pack(side="left")
        ttk.Button(row, text="Reset", command=lambda v=var, rv=reset_val: v.set(rv)).pack(side="left", padx=4)

    def _mk_metric(self, parent, title, var, font_size=32):
        card = ttk.Frame(parent, padding=8)
        card.pack(side="left", fill="both", expand=True, padx=6, pady=6)
        ttk.Label(card, text=title, anchor="center").pack(fill="x")
        lbl = ttk.Label(card, textvariable=var, anchor="center")
        lbl.pack(fill="both", expand=True)
        lbl.configure(font=("Segoe UI", font_size, "bold"))

    # MAVLink helpers
    def _set_msg_rate(self, msgid, hz):
        try:
            interval_us = int(1e6 / float(hz)) if hz > 0 else 0
            self.m.mav.command_long_send(
                self.m.target_system, self.m.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0, msgid, interval_us, 0,0,0,0,0
            )
        except Exception as e:
            print(f"[MSG_RATE] {msgid} @ {hz}Hz failed:", e)

    def _send_override(self):
        try:
            self.m.mav.rc_channels_override_send(
                self.m.target_system, self.m.target_component,
                int(self.roll.get()),   # CH1
                int(self.pitch.get()),  # CH2
                int(self.thr.get()),    # CH3
                int(self.yaw.get()),    # CH4
                int(self.servo5.get()), # CH5
                65535,                  # CH6
                int(self.servo7.get()), # CH7
                65535                   # CH8
            )
        except Exception as e:
            print("[RC_OVERRIDE] error:", e)

    def set_mode(self, mode_name):
        """Try to set mode by name; stops GUIDED keepalive if moving away from GUIDED."""
        try:
            mode_id = self.m.mode_mapping()[mode_name]
            self.m.set_mode(mode_id)
            print(f"[MODE] {mode_name}")
            if mode_name != "GUIDED":
                self.stop_guided_hold()
        except Exception as e:
            print(f"[MODE] Error setting {mode_name}:", e)

    def get_param(self, name, default=None, timeout=2.0):
        try:
            target = str(name).strip()
            self.m.mav.param_request_read_send(
                self.m.target_system, self.m.target_component,
                target.encode('ascii', 'ignore'), -1
            )
            t0 = time.time()
            while time.time() - t0 < timeout:
                p = self.m.recv_match(type="PARAM_VALUE", blocking=False)
                if not p:
                    time.sleep(0.02); continue
                pid = _param_id_to_str(getattr(p, "param_id", ""))
                if pid == target:
                    try:
                        return float(p.param_value)
                    except Exception:
                        return p.param_value
            print(f"[PARAM] timeout reading {target}")
        except Exception as e:
            print(f"[PARAM] read {name} failed:", e)
        return default

    def _hover_rc(self):
        hover = self.get_param("MOT_THST_HOVER", default=0.5)
        hover = clamp(hover if hover is not None else 0.5, 0.0, 1.0)
        rc3_hover = int(RC_MIN + hover * (RC_MAX - RC_MIN))
        return rc3_hover, hover

    # ALT_HOLD helper (RC pulses)
    def hold_alt(self):
        self.set_mode("ALT_HOLD")
        rc3_hover, hover = self._hover_rc()
        steps, cur = 10, self.thr.get()
        for i in range(1, steps+1):
            self.thr.set(int(cur + (rc3_hover - cur) * i/steps))
            self._send_override(); time.sleep(0.03)
        print(f"[ALT_HOLD] Hover={hover:.2f} -> RC3={rc3_hover}")

    def bump_alt(self, delta_m):
        up = self.get_param("PILOT_SPEED_UP", default=None)
        dn = self.get_param("PILOT_SPEED_DN", default=None)
        def _as_mps(val, fb): return (val/100.0 if (val and val>20) else (val if val else fb))
        v_up, v_dn = _as_mps(up, FALLBACK_SPEED_UP), _as_mps(dn, FALLBACK_SPEED_DN)
        rc3_hover, _ = self._hover_rc()
        sign = 1 if delta_m >= 0 else -1
        rate = v_up if sign>0 else v_dn
        frac = ALT_BUMP_FRAC
        dur = clamp(abs(delta_m) / max(rate*frac, 1e-3), ALT_BUMP_MIN_T, ALT_BUMP_MAX_T)
        rc_pulse = clamp(rc3_hover + sign*int(500*frac), RC_MIN, RC_MAX)
        def _do():
            self.set_mode("ALT_HOLD")
            self.thr.set(rc_pulse)
            t_end = time.time()+dur
            while time.time()<t_end and self.running:
                self._send_override(); time.sleep(0.03)
            self.thr.set(rc3_hover); self._send_override()
            print(f"[ALT_BUMP] dh={delta_m:+.2f}m dur={dur:.2f}s rc3={rc_pulse}")
        threading.Thread(target=_do, daemon=True).start()

    # Baro Alt-Hold controller
    def start_alt_hold_ctrl(self):
        self.set_mode("ALT_HOLD")
        rc3_hover, _ = self._hover_rc()
        self.thr.set(rc3_hover)
        tgt = self._alt_val if self._alt_val is not None else 0.0
        self.alt_tgt.set(round(float(tgt), 2))
        if self.alt_hold_enabled:
            print("[ALT-CTRL] already running"); return
        self.alt_hold_enabled = True
        self.alt_hold_thread = threading.Thread(target=self._alt_hold_loop, daemon=True)
        self.alt_hold_thread.start()
        print(f"[ALT-CTRL] started, target={self.alt_tgt.get():.2f} m")

    def stop_alt_hold_ctrl(self):
        if not self.alt_hold_enabled: return
        self.alt_hold_enabled = False
        rc3_hover, _ = self._hover_rc()
        self.thr.set(rc3_hover)
        print("[ALT-CTRL] stopped")

    def bump_target(self, delta_m: float):
        if not self.alt_hold_enabled:
            self.start_alt_hold_ctrl()
        self.alt_tgt.set(round(self.alt_tgt.get() + float(delta_m), 2))
        print(f"[ALT-CTRL] target -> {self.alt_tgt.get():.2f} m")

    def _alt_hold_loop(self):
        rc3_hover, _ = self._hover_rc()
        last_print = 0.0
        while self.running and self.alt_hold_enabled:
            alt = self._alt_val
            if alt is None:
                time.sleep(ALT_CTRL_DT); continue
            tgt = float(self.alt_tgt.get())
            err = tgt - float(alt)
            self.alt_err.set(round(err, 3))
            if abs(err) <= ALT_CTRL_TOL_M:
                rc3 = rc3_hover
            else:
                delta_us = int(err * ALT_CTRL_KP_US_PER_M)
                delta_us = clamp(delta_us, -ALT_CTRL_MAX_US, ALT_CTRL_MAX_US)
                rc3 = clamp(rc3_hover + delta_us, RC_MIN, RC_MAX)
            self.thr.set(int(rc3))
            now = time.time()
            if now - last_print >= ALT_CTRL_PRINT_PERIOD:
                print(f"[ALT-CTRL] alt={float(alt):.2f} m  tgt={tgt:.2f} m  err={err:.2f} m  rc3={rc3}")
                last_print = now
            time.sleep(ALT_CTRL_DT)

    # PS4 / RC TX loop
    def _send_loop(self):
        per = 1.0 / SEND_HZ
        while self.running:
            if self.js:
                try:
                    pygame.event.pump()
                    axis_roll  = apply_deadzone(self.js.get_axis(0))
                    axis_pitch = apply_deadzone(-self.js.get_axis(1))
                    axis_yaw   = apply_deadzone(self.js.get_axis(2))
                    axis_thr   = -self.js.get_axis(3)
                    r = int(RC_MID + axis_roll  * 500)
                    p = int(RC_MID + axis_pitch * 500)
                    y = int(RC_MID + axis_yaw   * 500)
                    t = RC_MIN if axis_thr <= 0 else int(RC_MIN + axis_thr * (RC_MAX - RC_MIN))
                    if not self.alt_hold_enabled:
                        self.thr.set(clamp(t, RC_MIN, RC_MAX))
                    if not self.yolo_enabled:
                        self.yaw.set(clamp(y, RC_MIN, RC_MAX))
                    self.roll.set(clamp(r, RC_MIN, RC_MAX))
                    self.pitch.set(clamp(p, RC_MIN, RC_MAX))
                    if self.js.get_button(0): self.force_arm()
                    if self.js.get_button(1): self.disarm()
                except Exception as e:
                    print("[PS4] joystick error:", e); self.js = None
            self._send_override()
            time.sleep(per)

    # Telemetry RX
    def _consider_alt(self, value, src):
        if value is None: return
        try:
            val = float(value)
        except Exception:
            return
        now = time.time()
        cur_pri = self._priority.get(self._alt_src or '', 0)
        new_pri = self._priority.get(src, 0)
        if new_pri < cur_pri and (now - self._alt_ts) < ALT_SOURCE_HOLD_S:
            return
        self._alt_src = src; self._alt_ts = now; self._alt_val = val
        self.t_alt.set(f"{val:.2f}")

    def _mav_telemetry(self):
        while self.running:
            try:
                msg = self.m.recv_match(blocking=False)
            except Exception:
                time.sleep(0.01); continue
            if not msg:
                time.sleep(0.005); continue

            t = msg.get_type()

            if t == "SYS_STATUS":
                voltage = msg.voltage_battery / 1000.0
                current = msg.current_battery / 100.0
                remaining = msg.battery_remaining
                self.batt_v.set(f"{voltage:.1f} V")
                if current > -9000: self.batt_a.set(f"{current:.1f} A")
                if remaining >= 0:  self.batt_pct.set(f"{remaining} %")

            elif t == "BATTERY_STATUS":
                if len(msg.voltages) > 0 and msg.voltages[0] > 0:
                    self.batt_v.set(f"{msg.voltages[0]/1000.0:.1f} V")

            elif t == "ATTITUDE":
                try:
                    roll_deg  = _rad2deg_wrap(float(msg.roll))
                    pitch_deg = _rad2deg_wrap(float(msg.pitch))
                    yaw_deg   = _rad2deg_wrap(float(msg.yaw))
                    self.t_roll.set(f"{roll_deg:+.1f}")
                    self.t_pitch.set(f"{pitch_deg:+.1f}")
                    self.t_yaw.set(f"{yaw_deg:+.1f}")
                    self._att_ts = time.time()
                except Exception:
                    pass

            if t == "ALTITUDE":
                self._consider_alt(getattr(msg, "altitude_relative", None), "ALTITUDE")

            if t == "GLOBAL_POSITION_INT":
                try:
                    vx, vy, vz = float(msg.vx)/100.0, float(msg.vy)/100.0, float(msg.vz)/100.0
                    self.t_speed3d.set(f"{(vx*vx+vy*vy+vz*vz)**0.5:.2f}")
                except Exception:
                    pass
                try:
                    self._consider_alt(float(msg.relative_alt)/1000.0, "GPI")
                except Exception:
                    pass
                try:
                    self._cur_latlon = (msg.lat/1e7, msg.lon/1e7)
                    if self._home_latlon and all(self._cur_latlon):
                        d = haversine_m(self._home_latlon[0], self._home_latlon[1],
                                        self._cur_latlon[0],  self._cur_latlon[1])
                        self.t_disthome.set(f"{d:.2f}")
                    else:
                        self.t_disthome.set("--")
                except Exception:
                    self.t_disthome.set("--")

            if t == "VFR_HUD":
                try: self.t_gspeed.set(f"{float(msg.groundspeed):.2f}")
                except Exception: pass
                try:
                    if (time.time() - self._att_ts) > 1.0:
                        self.t_yaw.set(f"{float(msg.heading):.1f}")
                except Exception:
                    pass
                try:
                    if (time.time() - self._alt_ts) > ALT_SOURCE_HOLD_S:
                        self._consider_alt(float(msg.alt), "VFR")
                except Exception:
                    pass

            if t == "HOME_POSITION":
                try:
                    self._home_latlon = (msg.latitude/1e7, msg.longitude/1e7)
                except Exception:
                    self._home_latlon = None

            if t == "NAV_CONTROLLER_OUTPUT":
                try:
                    dwp = float(msg.wp_dist)
                    self.t_wpdist.set(f"{dwp:.2f}" if dwp > 0 else "--")
                except Exception:
                    self.t_wpdist.set("--")

    # YOLO + ByteTrack (optional)
    def start_yolo(self):
        if not YOLO_OK:
            messagebox.showwarning("YOLO", "YOLO/Camera not available"); return
        if not BT_OK:
            messagebox.showwarning("ByteTrack", "ByteTrack not available"); return
        if self.yolo_enabled: return
        self.yolo_enabled = True
        self.yolo_status.set("YOLO: running (ByteTrack)")
        threading.Thread(target=self._yolo_loop, daemon=True).start()

    def stop_yolo(self):
        if not self.yolo_enabled: return
        self.yolo_enabled = False
        self.yolo_status.set("YOLO: stopped")
        self.yaw.set(RC_MID)

    def _yolo_loop(self):
        try:
            model = _YOLO(YOLO_MODEL_NAME)
        except Exception as e:
            print("YOLO load failed:", e); self.yolo_status.set("YOLO: load failed"); self.yolo_enabled=False; return

        cap = cv2.VideoCapture(YOLO_CAM_INDEX)
        if not cap.isOpened():
            print("No camera found"); self.yolo_status.set("YOLO: no camera"); self.yolo_enabled=False; return

        fps = cap.get(cv2.CAP_PROP_FPS)
        fps = float(fps) if fps and fps > 0 else 30.0
        bt_args = SimpleNamespace(track_thresh=BT_TRACK_THRESH, match_thresh=BT_MATCH_THRESH,
                                  track_buffer=BT_TRACK_BUFFER, frame_rate=fps, mot20=False)
        tracker = BYTETracker(bt_args)

        print("[YOLO+BT] tracking person by Yaw (conf >= 0.80). ESC/Q closes preview.")
        while self.running and self.yolo_enabled:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.01); continue

            h, w = frame.shape[:2]
            cx_ref = w * 0.5
            rc4_to_send = None

            try:
                results = model(frame, conf=YOLO_CONF, iou=YOLO_IOU, verbose=False)
                det_list = []
                for r in results:
                    if not hasattr(r, "boxes"): continue
                    for b in r.boxes:
                        cls = int(b.cls[0].item())
                        if cls != YOLO_PERSON_CLS: continue
                        x1, y1, x2, y2 = map(float, b.xyxy[0])
                        conf = float(b.conf[0].item()) if hasattr(b, "conf") else 0.0
                        det_list.append([x1, y1, x2, y2, conf, cls])

                dets = torch.tensor(det_list, dtype=torch.float32) if det_list else torch.empty((0,6), dtype=torch.float32)
                tracks = tracker.update(dets, (h, w), (h, w))

                annotated = frame.copy()
                if YOLO_SHOW_WINDOW:
                    cv2.line(annotated, (int(cx_ref), 0), (int(cx_ref), h), (255, 0, 0), 2)

                strong_tracks = []
                for t in tracks:
                    conf_t = float(getattr(t, "score", 0.0))
                    if conf_t >= DRAW_CONF_MIN:
                        strong_tracks.append((t, conf_t))
                        x1, y1, x2, y2 = t.tlbr.astype(int).tolist()
                        tid = int(t.track_id)
                        color = _id_color(tid)
                        cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                        cv2.putText(annotated, f"ID {tid} {conf_t:.2f}", (x1, max(0, y1-10)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                        now = time.time()
                        if now - self._id_last_print.get(tid, 0) >= self._print_cooldown:
                            print(f"[TRACK] ID={tid} conf={conf_t:.2f}")
                            self._id_last_print[tid] = now

                target_dx, src = None, None
                if strong_tracks:
                    best_abs, pick_t = 1e9, None
                    for t, conf_t in strong_tracks:
                        x1, y1, x2, y2 = t.tlbr
                        cx = 0.5 * (x1 + x2)
                        dx = cx - cx_ref
                        if abs(dx) < best_abs:
                            best_abs = abs(dx); target_dx = dx; src = "track"; pick_t = t
                    if YOLO_SHOW_WINDOW and pick_t is not None:
                        x1, y1, x2, y2 = pick_t.tlbr.astype(int).tolist()
                        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 128, 255), 2)
                elif FORCE_FALLBACK_TO_DET:
                    cand = [d for d in det_list if d[4] >= DRAW_CONF_MIN]
                    if cand:
                        det = max(cand, key=lambda d: d[4])
                        x1, y1, x2, y2, conf, cls = det
                        cx = 0.5 * (x1 + x2)
                        target_dx = cx - cx_ref
                        src = "det"
                        if YOLO_SHOW_WINDOW:
                            cv2.rectangle(annotated, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 255), 2)
                            cv2.putText(annotated, f"det {conf:.2f}", (int(x1), max(0, int(y1)-10)),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                if target_dx is not None:
                    if abs(target_dx) > YOLO_DB_PIX:
                        norm = clamp(target_dx / (w * 0.5), -1.0, 1.0)
                        yaw_cmd = clamp(YOLO_YAW_GAIN * norm, -1.0, 1.0)
                        rc4_to_send = int(RC_MID + yaw_cmd * 500)
                    else:
                        rc4_to_send = RC_MID
                    if self.thr.get() < YOLO_THR_MIN and not self.alt_hold_enabled:
                        self.thr.set(YOLO_THR_MIN)

                if rc4_to_send is not None:
                    self.yaw.set(clamp(rc4_to_send, RC_MIN, RC_MAX))
                    if YOLO_SHOW_WINDOW:
                        cv2.putText(annotated, f"CH4={rc4_to_send} src={src}",
                                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,200,255), 2)

                if YOLO_SHOW_WINDOW:
                    cv2.imshow("YOLO + ByteTrack (Person Yaw Tracking)", annotated)

            except Exception as e:
                print("[YOLO+BT] inference/tracking error:", e)

            if YOLO_SHOW_WINDOW:
                k = cv2.waitKey(1) & 0xFF
                if k in (27, ord('q')):
                    self.stop_yolo()
                    break

        try:
            cap.release()
            if YOLO_SHOW_WINDOW: cv2.destroyAllWindows()
        except Exception:
            pass
        print("[YOLO+BT] stopped")

    # GUIDED setpoint control (no RC pulses)
    def _time_boot_ms(self) -> int:
        return int((time.time() % 1e6) * 1000)

    def _send_guided_position(self, lat, lon, alt_rel_m):
        try:
            type_mask = (
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
            )
            self.m.mav.set_position_target_global_int_send(
                self._time_boot_ms(),
                self.m.target_system, self.m.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                type_mask,
                int(lat * 1e7), int(lon * 1e7), float(alt_rel_m),
                0,0,0, 0,0,0, 0.0, 0.0
            )
        except Exception as e:
            print("[GUIDED] set_position_target_global_int error:", e)

    def _enable_guided_keepalive(self, lat, lon, alt_rel_m):
        with self._guided_lock:
            self._guided_target = (lat, lon, float(alt_rel_m))
            self._guided_active = True

    def stop_guided_hold(self):
        with self._guided_lock:
            self._guided_active = False
            self._guided_target = None
        print("[GUIDED] keepalive stopped")

    def _guided_keepalive_loop(self):
        period = 1.0 / GUIDED_KEEPALIVE_HZ
        while self.running:
            tgt = None
            with self._guided_lock:
                if self._guided_active and self._guided_target:
                    tgt = self._guided_target
            if tgt:
                lat, lon, alt = tgt
                self._send_guided_position(lat, lon, alt)
            time.sleep(period)

    def hold_alt_guided_here(self):
        if not self._cur_latlon:
            messagebox.showwarning("GUIDED", "No GPS fix or location unknown."); return
        if self._alt_val is None:
            messagebox.showwarning("GUIDED", "Current altitude unknown."); return
        self.set_mode("GUIDED")
        lat, lon = self._cur_latlon
        alt = float(self._alt_val)
        self.alt_tgt.set(round(alt, 2))
        for _ in range(5):
            self._send_guided_position(lat, lon, alt); time.sleep(0.05)
        self._enable_guided_keepalive(lat, lon, alt)
        print(f"[GUIDED] Holding here at {alt:.2f} m AGL (keepalive)")

    def bump_alt_guided(self, delta_m: float):
        if not self._cur_latlon:
            messagebox.showwarning("GUIDED", "No GPS fix or location unknown."); return
        self.set_mode("GUIDED")
        if self.alt_tgt.get() == 0.0 and self._alt_val is not None:
            self.alt_tgt.set(round(float(self._alt_val), 2))
        new_tgt = round(float(self.alt_tgt.get()) + float(delta_m), 2)
        self.alt_tgt.set(new_tgt)
        lat, lon = self._cur_latlon
        for _ in range(5):
            self._send_guided_position(lat, lon, new_tgt); time.sleep(0.05)
        self._enable_guided_keepalive(lat, lon, new_tgt)
        print(f"[GUIDED] Target altitude -> {new_tgt:.2f} m (relative, keepalive)")

    # ARM/DISARM/Reset/Close
    def force_arm(self):
        self.m.mav.command_long_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0,0,0,0,0
        )
        print("[FORCE ARM]")

    def disarm(self):
        self.m.mav.command_long_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0,0,0,0,0,0
        )
        print("[DISARM]")

    def reset_all(self):
        self.roll.set(RC_MID); self.pitch.set(RC_MID); self.thr.set(RC_MIN); self.yaw.set(RC_MID)
        self.servo5.set(1500); self.servo7.set(1500)

    def on_close(self):
        if messagebox.askokcancel("Exit", "Close GUI?"):
            self.running = False
            self.stop_yolo()
            self.stop_alt_hold_ctrl()
            self.stop_guided_hold()
            time.sleep(0.05)
            self.root.destroy()


def main():
    root = tk.Tk()
    FakeSticks(root)
    root.mainloop()


if __name__ == "__main__":
    main()
