#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Fake RC Sticks GUI + PS4 Joystick
- GUI סליידרים + שלט PS4
- YOLO + ByteTrack: מעקב אדם לפי YAW + הצגת Bounding Box + Track ID (צבע קבוע לכל ID)
- ALT_HOLD (Pulse helper) + Alt-Hold Controller (סגור-לולאה) עם ▲/▼ 0.1m
- טלמטריה (בשורה אחת): Alt, GroundSpeed, Dist to WP, Yaw, 3D Speed, DistToHome
"""

import math, time, threading, tkinter as tk
from tkinter import ttk, messagebox
from pymavlink import mavutil
import pygame

# ========= YOLO (אופציונלי) =========
YOLO_OK = True
try:
    import cv2
    from ultralytics import YOLO as _YOLO
except Exception as _e:
    YOLO_OK = False
    _YOLO = None
    cv2 = None
    print("⚠️ YOLO/Camera unavailable:", _e)

# ========= ByteTrack =========
BT_OK = True
try:
    import numpy as np
    # Patch for deprecated numpy aliases (נדרש ע"י YOLOX/ByteTrack ישנים)
    if not hasattr(np, "float"): np.float = float
    if not hasattr(np, "int"):   np.int   = int
    if not hasattr(np, "bool"):  np.bool  = bool
    import torch
    from types import SimpleNamespace
    from yolox.tracker.byte_tracker import BYTETracker
except Exception as _e:
    BT_OK = False
    print("⚠️ ByteTrack unavailable:", _e)

# ========= תצורה =========
DEVICE = "COM40"  # לדוגמה: "COM14" / "udp:127.0.0.1:14550"
BAUD   = 115200
RC_MIN, RC_MAX, RC_MID = 1000, 2000, 1500
SEND_HZ = 20.0
DEADZONE = 0.1

# YOLO + ByteTrack (מעקב Yaw על אדם)
YOLO_DB_PIX = 40
YOLO_YAW_GAIN = 0.6
YOLO_THR_MIN = 1150
YOLO_SHOW_WINDOW = True
YOLO_CAM_INDEX = 0
YOLO_MODEL_NAME = "yolov8n.pt"
YOLO_PERSON_CLS = 0        # COCO: person=0

# ALT_HOLD – Pulse helper
ALT_BUMP_DEFAULT = 0.1
ALT_BUMP_FRAC = 0.30
ALT_BUMP_MIN_T = 0.08
ALT_BUMP_MAX_T = 1.00
FALLBACK_SPEED_UP = 1.5
FALLBACK_SPEED_DN = 1.0

# Altitude source anti-flicker
ALT_SOURCE_HOLD_S = 1.5

# Alt-Hold Controller (סגור-לולאה)
ALT_CTRL_TOL_M        = 0.05
ALT_CTRL_KP_US_PER_M  = 1200.0
ALT_CTRL_MAX_US       = 300
ALT_CTRL_DT           = 0.05
ALT_CTRL_PRINT_PERIOD = 0.2

def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v
def apply_deadzone(val, dz=DEADZONE): return 0.0 if abs(val) < dz else val

def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    a1, b1 = math.radians(lat1), math.radians(lon1)
    a2, b2 = math.radians(lat2), math.radians(lon2)
    da, db = a2-a1, b2-b1
    h = math.sin(da/2)**2 + math.cos(a1)*math.cos(a2)*math.sin(db/2)**2
    return 2*R*math.asin(math.sqrt(h))

# --- עזר קריאת פרמטרים (תואם bytes/str) ---
def _param_id_to_str(pid):
    if isinstance(pid, (bytes, bytearray)):
        try:
            return pid.decode('ascii', errors='ignore').rstrip('\x00')
        except Exception:
            return str(pid).rstrip('\x00')
    return str(pid).rstrip('\x00')

# --- צבע יציב לכל Track ID ---
def _id_color(track_id: int):
    # מפה דטרמיניסטית ופשוטה ID->RGB (קריאה מהירה ללא RNG)
    r = (37 * track_id) % 255
    g = (17 * track_id) % 255
    b = (29 * track_id) % 255
    # ודא ניגודיות מסוימת
    r = 60 if r < 60 else r
    g = 60 if g < 60 else g
    b = 60 if b < 60 else b
    return int(b), int(g), int(r)  # OpenCV BGR

class FakeSticks:
    def __init__(self, root):
        self.root = root
        self.root.title(f"Fake RC Sticks + PS4 ({DEVICE} @{BAUD})")

        # MAVLink
        try:
            self.m = mavutil.mavlink_connection(DEVICE, baud=BAUD)
            self.m.wait_heartbeat(timeout=5)
            ttk.Label(root, text=f"Connected: sys {self.m.target_system}, comp {self.m.target_component}").pack()

            # בקשת HOME
            try:
                self.m.mav.command_long_send(
                    self.m.target_system, self.m.target_component,
                    mavutil.mavlink.MAV_CMD_GET_HOME_POSITION, 0,
                    0,0,0,0,0,0,0
                )
            except Exception:
                pass

            # קצבי הודעות
            self._set_msg_rate(mavutil.mavlink.MAVLINK_MSG_ID_ALTITUDE,            10)
            self._set_msg_rate(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10)
            self._set_msg_rate(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,              5)
            self._set_msg_rate(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,           1)
        except Exception as e:
            messagebox.showerror("MAVLink", f"Connect failed: {e}")
            root.destroy(); return

        # PS4
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            self.js = pygame.joystick.Joystick(0); self.js.init()
            print(f"[PS4] Connected: {self.js.get_name()}")
        else:
            self.js = None; print("[PS4] No joystick detected")

        # RC sticks
        self.roll  = tk.IntVar(value=RC_MID)
        self.pitch = tk.IntVar(value=RC_MID)
        self.thr   = tk.IntVar(value=RC_MIN)
        self.yaw   = tk.IntVar(value=RC_MID)

        # Servo
        self.servo5 = tk.IntVar(value=1500)
        self.servo7 = tk.IntVar(value=1500)

        # Battery
        self.batt_v   = tk.StringVar(value="--.- V")
        self.batt_a   = tk.StringVar(value="--.- A")
        self.batt_pct = tk.StringVar(value="-- %")

        # Telemetry
        self.t_alt      = tk.StringVar(value="--")
        self.t_yaw      = tk.StringVar(value="--")
        self.t_wpdist   = tk.StringVar(value="--")
        self.t_disthome = tk.StringVar(value="--")
        self.t_gspeed   = tk.StringVar(value="--")
        self.t_speed3d  = tk.StringVar(value="--")

        # GPS/home
        self._cur_latlon  = None
        self._home_latlon = None

        # Altitude source handling
        self._alt_src   = None
        self._alt_ts    = 0.0
        self._alt_val   = None
        self._priority  = {'VFR':1, 'GPI':2, 'ALTITUDE':3}

        # Alt-Hold Controller state
        self.alt_hold_enabled = False
        self.alt_hold_thread  = None
        self.alt_tgt = tk.DoubleVar(value=0.0)
        self.alt_err = tk.DoubleVar(value=0.0)

        # ---------- GUI ----------
        lf = ttk.LabelFrame(root, text="RC Sticks", padding=8); lf.pack(fill="x", padx=8, pady=6)
        self._mk_slider(lf,"Roll (CH1)", self.roll, RC_MID)
        self._mk_slider(lf,"Pitch (CH2)",self.pitch,RC_MID)
        self._mk_slider(lf,"Throttle (CH3)",self.thr,RC_MIN)
        self._mk_slider(lf,"Yaw (CH4)",   self.yaw,RC_MID)

        ls = ttk.LabelFrame(root, text="Servos (CH5 & CH7)", padding=8); ls.pack(fill="x", padx=8, pady=6)
        self._mk_slider(ls,"Servo CH5", self.servo5, 1500)
        self._mk_slider(ls,"Servo CH7", self.servo7, 1500)

        lb = ttk.LabelFrame(root, text="Battery", padding=8); lb.pack(fill="x", padx=8, pady=6)
        ttk.Label(lb, text="Voltage:").grid(row=0,column=0,sticky="w"); ttk.Label(lb, textvariable=self.batt_v).grid(row=0,column=1,sticky="w")
        ttk.Label(lb, text="Current:").grid(row=1,column=0,sticky="w"); ttk.Label(lb, textvariable=self.batt_a).grid(row=1,column=1,sticky="w")
        ttk.Label(lb, text="Remaining:").grid(row=2,column=0,sticky="w"); ttk.Label(lb, textvariable=self.batt_pct).grid(row=2,column=1,sticky="w")

        # Telemetry – בשורה אחת
        tele = ttk.LabelFrame(root, text="Telemetry", padding=8); tele.pack(fill="x", padx=8, pady=6)
        self._mk_metric(tele, "Altitude (m)",      self.t_alt)
        self._mk_metric(tele, "GroundSpeed (m/s)", self.t_gspeed)
        self._mk_metric(tele, "Dist to WP (m)",    self.t_wpdist)
        self._mk_metric(tele, "Yaw (deg)",         self.t_yaw)
        self._mk_metric(tele, "3D Speed (m/s)",    self.t_speed3d)
        self._mk_metric(tele, "DistToHome (m)",    self.t_disthome)

        # YOLO
        ly = ttk.LabelFrame(root, text="YOLO Yaw Tracking", padding=8); ly.pack(fill="x", padx=8, pady=6)
        self.yolo_enabled = False
        ready_txt = "YOLO: ready"
        if not YOLO_OK: ready_txt = "YOLO: unavailable"
        if not BT_OK:   ready_txt += " (ByteTrack unavailable)"
        self.yolo_status = tk.StringVar(value=ready_txt)
        ttk.Label(ly, textvariable=self.yolo_status).pack(side="left", padx=4)
        ttk.Button(ly, text="▶ Start", command=self.start_yolo).pack(side="left", padx=4)
        ttk.Button(ly, text="⏹ Stop",  command=self.stop_yolo).pack(side="left", padx=4)

        # ALT_HOLD (Pulse helper)
        lm = ttk.LabelFrame(root, text="ALT_HOLD (Pulse helper)", padding=8); lm.pack(fill="x", padx=8, pady=6)
        ttk.Button(lm, text="Hold Alt (ALT_HOLD)", command=self.hold_alt).pack(side="left", expand=True, fill="x", padx=4)
        ttk.Button(lm, text="▲ +0.1 m (pulse)", command=lambda: self.bump_alt(+ALT_BUMP_DEFAULT)).pack(side="left", expand=True, fill="x", padx=4)
        ttk.Button(lm, text="▼ -0.1 m (pulse)", command=lambda: self.bump_alt(-ALT_BUMP_DEFAULT)).pack(side="left", expand=True, fill="x", padx=4)

        # Alt-Hold Controller (סגור-לולאה)
        lc = ttk.LabelFrame(root, text="Altitude Hold (Controller, baro)", padding=8); lc.pack(fill="x", padx=8, pady=6)
        ttk.Button(lc, text="Start Alt-Hold (CTRL)", command=self.start_alt_hold_ctrl).pack(side="left", expand=True, fill="x", padx=4)
        ttk.Button(lc, text="▲ +0.1 m", command=lambda: self.bump_target(+0.1)).pack(side="left", expand=True, fill="x", padx=4)
        ttk.Button(lc, text="▼ −0.1 m", command=lambda: self.bump_target(-0.1)).pack(side="left", expand=True, fill="x", padx=4)
        ttk.Button(lc, text="Stop", command=self.stop_alt_hold_ctrl).pack(side="left", expand=True, fill="x", padx=4)

        row = ttk.Frame(lc, padding=6); row.pack(fill="x", padx=4)
        ttk.Label(row, text="Target (m):").pack(side="left")
        ttk.Label(row, textvariable=self.alt_tgt, width=8).pack(side="left", padx=(0,12))
        ttk.Label(row, text="Error (m):").pack(side="left")
        ttk.Label(row, textvariable=self.alt_err, width=8).pack(side="left")

        # Bottom buttons
        btns = ttk.Frame(root, padding=8); btns.pack(fill="x")
        ttk.Button(btns,text="Force ARM", command=self.force_arm).pack(side="left",expand=True,fill="x",padx=4)
        ttk.Button(btns,text="DISARM",    command=self.disarm).pack(side="left",expand=True,fill="x",padx=4)
        ttk.Button(btns,text="Reset All", command=self.reset_all).pack(side="left",expand=True,fill="x",padx=4)
        ttk.Button(btns,text="Exit",      command=self.on_close).pack(side="left",expand=True,fill="x",padx=4)

        # Threads
        self.running = True
        threading.Thread(target=self._send_loop,     daemon=True).start()
        threading.Thread(target=self._mav_telemetry, daemon=True).start()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        try:
            self.root.state('zoomed')  # Windows: פתיחה במסך מלא
        except Exception:
            self.root.geometry("1200x800")

    # ---------- GUI helpers ----------
    def _mk_slider(self,parent,label,var,reset_val):
        row=ttk.Frame(parent); row.pack(fill="x", pady=4)
        ttk.Label(row,text=label,width=12).pack(side="left")
        s=ttk.Scale(row,from_=RC_MIN,to=RC_MAX,orient="horizontal",variable=var)
        s.pack(side="left",fill="x",expand=True,padx=8)
        ttk.Label(row,textvariable=var,width=5).pack(side="left")
        ttk.Button(row,text="Reset", command=lambda v=var,rv=reset_val: v.set(rv)).pack(side="left",padx=4)

    def _mk_metric(self, parent, title, var, font_size=32):
        card = ttk.Frame(parent, padding=8)
        card.pack(side="left", fill="both", expand=True, padx=6, pady=6)
        ttk.Label(card, text=title, anchor="center").pack(fill="x")
        lbl = ttk.Label(card, textvariable=var, anchor="center")
        lbl.pack(fill="both", expand=True)
        lbl.configure(font=("Segoe UI", font_size, "bold"))

    # ---------- MAVLink helpers ----------
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
        try:
            mode_id = self.m.mode_mapping()[mode_name]
            self.m.set_mode(mode_id)
            print(f"[MODE] {mode_name}")
        except Exception as e:
            print(f"[MODE] Error setting {mode_name}:", e)

    def get_param(self, name, default=None, timeout=2.0):
        """קריאת פרמטר בודד (חסינת bytes/str)."""
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

    # ---------- ALT HOLD (Pulse helper) ----------
    def hold_alt(self):
        self.set_mode("ALT_HOLD")
        rc3_hover, hover = self._hover_rc()
        steps, cur = 10, self.thr.get()
        for i in range(1, steps+1):
            self.thr.set(int(cur + (rc3_hover - cur) * i/steps))
            self._send_override(); time.sleep(0.03)
        print(f"[ALT_HOLD] Hover={hover:.2f} → RC3={rc3_hover}")

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
            print(f"[ALT_BUMP] Δh={delta_m:+.2f}m dur={dur:.2f}s RC3={rc_pulse}")
        threading.Thread(target=_do, daemon=True).start()

    # ---------- Alt-Hold Controller (סגור-לולאה) ----------
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

    # ---------- TX loop ----------
    def _send_loop(self):
        per=1.0/SEND_HZ
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

    # ---------- Altitude + Telemetry RX ----------
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
                try: self.t_yaw.set(f"{float(msg.heading):.2f}")
                except Exception: pass
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

    # ---------- YOLO + ByteTrack ----------
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
        # טען YOLO
        try:
            model = _YOLO(YOLO_MODEL_NAME)
        except Exception as e:
            print("⚠️ YOLO load failed:", e); self.yolo_status.set("YOLO: load failed"); self.yolo_enabled=False; return

        # מצלמה
        cap = cv2.VideoCapture(YOLO_CAM_INDEX)
        if not cap.isOpened():
            print("⚠️ No camera found"); self.yolo_status.set("YOLO: no camera"); self.yolo_enabled=False; return

        # אתחל ByteTrack עם FPS מהמצלמה (נופל ל-30 אם לא ידוע)
        fps = cap.get(cv2.CAP_PROP_FPS)
        fps = float(fps) if fps and fps > 0 else 30.0
        bt_args = SimpleNamespace(track_thresh=0.15, match_thresh=0.8, track_buffer=30, frame_rate=fps, mot20=False)
        tracker = BYTETracker(bt_args)

        print("[YOLO+BT] tracking person by YAW only. ESC/Q closes preview.")
        while self.running and self.yolo_enabled:
            ok, frame = cap.read()
            if not ok: time.sleep(0.01); continue
            h, w = frame.shape[:2]; cx_ref = w * 0.5
            rc4_to_send = None

            try:
                results = model(frame, verbose=False)
                # בנה דיטקציות רק ל-person (YOLO_PERSON_CLS)
                det_list = []
                for r in results:
                    if not hasattr(r, "boxes"): continue
                    for b in r.boxes:
                        cls = int(b.cls[0].item())
                        if cls != YOLO_PERSON_CLS:  # עקוב רק אחרי אדם
                            continue
                        x1, y1, x2, y2 = map(float, b.xyxy[0])
                        conf = float(b.conf[0].item()) if hasattr(b, "conf") else 0.0
                        det_list.append([x1, y1, x2, y2, conf, cls])

                # המרה ל-Tensor עבור ByteTrack
                if len(det_list) > 0:
                    dets = torch.tensor(det_list, dtype=torch.float32)
                else:
                    dets = torch.empty((0, 6), dtype=torch.float32)

                # עדכון מעקב
                tracks = tracker.update(dets, frame.shape, frame.shape)

                # בחר מטרה ל-YAW: ה-track שמרכזו הכי קרוב למרכז התמונה
                target = None
                min_abs_dx = 1e9

                # צייר בוקסים + ID
                annotated = frame.copy()
                # קו מרכז אנכי לעזר
                if YOLO_SHOW_WINDOW:
                    cv2.line(annotated, (int(cx_ref), 0), (int(cx_ref), h), (255, 0, 0), 2)

                for t in tracks:
                    tlbr = t.tlbr.astype(int)        # [x1,y1,x2,y2]
                    x1, y1, x2, y2 = tlbr.tolist()
                    track_id = int(t.track_id)
                    color = _id_color(track_id)

                    # ציור תיבה + תווית ID
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(annotated, f"ID {track_id}", (x1, max(0, y1-10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

                    # בחירת היעד הקרוב למרכז
                    cx = 0.5 * (x1 + x2)
                    dx_px = cx - cx_ref
                    if abs(dx_px) < min_abs_dx:
                        min_abs_dx = abs(dx_px)
                        target = dx_px

                # שליטת YAW על סמך היעד
                if target is not None:
                    if abs(target) > YOLO_DB_PIX:
                        norm = clamp(target / (w * 0.5), -1.0, 1.0)
                        yaw_cmd = clamp(YOLO_YAW_GAIN * norm, -1.0, 1.0)
                        rc4_to_send = int(RC_MID + yaw_cmd * 500)
                    else:
                        rc4_to_send = RC_MID

                    # שמור מינימום מצערת בעת מעקב (אם לא Alt-Hold)
                    if self.thr.get() < YOLO_THR_MIN and not self.alt_hold_enabled:
                        self.thr.set(YOLO_THR_MIN)

                # הצג חלון
                if YOLO_SHOW_WINDOW:
                    cv2.imshow("YOLO + ByteTrack (Person Yaw Tracking)", annotated)

            except Exception as e:
                print("[YOLO+BT] inference/tracking error:", e)

            # עדכן RC Yaw
            if rc4_to_send is not None:
                self.yaw.set(clamp(rc4_to_send, RC_MIN, RC_MAX))

            # יציאה ע"י ESC / q
            if YOLO_SHOW_WINDOW:
                k = cv2.waitKey(1) & 0xFF
                if k in (27, ord('q')): self.stop_yolo(); break

        # ניקוי
        try:
            cap.release()
            if YOLO_SHOW_WINDOW: cv2.destroyAllWindows()
        except Exception:
            pass
        print("[YOLO+BT] stopped")

    # ---------- ARM/DISARM/Reset/Close ----------
    def force_arm(self):
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0,0,0,0,0)
        print("[FORCE ARM]")

    def disarm(self):
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0,0,0,0,0,0)
        print("[DISARM]")

    def reset_all(self):
        self.roll.set(RC_MID); self.pitch.set(RC_MID); self.thr.set(RC_MIN); self.yaw.set(RC_MID)
        self.servo5.set(1500); self.servo7.set(1500)

    def on_close(self):
        if messagebox.askokcancel("Exit", "Close GUI?"):
            self.running=False
            self.stop_yolo()
            self.stop_alt_hold_ctrl()
            time.sleep(0.05)
            self.root.destroy()

def main():
    root=tk.Tk()
    FakeSticks(root)
    root.mainloop()

if __name__=="__main__":
    main()
