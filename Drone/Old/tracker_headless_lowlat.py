#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# tracker_headless_lowlat.py
"""
Fake RC Sticks GUI + PS4 + Optional YOLO/ByteTrack (Low-Latency build)
- Latest-frame only camera reader thread (drops frames, always uses newest)
- Camera low-buffer: BUFFERSIZE=1, FOURCC=MJPG, set FPS/Res, V4L2/DSHOW
- Torch/OpenCV threads capped to reduce overhead
- imgsz control + optional downscale pre-inference (RPi-friendly)
- Optional --no-bt to disable ByteTrack (detection-only = lower latency)
- --lowlat profile tweaks filters, smoothing and timeouts for responsiveness

Safety + Controls:
- Emergency Stop (Triangle): sets R/P/Y=1500, THR=1000, tries BRAKE/LOITER,
  optional MAV_CMD_DO_FLIGHTTERMINATION, force DISARM, holds 3s, then STABILIZE
- ARM on X, DISARM on O preserved; color highlight shown only while pressed

Run examples:
  python3 tracker_headless_lowlat.py --lowlat --no-bt --imgsz 384 --cam-fps 30 --preview 0
"""

import os, sys, glob, math, time, threading, argparse
import tkinter as tk
from tkinter import ttk, messagebox
from tkinter import font as tkfont
from pymavlink import mavutil
import pygame

# ---------- Parse CLI early (so env vars can apply before libs init) ----------
def _parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--lowlat", action="store_true", help="Aggressive low-latency profile")
    p.add_argument("--no-bt",  action="store_true", help="Disable ByteTrack (detection-only)")
    p.add_argument("--imgsz",  type=int, default=448, help="YOLO inference size (square)")
    p.add_argument("--det-downscale", type=float, default=0.67, help="Pre-inference downscale (0.2..1.0)")
    p.add_argument("--cam-index", type=int, default=0, help="Camera index")
    p.add_argument("--cam-w", type=int, default=640, help="Capture width")
    p.add_argument("--cam-h", type=int, default=480, help="Capture height")
    p.add_argument("--cam-fps", type=int, default=30, help="Capture FPS target")
    p.add_argument("--preview", type=int, default=1, help="Show annotated preview window (1/0)")
    return p.parse_args()

CLI = _parse_args()

# Environment knobs for fewer threads (helpful to reduce latency + jitter)
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")
os.environ.setdefault("PYTORCH_MPS_HIGH_WATERMARK_RATIO", "0.0")

# ---------- Compact UI Settings ----------
UI_SCALE = 0.90
METRIC_FONT_SIZE = 24
PADDING = 6
INNER_PAD = 4
LABEL_WIDTH = 12
VALUE_WIDTH = 4
GEOMETRY = "1100x840"

# ---------- Serial auto-detect ----------
if sys.platform.startswith("linux"):
    import glob as _glob
    cands = _glob.glob("/dev/ttyACM*") + _glob.glob("/dev/ttyUSB*")
    DEVICE = cands[0] if cands else "/dev/ttyACM0"
    BAUD = 115200
else:
    DEVICE = "COM40"
    BAUD = 115200

# ---------- Joystick Mapping ----------
LINUX_AXIS_ROLL  = 3
LINUX_AXIS_PITCH = 4  # inverted
LINUX_AXIS_YAW   = 0
LINUX_AXIS_THR   = 1  # inverted
WIN_AXIS_ROLL  = 0
WIN_AXIS_PITCH = 1  # inverted
WIN_AXIS_YAW   = 2
WIN_AXIS_THR   = 3  # inverted

if sys.platform.startswith("linux"):
    AXIS_ROLL, AXIS_PITCH, AXIS_YAW, AXIS_THR = LINUX_AXIS_ROLL, LINUX_AXIS_PITCH, LINUX_AXIS_YAW, LINUX_AXIS_THR
else:
    AXIS_ROLL, AXIS_PITCH, AXIS_YAW, AXIS_THR = WIN_AXIS_ROLL, WIN_AXIS_PITCH, WIN_AXIS_YAW, WIN_AXIS_THR

# Invert flags
PITCH_INVERT = True
THR_INVERT   = True
ROLL_INVERT  = False
YAW_INVERT   = False

# ---------- RC/Control ----------
RC_MIN, RC_MAX, RC_MID = 1000, 2000, 1500
SEND_HZ   = 20.0
DEADZONE  = 0.10

# ---------- SAFE throttle for altitude hold ----------
THR_SAFE_MAX = 1500
THR_SLEW_US_PER_SEC = 600

# ---------- YOLO / camera (lazy import; after env vars) ----------
YOLO_OK = True
try:
    import cv2
    from ultralytics import YOLO as _YOLO
    try:
        cv2.setNumThreads(0)
    except Exception:
        pass
except Exception as _e:
    YOLO_OK = False
    _YOLO = None
    cv2 = None
    print("Warning: YOLO/Camera unavailable:", _e)

# ---------- Optional: ByteTrack & deps ----------
BT_OK = True
try:
    import numpy as np
    if not hasattr(np, "float"): np.float = float
    if not hasattr(np, "int"):   np.int   = int
    if not hasattr(np, "bool"):  np.bool  = bool
    import torch
    from types import SimpleNamespace
    from yolox.tracker.byte_tracker import BYTETracker
    try:
        torch.set_num_threads(1)
        torch.set_num_interop_threads(1)
    except Exception:
        pass
except Exception as _e:
    BT_OK = False
    print("Warning: ByteTrack unavailable:", _e)

# ---------- ByteTrack tuning (can be disabled by --no-bt) ----------
BT_TRACK_THRESH  = 0.10
BT_MATCH_THRESH  = 0.80
BT_TRACK_BUFFER  = 30

# ---------- YOLO + ByteTrack tuning ----------
YOLO_DB_PIX      = 40
YOLO_THR_MIN     = 1150
YOLO_SHOW_WINDOW = bool(CLI.preview)
YOLO_CAM_INDEX   = CLI.cam_index
YOLO_MODEL_NAME  = "yolov8n.pt"
YOLO_PERSON_CLS  = 0
YOLO_CONF        = 0.15
YOLO_IOU         = 0.50
DRAW_CONF_MIN    = 0.60
DET_DOWNSCALE    = float(CLI.det_downscale)

# ---------- Lost-target scan ----------
LOST_TIMEOUT_S   = 1.0
SCAN_PERIOD_S    = 3.0
SCAN_AMPL_RC     = 140

# ---------- Altitude handling ----------
ALT_SOURCE_HOLD_S = 1.5
ALT_BUMP_DEFAULT = 0.1
ALT_BUMP_FRAC = 0.30
ALT_BUMP_MIN_T = 0.08
ALT_BUMP_MAX_T = 1.00
FALLBACK_SPEED_UP = 1.5
FALLBACK_SPEED_DN = 1.0

# ---------- Baro Alt-Hold controller ----------
ALT_CTRL_TOL_M        = 0.05
ALT_CTRL_KP_US_PER_M  = 1200.0
ALT_CTRL_MAX_US       = 300
ALT_CTRL_DT           = 0.05
ALT_CTRL_PRINT_PERIOD = 0.20

# ---------- GUIDED keepalive ----------
GUIDED_KEEPALIVE_HZ = 5.0

# ---------- Visual forward (Pitch from object size) ----------
VS_TGT_AREA_FRAC   = 0.025
VS_DB_AREA_FRAC    = 0.004
VS_KP_PITCH        = 0.70
VS_MAX_RC_DELTA    = 220
VS_LP_ALPHA        = 0.18
VS_PITCH_FORWARD_SIGN = -1

VS_THR_COMP_GAIN_US = 60
VS_THR_COMP_MAX_US  = 100

# ---------- Emergency Stop ----------
E_STOP_HOLD_S = 3.0
CMD_FLIGHT_TERMINATION = getattr(mavutil.mavlink, "MAV_CMD_DO_FLIGHTTERMINATION", 185)
DO_FLIGHT_TERMINATION_ON_ESTOP = True

# ---------- Colors ----------
BTN_GREEN = "#16a34a"
BTN_RED   = "#b91c1c"

# ---------- Tracking control defaults ----------
YAW_DB_PIX          = 30
YAW_KP_DEFAULT      = 0.60
YAW_KD_DEFAULT      = 0.20
YAW_MAX_RC_DELTA    = 320
YAW_LP_ALPHA        = 0.30

VS_YAW_OK_PIX_DEFAULT = 40
NEAR_STOP_AREA_FRAC_DEFAULT = 0.120
FAR_STOP_AREA_FRAC_DEFAULT  = 0.0015

# Low-latency profile tweaks
if CLI.lowlat:
    YOLO_SHOW_WINDOW = False if CLI.preview == 1 else bool(CLI.preview)
    YAW_LP_ALPHA     = 0.50
    VS_LP_ALPHA      = 0.30
    LOST_TIMEOUT_S   = 0.6
    SCAN_PERIOD_S    = 2.4
    DRAW_CONF_MIN    = 0.55

def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v
def apply_deadzone(val, dz=DEADZONE): return 0.0 if abs(val) < dz else val
def _rad2deg_wrap(rad: float) -> float:
    deg = math.degrees(rad)
    while deg > 180.0:  deg -= 360.0
    while deg <= -180.0: deg += 360.0
    return deg

# ---------- Latest-frame camera reader ----------
class LatestFrameCamera:
    def __init__(self, index, width, height, fps):
        self.index = index
        self.width = width
        self.height = height
        self.fps = fps
        self.cap = None
        self.last = None
        self.lock = threading.Lock()
        self.running = False

    def open(self):
        api = cv2.CAP_DSHOW if sys.platform.startswith("win") else cv2.CAP_V4L2
        self.cap = cv2.VideoCapture(self.index, api)
        if not self.cap.isOpened():
            # fallback to default API
            self.cap = cv2.VideoCapture(self.index)
        if not self.cap.isOpened():
            return False
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        try:
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        except Exception:
            pass
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS,          self.fps)
        self.running = True
        threading.Thread(target=self._reader, daemon=True).start()
        return True

    def _reader(self):
        # Tight loop: always keep only the newest frame
        while self.running:
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.005); continue
            with self.lock:
                self.last = frame

    def get_latest(self):
        with self.lock:
            if self.last is None:
                return None
            return self.last

    def release(self):
        self.running = False
        try:
            if self.cap: self.cap.release()
        except Exception:
            pass

# ---------- Main App ----------
class FakeSticks:
    def __init__(self, root):
        self.root = root
        self.root.title(f"Fake RC Sticks + PS4 ({DEVICE} @{BAUD})")

        # Apply compact UI scaling and fonts
        self._apply_compact_style()

        # MAVLink connect
        try:
            self.m = mavutil.mavlink_connection(DEVICE, baud=BAUD)
            self.m.wait_heartbeat(timeout=5)
            ttk.Label(root, text=f"Connected: sys {self.m.target_system}, comp {self.m.target_component}").pack()
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

        # Battery & telemetry
        self.batt_v   = tk.StringVar(value="--.- V")
        self.batt_a   = tk.StringVar(value="--.- A")
        self.batt_pct = tk.StringVar(value="-- %")

        self.t_alt      = tk.StringVar(value="--")
        self.t_gspeed   = tk.StringVar(value="--")
        self.t_wpdist   = tk.StringVar(value="--")
        self.t_disthome = tk.StringVar(value="--")
        self.t_speed3d  = tk.StringVar(value="--")
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

        # Controllers/holds state
        self.alt_hold_enabled = False
        self.alt_hold_thread  = None
        self.alt_tgt = tk.DoubleVar(value=0.0)
        self.alt_err = tk.DoubleVar(value=0.0)
        self._guided_active = False
        self._guided_target = None
        self._guided_lock   = threading.Lock()
        self._alt_hold_helper_active = False

        # Throttle smoothing state
        self._thr_last = RC_MIN
        self._thr_last_ts = time.time()

        # Visual forward state/toggles
        self._vs_pitch_rc = RC_MID
        self.vs_pitch_en    = tk.BooleanVar(value=True)
        self.vs_thr_comp_en = tk.BooleanVar(value=True)

        # Tracking state
        self._trk_id = None
        self._trk_last_seen = 0.0
        self._dx_lp = 0.0
        self._dx_prev = 0.0
        self._last_loop_ts = time.time()

        # Emergency/arm state
        self._e_stop_active = False
        self._e_stop_lock = threading.Lock()
        self.is_armed = False
        self._arm_inhibit_until = 0.0

        # ---- GUI ----
        lf = ttk.LabelFrame(root, text="RC Sticks", padding=PADDING); lf.pack(fill="x", padx=PADDING, pady=INNER_PAD)
        self._mk_slider(lf, "Roll (CH1)",     self.roll,  RC_MID)
        self._mk_slider(lf, "Pitch (CH2)",    self.pitch, RC_MID)
        self._mk_slider(lf, "Throttle (CH3)", self.thr,   RC_MIN)
        self._mk_slider(lf, "Yaw (CH4)",      self.yaw,   RC_MID)

        ls = ttk.LabelFrame(root, text="Servos (CH5 & CH7)", padding=PADDING); ls.pack(fill="x", padx=PADDING, pady=INNER_PAD)
        self._mk_slider(ls,"Servo CH5", self.servo5, 1500)
        self._mk_slider(ls,"Servo CH7", self.servo7, 1500)

        lb = ttk.LabelFrame(root, text="Battery", padding=PADDING); lb.pack(fill="x", padx=PADDING, pady=INNER_PAD)
        ttk.Label(lb, text="Voltage:").grid(row=0,column=0,sticky="w"); ttk.Label(lb, textvariable=self.batt_v).grid(row=0,column=1,sticky="w")
        ttk.Label(lb, text="Current:").grid(row=1,column=0,sticky="w"); ttk.Label(lb, textvariable=self.batt_a).grid(row=1,column=1,sticky="w")
        ttk.Label(lb, text="Remaining:").grid(row=2,column=0,sticky="w"); ttk.Label(lb, textvariable=self.batt_pct).grid(row=2,column=1,sticky="w")

        att = ttk.LabelFrame(root, text="Attitude (deg)", padding=PADDING); att.pack(fill="x", padx=PADDING, pady=INNER_PAD)
        self._mk_metric(att, "Roll (deg)",  self.t_roll)
        self._mk_metric(att, "Pitch (deg)", self.t_pitch)
        self._mk_metric(att, "Yaw (deg)",   self.t_yaw)

        tele = ttk.LabelFrame(root, text="Telemetry", padding=PADDING); tele.pack(fill="x", padx=PADDING, pady=INNER_PAD)
        self._mk_metric(tele, "Altitude (m)",      self.t_alt)
        self._mk_metric(tele, "GroundSpeed (m/s)", self.t_gspeed)
        self._mk_metric(tele, "Dist to WP (m)",    self.t_wpdist)
        self._mk_metric(tele, "3D Speed (m/s)",    self.t_speed3d)
        self._mk_metric(tele, "DistToHome (m)",    self.t_disthome)

        ly = ttk.LabelFrame(root, text="YOLO + ByteTrack (Yaw + Forward)", padding=PADDING); ly.pack(fill="x", padx=PADDING, pady=INNER_PAD)
        self.yolo_enabled = False
        ready_txt = "YOLO: ready" if YOLO_OK else "YOLO: unavailable"
        if not (BT_OK and not CLI.no_bt): ready_txt += " (ByteTrack off)"
        self.yolo_status = tk.StringVar(value=ready_txt)
        ttk.Label(ly, textvariable=self.yolo_status).pack(side="left", padx=INNER_PAD)
        ttk.Button(ly, text="Start", command=self.start_yolo).pack(side="left", padx=INNER_PAD)
        ttk.Button(ly, text="Stop",  command=self.stop_yolo).pack(side="left", padx=INNER_PAD)
        ttk.Checkbutton(ly, text="Forward from size (Pitch)", variable=self.vs_pitch_en).pack(side="left", padx=INNER_PAD)
        ttk.Checkbutton(ly, text="Throttle comp", variable=self.vs_thr_comp_en).pack(side="left", padx=INNER_PAD)

        tun = ttk.LabelFrame(root, text="Tracking Tunables (live)", padding=PADDING); tun.pack(fill="x", padx=PADDING, pady=INNER_PAD)
        self.kp_var   = tk.DoubleVar(value=YAW_KP_DEFAULT)
        self.kd_var   = tk.DoubleVar(value=YAW_KD_DEFAULT)
        self.yaw_ok_px_var = tk.DoubleVar(value=VS_YAW_OK_PIX_DEFAULT)
        self.far_area_var  = tk.DoubleVar(value=FAR_STOP_AREA_FRAC_DEFAULT)
        self.near_area_var = tk.DoubleVar(value=NEAR_STOP_AREA_FRAC_DEFAULT)
        self._mk_tunable(tun, "YAW_KP",  self.kp_var,   0.0, 1.5, 0.01, "{:.2f}")
        self._mk_tunable(tun, "YAW_KD",  self.kd_var,   0.0, 0.8,  0.01, "{:.2f}")
        row_bounds = ttk.Frame(tun); row_bounds.pack(fill="x", pady=INNER_PAD)
        self._mk_tunable(tun, "VS_YAW_OK_PIX", self.yaw_ok_px_var, 0.0, 200.0, 1.0, "{:.0f}")
        self._mk_tunable(row_bounds, "FAR_STOP_AREA_FRAC",  self.far_area_var,  0.000, 0.020, 0.001, "{:.3f}")
        self._mk_tunable(row_bounds, "NEAR_STOP_AREA_FRAC", self.near_area_var, 0.020, 0.300, 0.001, "{:.3f}")
        ttk.Button(tun, text="Reset to defaults", command=self._reset_tunables).pack(side="right", padx=INNER_PAD)

        mf = ttk.LabelFrame(root, text="Flight Modes", padding=PADDING)
        mf.pack(fill="x", padx=PADDING, pady=INNER_PAD)
        ttk.Button(mf, text="GUIDED",    command=lambda: self.set_mode("GUIDED")).pack(side="left", expand=True, fill="x", padx=INNER_PAD)
        ttk.Button(mf, text="STABILIZE", command=lambda: self.set_mode("STABILIZE")).pack(side="left", expand=True, fill="x", padx=INNER_PAD)

        btns = ttk.Frame(root, padding=PADDING); btns.pack(fill="x")
        self.arm_state = tk.StringVar(value="DISARMED")
        self.lbl_arm_state = tk.Label(btns, textvariable=self.arm_state,
                                      width=12, relief="groove", fg="white")
        self.lbl_arm_state.pack(side="left", padx=INNER_PAD)
        self._set_arm_state_ui(False)

        self.btn_arm = tk.Button(btns, text="ARM (X)",
                                 command=self.force_arm, width=14,
                                 activebackground=BTN_GREEN, activeforeground="white")
        self.btn_arm.pack(side="left", expand=True, fill="x", padx=INNER_PAD)

        self.btn_disarm = tk.Button(btns, text="DISARM (O)",
                                    command=self.disarm, width=14,
                                    activebackground=BTN_RED, activeforeground="white")
        self.btn_disarm.pack(side="left", expand=True, fill="x", padx=INNER_PAD)

        ttk.Button(btns, text="Reset All", command=self.reset_all).pack(side="left", expand=True, fill="x", padx=INNER_PAD)
        self.btn_estop = tk.Button(btns, text="Emergency Stop (Triangle)",
                                   command=self.emergency_stop_async,
                                   activebackground=BTN_RED, activeforeground="white")
        self.btn_estop.pack(side="left", expand=True, fill="x", padx=INNER_PAD)
        ttk.Button(btns, text="Exit", command=self.on_close).pack(side="left", expand=True, fill="x", padx=INNER_PAD)

        self._js_prev = {0:0, 1:0, 3:0}

        # Threads
        self.running = True
        threading.Thread(target=self._send_loop,             daemon=True).start()
        threading.Thread(target=self._mav_telemetry,         daemon=True).start()
        threading.Thread(target=self._guided_keepalive_loop, daemon=True).start()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.geometry(GEOMETRY)

    # ---------- ARM UI ----------
    def _set_arm_state_ui(self, armed: bool):
        try:
            if armed:
                self.arm_state.set("ARMED")
                self.lbl_arm_state.configure(bg=BTN_GREEN, fg="white")
            else:
                self.arm_state.set("DISARMED")
                self.lbl_arm_state.configure(bg=BTN_RED, fg="white")
        except Exception:
            pass

    # ---------- Helpers ----------
    def _mk_tunable(self, parent, text, var, vmin, vmax, step, fmt="{:.2f}"):
        row = ttk.Frame(parent); row.pack(fill="x", pady=INNER_PAD)
        ttk.Label(row, text=text, width=20).pack(side="left")
        sc = tk.Scale(row, from_=vmin, to=vmax, orient="horizontal",
                      resolution=step, showvalue=0, length=360)
        sc.configure(variable=var)
        sc.pack(side="left", fill="x", expand=True, padx=INNER_PAD)
        lbl = ttk.Label(row, width=8); lbl.pack(side="left")
        def _update_lbl(*_):
            try:
                lbl.configure(text=fmt.format(float(var.get())))
            except Exception:
                pass
        var.trace_add("write", lambda *_: _update_lbl())
        _update_lbl()

    def _reset_tunables(self):
        self.kp_var.set(YAW_KP_DEFAULT)
        self.kd_var.set(YAW_KD_DEFAULT)
        self.yaw_ok_px_var.set(VS_YAW_OK_PIX_DEFAULT)
        self.far_area_var.set(FAR_STOP_AREA_FRAC_DEFAULT)
        self.near_area_var.set(NEAR_STOP_AREA_FRAC_DEFAULT)

    def _send_rc_neutral_minthr(self):
        self.roll.set(RC_MID); self.pitch.set(RC_MID); self.yaw.set(RC_MID)
        self._set_thr(RC_MIN, apply_slew=False)
        self._send_override()

    def _try_cmd_many(self, fn, tries=3, delay=0.12, label="CMD"):
        ok = False
        for i in range(tries):
            try:
                fn()
                ok = True
            except Exception as e:
                print(f"[{label}] try {i+1}/{tries} failed:", e)
            time.sleep(delay)
        return ok

    def _clear_flight_termination(self):
        try:
            self.m.mav.command_long_send(
                self.m.target_system, self.m.target_component,
                CMD_FLIGHT_TERMINATION, 0,
                0, 0,0,0,0,0,0
            )
        except Exception as e:
            print("[FLIGHT_TERM_OFF] error:", e)

    # ---------- SAFE/hold helpers ----------
    def _is_hold_context(self) -> bool:
        return self.alt_hold_enabled or self._guided_active or self._alt_hold_helper_active

    def _set_thr(self, value: int, apply_slew: bool = None):
        if apply_slew is None:
            apply_slew = self._is_hold_context()
        rc = int(value)
        if self._is_hold_context():
            rc = clamp(rc, RC_MIN, THR_SAFE_MAX)
            now = time.time()
            dt = max(1e-3, now - self._thr_last_ts)
            max_step = int(THR_SLEW_US_PER_SEC * dt)
            if abs(rc - self._thr_last) > max_step:
                rc = self._thr_last + (max_step if rc > self._thr_last else -max_step)
            self._thr_last = rc
            self._thr_last_ts = now
        else:
            rc = clamp(rc, RC_MIN, RC_MAX)
        self.thr.set(rc)

    # ---------- UI helpers ----------
    def _apply_compact_style(self):
        try:
            self.root.tk.call('tk', 'scaling', UI_SCALE)
        except Exception:
            pass
        for name in ("TkDefaultFont","TkTextFont","TkHeadingFont","TkMenuFont",
                     "TkIconFont","TkFixedFont","TkTooltipFont","TkSmallCaptionFont"):
            try:
                f = tkfont.nametofont(name)
                sz = int(f.cget("size"))
                new_sz = int(round(sz * UI_SCALE)) if sz != 0 else sz
                if new_sz == 0: new_sz = -9 if sz < 0 else 9
                f.configure(size=new_sz)
            except Exception:
                continue
        try: ttk.Style(self.root)
        except Exception: pass

    def _mk_slider(self, parent, label, var, reset_val):
        row = ttk.Frame(parent); row.pack(fill="x", pady=INNER_PAD)
        ttk.Label(row, text=label, width=LABEL_WIDTH).pack(side="left")
        s = ttk.Scale(row, from_=RC_MIN, to=RC_MAX, orient="horizontal", variable=var)
        s.pack(side="left", fill="x", expand=True, padx=INNER_PAD)
        ttk.Label(row, textvariable=var, width=VALUE_WIDTH).pack(side="left")
        ttk.Button(row, text="Reset", command=lambda v=var, rv=reset_val: v.set(rv)).pack(side="left", padx=INNER_PAD)

    def _mk_metric(self, parent, title, var, font_size=METRIC_FONT_SIZE):
        card = ttk.Frame(parent, padding=INNER_PAD)
        card.pack(side="left", fill="both", expand=True, padx=INNER_PAD, pady=INNER_PAD)
        ttk.Label(card, text=title, anchor="center").pack(fill="x")
        lbl = ttk.Label(card, textvariable=var, anchor="center")
        lbl.pack(fill="both", expand=True)
        lbl.configure(font=("TkDefaultFont", font_size, "bold"))

    # ---------- MAVLink ----------
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
        rc1 = int(self.roll.get())
        rc2 = int(self.pitch.get())
        rc4 = int(self.yaw.get())
        rc5 = int(self.servo5.get())
        rc7 = int(self.servo7.get())
        rc3 = int(self.thr.get())
        if self._is_hold_context():
            rc3 = clamp(rc3, RC_MIN, THR_SAFE_MAX)
        else:
            rc3 = clamp(rc3, RC_MIN, RC_MAX)
        try:
            self.m.mav.rc_channels_override_send(
                self.m.target_system, self.m.target_component,
                rc1, rc2, rc3, rc4, rc5, 65535, rc7, 65535
            )
        except Exception as e:
            print("[RC_OVERRIDE] error:", e)

    def set_mode(self, mode_name):
        try:
            mode_id = self.m.mode_mapping()[mode_name]
            self.m.set_mode(mode_id)
            print(f"[MODE] {mode_name}")
            if mode_name != "GUIDED":
                self.stop_guided_hold()
        except Exception as e:
            print(f"[MODE] Error setting {mode_name}:", e)

    # ---------- Telemetry ----------
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
            if t == "HEARTBEAT":
                try:
                    armed_flag = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    if armed_flag != self.is_armed:
                        self.is_armed = armed_flag
                        self._set_arm_state_ui(self.is_armed)
                except Exception:
                    pass
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
                except Exception: pass
                try:
                    self._consider_alt(float(msg.relative_alt)/1000.0, "GPI")
                except Exception: pass
                try:
                    self._cur_latlon = (msg.lat/1e7, msg.lon/1e7)
                except Exception:
                    pass
            if t == "VFR_HUD":
                try: self.t_gspeed.set(f"{float(msg.groundspeed):.2f}")
                except Exception: pass
                try:
                    if (time.time() - self._att_ts) > 1.0:
                        self.t_yaw.set(f"{float(msg.heading):.1f}")
                except Exception: pass
                try:
                    if (time.time() - self._alt_ts) > ALT_SOURCE_HOLD_S:
                        self._consider_alt(float(msg.alt), "VFR")
                except Exception: pass
            if t == "HOME_POSITION":
                try:
                    self._home_latlon = (msg.latitude/1e7, msg.longitude/1e7)
                except Exception:
                    self._home_latlon = None

    # ---------- ALT HOLD / GUIDED helpers ----------
    def _hover_rc(self):
        def clamp01(x): return 0.0 if x is None else max(0.0, min(1.0, float(x)))
        try:
            hover = clamp01(self.get_param("MOT_THST_HOVER", default=0.5))
        except Exception:
            hover = 0.5
        rc3_hover = int(RC_MIN + hover * (RC_MAX - RC_MIN))
        return rc3_hover, hover

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
                pid = (getattr(p, "param_id", b"") or b"")
                if isinstance(pid, (bytes, bytearray)):
                    pid = pid.decode("ascii", "ignore").rstrip("\x00")
                if pid == target:
                    try: return float(p.param_value)
                    except Exception: return p.param_value
            print(f"[PARAM] timeout reading {target}")
        except Exception as e:
            print(f"[PARAM] read {name} failed:", e)
        return default

    def hold_alt(self):
        self.set_mode("ALT_HOLD")
        self._alt_hold_helper_active = True
        rc3_hover, hover = self._hover_rc()
        steps, cur = 10, self.thr.get()
        for i in range(1, steps+1):
            self._set_thr(int(cur + (rc3_hover - cur) * i/steps))
            self._send_override(); time.sleep(0.03)
        self._alt_hold_helper_active = False
        print(f"[ALT_HOLD] Hover={hover:.2f} -> RC3<= {THR_SAFE_MAX}")

    def start_alt_hold_ctrl(self):
        self.set_mode("ALT_HOLD")
        self.alt_hold_enabled = True
        rc3_hover, _ = self._hover_rc()
        self._set_thr(rc3_hover)
        tgt = self._alt_val if self._alt_val is not None else 0.0
        self.alt_tgt.set(round(float(tgt), 2))
        if self.alt_hold_thread and self.alt_hold_thread.is_alive():
            print("[ALT-CTRL] already running"); return
        self.alt_hold_thread = threading.Thread(target=self._alt_hold_loop, daemon=True)
        self.alt_hold_thread.start()
        print(f"[ALT-CTRL] started, target={self.alt_tgt.get():.2f} m, rc3<= {THR_SAFE_MAX}")

    def stop_alt_hold_ctrl(self):
        if not self.alt_hold_enabled: return
        self.alt_hold_enabled = False
        rc3_hover, _ = self._hover_rc()
        self.thr.set(rc3_hover)
        print("[ALT-CTRL] stopped")

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
            self._set_thr(int(rc3))
            now = time.time()
            if now - last_print >= ALT_CTRL_PRINT_PERIOD:
                print(f"[ALT-CTRL] alt={float(alt):.2f} m  tgt={tgt:.2f} m  err={err:.2f} m  rc3={int(self.thr.get())}")
                last_print = now
            time.sleep(ALT_CTRL_DT)

    # ---------- PS4 / RC TX loop ----------
    def _send_loop(self):
        per = 1.0 / SEND_HZ
        while self.running:
            if self.js and not self._e_stop_active:
                try:
                    pygame.event.pump()
                    axis_roll  = self.js.get_axis(AXIS_ROLL)
                    axis_pitch = self.js.get_axis(AXIS_PITCH)
                    axis_yaw   = self.js.get_axis(AXIS_YAW)
                    axis_thr   = self.js.get_axis(AXIS_THR)
                    if ROLL_INVERT:  axis_roll  = -axis_roll
                    if PITCH_INVERT: axis_pitch = -axis_pitch
                    if YAW_INVERT:   axis_yaw   = -axis_yaw
                    if THR_INVERT:   axis_thr   = -axis_thr

                    axis_roll  = apply_deadzone(axis_roll)
                    axis_pitch = apply_deadzone(axis_pitch)
                    axis_yaw   = apply_deadzone(axis_yaw)

                    r = int(RC_MID + axis_roll  * 500)
                    p = int(RC_MID + axis_pitch * 500)
                    y = int(RC_MID + axis_yaw   * 500)
                    t = RC_MIN if axis_thr <= 0 else int(RC_MIN + axis_thr * (RC_MAX - RC_MIN))

                    if (not self._is_hold_context()) and not (self.yolo_enabled and self.vs_thr_comp_en.get()):
                        self.thr.set(clamp(t, RC_MIN, RC_MAX))

                    if not self.yolo_enabled:
                        self.yaw.set(clamp(y, RC_MIN, RC_MAX))

                    if not (self.yolo_enabled and self.vs_pitch_en.get()):
                        self.pitch.set(clamp(p, RC_MIN, RC_MAX))

                    self.roll.set(clamp(r, RC_MIN, RC_MAX))

                    b0 = 1 if self.js.get_button(0) else 0
                    b1 = 1 if self.js.get_button(1) else 0
                    b3 = 1 if self.js.get_button(3) else 0

                    if b0 and not self._js_prev[0]:
                        self.force_arm(); self._blink_button(self.btn_arm, BTN_GREEN)
                    if b1 and not self._js_prev[1]:
                        self.disarm(); self._blink_button(self.btn_disarm, BTN_RED)
                    if b3 and not self._js_prev[3]:
                        self._blink_button(self.btn_estop, BTN_RED)
                        self.emergency_stop_async()

                    self._js_prev[0] = b0
                    self._js_prev[1] = b1
                    self._js_prev[3] = b3

                except Exception as e:
                    print("[PS4] joystick error:", e); self.js = None

            self._send_override()
            time.sleep(per)

    # ---------- YOLO + (optional) ByteTrack ----------
    def start_yolo(self):
        if not YOLO_OK:
            messagebox.showwarning("YOLO", "YOLO/Camera not available"); return
        if self.yolo_enabled: return
        self.yolo_enabled = True
        self.yolo_status.set("YOLO: running (latest-frame)")
        threading.Thread(target=self._yolo_loop_lowlat, daemon=True).start()

    def stop_yolo(self):
        if not self.yolo_enabled: return
        self.yolo_enabled = False
        self.yolo_status.set("YOLO: stopped")
        self.yaw.set(RC_MID)
        self._vs_pitch_rc = int(self._vs_pitch_rc + 0.7*(RC_MID - self._vs_pitch_rc))
        self.pitch.set(self._vs_pitch_rc)

    def _yolo_loop_lowlat(self):
        # Load model (CUDA/Half if available)
        try:
            model = _YOLO(YOLO_MODEL_NAME)
        except Exception as e:
            print("YOLO load failed:", e); self.yolo_status.set("YOLO: load failed"); self.yolo_enabled=False; return
        imgsz = int(CLI.imgsz)
        try:
            import torch
            if torch.cuda.is_available():
                model.to("cuda")
                try:
                    model.model.half()
                except Exception:
                    pass
        except Exception:
            pass

        # Camera: latest-frame reader
        cam = LatestFrameCamera(YOLO_CAM_INDEX, CLI.cam_w, CLI.cam_h, CLI.cam_fps)
        if not cam.open():
            print("No camera found"); self.yolo_status.set("YOLO: no camera"); self.yolo_enabled=False; return

        # Optional ByteTrack
        use_bt = (BT_OK and not CLI.no_bt)
        if use_bt:
            fps = float(CLI.cam_fps)
            bt_args = SimpleNamespace(track_thresh=BT_TRACK_THRESH, match_thresh=BT_MATCH_THRESH,
                                      track_buffer=BT_TRACK_BUFFER, frame_rate=fps, mot20=False)
            tracker = BYTETracker(bt_args)
        else:
            tracker = None

        self._trk_id = None
        self._trk_last_seen = 0.0
        self._dx_lp = 0.0
        self._dx_prev = 0.0
        self._last_loop_ts = time.time()

        print("[YOLO] low-latency: latest-frame, buffersize=1, MJPG, detection imgsz={}, downscale={}".format(imgsz, DET_DOWNSCALE))
        while self.running and self.yolo_enabled:
            frame = cam.get_latest()
            if frame is None:
                time.sleep(0.001); continue

            h, w = frame.shape[:2]
            cx_ref = w * 0.5
            rc4_to_send = None
            rc2_to_send = None
            sel_bbox = None
            src = None
            area_frac = None
            target_dx = None

            try:
                # Pre-inference downscale for RPi
                if DET_DOWNSCALE and 0.2 <= DET_DOWNSCALE < 1.0:
                    dw, dh = int(w * DET_DOWNSCALE), int(h * DET_DOWNSCALE)
                    infer_frame = cv2.resize(frame, (dw, dh), interpolation=cv2.INTER_AREA)
                    scale_x, scale_y = w / float(dw), h / float(dh)
                else:
                    infer_frame = frame
                    scale_x = scale_y = 1.0

                results = model(infer_frame, imgsz=imgsz, conf=YOLO_CONF, iou=YOLO_IOU, verbose=False)

                det_list = []
                for r in results:
                    if not hasattr(r, "boxes"): continue
                    for b in r.boxes:
                        cls = int(b.cls[0].item())
                        if cls != YOLO_PERSON_CLS: continue
                        x1, y1, x2, y2 = map(float, b.xyxy[0])
                        x1, x2 = x1 * scale_x, x2 * scale_x
                        y1, y2 = y1 * scale_y, y2 * scale_y
                        conf = float(b.conf[0].item()) if hasattr(b, "conf") else 0.0
                        det_list.append([x1, y1, x2, y2, conf, cls])

                if use_bt:
                    import torch as _torch
                    dets = _torch.tensor(det_list, dtype=_torch.float32) if det_list else _torch.empty((0,6), dtype=_torch.float32)
                    tracks = tracker.update(dets, (h, w), (h, w))
                    candidates = []
                    for t in tracks:
                        conf_t = float(getattr(t, "score", 0.0))
                        if conf_t < DRAW_CONF_MIN:
                            continue
                        x1, y1, x2, y2 = t.tlbr
                        cx = 0.5 * (x1 + x2)
                        dx = cx - cx_ref
                        area = ((x2 - x1) * (y2 - y1)) / float(max(1.0, w * h))
                        tid = int(t.track_id)
                        center_score = 1.0 - min(1.0, abs(dx) / (w * 0.5))
                        area_score = min(1.0, area / max(1e-6, VS_TGT_AREA_FRAC))
                        score = center_score + 0.5 * conf_t + 0.3 * area_score
                        if self._trk_id is not None and tid == self._trk_id:
                            score += 0.35
                        candidates.append((score, tid, dx, (float(x1), float(y1), float(x2), float(y2)), area, conf_t))
                    if candidates:
                        best = max(candidates, key=lambda c: c[0])
                        _, cand_id, cand_dx, cand_bbox, cand_area, _ = best
                        now = time.time()
                        if (self._trk_id is None) or (cand_id == self._trk_id) or ((now - self._trk_last_seen) > 1.2):
                            self._trk_id = cand_id
                        if cand_id == self._trk_id:
                            target_dx = cand_dx
                            sel_bbox = cand_bbox
                            area_frac = cand_area
                            src = "track"
                            self._trk_last_seen = now
                else:
                    # detection-only (choose best centered/high-conf)
                    # detection-only (choose best centered / high-confidence)
                    if det_list:
                        def _center_conf_score(d):
                            # d = [x1, y1, x2, y2, conf, cls]
                            center_x = 0.5 * (d[0] + d[2])
                            # ציון = אמון * עד כמה קרוב למרכז (מנורמל ל-0..1)
                            return d[4] * (1.0 - min(1.0, abs(center_x - cx_ref) / (w * 0.5)))

                        pick = max(det_list, key=_center_conf_score)
                        x1, y1, x2, y2, conf, _ = pick
                        dx = 0.5 * (x1 + x2) - cx_ref
                        sel_bbox = (x1, y1, x2, y2)
                        area_frac = ((x2 - x1) * (y2 - y1)) / float(max(1.0, w * h))
                        target_dx = dx
                        src = "det"

                # ------ Yaw controller ------
                now = time.time()
                dt = max(1e-3, now - self._last_loop_ts)
                self._last_loop_ts = now

                if target_dx is None:
                    if (now - self._trk_last_seen) > LOST_TIMEOUT_S:
                        self._trk_id = None
                    self._dx_lp = 0.0
                    self._dx_prev = 0.0
                    # lost-target scan
                    since = now - self._trk_last_seen
                    if since > LOST_TIMEOUT_S:
                        phase = (now % SCAN_PERIOD_S) / SCAN_PERIOD_S
                        sweep = math.sin(2.0 * math.pi * phase)
                        rc4_to_send = RC_MID + int(sweep * SCAN_AMPL_RC)
                        src = "scan"
                    else:
                        rc4_to_send = RC_MID
                else:
                    self._dx_lp += YAW_LP_ALPHA * (float(target_dx) - self._dx_lp)
                    d_err = (self._dx_lp - self._dx_prev) / dt
                    self._dx_prev = self._dx_lp

                    if abs(self._dx_lp) <= YAW_DB_PIX:
                        rc4_to_send = RC_MID
                    else:
                        err_norm = min(1.0, abs(self._dx_lp) / (w * 0.5))
                        kp_eff = float(self.kp_var.get()) * (0.85 + 0.30 * err_norm)
                        kd_eff = float(self.kd_var.get()) * (0.85 + 0.30 * err_norm)
                        e_norm = clamp(self._dx_lp / (w * 0.5), -1.0, 1.0)
                        d_norm = clamp(d_err     / (w * 0.5), -1.0, 1.0)
                        yaw_cmd = clamp(kp_eff * e_norm + kd_eff * d_norm, -1.0, 1.0)
                        rc4_to_send = int(RC_MID + yaw_cmd * YAW_MAX_RC_DELTA)

                    if (not self._is_hold_context()) and self.thr.get() < YOLO_THR_MIN:
                        self._set_thr(YOLO_THR_MIN)

                # ------ Pitch forward with soft-stop near NEAR ------
                if self.vs_pitch_en.get():
                    yaw_ok = float(self.yaw_ok_px_var.get())
                    far_sz = float(self.far_area_var.get())
                    near_sz = float(self.near_area_var.get())
                    if far_sz > near_sz:
                        far_sz, near_sz = near_sz, far_sz
                        self.far_area_var.set(far_sz); self.near_area_var.set(near_sz)

                    if sel_bbox is not None and (area_frac is not None) and abs(self._dx_lp) <= yaw_ok and (far_sz <= area_frac <= near_sz):
                        soft = clamp((near_sz - area_frac) / max(1e-6, (near_sz - VS_TGT_AREA_FRAC)), 0.0, 1.0) if near_sz > VS_TGT_AREA_FRAC else 1.0
                        rc2_raw = self._vs_pitch_from_area(area_frac)
                        rc2_to_send = int(RC_MID + soft * (rc2_raw - RC_MID))
                        if self.vs_thr_comp_en.get() and not self._is_hold_context():
                            base = max(self.thr.get(), YOLO_THR_MIN)
                            forward_strength = max(0.0, (rc2_to_send - RC_MID) / float(VS_MAX_RC_DELTA))
                            add = int(clamp(forward_strength * VS_THR_COMP_GAIN_US, 0, VS_THR_COMP_MAX_US))
                            self._set_thr(base + add)
                    else:
                        self._vs_pitch_rc = int(self._vs_pitch_rc + 0.35 * (RC_MID - self._vs_pitch_rc))
                        rc2_to_send = self._vs_pitch_rc
                else:
                    self._vs_pitch_rc = int(self._vs_pitch_rc + 0.2*(RC_MID - self._vs_pitch_rc))

                # ------ Apply RC (no drawing in lowlat unless preview) ------
                if rc4_to_send is not None:
                    self.yaw.set(clamp(rc4_to_send, RC_MIN, RC_MAX))
                if rc2_to_send is not None:
                    self.pitch.set(clamp(rc2_to_send, RC_MIN, RC_MAX))

                if YOLO_SHOW_WINDOW:
                    annotated = frame.copy()
                    if rc4_to_send is not None:
                        cv2.putText(annotated, f"CH4={rc4_to_send} src={src or '-'}",(10,28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,200,255), 2)
                    if rc2_to_send is not None:
                        ttxt = f"CH2={rc2_to_send}"
                        if area_frac is not None: ttxt += f" area={area_frac:.3f}"
                        cv2.putText(annotated, ttxt,(10,56), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,180,60), 2)
                    cv2.imshow("YOLO (low-latency)", annotated)
                    k = cv2.waitKey(1) & 0xFF
                    if k in (27, ord('q')):
                        self.stop_yolo(); break

            except Exception as e:
                print("[YOLO] error:", e)

        try:
            cam.release()
            if YOLO_SHOW_WINDOW: cv2.destroyAllWindows()
        except Exception:
            pass
        print("[YOLO] stopped")

    def _vs_pitch_from_area(self, area_frac: float) -> int:
        err = VS_TGT_AREA_FRAC - max(0.0, min(1.0, float(area_frac)))
        if abs(err) < VS_DB_AREA_FRAC:
            rc2 = RC_MID
        else:
            norm = clamp(err / max(VS_TGT_AREA_FRAC, 1e-6), -1.0, 1.0)
            rc2 = int(RC_MID + VS_PITCH_FORWARD_SIGN * norm * VS_KP_PITCH * VS_MAX_RC_DELTA)
        rc2 = int(self._vs_pitch_rc + VS_LP_ALPHA * (rc2 - self._vs_pitch_rc))
        rc2 = clamp(rc2, RC_MIN, RC_MAX)
        self._vs_pitch_rc = rc2
        return rc2

    # ---------- GUIDED ----------
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

    # ---------- Emergency Stop (async) ----------
    def emergency_stop_async(self):
        if self._e_stop_active:
            print("[E-STOP] already active; ignoring new request.")
            return
        threading.Thread(target=self._emergency_stop_worker, daemon=True).start()

    def _emergency_stop_worker(self):
        with self._e_stop_lock:
            print("[E-STOP] Triggered.")
            self._e_stop_active = True
            try: self.stop_yolo()
            except Exception: pass
            try: self.stop_alt_hold_ctrl()
            except Exception: pass
            try: self.stop_guided_hold()
            except Exception: pass
            self._alt_hold_helper_active = False

            t0 = time.time()
            while time.time() - t0 < 0.5:
                self._send_rc_neutral_minthr()
                time.sleep(0.05)

            try: self.set_mode("BRAKE")
            except Exception:
                try: self.set_mode("LOITER")
                except Exception: pass

            if DO_FLIGHT_TERMINATION_ON_ESTOP:
                def _do_termination_on():
                    self.m.mav.command_long_send(
                        self.m.target_system, self.m.target_component,
                        CMD_FLIGHT_TERMINATION, 0, 1, 0,0,0,0,0,0
                    )
                self._try_cmd_many(_do_termination_on, tries=3, delay=0.12, label="FLIGHT_TERM_ON")

            def _do_disarm_force():
                self.m.mav.command_long_send(
                    self.m.target_system, self.m.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 21196, 0,0,0,0,0
                )
            self._try_cmd_many(_do_disarm_force, tries=4, delay=0.12, label="DISARM_FORCE")

            t_end = time.time() + E_STOP_HOLD_S
            while self.running and time.time() < t_end:
                self._send_rc_neutral_minthr()
                time.sleep(0.05)

            self._clear_flight_termination()
            try: self.set_mode("STABILIZE")
            except Exception: pass
            self._arm_inhibit_until = 0.0
            self._e_stop_active = False
            print(f"[E-STOP] Done. Sticks held for {E_STOP_HOLD_S:.1f}s. You can ARM now (X).")

    # ---------- ARM/DISARM/Reset/Close ----------
    def force_arm(self):
        self._clear_flight_termination()
        if self._e_stop_active or time.time() < self._arm_inhibit_until:
            print("[FORCE ARM] Inhibited (E-STOP/cooldown).")
            return
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
            try: self.stop_yolo()
            except Exception: pass
            try: self.stop_alt_hold_ctrl()
            except Exception: pass
            try: self.stop_guided_hold()
            except Exception: pass
            time.sleep(0.05)
            self.root.destroy()

# ---------- Main ----------
def main():
    root = tk.Tk()
    FakeSticks(root)
    root.mainloop()

if __name__ == "__main__":
    main()
