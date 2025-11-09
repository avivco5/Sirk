#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ASCII-only, english comments

Hybrid GUI Trackerv2 (merged 2-16):
- YOLOv8 detection
- DeepSORT (Kalman + ReID) for robust identity memory and motion prediction
- PD yaw controller with gain scheduling + deadband + low-pass on dx
- Forward-from-size pitch with soft-stop near NEAR bound
- Gentle throttle compensation when moving forward
- Lost-target scan (sin sweep)
- Alt-Hold helper (RC pulses) + Baro Alt-Hold closed loop
- GUIDED "hold here" keepalive (relative alt) + +/- bumps + Go-To GPS target
- Emergency Stop routine (neutral sticks, BRAKE fallback, LOITER)
- Compact Tkinter UI + PS4 joystick mapping
- Optional OpenCV preview and/or MJPEG web server

New merged features (items 2-16):
2) JSON load/save runtime profile + buttons
3) Keyboard hotkeys for RC nudges and ESC to stop YOLO
4) Rotating log to file (hybrid.log)
5) Battery low beeper via CH5
6) Auto-reconnect MAVLink on drop
7) Health panel basics (GPS fix/sats)
8) FPS limiter when window hidden
9) Telemetry CSV recorder toggle
10) Safe-arm gate (modes + cooldown)
11) GUI toggle for auto throttle minimum while tracking
12) Camera index fallback scan
13) Live class filter entry
14) UDP publish of target bbox
15) Safe window close if ARMED (confirmation)
16) CLI presets (rpi-lite, laptop-cpu, cam-hires)

Notes:
- Keep comments ascii-only.
- This script depends on: ultralytics, opencv-python, deep-sort-realtime, pygame, pymavlink, flask (optional).
"""

import sys, glob
from serial.tools import list_ports
import math, time, threading, queue, re, os
import argparse
import tkinter as tk
from tkinter import ttk, messagebox
from tkinter import font as tkfont
from tkinter import simpledialog
import warnings
import json, socket, subprocess  # for UDP control and external process
import pathlib
import logging, logging.handlers

warnings.filterwarnings("ignore", category=FutureWarning)

# --- simple ENV getters (ascii-only) ---
def _get_env_str(name, default=None):
    try:
        v = os.environ.get(str(name), None)
        return str(v) if v is not None else default
    except Exception:
        return default

def _get_env_int(name, default=None):
    try:
        v = os.environ.get(str(name), None)
        return int(v) if v is not None else default
    except Exception:
        return default

# map/ctrl defaults
MAP_UDP_IP_DEFAULT = "127.0.0.1"

# Numpy import and alias patch
import numpy as np
if not hasattr(np, "float"): np.float = float
if not hasattr(np, "int"):   np.int   = int
if not hasattr(np, "bool"):  np.bool  = bool

try:
    import cv2
except Exception as e:
    cv2 = None
    print("Warning: OpenCV unavailable:", e)

try:
    from ultralytics import YOLO as _YOLO
except Exception as e:
    _YOLO = None
    print("Warning: YOLO unavailable:", e)

try:
    from deep_sort_realtime.deepsort_tracker import DeepSort
    DS_OK = True
except Exception as e:
    DS_OK = False
    print("Warning: DeepSORT unavailable:", e)

try:
    import pygame
except Exception as e:
    pygame = None
    print("Warning: pygame unavailable:", e)

from pymavlink import mavutil

# ---------------- Logger (item 4) ----------------
def _init_logger():
    logger = logging.getLogger("hybrid")
    if logger.handlers:
        return logger
    logger.setLevel(logging.INFO)
    fh = logging.handlers.RotatingFileHandler("hybrid.log", maxBytes=2_000_000, backupCount=3)
    fh.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    sh = logging.StreamHandler(sys.stdout)
    sh.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    logger.addHandler(fh)
    logger.addHandler(sh)
    return logger

LOGGER = _init_logger()

# ---------------- Compact UI Settings ----------------
UI_SCALE = 0.90
METRIC_FONT_SIZE = 24
PADDING = 6
INNER_PAD = 4
LABEL_WIDTH = 12
VALUE_WIDTH = 4
GEOMETRY = "1120x900"

# ---------------- Serial auto-detect ----------------
if sys.platform.startswith("linux"):
    cands = glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")
    DEVICE = cands[0] if cands else "/dev/ttyACM0"
    BAUD = 115200
else:
    DEVICE = None
    for p in list_ports.comports():
        name = (p.description or "").lower()
        if "mavlink" in name:
            DEVICE = p.device
            print(f"[AUTO] Found MAVLink device: {p.device} ({p.description})")
            break
    if DEVICE is None:
        print("[WARN] No MAVLink port found, defaulting to COM4")
        DEVICE = "COM4"
    BAUD = 115200

# ---------------- Joystick Mapping ----------------
LINUX_AXIS_ROLL  = 3
LINUX_AXIS_PITCH = 4  # inverted
LINUX_AXIS_YAW   = 0
LINUX_AXIS_THR   = 1  # inverted
WIN_AXIS_ROLL  = 2
WIN_AXIS_PITCH = 3  # inverted
WIN_AXIS_YAW   = 0
WIN_AXIS_THR   = 1  # inverted

# --- Tracking env defaults ---
TRACK_MODE     = (_get_env_str("TRACK_MODE", "udp") or "udp").lower()  # "udp", "external", "local"
TRACK_UDP_IP   = _get_env_str("TRACK_UDP_IP", MAP_UDP_IP_DEFAULT)
TRACK_UDP_PORT = _get_env_int("TRACK_UDP_PORT", 9003)
TRACK_CMD      = _get_env_str("TRACK_CMD", "python hybrid_tracker.py --no-window")

if sys.platform.startswith("linux"):
    AXIS_ROLL, AXIS_PITCH, AXIS_YAW, AXIS_THR = LINUX_AXIS_ROLL, LINUX_AXIS_PITCH, LINUX_AXIS_YAW, LINUX_AXIS_THR
else:
    AXIS_ROLL, AXIS_PITCH, AXIS_YAW, AXIS_THR = WIN_AXIS_ROLL, WIN_AXIS_PITCH, WIN_AXIS_YAW, WIN_AXIS_THR

# Invert flags
PITCH_INVERT = True
THR_INVERT   = True
ROLL_INVERT  = False
YAW_INVERT   = False

# ---------------- RC/Control ----------------
RC_MIN, RC_MAX, RC_MID = 1000, 2000, 1500
SEND_HZ   = 20.0
DEADZONE  = 0.10

# ---------------- SAFE throttle for altitude hold ----------------
THR_SAFE_MAX = 1500
THR_SLEW_US_PER_SEC = 600

# ---------------- YOLO / camera ----------------
YOLO_OK = _YOLO is not None and cv2 is not None

YOLO_DB_PIX      = 40
YOLO_THR_MIN     = 1150
YOLO_SHOW_WINDOW = True
YOLO_CAM_INDEX   = 0
YOLO_MODEL_NAME  = "yolov8n.pt"
YOLO_CONF        = 0.20
YOLO_IOU         = 0.50
DRAW_CONF_MIN    = 0.60

# Inference downscale for RPi (map back boxes to original size)
DET_DOWNSCALE    = 0.67

# --------- Class filter (CLI override) ----------
TARGET_CLASSES   = ["person"]
TARGET_CLASS_IDS = None

# ---------------- DeepSORT tuning ----------------
DS_MAX_AGE       = 45
DS_N_INIT        = 3
DS_MAX_IOU       = 0.7
DS_NN_BUDGET     = 100
DS_EMBEDDER      = "mobilenet"
DS_HALF          = True

# ---------------- Lost-target scan ----------------
LOST_TIMEOUT_S   = 1.0
SCAN_PERIOD_S    = 3.5
SCAN_AMPL_RC     = 140

# ---------------- Altitude handling ----------------
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

# ---------------- Visual forward (Pitch from object size) ----------------
VS_TGT_AREA_FRAC   = 0.025
VS_DB_AREA_FRAC    = 0.004
VS_KP_PITCH        = 0.70
VS_MAX_RC_DELTA    = 220
VS_LP_ALPHA        = 0.18
VS_PITCH_FORWARD_SIGN = -1  # negative: move forward when area is small

# Optional small throttle compensation (gentle)
VS_THR_COMP_GAIN_US = 60
VS_THR_COMP_MAX_US  = 100

# ---------------- Emergency Stop ----------------
E_STOP_HOLD_S = 3.0
CMD_FLIGHT_TERMINATION = getattr(mavutil.mavlink, "MAV_CMD_DO_FLIGHTTERMINATION", 185)
DO_FLIGHT_TERMINATION_ON_ESTOP = True

# ---------------- Colors ----------------
BTN_GREEN = "#16a34a"
BTN_RED   = "#b91c1c"

# ---------------- Tracking control defaults ----------------
YAW_DB_PIX                  = 30
YAW_KP_DEFAULT              = 0.60
YAW_KD_DEFAULT              = 0.20
YAW_MAX_RC_DELTA            = 320
YAW_LP_ALPHA                = 0.30

VS_YAW_OK_PIX_DEFAULT       = 40
NEAR_STOP_AREA_FRAC_DEFAULT = 0.120
FAR_STOP_AREA_FRAC_DEFAULT  = 0.0015

# ---------------- Profiles (item 2) ----------------
PROFILE_PATH = pathlib.Path("hybrid_profile.json")

# ---------------- Recorder (item 9) ----------------
REC_PATH = pathlib.Path("telemetry.csv")

# ---------------- PUB UDP for bbox (item 14) ----------------
PUB_UDP_IP = _get_env_str("PUB_UDP_IP", "127.0.0.1")
PUB_UDP_PORT = _get_env_int("PUB_UDP_PORT", 9103)

# ---------------- FPS limiter when hidden (item 8) ----------------
MAX_FPS_NO_WINDOW = 15.0

# ---------------- Utility funcs ----------------
def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v
def apply_deadzone(val, dz=DEADZONE): return 0.0 if abs(val) < dz else val

def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    a1, b1 = math.radians(lat1), math.radians(lon1)
    a2, b2 = math.radians(lat2), math.radians(lon2)
    da, db = a2 - a1, b2 - b1
    h = math.sin(da/2)**2 + math.cos(a1)*math.cos(a2)*math.sin(db/2)**2
    return 2 * R * math.asin(math.sqrt(h))

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

def _norm_name(s):
    return re.sub(r"[^a-z0-9]", "", str(s).lower())

# ---- CLI helpers ----
def _normalize_classnames_arg(arg):
    # supports: ["person","car"] or ["person,car"] or IDs like ["0","2"]
    if arg is None: return None
    if len(arg) == 1 and ("," in arg[0]):
        return [tok.strip() for tok in arg[0].split(",") if tok.strip() != ""]
    return arg

def _resolve_class_ids(model, wanted):
    # wanted: list[str|int] or None -> returns set[int] or None
    if wanted is None:
        return None
    if isinstance(wanted, (list, tuple)) and len(wanted) == 0:
        return None
    id2name = {}
    try:
        names = model.names if hasattr(model, "names") else None
        if isinstance(names, dict): id2name = names
        elif names is not None:     id2name = {i: str(n) for i, n in enumerate(names)}
    except Exception:
        pass
    ids = set()
    for w in wanted:
        if isinstance(w, int) or str(w).isdigit():
            ids.add(int(w))
        else:
            wn = _norm_name(w)
            for i, nm in id2name.items():
                if _norm_name(nm) == wn:
                    ids.add(i)
    return ids if len(ids) > 0 else None

def _parse_args():
    p = argparse.ArgumentParser(description="Hybrid GUI Tracker (YOLOv8 + DeepSORT)")
    p.add_argument("--tracker", choices=["deepsort", "bytetrack"], default="deepsort",
                   help="tracking backend (bytetrack falls back to deepsort in this build)")
    p.add_argument("--classnames", nargs="+", default=None,
                   help="target classes by names or IDs (e.g. person car or 0 2).")
    p.add_argument("--yolo-model", default="yolov8n.pt", help="YOLOv8 model path/name")
    p.add_argument("--det-downscale", type=float, default=0.67, help="inference downscale (0.2..1.0)")
    p.add_argument("--conf", type=float, default=0.20, help="YOLO confidence threshold")
    p.add_argument("--iou", type=float, default=0.50, help="YOLO NMS IoU threshold")
    p.add_argument("--cam-index", type=int, default=0, help="OpenCV camera index")
    p.add_argument("--no-window", action="store_true", help="disable OpenCV preview window")
    # item 16: presets
    p.add_argument("--preset", choices=["rpi-lite","laptop-cpu","cam-hires"], default=None,
                   help="quick preset for conf/iou/downscale")
    return p.parse_args()

# ================= CPU-only DeepSORT helper =================
def _make_deepsort_cpu():
    """
    Create a DeepSort instance that never touches CUDA, regardless of local GPU.
    """
    try:
        import torch
        os.environ["CUDA_VISIBLE_DEVICES"] = ""
        try:
            torch.backends.cudnn.enabled = False
        except Exception:
            pass
        try:
            torch.cuda.is_available = lambda: False
        except Exception:
            pass
    except Exception:
        pass

    from deep_sort_realtime.deepsort_tracker import DeepSort

    try:
        return DeepSort(
            max_age=DS_MAX_AGE,
            n_init=DS_N_INIT,
            max_iou_distance=DS_MAX_IOU,
            nn_budget=DS_NN_BUDGET,
            nms_max_overlap=1.0,
            embedder=DS_EMBEDDER,
            half=False,
            embedder_gpu=False,
            bgr=True
        )
    except TypeError:
        try:
            return DeepSort(
                max_age=DS_MAX_AGE,
                n_init=DS_N_INIT,
                max_iou_distance=DS_MAX_IOU,
                nn_budget=DS_NN_BUDGET,
                nms_max_overlap=1.0,
                embedder=DS_EMBEDDER,
                half=False,
                use_cuda=False,
                bgr=True
            )
        except TypeError:
            return DeepSort(
                max_age=DS_MAX_AGE,
                n_init=DS_N_INIT,
                max_iou_distance=DS_MAX_IOU,
                nn_budget=DS_NN_BUDGET,
                nms_max_overlap=1.0,
                embedder=DS_EMBEDDER,
                half=False,
                cpu=True,
                bgr=True
            )

# ================= MJPEG Web Server (optional) =================
class MJPEGServer:
    def __init__(self, host="0.0.0.0", port=5000):
        self.host, self.port = host, port
        self.frame_q = queue.Queue(maxsize=2)
        self._srv_thread = None

    def start(self):
        try:
            from flask import Flask, Response
        except Exception as e:
            print("Flask not installed. Web disabled:", e)
            return
        app = Flask(__name__)

        @app.route("/")
        def index():
            return "<html><body><h3>YOLOv8 + DeepSORT stream</h3><img src='/video' /></body></html>"

        def gen():
            while True:
                jpg = self.frame_q.get()
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")

        @app.route("/video")
        def video():
            return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

        def run():
            app.run(host=self.host, port=self.port, debug=False, threaded=True, use_reloader=False)

        self._srv_thread = threading.Thread(target=run, daemon=True)
        self._srv_thread.start()
        print(f"[WEB] MJPEG on http://{self.host}:{self.port}")

    def push(self, bgr):
        if cv2 is None: return
        ok, jpg = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ok:
            return
        if self.frame_q.full():
            try: self.frame_q.get_nowait()
            except: pass
        self.frame_q.put(jpg.tobytes())

# ================= Main GUI class =================
class FakeSticks:
    def __init__(self, root):
        self.root = root
        self.root.title(f"Hybrid Tracker (YOLOv8 + DeepSORT)  [{DEVICE} @{BAUD}]")

        # Apply compact UI scaling and fonts
        self._apply_compact_style()

        # Remove stale lock on Linux before opening the serial
        if sys.platform.startswith("linux"):
            try:
                dev_name = DEVICE.split("/")[-1]
                lockfile = f"/var/lock/LCK..{dev_name}"
                if os.path.exists(lockfile):
                    print(f"[INFO] Removing stale lock: {lockfile}")
                    os.remove(lockfile)
            except Exception as e:
                print("[WARN] Could not remove stale lock:", e)

        # MAVLink connect
        try:
            self.m = mavutil.mavlink_connection(DEVICE, baud=BAUD)
            self.m.wait_heartbeat(timeout=5)
            ttk.Label(root, text=f"Connected: sys {self.m.target_system}, comp {self.m.target_component}").pack()

            # Tracking control state and UDP control socket
            self.tracking_active = tk.BooleanVar(value=False)
            self.track_status = tk.StringVar(value="Tracking: idle")

            # separate UDP control channel (for external tracker listener)
            try:
                self._ctrl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self._ctrl_dst = (TRACK_UDP_IP, int(TRACK_UDP_PORT))
            except Exception as e:
                self._ctrl_sock = None
                self._ctrl_dst = None
                print("[TRACK-UDP] socket init failed:", e)

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
        if pygame is not None:
            try:
                pygame.init()
                pygame.joystick.init()
                if pygame.joystick.get_count() > 0:
                    self.js = pygame.joystick.Joystick(0); self.js.init()
                    print(f"[PS4] Connected: {self.js.get_name()}")
                else:
                    self.js = None; print("[PS4] No joystick detected")
            except Exception as e:
                self.js = None
                print("[PS4] init error:", e)
        else:
            self.js = None

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

        # Health (item 7)
        self.t_health = tk.StringVar(value="--")

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
        self.alt_tgt = tk.DoubleVar(value=0.0)
        self.alt_err = tk.DoubleVar(value=0.0)

        # GUIDED keepalive state
        self._guided_active = False
        self._guided_target = None  # (lat, lon, alt_rel)
        self._guided_lock   = threading.Lock()

        # Alt-hold helper active flag
        self._alt_hold_helper_active = False

        # Throttle smoothing state
        self._thr_last = RC_MIN
        self._thr_last_ts = time.time()

        # Visual forward state/toggles
        self._vs_pitch_rc = RC_MID
        self.vs_pitch_en    = tk.BooleanVar(value=True)
        self.vs_thr_comp_en = tk.BooleanVar(value=True)  # enable gentle throttle comp by default

        # Tracking / control state
        self._trk_id = None
        self._trk_last_seen = 0.0
        self._dx_lp = 0.0
        self._dx_prev = 0.0
        self._last_loop_ts = time.time()

        # Emergency Stop / ARM state
        self._e_stop_active = False
        self._e_stop_lock = threading.Lock()
        self.is_armed = False
        self._arm_inhibit_until = 0.0

        # ---- GUI: RC sliders ----
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

        # Health panel (item 7)
        hlf = ttk.LabelFrame(root, text="Health", padding=PADDING); hlf.pack(fill="x", padx=PADDING, pady=INNER_PAD)
        self._mk_metric(hlf, "Status", self.t_health, font_size=18)

        ly = ttk.LabelFrame(root, text="YOLO + DeepSORT (Yaw + Forward)", padding=PADDING); ly.pack(fill="x", padx=PADDING, pady=INNER_PAD)
        self.yolo_enabled = False
        ready_txt = "YOLO: ready" if YOLO_OK else "YOLO: unavailable"
        if not DS_OK: ready_txt += " (DeepSORT unavailable)"
        self.yolo_status = tk.StringVar(value=ready_txt)
        ttk.Label(ly, textvariable=self.yolo_status).pack(side="left", padx=INNER_PAD)
        ttk.Button(ly, text="Start", command=self.start_yolo).pack(side="left", padx=INNER_PAD)
        ttk.Button(ly, text="Stop",  command=self.stop_yolo).pack(side="left", padx=INNER_PAD)
        ttk.Checkbutton(ly, text="Forward from size (Pitch)", variable=self.vs_pitch_en).pack(side="left", padx=INNER_PAD)
        ttk.Checkbutton(ly, text="Throttle comp", variable=self.vs_thr_comp_en).pack(side="left", padx=INNER_PAD)
        # item 11: auto throttle min toggle
        self.auto_thr_min = tk.BooleanVar(value=True)
        ttk.Checkbutton(ly, text="Auto throttle min", variable=self.auto_thr_min).pack(side="left", padx=INNER_PAD)

        # item 13: live class filter entry
        frm_cls = ttk.Frame(ly); frm_cls.pack(side="left", padx=INNER_PAD)
        ttk.Label(frm_cls, text="Classes:").pack(side="left")
        self.cls_entry = ttk.Entry(frm_cls, width=16)
        self.cls_entry.insert(0, ",".join(TARGET_CLASSES) if TARGET_CLASSES else "")
        self.cls_entry.pack(side="left")
        ttk.Button(frm_cls, text="Apply", command=self._apply_classes_live).pack(side="left", padx=INNER_PAD)

        # Tracking control (external/local)
        trk = ttk.LabelFrame(root, text="Tracking control", padding=PADDING)
        trk.pack(fill="x", padx=PADDING, pady=INNER_PAD)
        ttk.Label(trk, textvariable=self.track_status).pack(side="left", padx=INNER_PAD)
        ttk.Button(trk, text="Start Tracking", command=self.btn_start_tracking).pack(side="left", padx=INNER_PAD)
        ttk.Button(trk, text="Stop Tracking",  command=self.btn_stop_tracking ).pack(side="left", padx=INNER_PAD)

        # Live Tunables
        tun = ttk.LabelFrame(root, text="Tracking Tunables (live)", padding=PADDING); tun.pack(fill="x", padx=PADDING, pady=INNER_PAD)
        self.kp_var   = tk.DoubleVar(value=YAW_KP_DEFAULT)
        self.kd_var   = tk.DoubleVar(value=YAW_KD_DEFAULT)
        self.yaw_ok_px_var = tk.DoubleVar(value=VS_YAW_OK_PIX_DEFAULT)
        self.far_area_var  = tk.DoubleVar(value=FAR_STOP_AREA_FRAC_DEFAULT)
        self.near_area_var = tk.DoubleVar(value=NEAR_STOP_AREA_FRAC_DEFAULT)
        self._mk_tunable(tun, "YAW_KP",  self.kp_var,   0.0, 1.5, 0.01, "{:.2f}")
        self._mk_tunable(tun, "YAW_KD",  self.kd_var,   0.0, 0.8,  0.01, "{:.2f}")
        self._mk_tunable(tun, "VS_YAW_OK_PIX", self.yaw_ok_px_var, 0.0, 200.0, 1.0, "{:.0f}")
        row_bounds = ttk.Frame(tun); row_bounds.pack(fill="x", pady=INNER_PAD)
        self._mk_tunable(row_bounds, "FAR_STOP_AREA_FRAC",  self.far_area_var,  0.000, 0.020, 0.001, "{:.3f}")
        self._mk_tunable(row_bounds, "NEAR_STOP_AREA_FRAC", self.near_area_var, 0.020, 0.300, 0.001, "{:.3f}")
        ttk.Button(tun, text="Reset to defaults", command=self._reset_tunables).pack(side="right", padx=INNER_PAD)

        # item 2: profile buttons
        rowp = ttk.Frame(tun); rowp.pack(fill="x", pady=INNER_PAD)
        ttk.Button(rowp, text="Save Profile", command=self.save_profile).pack(side="left", padx=INNER_PAD)
        ttk.Button(rowp, text="Load Profile", command=self.load_profile).pack(side="left", padx=INNER_PAD)

        # Flight modes + GUIDED helpers
        mf = ttk.LabelFrame(root, text="Flight Modes", padding=PADDING)
        mf.pack(fill="x", padx=PADDING, pady=INNER_PAD)
        ttk.Button(mf, text="GUIDED",    command=lambda: self.set_mode("GUIDED")).pack(side="left", expand=True, fill="x", padx=INNER_PAD)
        ttk.Button(mf, text="STABILIZE", command=lambda: self.set_mode("STABILIZE")).pack(side="left", expand=True, fill="x", padx=INNER_PAD)
        ttk.Button(mf, text="Hold Here", command=self.hold_alt_guided_here).pack(side="left", expand=True, fill="x", padx=INNER_PAD)
        ttk.Button(mf, text="Go To…",    command=self.prompt_go_to).pack(side="left", expand=True, fill="x", padx=INNER_PAD)
        ttk.Button(mf, text="+1m",       command=lambda: self.bump_alt_guided(+1.0)).pack(side="left", expand=True, fill="x", padx=INNER_PAD)
        ttk.Button(mf, text="-1m",       command=lambda: self.bump_alt_guided(-1.0)).pack(side="left", expand=True, fill="x", padx=INNER_PAD)

        # Recorder toggle (item 9)
        recf = ttk.Frame(root, padding=PADDING); recf.pack(fill="x")
        self.rec_enabled = tk.BooleanVar(value=False)
        ttk.Checkbutton(recf, text="Record telemetry CSV", variable=self.rec_enabled).pack(side="left", padx=INNER_PAD)

        btns = ttk.Frame(root, padding=PADDING); btns.pack(fill="x")
        # ARMED/DISARMED indicator
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

        # Joystick button edges
        self._js_prev = {0:0, 1:0, 3:0}

        # PUB UDP (item 14)
        try:
            self._pub_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._pub_dst = (PUB_UDP_IP, int(PUB_UDP_PORT))
        except Exception as e:
            self._pub_sock = None; self._pub_dst = None
            print("[PUB-UDP] init failed:", e)

        # Threads
        self.running = True
        threading.Thread(target=self._send_loop,             daemon=True).start()
        threading.Thread(target=self._mav_telemetry,         daemon=True).start()
        threading.Thread(target=self._guided_keepalive_loop, daemon=True).start()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.geometry(GEOMETRY)

        # Hotkeys (item 3)
        self._bind_hotkeys()

        # Load profile (item 2)
        self.load_profile()

    # ---------- Profile save/load (item 2) ----------
    def save_profile(self):
        try:
            data = {
                "kp": float(self.kp_var.get()),
                "kd": float(self.kd_var.get()),
                "yaw_ok_px": float(self.yaw_ok_px_var.get()),
                "far_area": float(self.far_area_var.get()),
                "near_area": float(self.near_area_var.get()),
                "vs_pitch_en": bool(self.vs_pitch_en.get()),
                "vs_thr_comp_en": bool(self.vs_thr_comp_en.get()),
                "yolo_conf": float(YOLO_CONF),
                "yolo_iou": float(YOLO_IOU),
                "det_downscale": float(DET_DOWNSCALE),
                "cam_index": int(YOLO_CAM_INDEX),
                "auto_thr_min": bool(self.auto_thr_min.get())
            }
            PROFILE_PATH.write_text(json.dumps(data, indent=2))
            print("[PROFILE] saved:", PROFILE_PATH)
        except Exception as e:
            print("[PROFILE] save error:", e)

    def load_profile(self):
        try:
            if PROFILE_PATH.exists():
                data = json.loads(PROFILE_PATH.read_text())
                self.kp_var.set(float(data.get("kp", self.kp_var.get())))
                self.kd_var.set(float(data.get("kd", self.kd_var.get())))
                self.yaw_ok_px_var.set(float(data.get("yaw_ok_px", self.yaw_ok_px_var.get())))
                self.far_area_var.set(float(data.get("far_area", self.far_area_var.get())))
                self.near_area_var.set(float(data.get("near_area", self.near_area_var.get())))
                self.vs_pitch_en.set(bool(data.get("vs_pitch_en", True)))
                self.vs_thr_comp_en.set(bool(data.get("vs_thr_comp_en", True)))
                self.auto_thr_min.set(bool(data.get("auto_thr_min", True)))
                print("[PROFILE] loaded:", PROFILE_PATH)
        except Exception as e:
            print("[PROFILE] load error:", e)

    # ---------- Hotkeys (item 3) ----------
    def _bind_hotkeys(self):
        self.root.bind("<Escape>", lambda e: self.stop_yolo())
        def _nudge(var, delta):
            v = int(var.get()) + int(delta)
            var.set(clamp(v, RC_MIN, RC_MAX))
            self._send_override()
        self.root.bind("<Key-q>", lambda e: _nudge(self.yaw, -25))
        self.root.bind("<Key-e>", lambda e: _nudge(self.yaw, +25))
        self.root.bind("<Key-w>", lambda e: _nudge(self.pitch, -25))
        self.root.bind("<Key-s>", lambda e: _nudge(self.pitch, +25))
        self.root.bind("<Key-a>", lambda e: _nudge(self.roll, -25))
        self.root.bind("<Key-d>", lambda e: _nudge(self.roll, +25))

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
    def _blink_button(self, btn, color, duration_ms=250, fg="white"):
        try:
            bg0 = btn.cget("background")
            fg0 = btn.cget("foreground")
            btn.configure(background=color, foreground=fg)
            btn.update_idletasks()
            def _restore():
                try: btn.configure(background=bg0, foreground=fg0)
                except Exception: pass
            self.root.after(duration_ms, _restore)
        except Exception:
            pass

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

    # ---------- Compact style ----------
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

    # ---------- GUI helpers ----------
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
                    try: return float(p.param_value)
                    except Exception: return p.param_value
            print(f"[PARAM] timeout reading {target}")
        except Exception as e:
            print(f"[PARAM] read {name} failed:", e)
        return default

    def _hover_rc(self):
        hover = self.get_param("MOT_THST_HOVER", default=0.5)
        hover = clamp(hover if hover is not None else 0.5, 0.0, 1.0)
        rc3_hover = int(RC_MIN + hover * (RC_MAX - RC_MIN))
        return rc3_hover, hover

    # ---------- ALT HOLD (RC helper) ----------
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
            self._alt_hold_helper_active = True
            self._set_thr(rc_pulse)
            t_end = time.time()+dur
            while time.time()<t_end and self.running:
                self._send_override(); time.sleep(0.03)
            self._set_thr(rc3_hover)
            self._alt_hold_helper_active = False
            print(f"[ALT_BUMP] dh={delta_m:+.2f}m dur={dur:.2f}s rc3<= {THR_SAFE_MAX}")
        threading.Thread(target=_do, daemon=True).start()

    # ---------- Baro Alt-Hold Controller ----------
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

                    b0 = 1 if self.js.get_button(0) else 0  # X
                    b1 = 1 if self.js.get_button(1) else 0  # O
                    b3 = 1 if self.js.get_button(3) else 0  # Triangle

                    if b0 and not self._js_prev[0]:
                        self.force_arm()
                        self._blink_button(self.btn_arm, BTN_GREEN)

                    if b1 and not self._js_prev[1]:
                        self.disarm()
                        self._blink_button(self.btn_disarm, BTN_RED)

                    if b3 and not self._js_prev[3]:
                        self._blink_button(self.btn_estop, BTN_RED)
                        self.emergency_stop_async()

                    self._js_prev[0] = b0
                    self._js_prev[1] = b1
                    self._js_prev[3] = b3

                except Exception as e:
                    print("[PS4] joystick error:", e); self.js = None

            # item 5: battery guard beeper on CH5
            self._battery_guard()

            self._send_override()
            time.sleep(per)

    # ---------- Battery guard (item 5) ----------
    # Example thresholds for 6S; tune to your pack or compute from cells
    BATT_LOW_V = 3.55 * 6.0
    BATT_CRIT_V = 3.45 * 6.0

    def _battery_guard(self):
        try:
            vtxt = self.batt_v.get().split()[0]
            v = float(vtxt)
            if v <= self.BATT_CRIT_V:
                self.servo5.set(1900)  # continuous alarm
            elif v <= self.BATT_LOW_V:
                self.servo5.set(1700)  # slow beep
            else:
                self.servo5.set(1500)
        except Exception:
            pass

    # ---------- Telemetry RX with reconnect (items 6,7,9) ----------
    def _mav_reconnect(self):
        try:
            self.m.close()
        except Exception:
            pass
        for _ in range(3):
            try:
                self.m = mavutil.mavlink_connection(DEVICE, baud=BAUD)
                self.m.wait_heartbeat(timeout=5)
                print("[MAV] reconnected")
                return True
            except Exception as e:
                print("[MAV] reconnect failed:", e)
                time.sleep(1.0)
        return False

    def _record_line(self, d):
        try:
            new = not REC_PATH.exists()
            with REC_PATH.open("a", encoding="utf-8") as f:
                if new:
                    f.write(",".join(d.keys()) + "\n")
                f.write(",".join(str(d[k]) for k in d.keys()) + "\n")
        except Exception as e:
            print("[REC] write error:", e)

    def _mav_telemetry(self):
        while self.running:
            try:
                msg = self.m.recv_match(blocking=False)
            except Exception:
                if self._mav_reconnect():
                    continue
                time.sleep(0.5); continue
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

            if t == "GPS_RAW_INT":
                try:
                    fix = int(msg.fix_type)
                    sats = int(msg.satellites_visible)
                    text = f"GPS:fix={fix} sats={sats}"
                    self.t_health.set(text)
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

            # item 9: recorder (coalesce once per loop)
            if self.rec_enabled.get():
                try:
                    d = {
                        "ts": round(time.time(), 3),
                        "alt": self._alt_val if self._alt_val is not None else "",
                        "roll": self.t_roll.get(),
                        "pitch": self.t_pitch.get(),
                        "yaw": self.t_yaw.get(),
                        "gspeed": self.t_gspeed.get(),
                        "rc1": int(self.roll.get()),
                        "rc2": int(self.pitch.get()),
                        "rc3": int(self.thr.get()),
                        "rc4": int(self.yaw.get()),
                    }
                    self._record_line(d)
                except Exception:
                    pass

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

    # ---------- Visual forward helpers ----------
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

    # ---------- YOLO + DeepSORT ----------
    def start_yolo(self):
        if not YOLO_OK:
            messagebox.showwarning("YOLO", "YOLO/Camera not available"); return
        if not DS_OK:
            messagebox.showwarning("DeepSORT", "DeepSORT not available"); return
        if self.yolo_enabled: return
        self.yolo_enabled = True
        self.yolo_status.set("YOLO: running (DeepSORT + forward)")
        threading.Thread(target=self._yolo_loop, daemon=True).start()

    def stop_yolo(self):
        if not self.yolo_enabled: return
        self.yolo_enabled = False
        self.yolo_status.set("YOLO: stopped")
        self.yaw.set(RC_MID)
        self._vs_pitch_rc = int(self._vs_pitch_rc + 0.7*(RC_MID - self._vs_pitch_rc))
        self.pitch.set(self._vs_pitch_rc)

    # ---------- Tracking control helpers ----------
    def _send_track_cmd(self, cmd: str):
        if not self._ctrl_sock or not self._ctrl_dst:
            print("[TRACK-UDP] socket not ready")
            return
        try:
            payload = json.dumps({"cmd": str(cmd)}).encode("utf-8")
            self._ctrl_sock.sendto(payload, self._ctrl_dst)
            print(f"[TRACK-UDP] sent '{cmd}' -> {self._ctrl_dst}")
        except Exception as e:
            print("[TRACK-UDP] send failed:", e)

    def btn_start_tracking(self):
        if self.tracking_active.get():
            print("[TRACK] already active")
            return
        mode = (TRACK_MODE or "udp").lower()
        try:
            if mode == "udp":
                self._send_track_cmd("start_tracking")
                self.tracking_active.set(True)
                self.track_status.set(f"Tracking: active (UDP {TRACK_UDP_IP}:{TRACK_UDP_PORT})")
            elif mode == "external":
                self._track_proc = subprocess.Popen(TRACK_CMD, shell=True)
                self.tracking_active.set(True)
                pid = self._track_proc.pid if self._track_proc else None
                self.track_status.set(f"Tracking: active (external) pid={pid}")
                print(f"[TRACK-EXT] started: {TRACK_CMD} (pid={pid})")
            elif mode == "local":
                self.tracking_active.set(True)
                self.track_status.set("Tracking: active (local)")
                print("[TRACK-LOCAL] active")
            else:
                print(f"[TRACK] unknown TRACK_MODE='{TRACK_MODE}', fallback to UDP")
                self._send_track_cmd("start_tracking")
                self.tracking_active.set(True)
                self.track_status.set(f"Tracking: active (UDP {TRACK_UDP_IP}:{TRACK_UDP_PORT})")
        except Exception as e:
            print("[TRACK] start failed:", e)

    def btn_stop_tracking(self):
        mode = (TRACK_MODE or "udp").lower()
        try:
            if mode == "udp":
                self._send_track_cmd("stop_tracking")
            elif mode == "external":
                if hasattr(self, "_track_proc") and self._track_proc and (self._track_proc.poll() is None):
                    try:
                        self._track_proc.terminate()
                        print(f"[TRACK-EXT] terminated pid={self._track_proc.pid}")
                    except Exception as e:
                        print("[TRACK-EXT] terminate error:", e)
                self._track_proc = None
            elif mode == "local":
                print("[TRACK-LOCAL] stopped")
        except Exception as e:
            print("[TRACK] stop failed:", e)
        finally:
            self.tracking_active.set(False)
            self.track_status.set("Tracking: idle")

    def _open_camera(self, index):
        if sys.platform.startswith("win"):
            return cv2.VideoCapture(index, cv2.CAP_DSHOW)
        return cv2.VideoCapture(index)

    def _publish_bbox(self, bbox, conf, area, dx, src):
        if not self._pub_sock or not self._pub_dst: return
        try:
            x1,y1,x2,y2 = bbox
            payload = {
                "ts": time.time(),
                "bbox": [float(x1), float(y1), float(x2), float(y2)],
                "conf": float(conf) if conf is not None else None,
                "area_frac": float(area) if area is not None else None,
                "dx": float(dx) if dx is not None else None,
                "src": str(src or "")
            }
            self._pub_sock.sendto(json.dumps(payload).encode("utf-8"), self._pub_dst)
        except Exception as e:
            print("[PUB-UDP] send err:", e)

    def _yolo_loop(self):
        # Load YOLO
        try:
            model = _YOLO(YOLO_MODEL_NAME)
        except Exception as e:
            print("YOLO load failed:", e)
            self.yolo_status.set("YOLO: load failed")
            self.yolo_enabled = False
            return

        # Force CPU (avoid invalid GPU kernel)
        try:
            import torch
            os.environ["CUDA_VISIBLE_DEVICES"] = ""
            try:
                torch.backends.cudnn.enabled = False
            except Exception:
                pass
            model.to("cpu")
            try:
                if hasattr(model, "model"):
                    model.model.float()
            except Exception:
                pass
            print("[YOLO] using CPU (float32)")
        except Exception as e:
            print("[YOLO] CPU setup warning:", e)

        # Resolve target class IDs from global TARGET_CLASSES (or live)
        global TARGET_CLASS_IDS
        TARGET_CLASS_IDS = _resolve_class_ids(model, TARGET_CLASSES)
        try:
            names = model.names if hasattr(model, "names") else None
            if TARGET_CLASS_IDS is None:
                if TARGET_CLASSES is None:
                    print("[CLASSES] tracking ALL classes")
                else:
                    print(f"[CLASSES] requested {TARGET_CLASSES} -> none matched, tracking ALL classes")
            else:
                if isinstance(names, dict):
                    human = [f"{i}:{names.get(i, '?')}" for i in sorted(TARGET_CLASS_IDS)]
                else:
                    human = [str(i) for i in sorted(TARGET_CLASS_IDS)]
                print("[CLASSES] tracking only IDs:", ", ".join(human))
        except Exception:
            pass

        # Camera open with fallback (item 12)
        cam_indices = [YOLO_CAM_INDEX, 0, 1, 2]
        cap = None
        for idx in cam_indices:
            tmp = self._open_camera(idx)
            if tmp.isOpened():
                cap = tmp
                # update global for profile save
                globals()["YOLO_CAM_INDEX"] = idx
                print(f"[CAM] opened index {idx}")
                break
        if cap is None:
            print("No camera found"); self.yolo_status.set("YOLO: no camera"); self.yolo_enabled=False; return

        fps = cap.get(cv2.CAP_PROP_FPS)
        fps = float(fps) if fps and fps > 0 else 30.0

        # DeepSORT init on CPU only
        try:
            tracker = _make_deepsort_cpu()
            print("[DeepSORT] using CPU embedder")
        except Exception as e:
            print("DeepSORT init failed:", e)
            self.yolo_status.set("DeepSORT: init failed")
            self.yolo_enabled = False
            try:
                cap.release()
            except Exception:
                pass
            return

        self._trk_id = None
        self._trk_last_seen = 0.0
        self._dx_lp = 0.0
        self._dx_prev = 0.0
        self._last_loop_ts = time.time()

        print("[YOLO+DS] yaw PD + forward gating. ESC/Q closes preview.")
        last_print = time.time()

        while self.running and self.yolo_enabled:
            t0 = time.time()
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.01); continue

            h, w = frame.shape[:2]
            cx_ref = w * 0.5
            rc4_to_send = None
            rc2_to_send = None
            sel_bbox = None
            area_frac = None
            target_dx = None
            src = None

            try:
                # Downscale for inference (map back)
                if DET_DOWNSCALE and 0.2 <= DET_DOWNSCALE < 1.0:
                    dw, dh = int(w * DET_DOWNSCALE), int(h * DET_DOWNSCALE)
                    infer_frame = cv2.resize(frame, (dw, dh), interpolation=cv2.INTER_AREA)
                    scale_x, scale_y = w / float(dw), h / float(dh)
                else:
                    infer_frame = frame
                    scale_x = scale_y = 1.0

                results = model(infer_frame, conf=YOLO_CONF, iou=YOLO_IOU, verbose=False)

                # Build detections for DeepSORT: [ltrbwh, conf, cls]
                det_list = []
                id2name = {}
                try:
                    nm = results[0].names if hasattr(results[0], "names") else None
                    if isinstance(nm, dict): id2name = nm
                except Exception:
                    pass

                for r in results:
                    if not hasattr(r, "boxes"): continue
                    for b in r.boxes:
                        cls = int(b.cls[0].item())
                        if TARGET_CLASS_IDS is not None and cls not in TARGET_CLASS_IDS:
                            continue
                        x1, y1, x2, y2 = map(float, b.xyxy[0])
                        x1, x2 = x1 * scale_x, x2 * scale_x
                        y1, y2 = y1 * scale_y, y2 * scale_y
                        conf = float(b.conf[0].item()) if hasattr(b, "conf") else 0.0
                        ltrbwh = [x1, y1, x2 - x1, y2 - y1]
                        det_list.append([ltrbwh, conf, cls])

                # Update DeepSORT
                t2 = time.time()
                tracks = tracker.update_tracks(det_list, frame=frame)
                t_track = (time.time() - t2) * 1000.0

                annotated = frame.copy()
                if YOLO_SHOW_WINDOW:
                    cv2.line(annotated, (int(cx_ref), 0), (int(cx_ref), h), (255, 0, 0), 2)

                # Select target with persistence bias
                candidates = []  # (score, id, dx, bbox, area, conf)
                for trk in tracks:
                    if not trk.is_confirmed():
                        continue
                    l, t, r, b = trk.to_ltrb()
                    cx = 0.5 * (l + r)
                    dx = cx - cx_ref
                    area = ((r - l) * (b - t)) / float(max(1.0, w * h))
                    tid = int(trk.track_id)
                    cls_id = trk.get_det_class()
                    conf_t = 0.0
                    if hasattr(trk, "det_conf") and trk.det_conf is not None:
                        try: conf_t = float(trk.det_conf)
                        except Exception: pass
                    elif hasattr(trk, "score") and trk.score is not None:
                        try: conf_t = float(trk.score)
                        except Exception: pass

                    center_score = 1.0 - min(1.0, abs(dx) / (w * 0.5))
                    area_score = min(1.0, area / max(1e-6, VS_TGT_AREA_FRAC))
                    score = center_score + 0.5 * conf_t + 0.3 * area_score
                    if self._trk_id is not None and tid == self._trk_id:
                        score += 0.35  # persistence bias

                    candidates.append((score, tid, dx, (float(l), float(t), float(r), float(b)), area, conf_t))

                    if YOLO_SHOW_WINDOW:
                        color = _id_color(tid)
                        cv2.rectangle(annotated, (int(l), int(t)), (int(r), int(b)), color, 2)
                        label = f"ID {tid}"
                        if conf_t > 0: label += f" {conf_t:.2f}"
                        if cls_id is not None and isinstance(id2name, dict):
                            label = f"{id2name.get(cls_id, str(cls_id))} {label}"
                        cv2.putText(annotated, label, (int(l), max(0, int(t)-10)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

                now = time.time()
                if candidates:
                    best = max(candidates, key=lambda c: c[0])
                    _, cand_id, cand_dx, cand_bbox, cand_area, _ = best
                    if (self._trk_id is None) or (cand_id == self._trk_id) or ((now - self._trk_last_seen) > 1.5):
                        self._trk_id = cand_id
                    if cand_id == self._trk_id:
                        target_dx = cand_dx
                        sel_bbox = cand_bbox
                        area_frac = cand_area
                        src = "track"
                        self._trk_last_seen = now
                        if YOLO_SHOW_WINDOW:
                            x1, y1, x2, y2 = map(int, sel_bbox)
                            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 128, 255), 2)

                # Yaw controller: PD + gain scheduling + deadband
                dt = max(1e-3, now - self._last_loop_ts)
                self._last_loop_ts = now

                if target_dx is None:
                    if (now - self._trk_last_seen) > LOST_TIMEOUT_S:
                        self._trk_id = None
                    self._dx_lp = 0.0
                    self._dx_prev = 0.0
                    # gentle scan
                    if (now - self._trk_last_seen) > LOST_TIMEOUT_S:
                        phase = (now % SCAN_PERIOD_S) / SCAN_PERIOD_S  # 0..1
                        sweep = math.sin(2.0 * math.pi * phase)        # -1..+1
                        rc4_to_send = RC_MID + int(sweep * SCAN_AMPL_RC)
                        src = "scan"
                else:
                    kp = float(self.kp_var.get())
                    kd = float(self.kd_var.get())

                    self._dx_lp += YAW_LP_ALPHA * (float(target_dx) - self._dx_lp)
                    d_err = (self._dx_lp - self._dx_prev) / dt
                    self._dx_prev = self._dx_lp

                    if abs(self._dx_lp) <= YAW_DB_PIX:
                        rc4_to_send = RC_MID
                    else:
                        err_norm = min(1.0, abs(self._dx_lp) / (w * 0.5))
                        kp_eff = kp * (0.85 + 0.30 * err_norm)
                        kd_eff = kd * (0.85 + 0.30 * err_norm)

                        e_norm = clamp(self._dx_lp / (w * 0.5), -1.0, 1.0)
                        d_norm = clamp(d_err     / (w * 0.5), -1.0, 1.0)
                        yaw_cmd = clamp(kp_eff * e_norm + kd_eff * d_norm, -1.0, 1.0)
                        rc4_to_send = int(RC_MID + yaw_cmd * YAW_MAX_RC_DELTA)

                    # item 11: auto throttle minimum while tracking (only if not in hold)
                    if self.auto_thr_min.get() and (not self._is_hold_context()) and self.thr.get() < YOLO_THR_MIN:
                        self._set_thr(YOLO_THR_MIN)

                # Pitch forward with soft-stop near NEAR
                yaw_ok = float(self.yaw_ok_px_var.get())
                far_sz = float(self.far_area_var.get())
                near_sz = float(self.near_area_var.get())
                if far_sz > near_sz:
                    far_sz, near_sz = near_sz, far_sz
                    self.far_area_var.set(far_sz)
                    self.near_area_var.set(near_sz)

                if self.vs_pitch_en.get():
                    if sel_bbox is not None and (area_frac is not None) and abs(self._dx_lp) <= yaw_ok and (far_sz <= area_frac <= near_sz):
                        if near_sz > VS_TGT_AREA_FRAC:
                            soft = clamp((near_sz - area_frac) / (near_sz - VS_TGT_AREA_FRAC), 0.0, 1.0)
                        else:
                            soft = 1.0
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

                # Apply RC + draw
                if rc4_to_send is not None:
                    self.yaw.set(clamp(rc4_to_send, RC_MIN, RC_MAX))
                    if YOLO_SHOW_WINDOW:
                        cv2.putText(annotated, f"CH4={rc4_to_send} src={src or '-'}",
                                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,200,255), 2)

                if rc2_to_send is not None:
                    self.pitch.set(clamp(rc2_to_send, RC_MIN, RC_MAX))
                    if YOLO_SHOW_WINDOW:
                        txt = f"CH2={rc2_to_send}"
                        if area_frac is not None: txt += f" area={area_frac:.3f}"
                        cv2.putText(annotated, txt, (10, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,180,60), 2)

                if sel_bbox is None and self.vs_pitch_en.get():
                    self._vs_pitch_rc = int(self._vs_pitch_rc + 0.2*(RC_MID - self._vs_pitch_rc))
                    self.pitch.set(self._vs_pitch_rc)

                # Publish bbox (item 14)
                if sel_bbox is not None:
                    self._publish_bbox(sel_bbox, None, area_frac, target_dx, src)

                # Preview
                if YOLO_SHOW_WINDOW:
                    cv2.imshow("YOLO + DeepSORT (Yaw PD + Forward)", annotated)

                # FPS prints
                t_loop = (time.time() - t0) * 1000.0
                if (time.time() - last_print) > 1.0:
                    print(f"[FPS] loop={1000.0/t_loop:.1f}/s track={t_track:.1f}ms end2end={t_loop:.1f}ms")
                    last_print = time.time()

            except Exception as e:
                print("[YOLO+DS] inference/tracking error:", e)

            # limiter when no window (item 8)
            if not YOLO_SHOW_WINDOW:
                dt = time.time() - t0
                min_dt = 1.0 / MAX_FPS_NO_WINDOW
                if dt < min_dt:
                    time.sleep(min_dt - dt)

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
        print("[YOLO+DS] stopped")

    # ---------- GUIDED setpoint control ----------
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
        print(f"[GUIDED] Holding here at {alt:.2f} m AGL (keepalive). RC3 capped at {THR_SAFE_MAX} while hold active.")

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
        print(f"[GUIDED] Target altitude -> {new_tgt:.2f} m (relative, keepalive). RC3 capped at {THR_SAFE_MAX} while hold active.")

    def go_to_gps(self, lat_deg: float, lon_deg: float, alt_rel_m: float):
        try:
            self.set_mode("GUIDED")
            for _ in range(5):
                self._send_guided_position(float(lat_deg), float(lon_deg), float(alt_rel_m))
                time.sleep(0.05)
            self._enable_guided_keepalive(float(lat_deg), float(lon_deg), float(alt_rel_m))
            self.alt_tgt.set(round(float(alt_rel_m), 2))
            print(f"[GUIDED] Go-to -> lat={lat_deg:.7f}, lon={lon_deg:.7f}, alt_rel={alt_rel_m:.2f} m (keepalive on)")
        except Exception as e:
            messagebox.showerror("GUIDED", f"Go-to failed: {e}")

    def prompt_go_to(self):
        try:
            lat = simpledialog.askfloat("Go To", "Latitude (deg):", parent=self.root)
            if lat is None:
                return
            lon = simpledialog.askfloat("Go To", "Longitude (deg):", parent=self.root)
            if lon is None:
                return
            default_alt = float(self._alt_val) if (self._alt_val is not None) else 10.0
            alt = simpledialog.askfloat("Go To", "Altitude AGL (m):", initialvalue=round(default_alt, 2), parent=self.root)
            if alt is None:
                return
            self.go_to_gps(float(lat), float(lon), float(alt))
        except Exception as e:
            messagebox.showerror("GUIDED", f"Input error: {e}")

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
                        CMD_FLIGHT_TERMINATION, 0,
                        1, 0,0,0,0,0,0
                    )
                self._try_cmd_many(_do_termination_on, tries=3, delay=0.12, label="FLIGHT_TERM_ON")

            def _do_disarm_force():
                self.m.mav.command_long_send(
                    self.m.target_system, self.m.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                    0, 21196, 0,0,0,0,0
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
    SAFE_ARM_MODES = {"GUIDED", "STABILIZE", "ALT_HOLD"}  # item 10

    def force_arm(self):
        self._clear_flight_termination()
        if self._e_stop_active or time.time() < self._arm_inhibit_until:
            print("[FORCE ARM] inhibited"); return
        try:
            cur_mode = None
            mm = self.m.mode_mapping()
            hb = self.m.recv_match(type="HEARTBEAT", blocking=False)
            if hb:
                for name, code in mm.items():
                    if code == hb.custom_mode:
                        cur_mode = name
                        break
            if cur_mode and cur_mode not in self.SAFE_ARM_MODES:
                print(f"[FORCE ARM] unsafe mode: {cur_mode}")
                return
        except Exception:
            pass
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
        # item 15: safety confirm if ARMED
        try:
            if self.is_armed:
                if not messagebox.askyesno("Exit", "Vehicle is ARMED. Disarm and exit?"):
                    return
        except Exception:
            pass
        if messagebox.askokcancel("Exit", "Close GUI?"):
            self.running = False
            try: self.stop_yolo()
            except Exception: pass
            try: self.stop_alt_hold_ctrl()
            except Exception: pass
            try: self.stop_guided_hold()
            except Exception: pass
            try:
                if self.m is not None:
                    self.m.close()
            except Exception as e:
                print("[MAVLink] close error:", e)
            time.sleep(0.1)
            # stop external tracker if running
            try:
                if hasattr(self, "_track_proc") and self._track_proc and (self._track_proc.poll() is None):
                    self._track_proc.terminate()
            except Exception:
                pass
            self._track_proc = None
            self.root.destroy()

    # ---------- Live classes (item 13) ----------
    def _apply_classes_live(self):
        text = self.cls_entry.get().strip()
        arr = [t.strip() for t in text.split(",")] if text else None
        global TARGET_CLASSES, TARGET_CLASS_IDS
        TARGET_CLASSES = arr
        TARGET_CLASS_IDS = None
        print("[CLASSES] will re-resolve on next model run:", TARGET_CLASSES)

# ================= main =================
def main():
    global YOLO_MODEL_NAME, DET_DOWNSCALE, YOLO_CONF, YOLO_IOU
    global YOLO_CAM_INDEX, YOLO_SHOW_WINDOW, TARGET_CLASSES

    args = _parse_args()

    # item 16: apply presets
    if args.preset == "rpi-lite":
        args.conf = 0.35; args.iou = 0.45; args.det_downscale = 0.5; args.no_window = True
    elif args.preset == "laptop-cpu":
        args.conf = 0.25; args.iou = 0.50; args.det_downscale = 0.67
    elif args.preset == "cam-hires":
        args.conf = 0.20; args.iou = 0.50; args.det_downscale = 1.0

    YOLO_MODEL_NAME  = args.yolo_model
    DET_DOWNSCALE    = args.det_downscale
    YOLO_CONF        = args.conf
    YOLO_IOU         = args.iou
    YOLO_CAM_INDEX   = args.cam_index
    YOLO_SHOW_WINDOW = not args.no_window

    TARGET_CLASSES = _normalize_classnames_arg(args.classnames)

    if args.tracker == "bytetrack":
        print("[WARN] --tracker bytetrack requested, but this build runs DeepSORT backend. Proceeding with DeepSORT.")

    if not YOLO_OK or not DS_OK or cv2 is None:
        print("ERROR: Missing dependencies. Required: OpenCV, ultralytics, deep-sort-realtime.")
        print("pip install opencv-python ultralytics deep-sort-realtime pygame pymavlink")
        return

    root = tk.Tk()
    FakeSticks(root)
    root.mainloop()

if __name__ == "__main__":
    main()
