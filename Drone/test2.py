#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ASCII-only, english comments

Hybrid Telemetry GUI (no RC sliders) + PS4 joystick + YOLOv8 + DeepSORT
- Clean telemetry-only UI (no RC sliders)
- PS4 controller bars for live sticks (ROLL/PITCH/YAW/THR)
- Triangle short press = cycle flight mode (STABILIZE -> ALT_HOLD -> GUIDED -> LOITER)
  Triangle long press  = Emergency Stop
- Yaw PD from bbox center + Pitch forward-only from bbox width with smooth blending
  so both yaw and pitch act simultaneously
- Yaw slew-rate limiter to prevent violent swings
- Optional throttle compensation while moving forward
- Start/Stop detection buttons
- Simple MAVLink connect (TCP/UDP/Serial auto)
- Single "Telemetry" block (attitude, battery, speed, distances, health)
- Compact logging
- FC <-> GUI mode sync: GUI reflects real FC mode (HEARTBEAT), and mode buttons send requests
"""

import os, sys, time, math, json, glob, socket, threading, argparse, logging, logging.handlers
import tkinter as tk
from tkinter import ttk, messagebox
from tkinter import font as tkfont

# Optional imports
try:
    import numpy as np
    if not hasattr(np, "float"): np.float = float
    if not hasattr(np, "int"):   np.int   = int
    if not hasattr(np, "bool"):  np.bool  = bool
except Exception:
    np = None

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
try:
    from serial.tools import list_ports
except Exception:
    list_ports = None

# -------------- Logging --------------
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

# -------------- Env helpers --------------
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

# -------------- Joystick axis mapping --------------
WIN_AXIS_ROLL  = 2
WIN_AXIS_PITCH = 3
WIN_AXIS_YAW   = 0
WIN_AXIS_THR   = 1
LINUX_AXIS_ROLL  = 3
LINUX_AXIS_PITCH = 4
LINUX_AXIS_YAW   = 0
LINUX_AXIS_THR   = 1
if sys.platform.startswith("linux"):
    AXIS_ROLL, AXIS_PITCH, AXIS_YAW, AXIS_THR = LINUX_AXIS_ROLL, LINUX_AXIS_PITCH, LINUX_AXIS_YAW, LINUX_AXIS_THR
else:
    AXIS_ROLL, AXIS_PITCH, AXIS_YAW, AXIS_THR = WIN_AXIS_ROLL, WIN_AXIS_PITCH, WIN_AXIS_YAW, WIN_AXIS_THR

# Inverts
ROLL_INVERT  = False
PITCH_INVERT = True
YAW_INVERT   = False
THR_INVERT   = True

# -------------- RC --------------
RC_MIN, RC_MAX, RC_MID = 1000, 2000, 1500
DEADZONE = 0.10
SEND_HZ  = 20.0
def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v
def apply_deadzone(val, dz=DEADZONE): return 0.0 if abs(val) < dz else val

THR_SAFE_MAX = 1500
THR_SLEW_US_PER_SEC = 600

# -------------- YOLO/Tracking defaults --------------
YOLO_OK = (_YOLO is not None and cv2 is not None)
YOLO_MODEL_NAME  = _get_env_str("YOLO_MODEL", "yolov8n.pt")
YOLO_CONF        = float(_get_env_str("YOLO_CONF", "0.20"))
YOLO_IOU         = float(_get_env_str("YOLO_IOU", "0.50"))
YOLO_CAM_INDEX   = int(_get_env_str("YOLO_CAM_INDEX", "0"))
YOLO_SHOW_WINDOW = True
DET_DOWNSCALE    = float(_get_env_str("DET_DOWNSCALE", "0.67"))
TARGET_CLASSES   = ["person"]
TARGET_CLASS_IDS = None

# -------------- Yaw PD, blending, slew --------------
YAW_KP_DEFAULT   = 0.3
YAW_KD_DEFAULT   = 0.20
YAW_MAX_RC_DELTA = 260
YAW_DB_PIX       = 40
YAW_LP_ALPHA     = 0.30
YAW_SLEW_US_PER_SEC = 1200

# -------------- Forward-only by bbox width (PID on distance) --------------
WIDTH_TGT_FRAC      = 0.50
FWD_ONLY_MAX_DELTA  = 350
FWD_ONLY_MIN_DELTA  = 40
FWD_CREEP_DELTA     = 50
DX_BLEND_FRAC       = 0.85
FWD_BLEND_MIN       = 0.60

FWD_KP = 600.0
FWD_KI = 0.0
FWD_KD = 120.0
FWD_INT_MAX = 300.0

# -------------- Throttle compensation --------------
VS_MAX_RC_DELTA      = 220
VS_THR_COMP_GAIN_US  = 60
VS_THR_COMP_MAX_US   = 100
YOLO_THR_MIN         = 1650

# -------------- Lost target scan --------------
LOST_TIMEOUT_S   = 1.0
SCAN_PERIOD_S    = 3.5
SCAN_AMPL_RC     = 140

# -------------- PUB UDP (optional bbox) --------------
PUB_UDP_IP   = _get_env_str("PUB_UDP_IP", "127.0.0.1")
PUB_UDP_PORT = _get_env_int("PUB_UDP_PORT", 9103)

# -------------- MAVLink defaults --------------
DEVICE = _get_env_str("MAVLINK_DEVICE", "tcp:172.20.186.151:5770")
BAUD   = _get_env_int("MAVLINK_BAUD", 115200)

# -------------- Utils --------------
def _rad2deg_wrap(rad: float) -> float:
    deg = math.degrees(rad)
    while deg > 180.0:  deg -= 360.0
    while deg <= -180.0: deg += 360.0
    return deg

def _param_id_to_str(pid):
    if isinstance(pid, (bytes, bytearray)):
        try:
            return pid.decode('ascii', errors='ignore').rstrip('\x00')
        except Exception:
            return str(pid).rstrip('\x00')
    return str(pid).rstrip('\x00')

def _norm_name(s):
    import re
    return re.sub(r"[^a-z0-9]", "", str(s).lower())

def _resolve_class_ids(model, wanted):
    if wanted is None: return None
    if isinstance(wanted, (list, tuple)) and len(wanted) == 0: return None
    id2name = {}
    try:
        names = model.names if hasattr(model, "names") else None
        if isinstance(names, dict): id2name = names
        elif names is not None: id2name = {i: str(n) for i, n in enumerate(names)}
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

def _normalize_mode_name(name: str, mode_list):
    if not name:
        return None
    key = name.strip().upper()
    aliases = {
        "ALT_HOLD": "ALT_HOLD",
        "ALTHOLD": "ALT_HOLD",
        "ALT HOLD": "ALT_HOLD",
        "LOITER": "LOITER",
        "STABILIZE": "STABILIZE",
        "STAB": "STABILIZE",
        "GUIDED": "GUIDED",
        "POSHOLD": "LOITER",
    }
    if key in aliases and aliases[key] in mode_list:
        return aliases[key]
    if key in mode_list:
        return key
    for m in mode_list:
        if m in key:
            return m
    return None

# -------------- DeepSORT factory (CPU) --------------
def _make_deepsort_cpu():
    try:
        import torch
        os.environ["CUDA_VISIBLE_DEVICES"] = ""
        try: torch.backends.cudnn.enabled = False
        except Exception: pass
        try: torch.cuda.is_available = lambda: False
        except Exception: pass
    except Exception:
        pass
    try:
        return DeepSort(
            max_age=45, n_init=3, max_iou_distance=0.7, nn_budget=100,
            nms_max_overlap=1.0, embedder="mobilenet", half=False,
            embedder_gpu=False, bgr=True
        )
    except TypeError:
        return DeepSort(
            max_age=45, n_init=3, max_iou_distance=0.7, nn_budget=100,
            nms_max_overlap=1.0, embedder="mobilenet", half=False,
            use_cuda=False, bgr=True
        )

# -------------- GUI --------------
UI_SCALE = 1.35  # enlarged fonts

class HybridGUI:

    def __init__(self, root, args):
        self.root = root
        self.args = args
        self.root.title("Hybrid Telemetry GUI (YOLO + DeepSORT)")

        # Vars
        self.running = True
        self.is_armed = False

        # Joystick state
        self.js = None
        self._js_inited = False
        self.js_status = tk.StringVar(value="No joystick detected")
        self._js_prev = {0:0, 1:0, 3:0}
        self.TRI_HOLD_FOR_ESTOP = 1.2
        self._tri_pressed = False
        self._tri_t0 = 0.0

        # RC channels
        self.roll  = tk.IntVar(value=RC_MID)
        self.pitch = tk.IntVar(value=RC_MID)
        self.yaw   = tk.IntVar(value=RC_MID)
        self.thr   = tk.IntVar(value=RC_MIN)

        # Pitch forward-only state
        self._vs_pitch_rc = RC_MID
        self._fwd_int = 0.0
        self._fwd_prev_err = 0.0
        self._fwd_prev_ts = time.time()
        self._dx_lp = 0.0
        self._dx_prev = 0.0
        self._last_loop_ts = time.time()
        self._trk_id = None
        self._trk_last_seen = 0.0

        # Yaw slew memory
        self._yaw_last = RC_MID
        self._yaw_last_ts = time.time()

        # Telemetry vars
        self.batt_v   = tk.StringVar(value="--.- V")
        self.batt_a   = tk.StringVar(value="--.- A")
        self.batt_pct = tk.StringVar(value="-- %")
        self.t_roll   = tk.StringVar(value="--")
        self.t_pitch  = tk.StringVar(value="--")
        self.t_yaw    = tk.StringVar(value="--")
        self.t_alt    = tk.StringVar(value="--")
        self.t_gspeed = tk.StringVar(value="--")
        self.t_wpdist = tk.StringVar(value="--")
        self.t_speed3d= tk.StringVar(value="--")
        self.t_disthome= tk.StringVar(value="--")
        self.t_health = tk.StringVar(value="--")
        self.arm_state= tk.StringVar(value="DISARMED")
        self._att_ts  = 0.0

        # Yaw PD live tunables
        self.kp_var = tk.DoubleVar(value=YAW_KP_DEFAULT)
        self.kd_var = tk.DoubleVar(value=YAW_KD_DEFAULT)
        self.yaw_ok_px_var = tk.DoubleVar(value=40.0)

        # Flags
        self.vs_pitch_en    = tk.BooleanVar(value=True)
        self.vs_thr_comp_en = tk.BooleanVar(value=True)
        self.auto_thr_min   = tk.BooleanVar(value=True)
        self.yolo_enabled   = False

        # Mode cycle and sync
        self.mode_list  = ["STABILIZE", "ALT_HOLD", "GUIDED", "LOITER"]
        self.mode_index = 0
        self.cur_mode   = tk.StringVar(value=self.mode_list[self.mode_index])
        self.mode_btns  = {}  # name -> tk.Button
        self._last_fc_mode_norm = None
        self._mode_before_yolo = None

        # Build
        self._apply_compact_style()
        self._build_gui()

        # MAVLink
        self.m = self._open_mavlink()

        # JS
        self._init_joystick()

        # Optional bbox UDP publisher
        try:
            self._pub_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._pub_dst = (PUB_UDP_IP, int(PUB_UDP_PORT))
        except Exception:
            self._pub_sock = None
            self._pub_dst = None

        # Threads
        threading.Thread(target=self._send_loop,     daemon=True).start()
        threading.Thread(target=self._mav_telemetry, daemon=True).start()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---------- UI ----------
    def _apply_compact_style(self, scale=UI_SCALE):
        try:
            self.root.tk.call('tk', 'scaling', float(scale))
        except Exception:
            pass
        for name in ("TkDefaultFont", "TkTextFont", "TkHeadingFont", "TkMenuFont",
                     "TkIconFont", "TkFixedFont", "TkTooltipFont", "TkSmallCaptionFont"):
            try:
                f = tkfont.nametofont(name)
                base = f.cget("size") or 10
                if base == 0: base = 10
                new_sz = int(max(8, round(abs(base)*float(scale))))
                f.configure(size=(-new_sz if base < 0 else new_sz))
            except Exception:
                continue
        try:
            ttk.Style(self.root)
        except Exception:
            pass

    def _mk_js_bar(self, parent, title, bind_var):
        row = ttk.Frame(parent); row.pack(fill="x", pady=2)
        ttk.Label(row, text=title, width=10).pack(side="left")
        canvas = tk.Canvas(row, height=10)
        canvas.pack(side="left", fill="x", expand=True, padx=6)
        def update_bar(percent):
            p = max(-1.0, min(1.0, float(percent)))
            w = max(1, canvas.winfo_width())
            mid = w//2
            canvas.delete("all")
            if p >= 0:
                canvas.create_rectangle(mid, 0, mid + int((w//2)*p), 10, fill="#4ade80", width=0)
            else:
                canvas.create_rectangle(mid + int((w//2)*p), 0, mid, 10, fill="#f87171", width=0)
            bind_var.set(f"{int(p*100):+d}%")
        return update_bar

    def _refresh_mode_buttons(self, active_name):
        for name, btn in self.mode_btns.items():
            if name == active_name:
                try:
                    btn.configure(bg="#16a34a", fg="white", activebackground="#16a34a", activeforeground="white")
                except Exception:
                    btn.configure(fg="green")
            else:
                try:
                    btn.configure(bg="SystemButtonFace", fg="black", activebackground="SystemButtonFace", activeforeground="black")
                except Exception:
                    btn.configure(fg="black")

    def _build_gui(self):
        top = ttk.Frame(self.root, padding=6); top.pack(fill="x")
        self.conn_lbl = ttk.Label(top, text=f"MAVLink: connecting {DEVICE} ...")
        self.conn_lbl.pack(side="left")

        modef = ttk.LabelFrame(self.root, text="Mode", padding=4)
        modef.pack(fill="x", padx=6, pady=4)

        # Left pane: current mode + joystick status
        left = ttk.Frame(modef); left.pack(side="left", padx=6)
        ttk.Label(left, textvariable=self.cur_mode, width=12).pack(side="left", padx=6)
        ttk.Label(left, textvariable=self.js_status).pack(side="left", padx=6)

        # Right pane: mode buttons
        row = ttk.Frame(modef); row.pack(side="right", padx=6)
        tk.Button(row, text="Prev", width=6, command=self._cycle_prev).pack(side="left", padx=2)
        for name in self.mode_list:
            btn = tk.Button(row, text=name, width=10, command=lambda n=name: self.set_mode(n))
            btn.pack(side="left", padx=2)
            self.mode_btns[name] = btn
        tk.Button(row, text="Next", width=6, command=self._cycle_mode).pack(side="left", padx=2)
        self._refresh_mode_buttons(self.cur_mode.get())

        # Sticks bars
        bars = ttk.LabelFrame(self.root, text="PS4 sticks", padding=4)
        bars.pack(fill="x", padx=6, pady=4)
        self.js_txt_roll     = tk.StringVar(value="0%")
        self.js_txt_pitch    = tk.StringVar(value="0%")
        self.js_txt_yaw      = tk.StringVar(value="0%")
        self.js_txt_throttle = tk.StringVar(value="0%")
        self._upd_roll   = self._mk_js_bar(bars, "ROLL",     self.js_txt_roll)
        self._upd_pitch  = self._mk_js_bar(bars, "PITCH",    self.js_txt_pitch)
        self._upd_yaw    = self._mk_js_bar(bars, "YAW",      self.js_txt_yaw)
        self._upd_thr    = self._mk_js_bar(bars, "THROTTLE", self.js_txt_throttle)

        # YOLO controls
        ly = ttk.LabelFrame(self.root, text="YOLO + DeepSORT", padding=6); ly.pack(fill="x", padx=6, pady=4)
        self.yolo_status = tk.StringVar(value=("ready" if YOLO_OK and DS_OK else "unavailable"))
        ttk.Label(ly, textvariable=self.yolo_status).pack(side="left", padx=4)
        ttk.Button(ly, text="Start", command=self.start_yolo).pack(side="left", padx=4)
        ttk.Button(ly, text="Stop",  command=self.stop_yolo).pack(side="left", padx=4)
        ttk.Checkbutton(ly, text="Pitch forward-only", variable=self.vs_pitch_en).pack(side="left", padx=6)
        ttk.Checkbutton(ly, text="Throttle comp", variable=self.vs_thr_comp_en).pack(side="left", padx=6)
        ttk.Checkbutton(ly, text="Auto throttle min", variable=self.auto_thr_min).pack(side="left", padx=6)

        # Yaw tunables
        tun = ttk.LabelFrame(self.root, text="Yaw PD", padding=6); tun.pack(fill="x", padx=6, pady=4)
        ttk.Label(tun, text="Kp").pack(side="left"); sp1 = tk.Scale(tun, from_=0.0, to=2.0, resolution=0.01, orient="horizontal", showvalue=1)
        sp1.configure(variable=self.kp_var); sp1.pack(side="left", fill="x", expand=True, padx=6)
        ttk.Label(tun, text="Kd").pack(side="left"); sp2 = tk.Scale(tun, from_=0.0, to=1.0, resolution=0.01, orient="horizontal", showvalue=1)
        sp2.configure(variable=self.kd_var); sp2.pack(side="left", fill="x", expand=True, padx=6)
        ttk.Label(tun, text="Yaw OK px").pack(side="left"); sp3 = tk.Scale(tun, from_=0.0, to=200.0, resolution=1.0, orient="horizontal", showvalue=1)
        sp3.configure(variable=self.yaw_ok_px_var); sp3.pack(side="left", fill="x", expand=True, padx=6)

        # Single Telemetry block (grid)
        tel = ttk.LabelFrame(self.root, text="Telemetry", padding=6); tel.pack(fill="x", padx=6, pady=6)

        fields = [
            ("Roll", self.t_roll),
            ("Pitch", self.t_pitch),
            ("Yaw", self.t_yaw),
            ("Voltage", self.batt_v),
            ("Current", self.batt_a),
            ("Remain", self.batt_pct),
            ("Alt (m)", self.t_alt),
            ("GndSpd", self.t_gspeed),
            ("3D Spd", self.t_speed3d),
            ("DistHome", self.t_disthome),
            ("WP Dist", self.t_wpdist),
            ("Health", self.t_health),
        ]
        cols_per_row = 6  # label+value pairs per row
        for i, (title, var) in enumerate(fields):
            r = i // cols_per_row
            c = (i % cols_per_row) * 2
            ttk.Label(tel, text=title).grid(row=r, column=c, sticky="w", padx=4, pady=2)
            ttk.Label(tel, textvariable=var).grid(row=r, column=c+1, sticky="w", padx=4, pady=2)
        for i in range(cols_per_row * 2):
            tel.grid_columnconfigure(i, weight=1)

        # Arm/Disarm/E-Stop buttons
        btns = ttk.Frame(self.root, padding=6); btns.pack(fill="x")
        self.lbl_arm_state = tk.Label(btns, textvariable=self.arm_state, width=12, relief="groove", fg="white")
        self.lbl_arm_state.pack(side="left", padx=6)
        self._set_arm_state_ui(False)
        self.btn_arm    = tk.Button(btns, text="ARM (X)",    command=self.force_arm, width=14)
        self.btn_disarm = tk.Button(btns, text="DISARM (O)", command=self.disarm, width=14)
        self.btn_estop  = tk.Button(btns, text="Emergency Stop (Triangle hold)", command=self.emergency_stop_async)
        self.btn_arm.pack(side="left", padx=4)
        self.btn_disarm.pack(side="left", padx=4)
        self.btn_estop.pack(side="left", padx=6)

    # ---------- MAVLink ----------
    def _open_mavlink(self):
        master, baud = self._resolve_master_and_baud(self.args.master, self.args.baud)
        self.conn_lbl.configure(text=f"MAVLink: {master} @ {baud}")
        try:
            if master.startswith("tcp:") or master.startswith("udp:"):
                print(f"[MAVLINK] Connecting via {master} ...")
                m = mavutil.mavlink_connection(master)
            else:
                print(f"[MAVLINK] Connecting serial {master} @ {baud}")
                m = mavutil.mavlink_connection(master, baud=baud)
            m.wait_heartbeat(timeout=5)
            print("[MAVLINK] Connected.")
            return m
        except Exception as e:
            messagebox.showerror("MAVLink", f"Connect failed: {e}")
            raise

    def _resolve_master_and_baud(self, cli_master=None, cli_baud=None):
        env_master = _get_env_str("MAVLINK_DEVICE", None)
        env_baud   = _get_env_int("MAVLINK_BAUD", None)
        if cli_master:
            return cli_master.strip(), int(cli_baud) if cli_baud else (env_baud or BAUD or 115200)
        if env_master:
            return env_master.strip(), env_baud or BAUD or 115200
        if DEVICE:
            return DEVICE, BAUD or 115200
        if sys.platform.startswith("linux"):
            cands = glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")
            return (cands[0] if cands else "/dev/ttyACM0"), 115200
        else:
            cand = "COM4"
            try:
                if list_ports is not None:
                    for p in list_ports.comports():
                        if "mavlink" in (p.description or "").lower():
                            cand = p.device; break
            except Exception:
                pass
            return cand, 115200

    def _set_arm_state_ui(self, armed: bool):
        try:
            if armed:
                self.arm_state.set("ARMED")
                self.lbl_arm_state.configure(bg="#16a34a", fg="white")
            else:
                self.arm_state.set("DISARMED")
                self.lbl_arm_state.configure(bg="#b91c1c", fg="white")
        except Exception:
            pass

    # ---------- Joystick ----------
    def _init_joystick(self):
        if self._js_inited: return
        self._js_inited = True
        if pygame is None:
            self.js_status.set("pygame not installed")
            return
        try:
            os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
            pygame.init()
            pygame.joystick.init()
            cnt = pygame.joystick.get_count()
            if cnt > 0:
                self.js = pygame.joystick.Joystick(0); self.js.init()
                name = self.js.get_name()
                self.js_status.set(f"Connected: {name}")
                print(f"[PS4] Connected: {name}")
            else:
                self.js = None
                self.js_status.set("No joystick detected")
                print("[PS4] No joystick detected")
        except Exception as e:
            self.js = None
            self.js_status.set(f"Joystick init error: {e}")
            print("[PS4] init error:", e)
        threading.Thread(target=self._joystick_watchdog, daemon=True).start()

    def _joystick_watchdog(self):
        while self.running:
            try:
                if pygame is not None:
                    pygame.event.pump()
                    cnt = pygame.joystick.get_count()
                    if cnt == 0 and self.js is not None:
                        try: self.js.quit()
                        except Exception: pass
                        self.js = None
                        self.js_status.set("Joystick disconnected")
                        print("[PS4] Disconnected")
                    if cnt > 0 and self.js is None:
                        try:
                            self.js = pygame.joystick.Joystick(0); self.js.init()
                            name = self.js.get_name()
                            self.js_status.set(f"Connected: {name}")
                            print(f"[PS4] Reconnected: {name}")
                        except Exception as e:
                            self.js = None
                            self.js_status.set(f"Reconnect failed: {e}")
            except Exception:
                pass
            time.sleep(0.5)

    # ---------- Core loops ----------
    def _send_loop(self):
        per = 1.0 / SEND_HZ
        while self.running:
            # read joystick
            if self.js:
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

                    # update bars
                    try:
                        self._upd_roll(axis_roll)
                        self._upd_pitch(axis_pitch)
                        self._upd_yaw(axis_yaw)
                        self._upd_thr(axis_thr)
                    except Exception:
                        pass

                    # map to RC, yaw/pitch may be overridden by YOLO block below
                    r = int(RC_MID + axis_roll  * 500)
                    p = int(RC_MID + axis_pitch * 500)
                    y = int(RC_MID + axis_yaw   * 500)
                    t = RC_MIN if axis_thr <= 0 else int(RC_MIN + axis_thr * (RC_MAX - RC_MIN))

                    self.roll.set(clamp(r, RC_MIN, RC_MAX))
                    if not self.yolo_enabled:
                        self.pitch.set(clamp(p, RC_MIN, RC_MAX))
                        self.yaw.set(clamp(y, RC_MIN, RC_MAX))
                    if not self.yolo_enabled or not self.vs_thr_comp_en.get():
                        self._set_thr(clamp(t, RC_MIN, RC_MAX))

                    # buttons
                    b0 = 1 if self.js.get_button(0) else 0  # X
                    b1 = 1 if self.js.get_button(1) else 0  # O
                    b3 = 1 if self.js.get_button(3) else 0  # Triangle
                    if b0 and not self._js_prev[0]:
                        self.force_arm()
                    if b1 and not self._js_prev[1]:
                        self.disarm()
                    if b3 and not self._js_prev[3]:
                        self._tri_pressed = True
                        self._tri_t0 = time.time()
                    elif (not b3) and self._js_prev[3]:
                        if self._tri_pressed:
                            dt = time.time() - self._tri_t0
                            self._tri_pressed = False
                            if dt >= self.TRI_HOLD_FOR_ESTOP:
                                self.emergency_stop_async()
                            else:
                                self._cycle_mode()
                    self._js_prev[0] = b0
                    self._js_prev[1] = b1
                    self._js_prev[3] = b3

                except Exception as e:
                    print("[PS4] joystick error:", e)
                    try: self.js.quit()
                    except Exception: pass
                    self.js = None  # watchdog will reconnect

            # send override
            self._send_override()
            time.sleep(per)

    def _set_thr(self, value: int, apply_slew: bool = True):
        rc = int(value)
        if apply_slew:
            now = time.time()
            if not hasattr(self, "_thr_last"):
                self._thr_last = RC_MIN
                self._thr_last_ts = now
            dt = max(1e-3, now - self._thr_last_ts)
            max_step = int(THR_SLEW_US_PER_SEC * dt)
            if abs(rc - self._thr_last) > max_step:
                rc = self._thr_last + (max_step if rc > self._thr_last else -max_step)
            self._thr_last = rc
            self._thr_last_ts = now
        self.thr.set(clamp(rc, RC_MIN, RC_MAX))

    def _send_override(self):
        rc1 = int(self.roll.get())
        rc2 = int(self.pitch.get())
        rc3 = int(self.thr.get())
        rc4 = int(self.yaw.get())
        try:
            if rc3 > THR_SAFE_MAX and self.yolo_enabled:
                rc3 = THR_SAFE_MAX
            self.m.mav.rc_channels_override_send(
                self.m.target_system, self.m.target_component,
                rc1, rc2, rc3, rc4, 65535, 65535, 65535, 65535
            )
        except Exception as e:
            print("[RC_OVERRIDE] error:", e)

    # ---------- Telemetry ----------
    def _mav_telemetry(self):
        while self.running:
            try:
                msg = self.m.recv_match(blocking=False)
            except Exception:
                time.sleep(0.2); continue
            if not msg:
                time.sleep(0.01); continue
            t = msg.get_type()

            if t == "HEARTBEAT":
                try:
                    armed_flag = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    if armed_flag != self.is_armed:
                        self.is_armed = armed_flag
                        self._set_arm_state_ui(self.is_armed)
                    mode_name = mavutil.mode_string_v10(msg)
                    norm = _normalize_mode_name(mode_name, self.mode_list)
                    if norm and norm != self._last_fc_mode_norm:
                        self._last_fc_mode_norm = norm
                        self.cur_mode.set(norm)
                        if norm in self.mode_list:
                            self.mode_index = self.mode_list.index(norm)
                        self._refresh_mode_buttons(norm)
                except Exception:
                    pass

            if t == "GPS_RAW_INT":
                try:
                    fix = int(msg.fix_type); sats = int(msg.satellites_visible)
                    self.t_health.set(f"GPS fix={fix} sats={sats}")
                except Exception:
                    pass

            if t == "ATTITUDE":
                try:
                    self.t_roll.set(f"{_rad2deg_wrap(float(msg.roll)):+.1f}")
                    self.t_pitch.set(f"{_rad2deg_wrap(float(msg.pitch)):+.1f}")
                    self.t_yaw.set(f"{_rad2deg_wrap(float(msg.yaw)):+.1f}")
                    self._att_ts = time.time()
                except Exception:
                    pass

            if t == "SYS_STATUS":
                try:
                    v = msg.voltage_battery/1000.0
                    c = msg.current_battery/100.0
                    r = msg.battery_remaining
                    self.batt_v.set(f"{v:.1f} V")
                    if c > -9000: self.batt_a.set(f"{c:.1f} A")
                    if r >= 0:    self.batt_pct.set(f"{r} %")
                except Exception:
                    pass

            if t == "GLOBAL_POSITION_INT":
                try:
                    vx, vy, vz = float(msg.vx)/100.0, float(msg.vy)/100.0, float(msg.vz)/100.0
                    self.t_speed3d.set(f"{(vx*vx+vy*vy+vz*vz)**0.5:.2f}")
                except Exception:
                    pass
                try:
                    self.t_alt.set(f"{float(msg.relative_alt)/1000.0:.2f}")
                except Exception:
                    pass

            if t == "VFR_HUD":
                try: self.t_gspeed.set(f"{float(msg.groundspeed):.2f}")
                except Exception: pass
                try:
                    if (time.time()-self._att_ts) > 1.0:
                        self.t_yaw.set(f"{float(msg.heading):.1f}")
                except Exception: pass

            if t == "NAV_CONTROLLER_OUTPUT":
                try:
                    dwp = float(msg.wp_dist)
                    self.t_wpdist.set(f"{dwp:.2f}" if dwp > 0 else "--")
                except Exception:
                    self.t_wpdist.set("--")

    # ---------- Modes, arm, stop ----------
    def _cycle_mode(self):
        try:
            self.mode_index = (self.mode_index + 1) % len(self.mode_list)
            next_mode = self.mode_list[self.mode_index]
            self.set_mode(next_mode)
        except Exception as e:
            print("[MODE] cycle error:", e)

    def _cycle_prev(self):
        try:
            self.mode_index = (self.mode_index - 1) % len(self.mode_list)
            prev_mode = self.mode_list[self.mode_index]
            self.set_mode(prev_mode)
        except Exception as e:
            print("[MODE] prev error:", e)

    def set_mode(self, mode_name):
        try:
            mode_id = self.m.mode_mapping()[mode_name]
            self.m.set_mode(mode_id)
            print(f"[MODE] {mode_name}")
            self.cur_mode.set(mode_name)
            if mode_name in self.mode_list:
                self.mode_index = self.mode_list.index(mode_name)
            self._refresh_mode_buttons(mode_name)
        except Exception as e:
            print(f"[MODE] Error setting {mode_name}:", e)

    def force_arm(self):
        try:
            self.m.mav.command_long_send(
                self.m.target_system, self.m.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0,0,0,0,0
            )
            print("[ARM]")
        except Exception as e:
            print("[ARM] error:", e)

    def disarm(self):
        try:
            self.m.mav.command_long_send(
                self.m.target_system, self.m.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0,0,0,0,0,0
            )
            print("[DISARM]")
        except Exception as e:
            print("[DISARM] error:", e)

    def emergency_stop_async(self):
        threading.Thread(target=self._emergency_stop_worker, daemon=True).start()

    def _emergency_stop_worker(self):
        print("[E-STOP] Triggered")
        try: self.stop_yolo()
        except Exception: pass
        t_end = time.time() + 3.0
        while self.running and time.time() < t_end:
            try:
                self.roll.set(RC_MID); self.pitch.set(RC_MID); self.yaw.set(RC_MID); self.thr.set(RC_MIN)
                self._send_override()
            except Exception: pass
            time.sleep(0.05)
        try:
            self.set_mode("STABILIZE")
        except Exception:
            pass
        print("[E-STOP] Done")

    # ---------- Control helpers ----------
    def _blend_from_dx(self, dx_pix, frame_w):
        if frame_w <= 0: return 1.0
        half = 0.5 * float(frame_w)
        norm = abs(float(dx_pix)) / max(1.0, half)
        t = norm / max(1e-6, DX_BLEND_FRAC)
        t = clamp(t, 0.0, 1.0)
        s = t * t * (3.0 - 2.0 * t)
        g = 1.0 - s
        return max(FWD_BLEND_MIN, g)

    def _pid_forward_only(self, err, dt):
        d_err = (err - self._fwd_prev_err) / max(1e-3, dt)
        self._fwd_prev_err = err
        self._fwd_int += err * dt * FWD_KI
        self._fwd_int = clamp(self._fwd_int, -FWD_INT_MAX, FWD_INT_MAX)
        delta_us = FWD_KP * err + self._fwd_int + FWD_KD * d_err
        delta_us = clamp(delta_us, FWD_ONLY_MIN_DELTA, FWD_ONLY_MAX_DELTA)
        rc2 = RC_MID - int(delta_us)
        return clamp(rc2, RC_MIN, RC_MID)

    def _slew(self, prev_rc, target_rc, rate_us_per_sec):
        now = time.time()
        dt = max(1e-3, now - getattr(self, "_slew_ts", now))
        self._slew_ts = now
        max_step = int(rate_us_per_sec * dt)
        if abs(target_rc - prev_rc) > max_step:
            return prev_rc + (max_step if target_rc > prev_rc else -max_step)
        return target_rc

    def _combine_yaw_pitch(self, rc4_yaw, rc2_fwd):
        dyaw = 0.0
        if rc4_yaw is not None:
            dyaw = (float(rc4_yaw) - RC_MID) / float(YAW_MAX_RC_DELTA)
            dyaw = clamp(dyaw, -1.0, 1.0)
        dfwd = 0.0
        if rc2_fwd is not None:
            dfwd = (RC_MID - float(rc2_fwd)) / float(FWD_ONLY_MAX_DELTA)
            dfwd = clamp(dfwd, 0.0, 1.0)
        mag = math.sqrt(dyaw*dyaw + dfwd*dfwd)
        if mag > 1.0:
            scale = 1.0 / mag
            dyaw *= scale
            dfwd *= scale
        rc4 = int(RC_MID + dyaw * YAW_MAX_RC_DELTA)
        rc2 = int(RC_MID - dfwd * FWD_ONLY_MAX_DELTA)
        rc4 = clamp(rc4, RC_MIN, RC_MAX)
        rc2 = clamp(rc2, RC_MIN, RC_MID)
        rc4 = self._slew(self._yaw_last, rc4, YAW_SLEW_US_PER_SEC)
        self._yaw_last = rc4
        return rc4, rc2

    # ---------- YOLO loop ----------
    def start_yolo(self):
        if not (YOLO_OK and DS_OK):
            messagebox.showwarning("YOLO", "YOLO/DeepSORT unavailable")
            return
        if self.yolo_enabled:
            return
        self.yolo_enabled = True
        self.yolo_status.set("running")

        # remember current FC mode and switch to ALT_HOLD for stable altitude hold
        try:
            self._mode_before_yolo = (self._last_fc_mode_norm or self.cur_mode.get())
        except Exception:
            self._mode_before_yolo = self.cur_mode.get()
        try:
            self.set_mode("ALT_HOLD")
            print("[YOLO] switched to ALT_HOLD for detection")
        except Exception as e:
            print("[YOLO] failed to switch to ALT_HOLD:", e)

        threading.Thread(target=self._yolo_loop, daemon=True).start()

    def stop_yolo(self):
        if not self.yolo_enabled:
            return
        self.yolo_enabled = False
        self.yolo_status.set("stopped")
        self.yaw.set(RC_MID)
        self._vs_pitch_rc = int(self._vs_pitch_rc + 0.7*(RC_MID - self._vs_pitch_rc))
        self.pitch.set(self._vs_pitch_rc)

        # restore previous mode if appropriate
        try:
            prev = self._mode_before_yolo
            self._mode_before_yolo = None
            if prev and prev != self._last_fc_mode_norm:
                if (self._last_fc_mode_norm == "ALT_HOLD") or (self._last_fc_mode_norm is None):
                    self.set_mode(prev)
                    print(f"[YOLO] restored mode -> {prev}")
        except Exception as e:
            print("[YOLO] restore mode failed:", e)

    def _publish_bbox(self, bbox, conf, area, dx, src):
        if not getattr(self, "_pub_sock", None) or not getattr(self, "_pub_dst", None):
            return
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
        except Exception:
            pass

    def _open_camera(self, index):
        if sys.platform.startswith("win"):
            return cv2.VideoCapture(index, cv2.CAP_DSHOW)
        return cv2.VideoCapture(index)

    def _yolo_loop(self):
        try:
            model = _YOLO(YOLO_MODEL_NAME)
        except Exception as e:
            print("YOLO load failed:", e); self.yolo_enabled=False; self.yolo_status.set("load failed"); return

        # Force CPU for predictability
        try:
            import torch
            os.environ["CUDA_VISIBLE_DEVICES"] = ""
            try: torch.backends.cudnn.enabled = False
            except Exception: pass
            model.to("cpu")
            try:
                if hasattr(model, "model"): model.model.float()
            except Exception: pass
            print("[YOLO] using CPU")
        except Exception:
            pass

        global TARGET_CLASS_IDS
        TARGET_CLASS_IDS = _resolve_class_ids(model, TARGET_CLASSES)

        cam_indices = [YOLO_CAM_INDEX, 0, 1, 2]
        cap = None
        for idx in cam_indices:
            tmp = self._open_camera(idx)
            if tmp.isOpened():
                cap = tmp
                print(f"[CAM] opened index {idx}")
                break
        if cap is None:
            print("No camera"); self.yolo_status.set("no camera"); self.yolo_enabled=False; return

        try:
            tracker = _make_deepsort_cpu()
        except Exception as e:
            print("DeepSORT init failed:", e); self.yolo_status.set("tracker fail"); self.yolo_enabled=False; cap.release(); return

        self._trk_id = None
        self._trk_last_seen = 0.0
        self._dx_lp = 0.0
        self._dx_prev = 0.0
        self._last_loop_ts = time.time()
        last_print = time.time()

        if YOLO_SHOW_WINDOW and cv2 is not None:
            cv2.namedWindow("YOLO+DS", cv2.WINDOW_NORMAL)

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
                # downscale for detection
                if DET_DOWNSCALE and 0.2 <= DET_DOWNSCALE < 1.0:
                    dw, dh = int(w*DET_DOWNSCALE), int(h*DET_DOWNSCALE)
                    infer_frame = cv2.resize(frame, (dw, dh), interpolation=cv2.INTER_AREA)
                    scale_x, scale_y = w/float(dw), h/float(dh)
                else:
                    infer_frame = frame
                    scale_x = scale_y = 1.0

                results = model(infer_frame, conf=YOLO_CONF, iou=YOLO_IOU, verbose=False)

                det_list = []
                for r in results:
                    if not hasattr(r, "boxes"): continue
                    for b in r.boxes:
                        cls = int(b.cls[0].item())
                        if TARGET_CLASS_IDS is not None and cls not in TARGET_CLASS_IDS:
                            continue
                        x1,y1,x2,y2 = map(float, b.xyxy[0])
                        x1, x2 = x1*scale_x, x2*scale_x
                        y1, y2 = y1*scale_y, y2*scale_y
                        conf = float(b.conf[0].item()) if hasattr(b, "conf") else 0.0
                        det_list.append([[x1, y1, x2-x1, y2-y1], conf, cls])

                tracks = tracker.update_tracks(det_list, frame=frame)

                annotated = frame.copy()
                if YOLO_SHOW_WINDOW and cv2 is not None:
                    cv2.line(annotated, (int(cx_ref), 0), (int(cx_ref), h), (255,0,0), 2)

                candidates = []
                for trk in tracks:
                    if not trk.is_confirmed(): continue
                    l,t,r,b = trk.to_ltrb()
                    cx = 0.5*(l+r)
                    dx = cx - cx_ref
                    area = ((r-l)*(b-t))/float(max(1.0, w*h))
                    tid = int(trk.track_id)
                    conf_t = 0.0
                    if hasattr(trk, "det_conf") and trk.det_conf is not None:
                        try: conf_t = float(trk.det_conf)
                        except Exception: pass
                    elif hasattr(trk, "score") and trk.score is not None:
                        try: conf_t = float(trk.score)
                        except Exception: pass

                    center_score = 1.0 - min(1.0, abs(dx)/(w*0.5))
                    width_frac = (r-l)/float(max(1.0, w))
                    area_score = min(1.0, width_frac / max(1e-6, WIDTH_TGT_FRAC))
                    score = center_score + 0.5*conf_t + 0.3*area_score
                    if self._trk_id is not None and tid == self._trk_id:
                        score += 0.35
                    candidates.append((score, tid, dx, (float(l), float(t), float(r), float(b)), area, conf_t))

                    if YOLO_SHOW_WINDOW and cv2 is not None:
                        color = (60+tid*37%195, 60+tid*17%195, 60+tid*29%195)
                        cv2.rectangle(annotated, (int(l), int(t)), (int(r), int(b)), color, 2)
                        cv2.putText(annotated, f"ID {tid}", (int(l), max(0,int(t)-10)),
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
                        if YOLO_SHOW_WINDOW and cv2 is not None:
                            x1,y1,x2,y2 = map(int, sel_bbox)
                            cv2.rectangle(annotated, (x1,y1), (x2,y2), (0,128,255), 2)

                dt = max(1e-3, now - self._last_loop_ts)
                self._last_loop_ts = now

                # Yaw PD or scan
                if target_dx is None:
                    if (now - self._trk_last_seen) > LOST_TIMEOUT_S:
                        self._trk_id = None
                        phase = (now % SCAN_PERIOD_S) / SCAN_PERIOD_S
                        sweep = math.sin(2.0 * math.pi * phase)
                        rc4_to_send = RC_MID + int(sweep * SCAN_AMPL_RC)
                        src = "scan"
                    self._dx_lp = 0.0
                    self._dx_prev = 0.0
                else:
                    kp = float(self.kp_var.get())
                    kd = float(self.kd_var.get())
                    self._dx_lp += YAW_LP_ALPHA * (float(target_dx) - self._dx_lp)
                    d_err = (self._dx_lp - self._dx_prev) / dt
                    self._dx_prev = self._dx_lp
                    if abs(self._dx_lp) <= YAW_DB_PIX:
                        rc4_to_send = RC_MID
                    else:
                        e_norm = clamp(self._dx_lp / (w * 0.5), -1.0, 1.0)
                        d_norm = clamp(d_err     / (w * 0.5), -1.0, 1.0)
                        err_norm = min(1.0, abs(self._dx_lp)/(w*0.5))
                        kp_eff = kp * (1.10 + 0.60*err_norm)
                        kd_eff = kd * (0.90 + 0.20*err_norm)
                        yaw_cmd = clamp(kp_eff * e_norm + kd_eff * d_norm, -1.0, 1.0)
                        rc4_to_send = int(RC_MID + yaw_cmd * YAW_MAX_RC_DELTA)

                    if self.auto_thr_min.get() and self.thr.get() < YOLO_THR_MIN:
                        self._set_thr(YOLO_THR_MIN)

                # Pitch forward-only blended with dx
                rc2_to_send = None
                now2 = time.time()
                dt_pid = max(1e-3, now2 - getattr(self, "_fwd_prev_ts", now2))
                self._fwd_prev_ts = now2

                if self.vs_pitch_en.get() and sel_bbox is not None and target_dx is not None:
                    x1,y1,x2,y2 = sel_bbox
                    width_frac = max(0.0, min(1.0, (x2 - x1)/float(max(1.0, w))))
                    err_w = max(0.0, WIDTH_TGT_FRAC - width_frac)
                    if err_w > 0.0:
                        rc2_cmd = self._pid_forward_only(err_w, dt_pid)
                        blend = self._blend_from_dx(self._dx_lp, w)
                        raw_delta = RC_MID - rc2_cmd
                        blended_delta = int(raw_delta * blend)
                        rc2_cmd_blended = RC_MID - blended_delta
                        self._vs_pitch_rc = int(self._vs_pitch_rc + 0.25*(rc2_cmd_blended - self._vs_pitch_rc))
                        rc2_to_send = self._vs_pitch_rc
                    else:
                        blend = self._blend_from_dx(self._dx_lp, w)
                        creep = max(1, int(FWD_CREEP_DELTA * blend))
                        rc2_cmd = RC_MID - creep
                        self._vs_pitch_rc = int(self._vs_pitch_rc + 0.25*(rc2_cmd - self._vs_pitch_rc))
                        rc2_to_send = self._vs_pitch_rc

                    if self.vs_thr_comp_en.get():
                        base = max(self.thr.get(), YOLO_THR_MIN)
                        forward_strength = max(0.0, (RC_MID - int(self._vs_pitch_rc))/float(VS_MAX_RC_DELTA))
                        add = int(clamp(forward_strength * VS_THR_COMP_GAIN_US, 0, VS_THR_COMP_MAX_US))
                        self._set_thr(base + add)
                else:
                    self._vs_pitch_rc = int(self._vs_pitch_rc + 0.20*(RC_MID - self._vs_pitch_rc))
                    rc2_to_send = self._vs_pitch_rc

                # Combine yaw and forward pitch
                if rc4_to_send is not None or rc2_to_send is not None:
                    c4, c2 = self._combine_yaw_pitch(
                        rc4_to_send if rc4_to_send is not None else RC_MID,
                        rc2_to_send if rc2_to_send is not None else RC_MID
                    )
                    self.yaw.set(c4)
                    self.pitch.set(c2)

                    if YOLO_SHOW_WINDOW and cv2 is not None:
                        cv2.putText(annotated, f"CH4={c4}", (10, 28),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)
                    if YOLO_SHOW_WINDOW and cv2 is not None:
                        cv2.putText(annotated, f"CH2={c2}", (10, 56),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 180, 60), 2)

                if sel_bbox is not None:
                    self._publish_bbox(sel_bbox, None, area_frac, target_dx, src)

                if YOLO_SHOW_WINDOW and cv2 is not None:
                    cv2.imshow("YOLO+DS", annotated)

            except Exception as e:
                print("[YOLO+DS] loop error:", e)

            if YOLO_SHOW_WINDOW and cv2 is not None:
                k = cv2.waitKey(1) & 0xFF
                if k in (27, ord('q')):
                    self.stop_yolo()
                    break

            t_loop = (time.time() - t0)
            if time.time() - last_print > 1.0:
                try:
                    print(f"[FPS] {1.0/max(1e-3,t_loop):.1f}/s  loop={t_loop*1000:.1f}ms")
                except Exception:
                    pass
                last_print = time.time()

        try:
            cap.release()
            if YOLO_SHOW_WINDOW and cv2 is not None: cv2.destroyAllWindows()
        except Exception:
            pass
        print("[YOLO+DS] stopped")

    # ---------- Window close ----------
    def on_close(self):
        if self.is_armed:
            if not messagebox.askyesno("Exit", "Vehicle is ARMED. Disarm and exit?"):
                return
        if messagebox.askokcancel("Exit", "Close GUI?"):
            self.running = False
            try: self.stop_yolo()
            except Exception: pass
            try: self.m.close()
            except Exception: pass
            try:
                if self.js: self.js.quit()
            except Exception: pass
            try:
                if pygame: pygame.quit()
            except Exception: pass
            self.root.destroy()

# -------------- CLI --------------
def _parse_args():
    p = argparse.ArgumentParser(description="Hybrid Telemetry GUI + PS4 + YOLO")
    p.add_argument("--master", default=None)
    p.add_argument("--baud", type=int, default=None)
    p.add_argument("--no-window", action="store_true")
    p.add_argument("--cam-index", type=int, default=YOLO_CAM_INDEX)
    p.add_argument("--conf", type=float, default=YOLO_CONF)
    p.add_argument("--iou", type=float, default=YOLO_IOU)
    p.add_argument("--det-downscale", type=float, default=DET_DOWNSCALE)
    return p.parse_args()

def main():
    global YOLO_CAM_INDEX, YOLO_CONF, YOLO_IOU, DET_DOWNSCALE, YOLO_SHOW_WINDOW
    args = _parse_args()
    YOLO_CAM_INDEX   = args.cam_index
    YOLO_CONF        = args.conf
    YOLO_IOU         = args.iou
    DET_DOWNSCALE    = args.det_downscale
    YOLO_SHOW_WINDOW = not args.no_window

    root = tk.Tk()
    app = HybridGUI(root, args)
    root.mainloop()

if __name__ == "__main__":
    main()
