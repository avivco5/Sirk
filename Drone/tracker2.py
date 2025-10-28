#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Headless RC + YOLO/ByteTrack (PS4 toggle) - LOW LATENCY
- Square toggles TRACK ON/OFF (camera opens lazily; latest-frame-only capture)
- Optional --track to autostart tracking on launch
- Minimal pipeline latency even if FPS drops
- Optional --lowlat profile tunes imgsz/filters/threads for lowest delay
- Optional --no-bt to disable ByteTrack (detection-only)
- ASCII-only file to avoid encoding issues
"""

import os
import sys
import glob
import math
import time
import threading
import argparse

from pymavlink import mavutil
import pygame

# ===== ANSI colors =====
ANSI_COLORS = sys.stdout.isatty()
def c_green(s): return ("\033[92m" + s + "\033[0m") if ANSI_COLORS else s
def c_red(s):   return ("\033[91m" + s + "\033[0m") if ANSI_COLORS else s

# ===== PS4 mapping =====
if sys.platform.startswith("linux"):
    AXIS_ROLL, AXIS_PITCH, AXIS_YAW, AXIS_THR = 3, 4, 0, 1
else:
    AXIS_ROLL, AXIS_PITCH, AXIS_YAW, AXIS_THR = 0, 1, 2, 3
BTN_X, BTN_O, BTN_SQUARE, BTN_TRIANGLE = 0, 1, 2, 3
PITCH_INVERT, THR_INVERT, ROLL_INVERT, YAW_INVERT = True, True, False, False

# ===== RC / control =====
RC_MIN, RC_MAX, RC_MID = 1000, 2000, 1500
SEND_HZ   = 20.0
DEADZONE  = 0.10
THR_SAFE_MAX = 1500
THR_SLEW_US_PER_SEC = 600
YOLO_THR_MIN = 1150

def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v
def apply_deadzone(val, dz=DEADZONE): return 0.0 if abs(val) < dz else val
def _rad2deg_wrap(rad):
    deg = math.degrees(rad)
    while deg > 180.0:  deg -= 360.0
    while deg <= -180.0: deg += 360.0
    return deg

# ===== Vision (optional) =====
YOLO_OK = True
try:
    import cv2
    from ultralytics import YOLO as _YOLO
except Exception as _e:
    YOLO_OK = False
    _YOLO = None
    cv2 = None
    print("Warning: YOLO/Camera unavailable:", _e)

BT_OK = True
try:
    import numpy as np
    if not hasattr(np, "float"): np.float = float
    if not hasattr(np, "int"):   np.int   = int
    if not hasattr(np, "bool"):  np.bool  = bool
    import torch
    from types import SimpleNamespace
    from yolox.tracker.byte_tracker import BYTETracker
except Exception as _e:
    BT_OK = False
    print("Warning: ByteTrack unavailable:", _e)

# ===== Defaults / tuning =====
# Yaw PD (+ scheduling)
YAW_DB_PIX       = 30
YAW_KP_DEFAULT   = 0.60
YAW_KD_DEFAULT   = 0.20
YAW_MAX_RC_DELTA = 320
YAW_LP_ALPHA     = 0.30  # can raise in --lowlat

# Forward (Pitch) from area
VS_TGT_AREA_FRAC   = 0.025
VS_DB_AREA_FRAC    = 0.004
VS_KP_PITCH        = 0.70
VS_MAX_RC_DELTA    = 220
VS_LP_ALPHA        = 0.18
VS_PITCH_FORWARD_SIGN = -1
VS_YAW_OK_PIX_DEFAULT = 40
FAR_STOP_AREA_FRAC_DEFAULT  = 0.0015
NEAR_STOP_AREA_FRAC_DEFAULT = 0.120

# Throttle comp
VS_THR_COMP_GAIN_US = 60
VS_THR_COMP_MAX_US  = 100

# Detector/Tracker
YOLO_MODEL_NAME  = "yolov8n.pt"
YOLO_PERSON_CLS  = 0
YOLO_CONF        = 0.15
YOLO_IOU         = 0.50
DRAW_CONF_MIN    = 0.60
BT_TRACK_THRESH  = 0.10
BT_MATCH_THRESH  = 0.80
BT_TRACK_BUFFER  = 30
FORCE_FALLBACK_TO_DET = True

# Downscale/size
DET_DOWNSCALE_DEFAULT = 0.67
IMGSZ_DEFAULT = 416  # smaller -> lower latency

# Lost-target scan
LOST_TIMEOUT_S = 0.7
SCAN_PERIOD_S  = 3.0
SCAN_AMPL_RC   = 150

# Print throttling
ATT_PRINT_PERIOD   = 0.5
BAT_PRINT_PERIOD   = 1.0
RC_PRINT_PERIOD    = 0.4
TRACK_PRINT_PERIOD = 0.20
DET_HOLD_S_DEFAULT = 0.7

# Serial default
if sys.platform.startswith("linux"):
    cands = glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")
    DEVICE_DEFAULT = cands[0] if cands else "/dev/ttyACM0"
else:
    DEVICE_DEFAULT = "COM40"
BAUD_DEFAULT = 115200

class HeadlessTracker:
    def __init__(self, args):
        self.args = args

        # Optional nicer priority
        try:
            if args.nice is not None:
                os.nice(args.nice)
        except Exception:
            pass

        # Limit threads for latency
        try:
            if 'torch' in sys.modules:
                import torch
                torch.set_num_threads(max(1, args.torch_threads))
        except Exception:
            pass
        try:
            if cv2:
                cv2.setNumThreads(max(1, args.cv_threads))
        except Exception:
            pass

        # MAVLink
        self.m = mavutil.mavlink_connection(args.device, baud=args.baud)
        self.m.wait_heartbeat(timeout=5)
        print(f"[LINK] Connected: sys={self.m.target_system} comp={self.m.target_component}")

        # RC state
        self.ch1 = RC_MID; self.ch2 = RC_MID; self.ch3 = RC_MIN; self.ch4 = RC_MID
        self.ch5 = 1500;   self.ch7 = 1500
        self.is_armed = False
        self.running  = True
        self._thr_last = RC_MIN; self._thr_last_ts = time.time()

        # Joystick
        pygame.init(); pygame.joystick.init()
        self.js = pygame.joystick.Joystick(0) if pygame.joystick.get_count() > 0 else None
        if self.js:
            self.js.init()
            print(f"[PS4] Connected: {self.js.get_name()}")
        else:
            print("[PS4] No joystick detected")

        # Toggle / vision
        self.track_enabled = bool(args.track)
        self.preview = args.preview and YOLO_OK
        self.model = None
        self.bt = None
        self.no_bt = args.no_bt

        # Camera & capture thread (latest-only)
        self.cap = None
        self.cam_index = args.cam
        self.cam_path = f"/dev/video{self.cam_index}" if sys.platform.startswith("linux") else f"cam{self.cam_index}"
        self.cap_w, self.cap_h, self.cap_fps = args.cap_w, args.cap_h, args.cap_fps
        self.fourcc = args.fourcc
        self.cap_buffersize = args.cap_buffersize

        self._cap_running = False
        self._frame = None
        self._frame_ts = 0.0
        self._cap_thread = None

        # Controller / selection
        self.kp, self.kd = args.yaw_kp, args.yaw_kd
        self.yaw_ok_px = args.yaw_ok_px
        self.far_area = args.far_area; self.near_area = args.near_area
        self._trk_id = None; self._trk_last_seen = 0.0
        self._dx_lp = 0.0; self._dx_prev = 0.0; self._last_loop_ts = time.time()
        self._vs_pitch_rc = RC_MID

        # Latency profile
        self.imgsz = args.imgsz
        self.det_downscale = args.det_downscale
        self.det_hold_s = args.det_hold_s
        if args.lowlat:
            self.imgsz = min(self.imgsz, 416)
            self.det_downscale = min(self.det_downscale, 0.60)
            self.det_hold_s = min(self.det_hold_s, 0.35)
            global YAW_LP_ALPHA
            YAW_LP_ALPHA = 0.50

        # Detection hold flag
        self._det_on = False; self._det_last_ts = 0.0

        # Telemetry prints
        self._last_print = {"RC":0.0, "ATT":0.0, "BAT":0.0, "TRACK":0.0}
        self.batt_v = None; self.batt_a = None; self.batt_pct = None

        self._setup_streams()

        threading.Thread(target=self._send_loop, daemon=True).start()
        threading.Thread(target=self._mav_rx_loop, daemon=True).start()
        threading.Thread(target=self._track_loop, daemon=True).start()

        # Initial banner and optional autostart
        if self.track_enabled:
            print("[TRACK]", c_green("ON"), "(autostart)")
            self._start_capture()
        else:
            print("[INFO]", c_red("TRACK: OFF"), "- press Square to toggle")
        if args.lowlat:
            print("[INFO] Low-latency profile ENABLED")

    # ---------- Streams ----------
    def _setup_streams(self):
        for mid, hz in (
            (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,            10),
            (mavutil.mavlink.MAVLINK_MSG_ID_ALTITUDE,            10),
            (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10),
            (mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,              5),
            (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,           2),
        ):
            try:
                interval_us = int(1e6/float(hz)) if hz>0 else 0
                self.m.mav.command_long_send(
                    self.m.target_system, self.m.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0, mid, interval_us, 0,0,0,0,0
                )
            except Exception as e:
                print(f"[MSG_RATE] {mid}@{hz}Hz failed:", e)

    # ---------- Safety ----------
    def _is_hold_context(self): return True
    def _set_thr(self, value):
        rc = int(value)
        now = time.time()
        dt = max(1e-3, now - self._thr_last_ts)
        max_step = int(THR_SLEW_US_PER_SEC * dt)
        rc = clamp(rc, RC_MIN, THR_SAFE_MAX)
        if abs(rc - self._thr_last) > max_step:
            rc = self._thr_last + (max_step if rc > self._thr_last else -max_step)
        self._thr_last = rc; self._thr_last_ts = now
        self.ch3 = rc

    # ---------- RC send ----------
    def _send_override(self):
        rc1,rc2,rc3,rc4,rc5,rc7 = self.ch1,self.ch2,self.ch3,self.ch4,self.ch5,self.ch7
        rc3 = clamp(rc3, RC_MIN, THR_SAFE_MAX if self._is_hold_context() else RC_MAX)
        try:
            self.m.mav.rc_channels_override_send(
                self.m.target_system, self.m.target_component,
                rc1, rc2, rc3, rc4, rc5, 65535, rc7, 65535
            )
        except Exception as e:
            print("[RC_OVERRIDE] error:", e)
        now = time.time()
        if now - self._last_print["RC"] >= RC_PRINT_PERIOD:
            trk = c_green("TRACK: ON") if self.track_enabled else c_red("TRACK: OFF")
            det = (" | DET: " + ("YES" if self._det_on else "NO")) if self.track_enabled else ""
            print(f"[RC] CH1={rc1} CH2={rc2} CH3={rc3} CH4={rc4} | {trk}{det}")
            self._last_print["RC"] = now

    # ---------- Joystick/RC loop ----------
    def _send_loop(self):
        per = 1.0 / SEND_HZ
        js_prev = {BTN_X:0, BTN_O:0, BTN_SQUARE:0, BTN_TRIANGLE:0}
        while self.running:
            try:
                if self.js is None and pygame.joystick.get_count() > 0:
                    self.js = pygame.joystick.Joystick(0); self.js.init()
                    print(f"[PS4] Connected: {self.js.get_name()}")
                if self.js:
                    pygame.event.pump()
                    ax_r  = self.js.get_axis(AXIS_ROLL)
                    ax_p  = self.js.get_axis(AXIS_PITCH)
                    ax_y  = self.js.get_axis(AXIS_YAW)
                    ax_t  = self.js.get_axis(AXIS_THR)
                    if ROLL_INVERT:  ax_r = -ax_r
                    if PITCH_INVERT: ax_p = -ax_p
                    if YAW_INVERT:   ax_y = -ax_y
                    if THR_INVERT:   ax_t = -ax_t
                    ax_r = apply_deadzone(ax_r); ax_p = apply_deadzone(ax_p); ax_y = apply_deadzone(ax_y)
                    r = int(RC_MID + ax_r * 500)
                    p = int(RC_MID + ax_p * 500)
                    y = int(RC_MID + ax_y * 500)
                    t = RC_MIN if ax_t <= 0 else int(RC_MIN + ax_t * (RC_MAX - RC_MIN))

                    # Roll always from joystick
                    self.ch1 = clamp(r, RC_MIN, RC_MAX)

                    if self.track_enabled:
                        self._set_thr(max(t, YOLO_THR_MIN))
                    else:
                        self.ch4 = clamp(y, RC_MIN, RC_MAX)
                        self._set_thr(clamp(t, RC_MIN, RC_MAX))
                        self.ch2 = clamp(p, RC_MIN, RC_MAX)

                    # Buttons
                    bX = 1 if self.js.get_button(BTN_X) else 0
                    bO = 1 if self.js.get_button(BTN_O) else 0
                    bS = 1 if self.js.get_button(BTN_SQUARE) else 0
                    bT = 1 if self.js.get_button(BTN_TRIANGLE) else 0
                    if bX and not js_prev[BTN_X]:  self._arm()
                    if bO and not js_prev[BTN_O]:  self._disarm()
                    if bT and not js_prev[BTN_TRIANGLE]: self._e_stop()
                    if bS and not js_prev[BTN_SQUARE]:
                        self.track_enabled = not self.track_enabled
                        print("[TRACK]", c_green("ON") if self.track_enabled else c_red("OFF"))
                        if not self.track_enabled:
                            self._det_on = False
                            self._stop_capture()
                        else:
                            self._start_capture()

                    js_prev[BTN_X]=bX; js_prev[BTN_O]=bO; js_prev[BTN_SQUARE]=bS; js_prev[BTN_TRIANGLE]=bT

            except Exception as e:
                print("[TX] joystick error:", e)
                self.js = None

            self._send_override()
            time.sleep(per)

    # ---------- MAV RX ----------
    def _mav_rx_loop(self):
        while self.running:
            try:
                msg = self.m.recv_match(blocking=False)
            except Exception:
                time.sleep(0.01); continue
            if not msg:
                now = time.time()
                if now - self._last_print["BAT"] >= BAT_PRINT_PERIOD:
                    v = f"{self.batt_v:.1f} V" if self.batt_v is not None else "-- V"
                    a = f"{self.batt_a:.1f} A" if self.batt_a is not None else "-"
                    p = f"{int(self.batt_pct)} %" if self.batt_pct is not None else "-"
                    print(f"[BAT] {v} {a} {p}")
                    self._last_print["BAT"] = now
                time.sleep(0.003); continue

            t = msg.get_type(); now = time.time()
            if t == "HEARTBEAT":
                try:
                    armed_flag = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    if armed_flag != self.is_armed:
                        self.is_armed = armed_flag
                        print("[ARM_STATE]", "ARMED" if self.is_armed else "DISARMED")
                except Exception:
                    pass
            elif t == "SYS_STATUS":
                try:
                    vb = getattr(msg, "voltage_battery", 0)
                    self.batt_v = (vb/1000.0) if vb > 0 else self.batt_v
                    cb = getattr(msg, "current_battery", None)
                    self.batt_a = (cb/100.0) if (cb is not None and cb > -9000) else self.batt_a
                    rb = getattr(msg, "battery_remaining", None)
                    self.batt_pct = float(rb) if (rb is not None and rb >= 0) else self.batt_pct
                except Exception:
                    pass
            elif t == "ATTITUDE":
                if now - self._last_print["ATT"] >= ATT_PRINT_PERIOD:
                    try:
                        r = _rad2deg_wrap(float(msg.roll))
                        p = _rad2deg_wrap(float(msg.pitch))
                        y = _rad2deg_wrap(float(msg.yaw))
                        print(f"[ATT] R={r:+.1f} P={p:+.1f} Y={y:+.1f}")
                    except Exception:
                        pass
                    self._last_print["ATT"] = now

    # ---------- ARM/DISARM/E-STOP ----------
    def _arm(self):
        try:
            self.m.mav.command_long_send(
                self.m.target_system, self.m.target_component,
                getattr(mavutil.mavlink, "MAV_CMD_DO_FLIGHTTERMINATION", 185), 0,
                0,0,0,0,0,0,0
            )
        except Exception:
            pass
        try:
            self.m.mav.command_long_send(
                self.m.target_system, self.m.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0,0,0,0,0
            )
            print("[ARM] requested")
        except Exception as e:
            print("[ARM] error:", e)

    def _disarm(self):
        try:
            self.m.mav.command_long_send(
                self.m.target_system, self.m.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0,0,0,0,0,0
            )
            print("[DISARM] requested")
        except Exception as e:
            print("[DISARM] error:", e)

    def _e_stop(self):
        try:
            print("[E-STOP] triggered")
            t0 = time.time()
            while time.time() - t0 < 0.4:
                self.ch1=RC_MID; self.ch2=RC_MID; self.ch4=RC_MID; self._set_thr(RC_MIN)
                self._send_override(); time.sleep(0.05)
            self._disarm()
            t1 = time.time()
            while time.time() - t1 < 1.8:
                self.ch1=RC_MID; self.ch2=RC_MID; self.ch4=RC_MID; self._set_thr(RC_MIN)
                self._send_override(); time.sleep(0.05)
            print("[E-STOP] done")
        except Exception as e:
            print("[E-STOP] error:", e)

    # ---------- Model / cam ----------
    def _ensure_model(self):
        if self.model is not None:
            return True
        if not YOLO_OK:
            return False
        try:
            self.model = _YOLO(YOLO_MODEL_NAME)
            # CUDA/Half (Jetson Orin Nano)
            try:
                import torch
                if torch.cuda.is_available():
                    self.model.to("cuda")
                    try:
                        self.model.model.half()
                    except Exception:
                        pass
            except Exception:
                pass
            print("[TRACK] model loaded:", YOLO_MODEL_NAME, f"(imgsz={self.imgsz})")
            return True
        except Exception as e:
            print("[TRACK] model load failed:", e)
            return False

    def _open_cam(self):
        if self.cap is not None:
            return True
        if not YOLO_OK:
            return False

        # Open with backend hints
        if sys.platform.startswith("win"):
            cap = cv2.VideoCapture(self.cam_index, cv2.CAP_DSHOW)
        else:
            cap = cv2.VideoCapture(self.cam_index, cv2.CAP_V4L2)

        if not cap or not cap.isOpened():
            print(f"[CAM] cannot open index {self.cam_index}")
            return False

        # Try configure for lower latency
        try:
            if self.fourcc:
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
            if self.cap_w:   cap.set(cv2.CAP_PROP_FRAME_WIDTH,  int(self.cap_w))
            if self.cap_h:   cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.cap_h))
            if self.cap_fps: cap.set(cv2.CAP_PROP_FPS,          float(self.cap_fps))
            if self.cap_buffersize is not None:
                cap.set(cv2.CAP_PROP_BUFFERSIZE, int(self.cap_buffersize))
        except Exception:
            pass

        self.cap = cap
        print(f"[CAM] opened: {self.cam_path} fourcc={self.fourcc or '-'} "
              f"{int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}@{self.cap.get(cv2.CAP_PROP_FPS):.1f} "
              f"buf={self.cap_buffersize if self.cap_buffersize is not None else '-'}")
        return True

    def _start_capture(self):
        if self._cap_running:
            return
        if not self._open_cam():
            return
        self._cap_running = True
        self._frame = None; self._frame_ts = 0.0
        self._cap_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._cap_thread.start()

    def _stop_capture(self):
        self._cap_running = False
        try:
            if self._cap_thread:
                self._cap_thread.join(timeout=0.2)
        except Exception:
            pass
        self._cap_thread = None
        try:
            if self.cap is not None:
                self.cap.release()
                print(f"[CAM] released: {self.cam_path}")
        except Exception:
            pass
        self.cap = None

    def _capture_loop(self):
        """Grabs frames continuously and keeps ONLY the latest (drops the rest)."""
        while self.running and self.track_enabled and self.cap and self._cap_running:
            ok, frame = self.cap.read()
            if not ok or frame is None:
                time.sleep(0.002)
                continue
            self._frame = frame
            self._frame_ts = time.time()
            # no sleep here to avoid queue build-up; camera read blocks as needed

    # ---------- Pitch helper ----------
    def _vs_pitch_from_area(self, area_frac):
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

    # ---------- Tracking loop ----------
    def _track_loop(self):
        if not self._ensure_model():
            print("[TRACK] disabled (no model).")
            return

        if BT_OK and (not self.no_bt):
            self.bt = None  # will init after camera open
        else:
            self.bt = None
            if self.no_bt:
                print("[TRACK] ByteTrack disabled (--no-bt)")

        print("[TRACK] thread started")
        while self.running:
            if not self.track_enabled:
                time.sleep(0.03)
                continue

            if not self._cap_running:
                self._start_capture()
                if BT_OK and (not self.no_bt) and self.cap is not None and self.bt is None:
                    try:
                        fps = self.cap.get(cv2.CAP_PROP_FPS)
                        fps = float(fps) if fps and fps > 0 else 30.0
                        bt_args = SimpleNamespace(track_thresh=BT_TRACK_THRESH, match_thresh=BT_MATCH_THRESH,
                                                  track_buffer=BT_TRACK_BUFFER, frame_rate=fps, mot20=False)
                        self.bt = BYTETracker(bt_args)
                        print(f"[TRACK] ByteTrack ready (fps={fps:.1f})")
                    except Exception as e:
                        print("[TRACK] ByteTrack init failed:", e)
                        self.bt = None

            frame = self._frame
            if frame is None:
                time.sleep(0.002)
                continue

            h, w = frame.shape[:2]
            cx_ref = w * 0.5
            rc4_to_send = None
            rc2_to_send = None
            sel_area = None
            src = None
            target_dx = None

            try:
                # Downscale for RPi
                infer_frame = frame
                scale_x = scale_y = 1.0
                if self.det_downscale and 0.2 <= self.det_downscale < 1.0:
                    dw, dh = int(w * self.det_downscale), int(h * self.det_downscale)
                    infer_frame = cv2.resize(frame, (dw, dh), interpolation=cv2.INTER_AREA)
                    scale_x, scale_y = w/float(dw), h/float(dh)

                # Inference (explicit imgsz for latency)
                results = self.model(infer_frame, conf=YOLO_CONF, iou=YOLO_IOU, verbose=False, imgsz=self.imgsz)

                # Build detections
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

                # DET hold
                now = time.time()
                if det_list:
                    self._det_on = True; self._det_last_ts = now
                else:
                    if self._det_on and (now - self._det_last_ts > self.det_hold_s):
                        self._det_on = False

                # Tracks or fallback
                candidates = []; tracks_cnt = 0
                if self.bt is not None and det_list:
                    dets = torch.tensor(det_list, dtype=torch.float32)
                    tracks = self.bt.update(dets, (h, w), (h, w))
                    tracks_cnt = len(tracks)
                    for t in tracks:
                        conf_t = float(getattr(t, "score", 0.0))
                        if conf_t < DRAW_CONF_MIN: continue
                        x1, y1, x2, y2 = t.tlbr
                        cx = 0.5*(x1+x2); dx = cx - cx_ref
                        area = ((x2-x1)*(y2-y1))/float(max(1.0, w*h))
                        center_score = 1.0 - min(1.0, abs(dx)/(w*0.5))
                        area_score = min(1.0, area / max(1e-6, VS_TGT_AREA_FRAC))
                        score = center_score + 0.5*conf_t + 0.3*area_score
                        if self._trk_id is not None and int(t.track_id)==self._trk_id: score += 0.35
                        candidates.append((score, int(t.track_id), dx, area))
                if candidates:
                    best = max(candidates, key=lambda c: c[0])
                    _, self._trk_id, target_dx, sel_area = best
                    src = "track"
                    self._trk_last_seen = now
                elif FORCE_FALLBACK_TO_DET and det_list:
                    pick = max(det_list, key=lambda d: (d[4])*(1.0-min(1.0, abs((0.5*(d[0]+d[2]) - cx_ref))/(w*0.5))))
                    x1,y1,x2,y2,conf,_ = pick
                    target_dx = 0.5*(x1+x2) - cx_ref
                    sel_area = ((x2-x1)*(y2-y1))/float(max(1.0, w*h))
                    src = "det"

                # Yaw PD (+ scheduling)
                dt = max(1e-3, now - self._last_loop_ts); self._last_loop_ts = now
                if target_dx is None:
                    if (now - self._trk_last_seen) > LOST_TIMEOUT_S:
                        self._trk_id = None
                    self._dx_lp = 0.0; self._dx_prev = 0.0; rc4_to_send = None
                else:
                    self._dx_lp += YAW_LP_ALPHA * (float(target_dx) - self._dx_lp)
                    d_err = (self._dx_lp - self._dx_prev) / dt; self._dx_prev = self._dx_lp
                    if abs(self._dx_lp) <= YAW_DB_PIX:
                        rc4_to_send = RC_MID
                    else:
                        err_norm = min(1.0, abs(self._dx_lp)/(w*0.5))
                        kp_eff = self.kp * (0.85 + 0.30*err_norm)
                        kd_eff = self.kd * (0.85 + 0.30*err_norm)
                        e_norm = clamp(self._dx_lp/(w*0.5), -1.0, 1.0)
                        d_norm = clamp(d_err    /(w*0.5), -1.0, 1.0)
                        yaw_cmd = clamp(kp_eff*e_norm + kd_eff*d_norm, -1.0, 1.0)
                        rc4_to_send = int(RC_MID + yaw_cmd * YAW_MAX_RC_DELTA)
                    if self.ch3 < YOLO_THR_MIN:
                        self._set_thr(YOLO_THR_MIN)

                # Lost-target scan
                if target_dx is None:
                    since = now - self._trk_last_seen
                    if since > LOST_TIMEOUT_S:
                        phase = (now % SCAN_PERIOD_S) / SCAN_PERIOD_S
                        sweep = math.sin(2.0*math.pi*phase)
                        rc4_to_send = RC_MID + int(sweep * SCAN_AMPL_RC)
                        src = "scan"

                # Forward pitch with soft-stop
                if self.track_enabled:
                    if sel_area is not None and abs(self._dx_lp) <= self.yaw_ok_px and (self.far_area <= sel_area <= self.near_area):
                        soft = clamp((self.near_area - sel_area)/max(1e-6, (self.near_area - VS_TGT_AREA_FRAC)), 0.0, 1.0)
                        rc2_raw = self._vs_pitch_from_area(sel_area)
                        rc2_to_send = int(RC_MID + soft*(rc2_raw - RC_MID))
                        base = max(self.ch3, YOLO_THR_MIN)
                        forward_strength = max(0.0, (rc2_to_send - RC_MID)/float(VS_MAX_RC_DELTA))
                        add = int(clamp(forward_strength*VS_THR_COMP_GAIN_US, 0, VS_THR_COMP_MAX_US))
                        self._set_thr(base + add)
                    else:
                        self._vs_pitch_rc = int(self._vs_pitch_rc + 0.35*(RC_MID - self._vs_pitch_rc))
                        rc2_to_send = self._vs_pitch_rc

                # Apply
                if rc4_to_send is not None: self.ch4 = clamp(rc4_to_send, RC_MIN, RC_MAX)
                if rc2_to_send is not None: self.ch2 = clamp(rc2_to_send, RC_MIN, RC_MAX)

                # Logs
                if now - self._last_print["TRACK"] >= TRACK_PRINT_PERIOD:
                    det_yes = "YES" if self._det_on else "NO"
                    rc4s = self.ch4 if rc4_to_send is not None else RC_MID
                    trks = (len(tracks) if 'tracks' in locals() else 0)
                    print(f"[TRACK] src={src or '-'} rc4={rc4s} cam={self.cam_path} det={det_yes} dets={len(det_list)} tracks={trks}")
                    self._last_print["TRACK"] = now

                # Optional preview
                if self.preview:
                    cv2.imshow("Tracker (preview)", frame)
                    k = cv2.waitKey(1) & 0xFF
                    if k in (27, ord('q')):
                        self.track_enabled = False
                        self._det_on = False

            except Exception as e:
                print("[YOLO+BT] error:", e)

            if not self.track_enabled and self._cap_running:
                self._stop_capture()

        # cleanup
        self._stop_capture()
        try:
            if self.preview:
                cv2.destroyAllWindows()
        except Exception:
            pass
        print("[TRACK] thread stopped")

# ------------------ CLI ------------------
def parse_args():
    p = argparse.ArgumentParser(description="Headless YOLO/ByteTrack RC tracker (low-latency)")
    p.add_argument("--device", default=DEVICE_DEFAULT, help="MAVLink serial (e.g., /dev/ttyACM0)")
    p.add_argument("--baud", type=int, default=BAUD_DEFAULT)
    p.add_argument("--cam", type=int, default=0, help="camera index (default 0)")
    p.add_argument("--preview", action="store_true", help="OpenCV preview window")
    p.add_argument("--lowlat", action="store_true", help="Aggressive low-latency profile")
    p.add_argument("--no-bt", action="store_true", help="Disable ByteTrack (detection-only)")
    p.add_argument("--track", action="store_true", help="Start with tracking ON on launch")
    # Camera caps
    p.add_argument("--cap-w", type=int, default=640)
    p.add_argument("--cap-h", type=int, default=360)
    p.add_argument("--cap-fps", type=int, default=20)
    p.add_argument("--fourcc", default="MJPG", help="FOURCC e.g. MJPG/YUYV; empty to skip")
    p.add_argument("--cap-buffersize", type=int, default=1, help="Driver queue size if supported")
    # Detector sizing/threads
    p.add_argument("--imgsz", type=int, default=IMGSZ_DEFAULT)
    p.add_argument("--det-downscale", type=float, default=DET_DOWNSCALE_DEFAULT)
    p.add_argument("--det-hold-s", type=float, default=DET_HOLD_S_DEFAULT)
    p.add_argument("--torch-threads", type=int, default=2)
    p.add_argument("--cv-threads", type=int, default=1)
    p.add_argument("--nice", type=int, default=None, help="process niceness, e.g., -5")
    # Tunables
    p.add_argument("--yaw-kp", type=float, default=YAW_KP_DEFAULT)
    p.add_argument("--yaw-kd", type=float, default=YAW_KD_DEFAULT)
    p.add_argument("--yaw-ok-px", type=float, default=VS_YAW_OK_PIX_DEFAULT)
    p.add_argument("--far-area", type=float, default=FAR_STOP_AREA_FRAC_DEFAULT)
    p.add_argument("--near-area", type=float, default=NEAR_STOP_AREA_FRAC_DEFAULT)
    return p.parse_args()

def main():
    args = parse_args()
    print(f"[ARGS] dev={args.device} baud={args.baud} cam={args.cam} "
          f"imgsz={args.imgsz} downscale={args.det_downscale} lowlat={'ON' if args.lowlat else 'OFF'} "
          f"cap={args.cap_w}x{args.cap_h}@{args.cap_fps} fourcc={args.fourcc} buf={args.cap_buffersize} "
          f"bt={'OFF' if args.no_bt else 'ON'} track={'ON' if args.track else 'OFF'}")
    tracker = HeadlessTracker(args)
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        tracker.running = False
        print("\n[EXIT] KeyboardInterrupt")

if __name__ == "__main__":
    main()
