#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Headless RC + YOLO/ByteTrack tracker for drones (no GUI, print-only)
- Safe throttle caps in hold contexts
- Yaw PD with simple gain-scheduling
- Forward-from-size (Pitch) with soft-stop near NEAR bound
- Lost-target gentle yaw scan
- RPi-ready downscale for inference
- CUDA/Half auto-enable when available (Jetson Orin Nano)
- Optional PS4 joystick via pygame
- ASCII-only prints for fast logging

Example logs:
CH3=1150 CH4=1437 | TRACK: ON | DET: YES
[TRACK] src=det rc4=1430 cam=/dev/video0 det=YES dets=1 tracks=0
[ATT] R=-5.6 P=-1.1 Y=-2.2
[BAT] 15.9 V  3.2 A  87 %
"""

import os, sys, glob, math, time, threading, argparse
from collections import deque

# ---- MAVLink / Joystick ----
from pymavlink import mavutil
import pygame

# ---- Vision (optional) ----
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

# ------------------ Defaults / Tuning ------------------
# RC
RC_MIN, RC_MAX, RC_MID = 1000, 2000, 1500
SEND_HZ   = 20.0
DEADZONE  = 0.10

# Safety throttle for hold contexts
THR_SAFE_MAX = 1500
THR_SLEW_US_PER_SEC = 600
YOLO_THR_MIN = 1150

# Yaw PD (+ scheduling)
YAW_DB_PIX       = 30
YAW_KP_DEFAULT   = 0.60
YAW_KD_DEFAULT   = 0.20
YAW_MAX_RC_DELTA = 320
YAW_LP_ALPHA     = 0.30

# Forward (Pitch) from target area
VS_TGT_AREA_FRAC   = 0.025
VS_DB_AREA_FRAC    = 0.004
VS_KP_PITCH        = 0.70
VS_MAX_RC_DELTA    = 220
VS_LP_ALPHA        = 0.18
VS_PITCH_FORWARD_SIGN = -1
VS_YAW_OK_PIX_DEFAULT = 40
FAR_STOP_AREA_FRAC_DEFAULT  = 0.0015
NEAR_STOP_AREA_FRAC_DEFAULT = 0.120

# Throttle compensation (gentle)
VS_THR_COMP_GAIN_US = 60
VS_THR_COMP_MAX_US  = 100

# YOLO/Tracker thresholds
YOLO_MODEL_NAME  = "yolov8n.pt"  # switch to yolov8s.pt on Jetson later
YOLO_PERSON_CLS  = 0
YOLO_CONF        = 0.15
YOLO_IOU         = 0.50
DRAW_CONF_MIN    = 0.60
BT_TRACK_THRESH  = 0.10
BT_MATCH_THRESH  = 0.80
BT_TRACK_BUFFER  = 30
FORCE_FALLBACK_TO_DET = True

# Inference downscale (RPi)
DET_DOWNSCALE = 0.67  # set 1.0 to disable

# Lost-target scan
LOST_TIMEOUT_S = 1.0
SCAN_PERIOD_S  = 3.5
SCAN_AMPL_RC   = 140

# Telemetry priorities / timing
ALT_SOURCE_HOLD_S = 1.5
ALT_CTRL_PRINT_PERIOD = 0.5
ATT_PRINT_PERIOD = 0.5
BAT_PRINT_PERIOD = 2.0
RC_PRINT_PERIOD  = 0.4
TRACK_PRINT_PERIOD = 0.25

# Serial auto-detect
if sys.platform.startswith("linux"):
    cands = glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")
    DEVICE_DEFAULT = cands[0] if cands else "/dev/ttyACM0"
else:
    DEVICE_DEFAULT = "COM40"
BAUD_DEFAULT = 115200

# Joystick axes (PS4)
if sys.platform.startswith("linux"):
    AXIS_ROLL, AXIS_PITCH, AXIS_YAW, AXIS_THR = 3, 4, 0, 1
else:
    AXIS_ROLL, AXIS_PITCH, AXIS_YAW, AXIS_THR = 0, 1, 2, 3
PITCH_INVERT, THR_INVERT, ROLL_INVERT, YAW_INVERT = True, True, False, False

# ------------------ Helpers ------------------
def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v
def apply_deadzone(val, dz=DEADZONE): return 0.0 if abs(val) < dz else val

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

# ------------------ Core Class ------------------
class HeadlessTracker:
    def __init__(self, args):
        self.args = args

        # MAVLink
        self.m = mavutil.mavlink_connection(args.device, baud=args.baud)
        self.m.wait_heartbeat(timeout=5)

        # Joystick
        pygame.init(); pygame.joystick.init()
        self.js = None
        if pygame.joystick.get_count() > 0:
            self.js = pygame.joystick.Joystick(0); self.js.init()
            print(f"[PS4] Connected: {self.js.get_name()}")
        else:
            print("[PS4] No joystick detected")

        # RC channels
        self.ch1 = RC_MID  # roll
        self.ch2 = RC_MID  # pitch
        self.ch3 = RC_MIN  # throttle
        self.ch4 = RC_MID  # yaw
        self.ch5 = 1500
        self.ch7 = 1500

        # State
        self.running = True
        self.is_armed = False
        self._thr_last = RC_MIN
        self._thr_last_ts = time.time()
        self._e_stop_active = False
        self._arm_inhibit_until = 0.0

        # Telemetry buffers
        self._att_ts = 0.0
        self.roll_deg = self.pitch_deg = self.yaw_deg = None

        self._alt_src  = None
        self._alt_ts   = 0.0
        self.alt_rel   = None

        self.groundspeed = None
        self.batt_v = None
        self.batt_a = None
        self.batt_pct = None

        # Tracking / control state
        self.yolo_enabled = False
        self.preview = args.preview and YOLO_OK
        self.cam_index = args.cam
        self.cam_path = f"/dev/video{self.cam_index}" if sys.platform.startswith("linux") else f"cam{self.cam_index}"
        self._trk_id = None
        self._trk_last_seen = 0.0
        self._dx_lp = 0.0
        self._dx_prev = 0.0
        self._last_loop_ts = time.time()
        self._last_det_flag = False
        self._vs_pitch_rc = RC_MID

        # Tunables (from args)
        self.kp = args.yaw_kp
        self.kd = args.yaw_kd
        self.yaw_ok_px = args.yaw_ok_px
        self.far_area = args.far_area
        self.near_area = args.near_area

        # Flags
        self.forward_en = True
        self.thr_comp_en = True

        # Print throttling
        self._last_print = {
            "RC": 0.0, "ATT": 0.0, "BAT": 0.0, "ALT": 0.0, "TRACK": 0.0
        }

        print(f"[LINK] Connected: sys={self.m.target_system} comp={self.m.target_component}")
        self._setup_streams()

        # Threads
        threading.Thread(target=self._send_loop, daemon=True).start()
        threading.Thread(target=self._mav_rx_loop, daemon=True).start()

        # Start detection/tracking if available
        if YOLO_OK and BT_OK and not args.no_yolo:
            self.yolo_enabled = True
            threading.Thread(target=self._yolo_loop, daemon=True).start()
            print("[DETECT] ON")
        else:
            print("[DETECT] OFF (YOLO/BT unavailable or --no-yolo)")

    # ---------- Streams ----------
    def _setup_streams(self):
        # set msg rates
        for mid, hz in (
            (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,            10),
            (mavutil.mavlink.MAVLINK_MSG_ID_ALTITUDE,            10),
            (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10),
            (mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,              5),
            (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,           1),
        ):
            try:
                interval_us = int(1e6 / float(hz)) if hz > 0 else 0
                self.m.mav.command_long_send(
                    self.m.target_system, self.m.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0, mid, interval_us, 0,0,0,0,0
                )
            except Exception as e:
                print(f"[MSG_RATE] {mid}@{hz}Hz failed:", e)

    # ---------- Safety/hold ----------
    def _is_hold_context(self):
        # Headless: treat GUIDED/ALT_HOLD as hold contexts via CH3 cap only when needed
        # (We don't toggle modes here; assume external flight mode)
        return True  # conservative: always cap throttle changes

    def _set_thr(self, value: int):
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
        self.ch3 = rc

    # ---------- RC send ----------
    def _send_override(self):
        rc1, rc2, rc3, rc4, rc5, rc7 = self.ch1, self.ch2, self.ch3, self.ch4, self.ch5, self.ch7
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
            trk = "ON" if self.yolo_enabled else "OFF"
            det = "YES" if self._last_det_flag else "NO"
            print(f"[RC] CH1={rc1} CH2={rc2} CH3={rc3} CH4={rc4} | TRACK: {trk} | DET: {det}")
            self._last_print["RC"] = now

    # ---------- Joystick/RC loop ----------
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

                    # throttle (unless YOLO comp active)
                    if not (self.yolo_enabled and self.thr_comp_en):
                        self._set_thr(clamp(t, RC_MIN, RC_MAX))
                    # yaw manual only if no tracker
                    if not self.yolo_enabled:
                        self.ch4 = clamp(y, RC_MIN, RC_MAX)
                    # pitch manual unless forward enabled
                    if not (self.yolo_enabled and self.forward_en):
                        self.ch2 = clamp(p, RC_MIN, RC_MAX)

                    self.ch1 = clamp(r, RC_MIN, RC_MAX)

                    # Buttons: X=ARM (0), O=DISARM (1), TRI=E-STOP (3)
                    b0 = 1 if self.js.get_button(0) else 0
                    b1 = 1 if self.js.get_button(1) else 0
                    b3 = 1 if self.js.get_button(3) else 0
                    # Just log requests (actual arming handled by FC)
                    if b0: print("[ARM] requested")
                    if b1: print("[DISARM] requested")
                    if b3:
                        print("[E-STOP] requested")
                        self._e_stop()

                except Exception as e:
                    print("[PS4] joystick error:", e); self.js = None

            self._send_override()
            time.sleep(per)

    # ---------- MAV RX ----------
    def _mav_rx_loop(self):
        last_att = last_bat = 0.0
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
                        print("[ARMED]" if self.is_armed else "[DISARMED]")
                except Exception:
                    pass

            elif t == "SYS_STATUS":
                voltage = msg.voltage_battery / 1000.0
                current = msg.current_battery / 100.0
                remaining = msg.battery_remaining
                self.batt_v = voltage
                self.batt_a = current if current > -9000 else None
                self.batt_pct = remaining if remaining >= 0 else None

                now = time.time()
                if now - self._last_print["BAT"] >= BAT_PRINT_PERIOD:
                    v = f"{self.batt_v:.1f} V" if self.batt_v is not None else "-- V"
                    a = f"{self.batt_a:.1f} A" if self.batt_a is not None else "- A"
                    p = f"{self.batt_pct} %" if self.batt_pct is not None else "-"
                    print(f"[BAT] {v} {a} {p}")
                    self._last_print["BAT"] = now

            elif t == "ATTITUDE":
                try:
                    self.roll_deg  = _rad2deg_wrap(float(msg.roll))
                    self.pitch_deg = _rad2deg_wrap(float(msg.pitch))
                    self.yaw_deg   = _rad2deg_wrap(float(msg.yaw))
                    self._att_ts = time.time()
                    if self._att_ts - self._last_print["ATT"] >= ATT_PRINT_PERIOD:
                        print(f"[ATT] R={self.roll_deg:+.1f} P={self.pitch_deg:+.1f} Y={self.yaw_deg:+.1f}")
                        self._last_print["ATT"] = self._att_ts
                except Exception:
                    pass

            elif t == "VFR_HUD":
                try:
                    self.groundspeed = float(msg.groundspeed)
                except Exception:
                    pass

            elif t == "ALTITUDE":
                try:
                    self.alt_rel = float(getattr(msg, "altitude_relative", None))
                    self._alt_src = "ALTITUDE"; self._alt_ts = time.time()
                except Exception:
                    pass

            elif t == "GLOBAL_POSITION_INT":
                try:
                    if (time.time() - self._alt_ts) > ALT_SOURCE_HOLD_S:
                        self.alt_rel = float(msg.relative_alt)/1000.0
                        self._alt_src = "GPI"; self._alt_ts = time.time()
                except Exception:
                    pass

    # ---------- Estop ----------
    def _e_stop(self):
        if self._e_stop_active: return
        self._e_stop_active = True
        try:
            # Neutralize sticks + min throttle briefly
            t0 = time.time()
            while time.time() - t0 < 0.5:
                self.ch1 = RC_MID; self.ch2 = RC_MID; self.ch4 = RC_MID; self._set_thr(RC_MIN)
                self._send_override(); time.sleep(0.05)
            # Request BRAKE then DISARM force via MAV_CMD if desired (optional; keeping minimal here)
        finally:
            self._e_stop_active = False

    # ---------- Visual forward ----------
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

    # ---------- YOLO + ByteTrack ----------
    def _yolo_loop(self):
        try:
            model = _YOLO(YOLO_MODEL_NAME)
        except Exception as e:
            print("YOLO load failed:", e); self.yolo_enabled=False; return

        # Try CUDA/Half if available (Jetson)
        try:
            if torch.cuda.is_available():
                model.to("cuda")
                try: model.model.half()
                except Exception: pass
        except Exception:
            pass

        # Camera open
        if sys.platform.startswith("win"):
            cap = cv2.VideoCapture(self.cam_index, cv2.CAP_DSHOW)
        else:
            cap = cv2.VideoCapture(self.cam_index)
        if not cap.isOpened():
            print(f"[CAM] cannot open index {self.cam_index}")
            self.yolo_enabled=False; return

        fps = cap.get(cv2.CAP_PROP_FPS)
        fps = float(fps) if fps and fps > 0 else 30.0
        bt_args = SimpleNamespace(track_thresh=BT_TRACK_THRESH, match_thresh=BT_MATCH_THRESH,
                                  track_buffer=BT_TRACK_BUFFER, frame_rate=fps, mot20=False)
        tracker = BYTETracker(bt_args)

        print(f"[CAM] opened: {self.cam_path}")
        print("[TRACK] ON")

        while self.running and self.yolo_enabled:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.01); continue

            h, w = frame.shape[:2]
            cx_ref = w * 0.5
            rc4_to_send = None
            rc2_to_send = None
            sel_bbox = None
            src = None
            area_frac = None
            target_dx = None

            try:
                # Downscale for inference
                if DET_DOWNSCALE and 0.2 <= DET_DOWNSCALE < 1.0:
                    dw, dh = int(w * DET_DOWNSCALE), int(h * DET_DOWNSCALE)
                    infer_frame = cv2.resize(frame, (dw, dh), interpolation=cv2.INTER_AREA)
                    scale_x, scale_y = w / float(dw), h / float(dh)
                else:
                    infer_frame = frame
                    scale_x = scale_y = 1.0

                results = model(infer_frame, conf=YOLO_CONF, iou=YOLO_IOU, verbose=False)
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

                dets = torch.tensor(det_list, dtype=torch.float32) if det_list else torch.empty((0,6), dtype=torch.float32)
                tracks = tracker.update(dets, (h, w), (h, w))

                # choose target
                candidates = []
                for t in tracks:
                    conf_t = float(getattr(t, "score", 0.0))
                    if conf_t < DRAW_CONF_MIN: continue
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
                    if (self._trk_id is None) or (cand_id == self._trk_id) or ((now - self._trk_last_seen) > 1.5):
                        self._trk_id = cand_id
                    if cand_id == self._trk_id:
                        target_dx = cand_dx
                        sel_bbox = cand_bbox
                        area_frac = cand_area
                        src = "track"
                        self._trk_last_seen = now

                elif FORCE_FALLBACK_TO_DET and det_list:
                    pick = max(det_list, key=lambda d: (d[4]) * (1.0 - min(1.0, abs((0.5*(d[0]+d[2]) - cx_ref)) / (w*0.5))))
                    x1, y1, x2, y2, conf, _ = pick
                    dx = 0.5 * (x1 + x2) - cx_ref
                    sel_bbox = (x1, y1, x2, y2)
                    area_frac = ((x2 - x1) * (y2 - y1)) / float(max(1.0, w * h))
                    target_dx = dx
                    src = "det"

                # Yaw PD + scheduling
                now = time.time()
                dt = max(1e-3, now - self._last_loop_ts)
                self._last_loop_ts = now

                if target_dx is None:
                    if (now - self._trk_last_seen) > LOST_TIMEOUT_S:
                        self._trk_id = None
                    self._dx_lp = 0.0
                    self._dx_prev = 0.0
                    rc4_to_send = None
                else:
                    self._dx_lp += YAW_LP_ALPHA * (float(target_dx) - self._dx_lp)
                    d_err = (self._dx_lp - self._dx_prev) / dt
                    self._dx_prev = self._dx_lp

                    if abs(self._dx_lp) <= YAW_DB_PIX:
                        rc4_to_send = RC_MID
                    else:
                        err_norm = min(1.0, abs(self._dx_lp) / (w * 0.5))
                        kp_eff = self.kp * (0.85 + 0.30 * err_norm)
                        kd_eff = self.kd * (0.85 + 0.30 * err_norm)

                        e_norm = clamp(self._dx_lp / (w * 0.5), -1.0, 1.0)
                        d_norm = clamp(d_err     / (w * 0.5), -1.0, 1.0)
                        yaw_cmd = clamp(kp_eff * e_norm + kd_eff * d_norm, -1.0, 1.0)
                        rc4_to_send = int(RC_MID + yaw_cmd * YAW_MAX_RC_DELTA)

                    if self.ch3 < YOLO_THR_MIN:
                        self._set_thr(YOLO_THR_MIN)

                # Lost-target gentle scan
                if target_dx is None:
                    since = now - self._trk_last_seen
                    if since > LOST_TIMEOUT_S:
                        phase = (now % SCAN_PERIOD_S) / SCAN_PERIOD_S
                        sweep = math.sin(2.0 * math.pi * phase)
                        rc4_to_send = RC_MID + int(sweep * SCAN_AMPL_RC)
                        src = "scan"

                # Pitch forward with soft-stop near NEAR
                if self.forward_en:
                    if sel_bbox is not None and (area_frac is not None) \
                       and abs(self._dx_lp) <= self.yaw_ok_px \
                       and (self.far_area <= area_frac <= self.near_area):
                        if self.near_area > VS_TGT_AREA_FRAC:
                            soft = clamp((self.near_area - area_frac) / (self.near_area - VS_TGT_AREA_FRAC), 0.0, 1.0)
                        else:
                            soft = 1.0
                        rc2_raw = self._vs_pitch_from_area(area_frac)
                        rc2_to_send = int(RC_MID + soft * (rc2_raw - RC_MID))
                        if self.thr_comp_en:
                            base = max(self.ch3, YOLO_THR_MIN)
                            forward_strength = max(0.0, (rc2_to_send - RC_MID) / float(VS_MAX_RC_DELTA))
                            add = int(clamp(forward_strength * VS_THR_COMP_GAIN_US, 0, VS_THR_COMP_MAX_US))
                            self._set_thr(base + add)
                    else:
                        self._vs_pitch_rc = int(self._vs_pitch_rc + 0.35 * (RC_MID - self._vs_pitch_rc))
                        rc2_to_send = self._vs_pitch_rc
                else:
                    self._vs_pitch_rc = int(self._vs_pitch_rc + 0.2*(RC_MID - self._vs_pitch_rc))

                # Apply RC from tracker
                if rc4_to_send is not None:
                    self.ch4 = clamp(rc4_to_send, RC_MIN, RC_MAX)
                if rc2_to_send is not None:
                    self.ch2 = clamp(rc2_to_send, RC_MIN, RC_MAX)

                # Prints
                self._last_det_flag = bool(det_list)
                nowp = time.time()
                if nowp - self._last_print["TRACK"] >= TRACK_PRINT_PERIOD:
                    det_yes = "YES" if det_list else "NO"
                    trks = len(tracks) if 'tracks' in locals() else 0
                    src_s = src or "-"
                    rc4s = self.ch4 if rc4_to_send is not None else RC_MID
                    print(f"[TRACK] src={src_s} rc4={rc4s} cam={self.cam_path} det={det_yes} dets={len(det_list)} tracks={trks}")
                    self._last_print["TRACK"] = nowp

                # Preview (optional)
                if self.preview:
                    annotated = frame.copy()
                    if sel_bbox is not None:
                        x1,y1,x2,y2 = map(int, sel_bbox)
                        cv2.rectangle(annotated, (x1,y1),(x2,y2),(0,128,255),2)
                    cv2.imshow("Tracker (preview)", annotated)
                    k = cv2.waitKey(1) & 0xFF
                    if k in (27, ord('q')):
                        self.yolo_enabled = False
                        break

            except Exception as e:
                print("[YOLO+BT] error:", e)

        # cleanup
        try:
            cap.release()
            if self.preview: cv2.destroyAllWindows()
            print(f"[CAM] released: {self.cam_path}")
        except Exception:
            pass
        print("[TRACK] OFF")

# ------------------ CLI ------------------
def parse_args():
    p = argparse.ArgumentParser(description="Headless YOLO/ByteTrack RC tracker")
    p.add_argument("--device", default=DEVICE_DEFAULT, help="MAVLink serial (e.g., /dev/ttyACM0 or COM40)")
    p.add_argument("--baud", type=int, default=BAUD_DEFAULT)
    p.add_argument("--cam", type=int, default=0, help="camera index (default 0)")
    p.add_argument("--no-yolo", action="store_true", help="disable detection/tracking")
    p.add_argument("--preview", action="store_true", help="show OpenCV preview window")
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
          f"kp={args.yaw_kp} kd={args.yaw_kd} yaw_ok={args.yaw_ok_px} "
          f"far={args.far_area} near={args.near_area} preview={'ON' if args.preview else 'OFF'} "
          f"detect={'OFF' if args.no_yolo else 'ON'}")

    tracker = HeadlessTracker(args)
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        tracker.running = False
        print("\n[EXIT] KeyboardInterrupt")

if __name__ == "__main__":
    main()
