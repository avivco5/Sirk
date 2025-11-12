#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ASCII-only, english comments

Tracker + MAVLink proxy (+ optional YOLO/DeepSORT + visual tracking RC override).
- Connects to ArduPilot SITL over TCP.
- Forwards MAVLink to GUI on UDP 14550.
- Listens for control commands on UDP 9104.
- Publishes bbox JSON to UDP 9103 when YOLO is running.
- Supports: ARM/DISARM, EMERGENCY_STOP, SET_MODE, HOLD, ALT_BUMP, GOTO_GPS, TAKEOFF.
- New: START_YOLO / STOP_YOLO / SET_CLASSES, START_TRACKING / STOP_TRACKING
- Visual controller: PD/LPF on dx for yaw, area-based forward pitch with deadband+LPF.
"""

import os
import cv2
import json
import time
import math
import socket
import threading

from pymavlink import mavutil

# ============================= Global time base ==============================
BOOT_TIME_MONO = None  # time since boot for MAVLink messages

# ============================= MAVLink Settings =============================
# TCP to WSL SITL (replace IP if your WSL IP changes)
MAVLINK_DEVICE = os.environ.get("MAVLINK_DEVICE", "tcp:172.20.186.151:5760")
MAVLINK_BAUD   = 115200
MAVLINK_CONN   = None

GUIDED_KEEPALIVE_PERIOD = 0.5  # seconds
SAFE_DEFAULT_ALT = 5.0         # meters

# Debounce for ARM/DISARM
last_arm_sent_ts = 0.0
last_disarm_sent_ts = 0.0

# Auto behaviors
MAV_MODE_FLAG_SAFETY_ARMED = 0x80
AUTO_TAKEOFF_ON_GOTO = True
AUTO_TAKEOFF_ALT     = 5.0

# ============================ Communication Ports ===========================
UDP_IP        = '127.0.0.1'
UDP_PORT_TEL  = 14550   # GUI listens here for forwarded MAVLink
UDP_PORT_CMD  = 9104    # Commands in from GUI (JSON)
UDP_PORT_BBOX = 9103    # BBox out to GUI (JSON)

# =============================== MAV Helpers ================================
APM_COPTER_MODES = {
    "STABILIZE": 0, "ACRO": 1, "ALT_HOLD": 2, "AUTO": 3, "GUIDED": 4, "LOITER": 5,
    "RTL": 6, "CIRCLE": 7, "LAND": 9, "DRIFT": 11, "SPORT": 13, "FLIP": 14,
    "AUTOTUNE": 15, "POSHOLD": 16, "BRAKE": 17, "THROW": 18, "AVOID_ADSB": 19,
    "GUIDED_NOGPS": 20, "SMART_RTL": 21, "FLOWHOLD": 22, "FOLLOW": 23,
    "ZIGZAG": 24, "SYSTEMID": 25, "AUTOROTATE": 26, "AUTO_RTL": 27
}

# put near other imports / threads
def _gcs_heartbeat_loop():
    # send GCS heartbeat at 1 Hz so FC recognizes us as a ground station
    while True:
        try:
            if MAVLINK_CONN:
                MAVLINK_CONN.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0
                )
        except Exception:
            pass
        time.sleep(1.0)


def current_mode_name():
    try:
        return MAVLINK_CONN.flightmode
    except Exception:
        return None

def is_armed():
    hb = MAVLINK_CONN.messages.get('HEARTBEAT') if MAVLINK_CONN else None
    if not hb:
        return False
    return bool(hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)

def wait_for_mode(target_mode, timeout=8.0):
    # wait until heartbeat shows the requested flightmode
    t0 = time.time()
    target_mode = (target_mode or "").upper()
    while time.time() - t0 < timeout:
        try:
            # pump one message to refresh flightmode
            MAVLINK_CONN.recv_match(blocking=False)
        except Exception:
            pass
        try:
            if (MAVLINK_CONN.flightmode or "").upper() == target_mode:
                return True
        except Exception:
            pass
        time.sleep(0.05)
    return False


def set_mode(mode_name, allow_nogps_fallback=True):
    # robust ArduCopter mode set: SET_MODE + DO_SET_MODE + diagnostics
    if MAVLINK_CONN is None or not mode_name:
        return False
    try:
        mode_name_u = mode_name.upper()
        mode_id = APM_COPTER_MODES.get(mode_name_u)
        if mode_id is None:
            print(f"[MAV-CMD] Unknown mode '{mode_name}'")
            return False

        ts = MAVLINK_CONN.target_system or 1
        tc = MAVLINK_CONN.target_component or 1

        # send both flavors
        try:
            MAVLINK_CONN.mav.set_mode_send(
                ts, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id
            )
        except Exception as e:
            print(f"[MAV-CMD] set_mode_send warn: {e}")

        try:
            MAVLINK_CONN.mav.command_long_send(
                ts, tc, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # base_mode
                mode_id, 0, 0, 0, 0, 0
            )
        except Exception as e:
            print(f"[MAV-CMD] DO_SET_MODE warn: {e}")

        ok = wait_for_mode(mode_name_u, timeout=8.0)
        print(f"[MAV-CMD] Mode -> {mode_name_u} ({'ok' if ok else 'pending'})")
        if ok:
            return True

        # not ok: print blockers and optionally fallback to GUIDED_NOGPS
        print_guided_blockers()
        if (mode_name_u == "GUIDED") and allow_nogps_fallback:
            print("[MAV-CMD] Trying GUIDED_NOGPS fallback (diagnostic)...")
            gid = APM_COPTER_MODES.get("GUIDED_NOGPS")
            if gid is not None:
                try:
                    MAVLINK_CONN.mav.set_mode_send(
                        ts, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, gid
                    )
                    MAVLINK_CONN.mav.command_long_send(
                        ts, tc, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, gid, 0,0,0,0,0
                    )
                except Exception as e:
                    print(f"[MAV-CMD] GUIDED_NOGPS warn: {e}")
                ok2 = wait_for_mode("GUIDED_NOGPS", timeout=5.0)
                print(f"[MAV-CMD] Mode -> GUIDED_NOGPS ({'ok' if ok2 else 'pending'})")
                return ok2
        return False
    except Exception as e:
        print(f"[MAV-CMD] set_mode error: {e}")
        return False



def request_telemetry_streams():
    if MAVLINK_CONN is None:
        return
    try:
        MAVLINK_CONN.mav.request_data_stream_send(
            MAVLINK_CONN.target_system,
            MAVLINK_CONN.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            10, 1
        )
    except Exception:
        pass

# ASCII-only, english comments

def _pump_msgs_for(seconds=0.8):
    # non-blocking pump to refresh MAVLINK_CONN.messages
    t0 = time.time()
    while time.time() - t0 < seconds:
        try:
            MAVLINK_CONN.recv_match(blocking=False)
        except Exception:
            pass
        time.sleep(0.02)

def _get_fix_type():
    # returns GPS fix type or None
    try:
        gps = MAVLINK_CONN.messages.get("GPS_RAW_INT")
        return int(gps.fix_type) if gps else None
    except Exception:
        return None

def _get_ekf_flags():
    # returns EKF flags bitmask or None
    try:
        ekf = MAVLINK_CONN.messages.get("EKF_STATUS_REPORT")
        return int(ekf.flags) if ekf else None
    except Exception:
        return None

def _ekf_ok(flags):
    # minimal healthy subset: attitude + velocity + pos rel + pos abs
    if flags is None:
        return False
    need = (1 | 2 | 8 | 16)  # bits from ArduPilot EKF_STATUS_REPORT
    return (flags & need) == need

def print_guided_blockers():
    # quick one-shot diagnostics to explain GUIDED blockers
    _pump_msgs_for(0.5)
    fix = _get_fix_type()
    ekf = _get_ekf_flags()
    hb  = MAVLINK_CONN.messages.get("HEARTBEAT")
    txt = MAVLINK_CONN.messages.get("STATUSTEXT")

    if fix is not None:
        print(f"[DIAG] GPS fix_type={fix} (need >=3 for 3D)")
    else:
        print("[DIAG] GPS fix_type: None")

    if ekf is not None:
        print(f"[DIAG] EKF flags=0x{ekf:08X} ok={_ekf_ok(ekf)}")
    else:
        print("[DIAG] EKF flags: None yet")

    if hb:
        armed = bool(hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
        print(f"[DIAG] flightmode='{getattr(MAVLINK_CONN,'flightmode',None)}' armed={armed}")

    if txt:
        try:
            print("[DIAG] STATUSTEXT:", txt.text)
        except Exception:
            pass

def ensure_guided_ready(max_wait=8.0):
    # wait for GPS 3D fix and EKF usable; return True if ready
    t0 = time.time()
    while time.time() - t0 < max_wait:
        _pump_msgs_for(0.25)
        fix = _get_fix_type()
        ekf = _get_ekf_flags()
        if (fix is not None and fix >= 3) and _ekf_ok(ekf):
            return True
        time.sleep(0.25)
    print_guided_blockers()
    return False


def set_mode(mode_name, allow_nogps_fallback=True):
    # robust ArduCopter mode set: SET_MODE + DO_SET_MODE + readiness checks
    if MAVLINK_CONN is None or not mode_name:
        return False
    try:
        mode_name_u = mode_name.upper()
        mode_id = APM_COPTER_MODES.get(mode_name_u)
        if mode_id is None:
            print(f"[MAV-CMD] Unknown mode '{mode_name}'")
            return False

        # for GUIDED, try to ensure sensors are ready
        if mode_name_u == "GUIDED":
            ensure_guided_ready(max_wait=6.0)

        ts = MAVLINK_CONN.target_system or 1
        tc = MAVLINK_CONN.target_component or 1

        # 1) SET_MODE with CUSTOM flag
        try:
            MAVLINK_CONN.mav.set_mode_send(
                ts, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id
            )
        except Exception as e:
            print(f"[MAV-CMD] set_mode_send warn: {e}")

        # 2) DO_SET_MODE fallback
        try:
            MAVLINK_CONN.mav.command_long_send(
                ts, tc, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # base_mode
                mode_id, 0, 0, 0, 0, 0
            )
        except Exception as e:
            print(f"[MAV-CMD] DO_SET_MODE warn: {e}")

        ok = wait_for_mode(mode_name_u, timeout=8.0)
        print(f"[MAV-CMD] Mode -> {mode_name_u} ({'ok' if ok else 'pending'})")
        if ok:
            return True

        # diagnostics + optional fallback
        if (mode_name_u == "GUIDED") and allow_nogps_fallback:
            print("[MAV-CMD] Trying GUIDED_NOGPS fallback...")
            gid = APM_COPTER_MODES.get("GUIDED_NOGPS")
            if gid is not None:
                try:
                    MAVLINK_CONN.mav.set_mode_send(
                        ts, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, gid
                    )
                    MAVLINK_CONN.mav.command_long_send(
                        ts, tc, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, gid, 0,0,0,0,0
                    )
                except Exception as e:
                    print(f"[MAV-CMD] GUIDED_NOGPS warn: {e}")
                ok2 = wait_for_mode("GUIDED_NOGPS", timeout=5.0)
                print(f"[MAV-CMD] Mode -> GUIDED_NOGPS ({'ok' if ok2 else 'pending'})")
                return ok2
        return False
    except Exception as e:
        print(f"[MAV-CMD] set_mode error: {e}")
        return False


def arm_motors():
    global last_arm_sent_ts
    if MAVLINK_CONN is None:
        return
    if time.time() - last_arm_sent_ts < 0.5:
        return
    last_arm_sent_ts = time.time()
    try:
        MAVLINK_CONN.arducopter_arm()
        print("[MAV-CMD] ARM sent")
    except Exception as e:
        print(f"[MAV-CMD] ARM error: {e}")

def disarm_motors():
    global last_disarm_sent_ts
    if MAVLINK_CONN is None:
        return
    if time.time() - last_disarm_sent_ts < 0.5:
        return
    last_disarm_sent_ts = time.time()
    try:
        MAVLINK_CONN.arducopter_disarm()
        print("[MAV-CMD] DISARM sent")
    except Exception as e:
        print(f"[MAV-CMD] DISARM error: {e}")

def go_to_gps(lat_deg, lon_deg, alt_rel_m):
    if MAVLINK_CONN is None:
        return
    if lat_deg is None or lon_deg is None:
        print("[MAV-CMD] GOTO_GPS missing lat/lon"); return
    try:
        if current_mode_name() != 'GUIDED':
            if not set_mode('GUIDED'): return
        if not is_armed() and AUTO_TAKEOFF_ON_GOTO:
            arm_motors()
            time.sleep(0.2)
            takeoff_to_alt(AUTO_TAKEOFF_ALT)
            time.sleep(0.2)

        type_mask = 0b0000111111000111  # ignore vel/accel/yaw
        MAVLINK_CONN.mav.set_position_target_global_int_send(
            int((time.monotonic() - (BOOT_TIME_MONO or time.monotonic())) * 1000),
            MAVLINK_CONN.target_system, MAVLINK_CONN.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            type_mask,
            int(float(lat_deg) * 1e7), int(float(lon_deg) * 1e7), float(alt_rel_m),
            0, 0, 0, 0, 0, 0, 0.0, 0.0
        )
        print(f"[MAV-CMD] GOTO -> {lat_deg:.7f},{lon_deg:.7f} alt={alt_rel_m:.1f}")
    except Exception as e:
        print(f"[MAV-CMD] GOTO error: {e}")

def hold_here_current_alt():
    if MAVLINK_CONN is None:
        return
    gpos = MAVLINK_CONN.messages.get('GLOBAL_POSITION_INT')
    vfr  = MAVLINK_CONN.messages.get('VFR_HUD')
    if not gpos:
        print("[MAV-CMD] HOLD failed: no GPS"); return
    lat = gpos.lat / 1e7
    lon = gpos.lon / 1e7
    alt = float(vfr.alt) if vfr else SAFE_DEFAULT_ALT
    if current_mode_name() != 'GUIDED':
        if not set_mode('GUIDED'): return
    go_to_gps(lat, lon, alt)
    print(f"[MAV-CMD] HOLD @ Lat:{lat:.5f}, Lon:{lon:.5f}, Alt:{alt:.1f}m")

def bump_alt_guided(delta_alt):
    if MAVLINK_CONN is None:
        return
    gpos = MAVLINK_CONN.messages.get('GLOBAL_POSITION_INT')
    vfr  = MAVLINK_CONN.messages.get('VFR_HUD')
    if not gpos:
        print("[MAV-CMD] ALT_BUMP failed: No GPS."); return
    lat = gpos.lat / 1e7
    lon = gpos.lon / 1e7
    base_alt = float(vfr.alt) if vfr else SAFE_DEFAULT_ALT
    new_alt = max(1.0, base_alt + float(delta_alt))
    go_to_gps(lat, lon, new_alt)
    print(f"[MAV-CMD] ALT_BUMP -> {new_alt:.1f}m")

def takeoff_to_alt(alt_m):
    if MAVLINK_CONN is None:
        return
    if current_mode_name() != 'GUIDED':
        if not set_mode('GUIDED'):
            print("[MAV-CMD] TAKEOFF: failed to set GUIDED"); return
    arm_motors()
    time.sleep(0.2)
    MAVLINK_CONN.mav.command_long_send(
        MAVLINK_CONN.target_system, MAVLINK_CONN.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0,0,0,0, 0,0, float(alt_m)
    )
    print(f"[MAV-CMD] TAKEOFF to {float(alt_m):.1f}m sent.")

def emergency_stop():
    if MAVLINK_CONN is None:
        return
    try:
        # neutral sticks with minimal throttle for a short period
        t0 = time.time()
        while time.time() - t0 < 0.5:
            send_rc_override(RC_MID, RC_MID, RC_MIN, RC_MID)
            time.sleep(0.05)
        # BRAKE (fallback LOITER)
        try:
            set_mode("BRAKE")
        except Exception:
            try:
                set_mode("LOITER")
            except Exception:
                pass
        print("[E-STOP] applied")
    except Exception as e:
        print(f"[E-STOP] error: {e}")

# ============================ RC Override Helpers ===========================
RC_MIN = 1000
RC_MID = 1500
RC_MAX = 2000
VS_MAX_RC_DELTA = 400          # max deviation from mid for yaw/pitch
THR_SAFE_MAX    = 1650         # cap throttle when tracking forward

def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

def send_rc_override(roll=None, pitch=None, thr=None, yaw=None):
    if MAVLINK_CONN is None:
        return
    try:
        ch = [0]*8
        # ArduCopter default: ch1 roll, ch2 pitch, ch3 throttle, ch4 yaw
        if roll  is not None: ch[0] = int(clamp(int(roll),  RC_MIN, RC_MAX))
        if pitch is not None: ch[1] = int(clamp(int(pitch), RC_MIN, RC_MAX))
        if thr   is not None: ch[2] = int(clamp(int(thr),   RC_MIN, RC_MAX))
        if yaw   is not None: ch[3] = int(clamp(int(yaw),   RC_MIN, RC_MAX))
        MAVLINK_CONN.mav.rc_channels_override_send(
            MAVLINK_CONN.target_system, MAVLINK_CONN.target_component,
            ch[0], ch[1], ch[2], ch[3], ch[4], ch[5], ch[6], ch[7]
        )
    except Exception as e:
        print(f"[RC-OVR] error: {e}")

# ============================ Vision / Tracking =============================
# YOLO settings (CPU)
YOLO_MODEL_NAME = os.environ.get("YOLO_MODEL", "../../yolov8n.pt")
TARGET_CLASSES  = None   # string like "0,1,2" or None to use all
_yolo_thread    = None
_yolo_stop      = threading.Event()

_track_thread   = None
_track_stop     = threading.Event()
_tracking_on    = False

# Shared state for bbox
_bbox_lock      = threading.Lock()
_last_bbox      = None    # (x1,y1,x2,y2)
_last_dx        = 0.0     # -1..1 (center offset)
_last_area_frac = 0.0     # 0..1

# Controller params (inspired by Hybrid logic)
YAW_KP      = 0.7
YAW_DB      = 0.03
YAW_LP_A    = 0.35

VS_TGT_AREA_FRAC = 0.10    # desired normalized area (fraction of frame)
VS_DB_AREA_FRAC  = 0.02
VS_KP_PITCH      = 0.9
VS_PITCH_SIGN    = +1.0    # positive -> forward when target is smaller than desired
VS_LP_ALPHA      = 0.25

# Internal LP state
_yaw_rc_lp   = RC_MID
_pitch_rc_lp = RC_MID

def _publish_bbox_udp(bbox, conf, area, dx, src="yolo"):
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        payload = {
            "ts": time.time(),
            "bbox": [float(bbox[0]), float(bbox[1]), float(bbox[2]), float(bbox[3])],
            "conf": float(conf) if conf is not None else None,
            "area_frac": float(area) if area is not None else None,
            "dx": float(dx) if dx is not None else None,
            "src": str(src or "")
        }
        sock.sendto(json.dumps(payload).encode("utf-8"), (UDP_IP, UDP_PORT_BBOX))
    except Exception as e:
        print("[PUB-UDP] send err:", e)

def _resolve_class_ids(model, classes_csv):
    if not classes_csv:
        return None
    try:
        wanted = set(int(x.strip()) for x in str(classes_csv).split(",") if x.strip() != "")
    except Exception:
        return None
    try:
        names = model.names if hasattr(model, "names") else None
        if names is None:
            return sorted(wanted)
        if isinstance(names, dict):
            max_id = max(names.keys()) if names else -1
        else:
            max_id = len(names) - 1
        return sorted([i for i in wanted if 0 <= i <= max_id])
    except Exception:
        return sorted(wanted)

def _yolo_loop_worker():
    global _last_bbox, _last_dx, _last_area_frac
    from ultralytics import YOLO as _YOLO
    try:
        model = _YOLO(YOLO_MODEL_NAME)
    except Exception as e:
        print("YOLO load failed:", e); return

    # force CPU
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
        print("[YOLO] CPU float32")
    except Exception as e:
        print("[YOLO] CPU setup warn:", e)

    # open camera (index 0 fallback)
    cap = None
    for idx in [0, 1, 2]:
        tmp = cv2.VideoCapture(idx)
        if tmp.isOpened():
            cap = tmp
            print(f"[CAM] opened index {idx}")
            break
    if cap is None:
        print("[YOLO] No camera found"); return

    try:
        while not _yolo_stop.is_set():
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.01); continue

            h, w = frame.shape[:2]
            res = model(frame, verbose=False)[0]
            best = None
            for box in res.boxes:
                cls_id = int(box.cls[0].item()) if hasattr(box, "cls") else -1
                if TARGET_CLASSES is not None:
                    # allow only wanted classes
                    try:
                        allow = cls_id in set(int(x) for x in str(TARGET_CLASSES).split(",") if x.strip()!="")
                        if not allow: continue
                    except Exception:
                        pass
                conf = float(box.conf[0].item()) if hasattr(box, "conf") else 0.0
                x1,y1,x2,y2 = [float(v) for v in box.xyxy[0].tolist()]
                area_frac = max(0.0, min(1.0, ((x2-x1)*(y2-y1)) / float(w*h)))
                cx = 0.5*(x1+x2); dx = ((cx / w) - 0.5) * 2.0
                best = (x1,y1,x2,y2, conf, area_frac, dx)
                break  # take first for simplicity

            if best is not None:
                x1,y1,x2,y2, conf, area_frac, dx = best
                with _bbox_lock:
                    _last_bbox = (x1,y1,x2,y2)
                    _last_dx   = float(dx)
                    _last_area_frac = float(area_frac)
                _publish_bbox_udp((x1,y1,x2,y2), conf, area_frac, dx, "yolo")
            else:
                with _bbox_lock:
                    _last_bbox = None
            time.sleep(0.01)
    except Exception as e:
        print("[YOLO] loop error:", e)
    finally:
        try: cap.release()
        except Exception: pass
        print("[YOLO] stopped")

def _rc_from_dx(dx):
    global _yaw_rc_lp
    # deadband
    err = float(dx)
    if abs(err) < YAW_DB:
        rc = RC_MID
    else:
        rc = int(RC_MID + clamp(err, -1.0, 1.0) * YAW_KP * VS_MAX_RC_DELTA)
    # LPF
    _yaw_rc_lp = int(_yaw_rc_lp + YAW_LP_A*(rc - _yaw_rc_lp))
    return clamp(_yaw_rc_lp, RC_MIN, RC_MAX)

def _rc_pitch_from_area(area_frac):
    global _pitch_rc_lp
    # like _vs_pitch_from_area in Hybrid (deadband + proportional + LPF)
    err = VS_TGT_AREA_FRAC - max(0.0, min(1.0, float(area_frac)))
    if abs(err) < VS_DB_AREA_FRAC:
        rc2 = RC_MID
    else:
        norm = clamp(err / max(VS_TGT_AREA_FRAC, 1e-6), -1.0, 1.0)
        rc2 = int(RC_MID + VS_PITCH_SIGN * norm * VS_KP_PITCH * VS_MAX_RC_DELTA)
    _pitch_rc_lp = int(_pitch_rc_lp + VS_LP_ALPHA*(rc2 - _pitch_rc_lp))
    return clamp(_pitch_rc_lp, RC_MIN, RC_MAX)

def _tracking_loop_worker():
    # sends RC overrides while tracking flag is on, based on latest bbox
    while not _track_stop.is_set():
        if not _tracking_on or MAVLINK_CONN is None:
            time.sleep(0.02); continue
        try:
            # allow tracking only when armed
            if not is_armed():
                time.sleep(0.02); continue

            with _bbox_lock:
                bbox = _last_bbox
                dx   = _last_dx
                area = _last_area_frac

            if bbox is None:
                # no target: slowly center sticks
                send_rc_override(None, int(_pitch_rc_lp + 0.6*(RC_MID - _pitch_rc_lp)),
                                 RC_MIN, int(_yaw_rc_lp + 0.6*(RC_MID - _yaw_rc_lp)))
                time.sleep(0.05); continue

            yaw_rc   = _rc_from_dx(dx)
            pitch_rc = _rc_pitch_from_area(area)
            thr_rc   = min(THR_SAFE_MAX, RC_MID + 50)  # gentle throttle cap during tracking
            send_rc_override(None, pitch_rc, thr_rc, yaw_rc)
        except Exception as e:
            print("[TRACK] loop err:", e)
        time.sleep(0.02)
    print("[TRACK] stopped")

# ============================== Connection/Loops =============================
def mav_connect():
    global MAVLINK_CONN, BOOT_TIME_MONO
    print(f"[MAV] Connecting to {MAVLINK_DEVICE} @ {MAVLINK_BAUD}.")
    try:
        MAVLINK_CONN = mavutil.mavlink_connection(MAVLINK_DEVICE, baud=MAVLINK_BAUD)
        MAVLINK_CONN.wait_heartbeat(timeout=10)
        print(f"[MAV] Heartbeat received from sys {MAVLINK_CONN.target_system}, comp {MAVLINK_CONN.target_component}")
        try:
            MAVLINK_CONN.mav.srcSystem = 255
            MAVLINK_CONN.mav.srcComponent = 190
        except Exception:
            pass
        if MAVLINK_CONN.target_system in (None, 0):
            MAVLINK_CONN.target_system = 1
        if MAVLINK_CONN.target_component in (None, 0):
            MAVLINK_CONN.target_component = 1
        print(f"[MAV] Using targets sys={MAVLINK_CONN.target_system}, comp={MAVLINK_CONN.target_component}")
        BOOT_TIME_MONO = time.monotonic()
        request_telemetry_streams()
        return True
    except Exception as e:
        print(f"[MAV] Connection FAILED: {e}")
        return False

def mav_forwarding_loop():
    if MAVLINK_CONN is None:
        return
    print("[MAV] Starting telemetry forwarding loop.")
    try:
        udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    except Exception as e:
        print(f"[UDP-TEL] Failed to init socket: {e}")
        return
    period = 1.0 / 50.0
    while True:
        try:
            msg = MAVLINK_CONN.recv_match(blocking=False)
            if msg:
                buf = msg.get_msgbuf()
                if buf:
                    udp_sock.sendto(buf, (UDP_IP, UDP_PORT_TEL))
            time.sleep(period)
        except Exception as e:
            if "Bad file descriptor" not in str(e):
                print(f"[MAV-TX] Error: {e}")
            time.sleep(0.1)

def command_listener_loop():
    # Listen for JSON commands from GUI on UDP 9104
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((UDP_IP, UDP_PORT_CMD))
        print(f"[CMD] Listening for commands on {UDP_IP}:{UDP_PORT_CMD}")
    except Exception as e:
        print(f"[CMD] Failed to bind command listener: {e}")
        return

    global TARGET_CLASSES, _tracking_on
    while True:
        try:
            data, _ = sock.recvfrom(4096)
            obj = json.loads(data.decode("utf-8"))
            cmd = obj.get("command") or obj.get("cmd")  # support both styles
            d   = obj.get("data", {})

            if cmd == "GOTO_GPS":
                go_to_gps(d.get("lat"), d.get("lon"), d.get("alt"))
            elif cmd == "HOLD_HERE":
                hold_here_current_alt()
            elif cmd == "ALT_BUMP":
                bump_alt_guided(d.get("delta", 0))
            elif cmd == "TAKEOFF":
                takeoff_to_alt(float(d.get("alt", SAFE_DEFAULT_ALT)))
            elif cmd == "SET_MODE":
                set_mode(d.get("mode"))
            elif cmd == "ARM":
                arm_motors()
            elif cmd == "DISARM":
                disarm_motors()
            elif cmd == "EMERGENCY_STOP":
                emergency_stop()

            # --- new vision / tracking cmds ---
            elif cmd == "START_YOLO":
                start_yolo()
            elif cmd == "STOP_YOLO":
                stop_yolo()
            elif cmd == "SET_CLASSES":
                TARGET_CLASSES = (d.get("classes") or None)
                print(f"[YOLO] TARGET_CLASSES={TARGET_CLASSES}")
            elif cmd == "START_TRACKING":
                _tracking_on = True; print("[TRACK] enabled")
            elif cmd == "STOP_TRACKING":
                _tracking_on = False; print("[TRACK] disabled")
            else:
                print(f"[CMD] Unknown command: {cmd}")
        except Exception as e:
            print(f"[CMD] Error parsing/executing command: {e}")

def _gcs_heartbeat_loop():
    # send GCS heartbeat at 1 Hz so FC recognizes us as a ground station
    while True:
        try:
            if MAVLINK_CONN:
                MAVLINK_CONN.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0
                )
        except Exception:
            pass
        time.sleep(1.0)

def guided_keepalive_loop():
    while True:
        try:
            if MAVLINK_CONN and current_mode_name() == 'GUIDED' and is_armed():
                gpos = MAVLINK_CONN.messages.get('GLOBAL_POSITION_INT')
                vfr  = MAVLINK_CONN.messages.get('VFR_HUD')
                if gpos and vfr:
                    lat = gpos.lat / 1e7
                    lon = gpos.lon / 1e7
                    alt = float(vfr.alt)
                    go_to_gps(lat, lon, alt)
        except Exception as e:
            print(f"[KEEPALIVE] error: {e}")
        time.sleep(GUIDED_KEEPALIVE_PERIOD)

# ============================== Start/Stop YOLO ==============================
def start_yolo():
    global _yolo_thread
    if _yolo_thread and _yolo_thread.is_alive():
        print("[YOLO] already running"); return
    _yolo_stop.clear()
    _yolo_thread = threading.Thread(target=_yolo_loop_worker, daemon=True)
    _yolo_thread.start()
    print("[YOLO] started")

def stop_yolo():
    if _yolo_thread and _yolo_thread.is_alive():
        _yolo_stop.set()
    print("[YOLO] stop requested")

def _get_fix_type():
    # returns GPS fix type or None
    try:
        gps = MAVLINK_CONN.messages.get("GPS_RAW_INT")
        if gps:
            return int(gps.fix_type)
    except Exception:
        pass
    return None

def _get_ekf_flags():
    # returns ekf flags bitmask or None
    try:
        ekf = MAVLINK_CONN.messages.get("EKF_STATUS_REPORT")
        if ekf:
            return int(ekf.flags)
    except Exception:
        pass
    return None

def _pump_msgs_for(seconds=1.0):
    t0 = time.time()
    while time.time() - t0 < seconds:
        try:
            MAVLINK_CONN.recv_match(blocking=False)
        except Exception:
            pass
        time.sleep(0.02)

def print_guided_blockers():
    # print a quick summary of blockers for GUIDED
    _pump_msgs_for(0.5)
    fix = _get_fix_type()
    ekf = _get_ekf_flags()
    hb  = MAVLINK_CONN.messages.get("HEARTBEAT")
    sts = MAVLINK_CONN.messages.get("SYS_STATUS")
    txt = MAVLINK_CONN.messages.get("STATUSTEXT")

    if fix is not None:
        print(f"[DIAG] GPS fix_type={fix} (3=3D fix)")
    else:
        print("[DIAG] GPS fix_type: None")

    if ekf is not None:
        print(f"[DIAG] EKF flags=0x{ekf:08X}")
    else:
        print("[DIAG] EKF flags: None (yet)")

    if hb:
        print(f"[DIAG] flightmode='{getattr(MAVLINK_CONN,'flightmode',None)}' armed={(hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)!=0}")

    if txt:
        try:
            print(f"[DIAG] STATUSTEXT: {txt.text}")
        except Exception:
            pass


# ================================ Main ======================================
if __name__ == "__main__":
    if not mav_connect():
        print("Starting without MAVLink connection.")

    # SITL-only helpers to speed up readiness (do this once when debugging)
    try:
        MAVLINK_CONN.mav.param_set_send(MAVLINK_CONN.target_system, MAVLINK_CONN.target_component,
                                        b"AHRS_EKF_TYPE", 3, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        MAVLINK_CONN.mav.param_set_send(MAVLINK_CONN.target_system, MAVLINK_CONN.target_component,
                                        b"SIM_GPS_DISABLE", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        MAVLINK_CONN.mav.param_set_send(MAVLINK_CONN.target_system, MAVLINK_CONN.target_component,
                                        b"ARMING_CHECK", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    except Exception as e:
        print("[PARAM] set warn:", e)

    threading.Thread(target=_gcs_heartbeat_loop, daemon=True).start()

    # start threads
    threading.Thread(target=mav_forwarding_loop, daemon=True).start()
    threading.Thread(target=command_listener_loop, daemon=True).start()
    threading.Thread(target=guided_keepalive_loop, daemon=True).start()

    # tracking loop
    _track_stop.clear()
    _track_thread = threading.Thread(target=_tracking_loop_worker, daemon=True)
    _track_thread.start()

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        _yolo_stop.set()
        _track_stop.set()
        print("Tracker shutdown complete.")
