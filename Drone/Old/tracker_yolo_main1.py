#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ASCII-only, english comments

Tracker + MAVLink proxy + YOLO/DeepSORT (optional).
- Connects to ArduPilot SITL by default (UDP 14551).
- Forwards all MAVLink to GUI via UDP 14550.
- Listens to control commands on UDP 9104.
- Supports ARM/DISARM, EMERGENCY STOP, SET_MODE, HOLD, ALT_BUMP, GOTO_GPS, TAKEOFF.
"""

import sys
import time
import socket
import threading
import json
import math
import numpy as np

import cv2
from ultralytics import YOLO
from deep_sort_realtime.deepsort_tracker import DeepSort

from pymavlink import mavutil
try:
    from pymavlink.dialects.v20 import ardupilotmega as mavlink_dialect
except ImportError:
    from pymavlink.dialects.v20 import common as mavlink_dialect

# ============================= MAVLink Settings =============================

# For real FC on Windows uncomment and set COM and baud:
# MAVLINK_DEVICE = 'COM4'
# MAVLINK_BAUD   = 57600

# Default: SITL via UDP (the tracker listens locally on 14551)
#MAVLINK_DEVICE = 'udp:0.0.0.0:14551'
#MAVLINK_BAUD = 115200
#MAVLINK_CONN = None

# New: connect to SITL over TCP (WSL eth0 IP)
MAVLINK_DEVICE = 'tcp:172.20.186.151:5760'
MAVLINK_BAUD = 115200
GUIDED_KEEPALIVE_PERIOD = 0.5  # seconds
SAFE_DEFAULT_ALT = 5.0         # meters

# Debounce for ARM/DISARM
last_arm_sent_ts = 0.0
last_disarm_sent_ts = 0.0

# ============================ Communication Ports ===========================

UDP_IP = '127.0.0.1'
UDP_PORT_TEL = 14550   # GUI listens here for forwarded MAVLink
UDP_PORT_BBOX = 9103   # YOLO bbox out
UDP_PORT_CMD = 9104    # Commands in from GUI

# ==================== YOLO / DeepSORT Settings and State ====================

YOLO_CAM_INDEX = 0
YOLO_MODEL_NAME = 'yolov8n.pt'
YOLO_CONF = 0.25
YOLO_IOU = 0.50
TARGET_CLASSES = [0]   # person
YOLO_SHOW_WINDOW = True

yolo_runner_active = False
current_alt_guided = None
yolo_thread = None


# =============================== MAV Helpers ================================

# ===== replace set_mode() with ArduCopter-accurate version =====
APM_COPTER_MODES = {
    "STABILIZE": 0, "ACRO": 1, "ALT_HOLD": 2, "AUTO": 3, "GUIDED": 4, "LOITER": 5,
    "RTL": 6, "CIRCLE": 7, "LAND": 9, "DRIFT": 11, "SPORT": 13, "FLIP": 14,
    "AUTOTUNE": 15, "POSHOLD": 16, "BRAKE": 17, "THROW": 18, "AVOID_ADSB": 19,
    "GUIDED_NOGPS": 20, "SMART_RTL": 21, "FLOWHOLD": 22, "FOLLOW": 23,
    "ZIGZAG": 24, "SYSTEMID": 25, "AUTOROTATE": 26, "AUTO_RTL": 27
}

def set_mode(mode_name: str):
    """Set ArduCopter flight mode using custom_mode numbers."""
    if MAVLINK_CONN is None:
        return False
    mode_name = (mode_name or "").upper()
    if mode_name not in APM_COPTER_MODES:
        print(f"[MAV] Mode {mode_name} not recognized for ArduCopter.")
        return False
    custom_mode = APM_COPTER_MODES[mode_name]
    MAVLINK_CONN.mav.set_mode_send(
        MAVLINK_CONN.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        custom_mode
    )
    print(f"[MAV] Changing mode to {mode_name} (custom_mode={custom_mode})")
    return True




def arm_motors():
    """Arm motors with debounce."""
    global last_arm_sent_ts
    if MAVLINK_CONN is None:
        return False
    now = time.time()
    if now - last_arm_sent_ts < 1.0:
        print("[MAV-CMD] ARM ignored (debounce)")
        return False
    MAVLINK_CONN.mav.command_long_send(
        MAVLINK_CONN.target_system,
        MAVLINK_CONN.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    last_arm_sent_ts = now
    print("[MAV-CMD] Motors ARMED (sent COMMAND_LONG).")
    return True


def disarm_motors():
    """Disarm motors with debounce."""
    global last_disarm_sent_ts
    if MAVLINK_CONN is None:
        return False
    now = time.time()
    if now - last_disarm_sent_ts < 1.0:
        print("[MAV-CMD] DISARM ignored (debounce)")
        return False
    MAVLINK_CONN.mav.command_long_send(
        MAVLINK_CONN.target_system,
        MAVLINK_CONN.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    last_disarm_sent_ts = now
    print("[MAV-CMD] Motors DISARMED (sent COMMAND_LONG).")
    return True


def emergency_stop():
    """Immediate disarm and attempt safe mode."""
    if MAVLINK_CONN is None:
        return
    disarm_motors()
    print("[MAV-CMD] !!! EMERGENCY STOP: Motors DISARMED !!!")
    try:
        set_mode('LOITER')
    except Exception:
        try:
            set_mode('LAND')
        except Exception:
            pass
    print("[MAV-CMD] Emergency stop routine complete.")


def request_telemetry_streams():
    """Request GPS and VFR_HUD streams."""
    if MAVLINK_CONN is None:
        return
    MAVLINK_CONN.mav.request_data_stream_send(
        MAVLINK_CONN.target_system,
        MAVLINK_CONN.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION,
        1,
        1
    )
    MAVLINK_CONN.mav.request_data_stream_send(
        MAVLINK_CONN.target_system,
        MAVLINK_CONN.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
        10,
        1
    )
    print("[MAV] Requested VFR_HUD and GPS streams.")


def go_to_gps(lat, lon, alt):
    """GUIDED goto to absolute GPS, relative alt."""
    global current_alt_guided
    if MAVLINK_CONN is None:
        return
    if lat is None or lon is None or alt is None:
        print(f"[MAV-CMD] GOTO_GPS FAILED: Missing lat/lon/alt ({lat}, {lon}, {alt})")
        return
    if MAVLINK_CONN.flightmode != 'GUIDED':
        if not set_mode('GUIDED'):
            return
    current_alt_guided = alt
    MAVLINK_CONN.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(
        int(time.time() * 1000),
        MAVLINK_CONN.target_system,
        MAVLINK_CONN.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,
        int(lat * 1e7),
        int(lon * 1e7),
        alt,
        0, 0, 0,
        0, 0, 0,
        0, 0
    ))
    print(f"[MAV-CMD] GOTO Lat:{lat:.5f}, Lon:{lon:.5f}, Alt:{alt:.1f}m")


def hold_here_current_alt():
    """Hold at current GPS and current or safe altitude."""
    global current_alt_guided
    if MAVLINK_CONN is None:
        print("[MAV-CMD] HOLD FAILED: No connection.")
        return
    msg = MAVLINK_CONN.recv_match(type=['GLOBAL_POSITION_INT'], blocking=True, timeout=1.0)
    lat, lon = (msg.lat / 1e7, msg.lon / 1e7) if msg else (None, None)

    alt_msg = MAVLINK_CONN.recv_match(type='VFR_HUD', blocking=True, timeout=0.1)
    alt = alt_msg.alt if alt_msg else SAFE_DEFAULT_ALT
    if not alt_msg:
        print(f"[MAV-CMD] HOLD WARNING: Using safe alt {alt:.1f}m")

    if lat is None or lon is None:
        print("[MAV-CMD] HOLD FAILED: No GPS fix.")
        return
    if MAVLINK_CONN.flightmode != 'GUIDED':
        if not set_mode('GUIDED'):
            return
    current_alt_guided = alt
    go_to_gps(lat, lon, alt)
    print(f"[MAV-CMD] HOLD @ Lat:{lat:.5f}, Lon:{lon:.5f}, Alt:{alt:.1f}m")


def bump_alt_guided(delta_alt):
    """Change current GUIDED target altitude by delta."""
    global current_alt_guided
    if MAVLINK_CONN is None:
        return
    if current_alt_guided is None:
        print("[MAV-CMD] ALT_BUMP failed: call HOLD or GOTO first.")
        return
    new_alt = max(1.0, current_alt_guided + float(delta_alt))
    msg = MAVLINK_CONN.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1.0)
    lat, lon = (msg.lat / 1e7, msg.lon / 1e7) if msg else (None, None)
    if lat is None or lon is None:
        print("[MAV-CMD] ALT_BUMP failed: No GPS.")
        return
    go_to_gps(lat, lon, new_alt)
    print(f"[MAV-CMD] ALT_BUMP -> {new_alt:.1f}m")


def takeoff_to_alt(alt_m):
    """Arm (if needed), set GUIDED, and takeoff to alt using NAV_TAKEOFF."""
    if MAVLINK_CONN is None:
        return
    # ensure GUIDED
    if MAVLINK_CONN.flightmode != 'GUIDED':
        if not set_mode('GUIDED'):
            print("[MAV-CMD] TAKEOFF: failed to set GUIDED")
            return
    # ensure armed
    arm_motors()
    time.sleep(0.2)
    # send takeoff
    MAVLINK_CONN.mav.command_long_send(
        MAVLINK_CONN.target_system,
        MAVLINK_CONN.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,
        0, 0,
        float(alt_m)
    )
    print(f"[MAV-CMD] TAKEOFF to {alt_m:.1f}m sent.")


def mav_connect():
    """Connect to MAVLink and set up streams."""
    global MAVLINK_CONN
    print(f"[MAV] Connecting to {MAVLINK_DEVICE} @ {MAVLINK_BAUD}...")
    try:
        MAVLINK_CONN = mavutil.mavlink_connection(MAVLINK_DEVICE, baud=MAVLINK_BAUD)
        MAVLINK_CONN.wait_heartbeat(timeout=10)
        print(f"[MAV] Heartbeat received from sys {MAVLINK_CONN.target_system}, comp {MAVLINK_CONN.target_component}")

        # set our sender ids (acts like a GCS)
        try:
            MAVLINK_CONN.mav.srcSystem = 255
            MAVLINK_CONN.mav.srcComponent = 190
        except Exception:
            pass

        # force targets for ArduCopter SITL if zeros
        if MAVLINK_CONN.target_system in (None, 0):
            MAVLINK_CONN.target_system = 1
        if MAVLINK_CONN.target_component in (None, 0):
            MAVLINK_CONN.target_component = 1
        print(f"[MAV] Using targets sys={MAVLINK_CONN.target_system}, comp={MAVLINK_CONN.target_component}")

        request_telemetry_streams()
        return True
    except Exception as e:
        print(f"[MAV] Connection FAILED: {e}")
        return False


def mav_forwarding_loop():
    """Forward every MAVLink message to UDP 14550 for the GUI."""
    if MAVLINK_CONN is None:
        return
    print("[MAV] Starting telemetry forwarding loop...")
    try:
        udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    except Exception as e:
        print(f"[UDP-TEL] Failed to init socket: {e}")
        return

    period = 1.0 / 50.0
    while True:
        start_time = time.time()
        try:
            msg = MAVLINK_CONN.recv_match(blocking=False)
            if msg:
                buf = msg.get_msgbuf()
                if buf:
                    udp_sock.sendto(buf, (UDP_IP, UDP_PORT_TEL))
            dt = time.time() - start_time
            if period - dt > 0:
                time.sleep(period - dt)
        except Exception as e:
            if "Bad file descriptor" not in str(e):
                print(f"[MAV-TX] Error: {e}")
            time.sleep(0.1)

# ===== optional: small helper to read COMMAND_ACK =====
def wait_ack(cmd_id, timeout=1.0):
    """Wait for COMMAND_ACK for given MAV_CMD_* id."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = MAVLINK_CONN.recv_match(type='COMMAND_ACK', blocking=False)
        if msg and msg.command == cmd_id:
            print(f"[MAV-ACK] {mavutil.mavlink.enums['MAV_CMD'][cmd_id].name}: {msg.result}")
            return True
        time.sleep(0.01)
    return False

# ============================== YOLO Controller =============================

def start_yolo_detection():
    """Start YOLO + DeepSORT in a background thread."""
    global yolo_runner_active, yolo_thread
    if yolo_runner_active:
        print("[YOLO] Detection is already active.")
        return

    def yolo_execution_target():
        global yolo_runner_active, TARGET_CLASSES
        print(f"[YOLO] Loading model, camera {YOLO_CAM_INDEX}, DeepSORT...")
        print(f"[YOLO] Initial filter classes: {TARGET_CLASSES}")
        try:
            cap = cv2.VideoCapture(YOLO_CAM_INDEX)
            if not cap.isOpened():
                raise IOError(f"Cannot open camera index {YOLO_CAM_INDEX}")
            model = YOLO(YOLO_MODEL_NAME)
            tracker = DeepSort(max_age=5, nms_max_overlap=0.7, embedder='mobilenet', half=True)
            udp_bbox = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            yolo_runner_active = True
            print("[YOLO] Initialized. Starting loop.")
        except Exception as e:
            print(f"[YOLO-INIT] FAILED: {e}")
            yolo_runner_active = False
            return

        while yolo_runner_active:
            start_t = time.time()
            ret, frame = cap.read()
            if not ret:
                print("[YOLO] Camera read failed, retry...")
                time.sleep(0.1)
                continue

            results = model.predict(source=frame, conf=YOLO_CONF, iou=YOLO_IOU,
                                    classes=TARGET_CLASSES, verbose=False)

            detections = []
            for r in results:
                for box in r.boxes:
                    x1, y1, x2, y2 = [int(i) for i in box.xyxy[0]]
                    confidence = float(box.conf[0])
                    class_id = int(box.cls[0])
                    detections.append(([x1, y1, x2 - x1, y2 - y1], confidence, class_id))

            tracks = tracker.update_tracks(detections, frame=frame)

            found_target = False
            if tracks:
                primary = tracks[0]
                if primary.is_confirmed():
                    ltrb = primary.to_ltrb()
                    track_id = primary.track_id
                    target_bbox = [ltrb[0], ltrb[1], ltrb[2], ltrb[3], track_id]
                    udp_bbox.sendto(json.dumps({"bbox": target_bbox}).encode('utf-8'),
                                    (UDP_IP, UDP_PORT_BBOX))
                    found_target = True
                    if YOLO_SHOW_WINDOW:
                        x1i, y1i, x2i, y2i = [int(p) for p in ltrb]
                        cv2.rectangle(frame, (x1i, y1i), (x2i, y2i), (0, 255, 0), 2)
                        cv2.putText(frame, f"ID:{track_id}", (x1i, y1i - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

            if not found_target:
                udp_bbox.sendto(json.dumps({"bbox": None}).encode('utf-8'),
                                (UDP_IP, UDP_PORT_BBOX))

            if YOLO_SHOW_WINDOW:
                fps = 1.0 / max(1e-6, (time.time() - start_t))
                cv2.putText(frame, f"FPS:{fps:.1f}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.imshow("YOLO/DeepSORT Tracker", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    yolo_runner_active = False

            time.sleep(0.001)

        cap.release()
        cv2.destroyAllWindows()
        yolo_runner_active = False
        print("[YOLO] Detection loop finished.")

    yolo_thread = threading.Thread(target=yolo_execution_target, daemon=True)
    yolo_thread.start()
    print("[YOLO] Started detection thread.")


def stop_yolo_detection():
    """Stop YOLO thread."""
    global yolo_runner_active, yolo_thread
    if not yolo_runner_active:
        print("[YOLO] Detection already inactive.")
        return
    yolo_runner_active = False
    print("[YOLO] Stop signal sent.")


# ============================ GUIDED Keepalive Loop ==========================

def guided_keepalive_loop():
    """Resend GUIDED setpoint periodically to avoid timeouts."""
    global current_alt_guided
    last_warn = 0
    while True:
        if MAVLINK_CONN and MAVLINK_CONN.flightmode == 'GUIDED' and current_alt_guided is not None:
            msg = MAVLINK_CONN.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
            lat, lon = (msg.lat / 1e7, msg.lon / 1e7) if msg else (None, None)
            alt = current_alt_guided
            if lat is not None and lon is not None:
                go_to_gps(lat, lon, alt)
            else:
                now = time.time()
                if now - last_warn > 5.0:
                    print("[MAV-CMD] WARNING: Keepalive skipped, no GPS.")
                    last_warn = now
        time.sleep(GUIDED_KEEPALIVE_PERIOD)


# ============================== Command Listener ============================

def command_listener_loop():
    """Listen for JSON commands from GUI on UDP 9104."""
    global TARGET_CLASSES
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((UDP_IP, UDP_PORT_CMD))
        print(f"[CMD] Listening for commands on {UDP_IP}:{UDP_PORT_CMD}")
    except Exception as e:
        print(f"[CMD] Failed to bind command listener: {e}")
        return

    while True:
        try:
            data, addr = sock.recvfrom(1024)
            cmd_obj = json.loads(data.decode('utf-8'))
            cmd = cmd_obj.get("command")
            cmd_data = cmd_obj.get("data", {})

            if cmd == "GOTO_GPS":
                go_to_gps(cmd_data.get("lat"), cmd_data.get("lon"), cmd_data.get("alt"))

            elif cmd == "HOLD_HERE":
                hold_here_current_alt()

            elif cmd == "ALT_BUMP":
                bump_alt_guided(cmd_data.get("delta"))

            elif cmd == "TAKEOFF":
                tgt_alt = cmd_data.get("alt", SAFE_DEFAULT_ALT)
                try:
                    takeoff_to_alt(float(tgt_alt))
                except Exception as e:
                    print(f"[CMD] TAKEOFF parse error: {e}")

            elif cmd == "START_YOLO":
                start_yolo_detection()

            elif cmd == "STOP_YOLO":
                stop_yolo_detection()

            elif cmd == "SET_CLASSES":
                classes_str = cmd_data.get("classes", "")
                try:
                    new_classes = [int(c.strip()) for c in classes_str.split(',') if c.strip().isdigit()]
                    TARGET_CLASSES = new_classes if new_classes else []
                    print(f"[CMD] Updated TARGET_CLASSES to: {TARGET_CLASSES}")
                except Exception as e:
                    print(f"[CMD] SET_CLASSES parsing error: {e}. Keeping old classes: {TARGET_CLASSES}")

            elif cmd == "SET_MODE":
                set_mode(cmd_data.get("mode"))

            elif cmd == "ARM":
                arm_motors()

            elif cmd == "DISARM":
                disarm_motors()

            elif cmd == "EMERGENCY_STOP":
                emergency_stop()

            else:
                print(f"[CMD] Unknown command: {cmd}")

        except Exception as e:
            print(f"[CMD] Error parsing/executing command: {e}")


# =================================== Main ===================================

if __name__ == "__main__":

    if not mav_connect():
        print("Starting without MAVLink connection.")

    threading.Thread(target=mav_forwarding_loop, daemon=True).start()
    threading.Thread(target=command_listener_loop, daemon=True).start()
    threading.Thread(target=guided_keepalive_loop, daemon=True).start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Tracker shutdown initiated.")

    print("Tracker shutdown complete.")
