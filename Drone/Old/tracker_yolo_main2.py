#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ASCII-only, english comments

Tracker + MAVLink proxy (+ optional YOLO/DeepSORT).
- Connects to ArduPilot SITL over TCP (WSL eth0 IP, port 5760).
- Forwards MAVLink to GUI on UDP 14550.
- Listens for control commands on UDP 9104.
- Supports: ARM/DISARM, EMERGENCY_STOP, SET_MODE, HOLD, ALT_BUMP, GOTO_GPS, TAKEOFF.
"""

import time
import socket
import threading
import json

# Optional vision deps (kept import-safe)
try:
    import cv2  # noqa: F401
    from ultralytics import YOLO  # noqa: F401
    from deep_sort_realtime.deepsort_tracker import DeepSort  # noqa: F401
except Exception:
    pass

from pymavlink import mavutil

# ============================= Global time base ==============================
# time since boot for MAVLink messages
BOOT_TIME_MONO = None  # set after connection

# ============================= MAVLink Settings =============================

# For real FC on Windows uncomment and set COM and baud:
# MAVLINK_DEVICE = 'COM4'
# MAVLINK_BAUD   = 57600

# TCP to WSL SITL (replace IP if your WSL IP changes)
MAVLINK_DEVICE = 'tcp:172.20.186.151:5760'
MAVLINK_BAUD = 115200
MAVLINK_CONN = None

GUIDED_KEEPALIVE_PERIOD = 0.5  # seconds
SAFE_DEFAULT_ALT = 5.0         # meters

# Debounce for ARM/DISARM
last_arm_sent_ts = 0.0
last_disarm_sent_ts = 0.0

# Auto behaviors
MAV_MODE_FLAG_SAFETY_ARMED = 0x80
AUTO_TAKEOFF_ON_GOTO = True
AUTO_TAKEOFF_ALT = 5.0

# ============================ Communication Ports ===========================

UDP_IP = '127.0.0.1'
UDP_PORT_TEL = 14550   # GUI listens here for forwarded MAVLink
UDP_PORT_CMD = 9104    # Commands in from GUI (JSON)

# =============================== MAV Helpers ================================

APM_COPTER_MODES = {
    "STABILIZE": 0, "ACRO": 1, "ALT_HOLD": 2, "AUTO": 3, "GUIDED": 4, "LOITER": 5,
    "RTL": 6, "CIRCLE": 7, "LAND": 9, "DRIFT": 11, "SPORT": 13, "FLIP": 14,
    "AUTOTUNE": 15, "POSHOLD": 16, "BRAKE": 17, "THROW": 18, "AVOID_ADSB": 19,
    "GUIDED_NOGPS": 20, "SMART_RTL": 21, "FLOWHOLD": 22, "FOLLOW": 23,
    "ZIGZAG": 24, "SYSTEMID": 25, "AUTOROTATE": 26, "AUTO_RTL": 27
}

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

def wait_for_mode(target_mode, timeout=3.0):
    t0 = time.time()
    target_mode = (target_mode or "").upper()
    while time.time() - t0 < timeout:
        m = current_mode_name()
        if m and m.upper() == target_mode:
            return True
        time.sleep(0.05)
    return False

def wait_for_arm_state(should_be_armed=True, timeout=3.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if is_armed() == should_be_armed:
            return True
        time.sleep(0.05)
    return False

def set_mode(mode_name: str):
    """Set ArduCopter flight mode using DO_SET_MODE + custom_mode, then wait."""
    if MAVLINK_CONN is None:
        return False
    mode_name = (mode_name or "").upper()
    if mode_name not in APM_COPTER_MODES:
        print(f"[MAV] Mode {mode_name} not recognized for ArduCopter.")
        return False
    custom_mode = int(APM_COPTER_MODES[mode_name])
    MAVLINK_CONN.mav.command_long_send(
        MAVLINK_CONN.target_system,
        MAVLINK_CONN.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # base_mode (param1)
        custom_mode,                                        # custom_mode (param2)
        0, 0, 0, 0, 0
    )
    ok = wait_for_mode(mode_name, timeout=2.0)
    print(f"[MAV] Changing mode to {mode_name} via DO_SET_MODE (ok={ok})")
    return ok

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
    """Immediate disarm and try to go LOITER/LAND."""
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
        1, 1
    )
    MAVLINK_CONN.mav.request_data_stream_send(
        MAVLINK_CONN.target_system,
        MAVLINK_CONN.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
        10, 1
    )
    print("[MAV] Requested VFR_HUD and GPS streams.")

def go_to_gps(lat, lon, alt):
    """GUIDED goto to absolute GPS, relative alt. Arms/takeoff if needed."""
    if MAVLINK_CONN is None:
        return
    if lat is None or lon is None or alt is None:
        print(f"[MAV-CMD] GOTO_GPS FAILED: Missing lat/lon/alt ({lat}, {lon}, {alt})")
        return

    # 1) ensure GUIDED
    if current_mode_name() != 'GUIDED':
        if not set_mode('GUIDED'):
            print("[MAV-CMD] GOTO aborted: could not enter GUIDED")
            return

    # 2) auto arm + takeoff if requested and on ground/low alt
    vfr = MAVLINK_CONN.messages.get('VFR_HUD')
    cur_alt = float(vfr.alt) if vfr else 0.0
    if AUTO_TAKEOFF_ON_GOTO and not is_armed():
        arm_motors()
        wait_for_arm_state(True, 3.0)
    if AUTO_TAKEOFF_ON_GOTO and cur_alt < 1.5:
        try:
            takeoff_to_alt(max(float(alt), AUTO_TAKEOFF_ALT))
            time.sleep(1.0)
        except Exception as e:
            print(f"[MAV-CMD] TAKEOFF error: {e}")

    # 3) send setpoint
    lat_i = int(float(lat) * 1e7)      # int32
    lon_i = int(float(lon) * 1e7)      # int32
    alt_f = float(alt)                 # float
    type_mask = 0b0000111111111000     # uint16

    now_ms = int((time.monotonic() - BOOT_TIME_MONO) * 1000) & 0xFFFFFFFF  # uint32

    MAVLINK_CONN.mav.set_position_target_global_int_send(
        now_ms,
        MAVLINK_CONN.target_system,
        MAVLINK_CONN.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        type_mask,
        lat_i, lon_i, alt_f,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0
    )
    print(f"[MAV-CMD] GOTO Lat:{lat_i/1e7:.5f}, Lon:{lon_i/1e7:.5f}, Alt:{alt_f:.1f}m")

def hold_here_current_alt():
    """Hold at current GPS and current or safe altitude."""
    if MAVLINK_CONN is None:
        print("[MAV-CMD] HOLD FAILED: No connection.")
        return

    gpos = MAVLINK_CONN.messages.get('GLOBAL_POSITION_INT')
    vfr  = MAVLINK_CONN.messages.get('VFR_HUD')

    if not gpos:
        print("[MAV-CMD] HOLD FAILED: No GPS fix.")
        return

    lat = gpos.lat / 1e7
    lon = gpos.lon / 1e7
    alt = float(vfr.alt) if vfr else SAFE_DEFAULT_ALT
    if not vfr:
        print(f"[MAV-CMD] HOLD WARNING: Using safe alt {alt:.1f}m")

    if current_mode_name() != 'GUIDED':
        if not set_mode('GUIDED'):
            return
    go_to_gps(lat, lon, alt)
    print(f"[MAV-CMD] HOLD @ Lat:{lat:.5f}, Lon:{lon:.5f}, Alt:{alt:.1f}m")

def bump_alt_guided(delta_alt):
    """Change current GUIDED target altitude by delta."""
    if MAVLINK_CONN is None:
        return
    gpos = MAVLINK_CONN.messages.get('GLOBAL_POSITION_INT')
    vfr  = MAVLINK_CONN.messages.get('VFR_HUD')
    if not gpos:
        print("[MAV-CMD] ALT_BUMP failed: No GPS.")
        return
    lat = gpos.lat / 1e7
    lon = gpos.lon / 1e7
    base_alt = float(vfr.alt) if vfr else SAFE_DEFAULT_ALT
    new_alt = max(1.0, base_alt + float(delta_alt))
    go_to_gps(lat, lon, new_alt)
    print(f"[MAV-CMD] ALT_BUMP -> {new_alt:.1f}m")

def takeoff_to_alt(alt_m):
    """Arm (if needed), set GUIDED, and takeoff to alt using NAV_TAKEOFF."""
    if MAVLINK_CONN is None:
        return
    if current_mode_name() != 'GUIDED':
        if not set_mode('GUIDED'):
            print("[MAV-CMD] TAKEOFF: failed to set GUIDED")
            return
    arm_motors()
    time.sleep(0.2)
    MAVLINK_CONN.mav.command_long_send(
        MAVLINK_CONN.target_system,
        MAVLINK_CONN.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,
        0, 0,
        float(alt_m)
    )
    print(f"[MAV-CMD] TAKEOFF to {float(alt_m):.1f}m sent.")

# ============================== Connection/Loops =============================

def mav_connect():
    global MAVLINK_CONN, BOOT_TIME_MONO
    print(f"[MAV] Connecting to {MAVLINK_DEVICE} @ {MAVLINK_BAUD}...")
    try:
        MAVLINK_CONN = mavutil.mavlink_connection(MAVLINK_DEVICE, baud=MAVLINK_BAUD)
        MAVLINK_CONN.wait_heartbeat(timeout=10)
        print(f"[MAV] Heartbeat received from sys {MAVLINK_CONN.target_system}, comp {MAVLINK_CONN.target_component}")

        # act like GCS ids
        try:
            MAVLINK_CONN.mav.srcSystem = 255
            MAVLINK_CONN.mav.srcComponent = 190
        except Exception:
            pass

        # force targets for SITL
        if MAVLINK_CONN.target_system in (None, 0):
            MAVLINK_CONN.target_system = 1
        if MAVLINK_CONN.target_component in (None, 0):
            MAVLINK_CONN.target_component = 1
        print(f"[MAV] Using targets sys={MAVLINK_CONN.target_system}, comp={MAVLINK_CONN.target_component}")

        # mark boot time (monotonic)
        BOOT_TIME_MONO = time.monotonic()

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
    """Listen for JSON commands from GUI on UDP 9104."""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((UDP_IP, UDP_PORT_CMD))
        print(f"[CMD] Listening for commands on {UDP_IP}:{UDP_PORT_CMD}")
    except Exception as e:
        print(f"[CMD] Failed to bind command listener: {e}")
        return

    while True:
        try:
            data, _ = sock.recvfrom(2048)
            cmd_obj = json.loads(data.decode('utf-8'))
            cmd = cmd_obj.get("command")
            d = cmd_obj.get("data", {})

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
            else:
                print(f"[CMD] Unknown command: {cmd}")
        except Exception as e:
            print(f"[CMD] Error parsing/executing command: {e}")

def guided_keepalive_loop():
    """Resend a GUIDED setpoint only when GUIDED and armed."""
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

def disable_fence():
    """Disable geofence so far GOTO is allowed."""
    if MAVLINK_CONN is None:
        return
    MAVLINK_CONN.mav.param_set_send(
        MAVLINK_CONN.target_system,
        MAVLINK_CONN.target_component,
        b"FENCE_ENABLE",
        float(0),
        mavutil.mavlink.MAV_PARAM_TYPE_INT8
    )
    print("[MAV-CMD] FENCE_ENABLE=0 sent")

def set_home_here():
    """Set home to current GPS and use current as EKF origin/home."""
    if MAVLINK_CONN is None:
        return
    gpos = MAVLINK_CONN.messages.get('GLOBAL_POSITION_INT')
    if not gpos:
        print("[MAV-CMD] SET_HOME failed: no GPS yet")
        return
    lat = gpos.lat / 1e7
    lon = gpos.lon / 1e7
    alt = 10.0
    MAVLINK_CONN.mav.command_long_send(
        MAVLINK_CONN.target_system,
        MAVLINK_CONN.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        0,
        1,      # use current
        0, 0, 0,
        0, 0, 0
    )
    print(f"[MAV-CMD] Home set to current (lat={lat:.5f}, lon={lon:.5f})")




# =================================== Main ===================================

if __name__ == "__main__":
    if not mav_connect():
        print("Starting without MAVLink connection.")

    threading.Thread(target=mav_forwarding_loop, daemon=True).start()
    threading.Thread(target=command_listener_loop, daemon=True).start()
    threading.Thread(target=guided_keepalive_loop, daemon=True).start()

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("Tracker shutdown initiated.")
    print("Tracker shutdown complete.")
