#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ASCII-only comments.

Dash map for drone telemetry + MAVLink helpers:
- Reads MAVLink from serial FC on COM4 (default) and updates the map live
- (Optional) Also listens to UDP JSON on port 9002 and merges fields if present
- Logs track to CSV 'logs/gps_log_YYYY-mm-dd_HH-MM-SS.csv'
- Shows live marker + blue polyline + bottom telemetry strip + follow/fit controls
- "Go-To" control draws a red->green line from current position to the target (green when close)
- Auto-clears the Go-To when within 5 m, and shows a "Target" marker while active
- Adds a heading arrow (polyline) based on yaw
- Adds MAVLink: auto-connect, GUIDED hold-here, Go-To (lat/lon/alt), +/-1m bumps, keepalive thread
- Overlay toolbars stay above the map (z-index) without blocking map gestures.

Dependencies:
  pip install dash dash-leaflet pymavlink
"""

import os
import csv
import json
import math
import socket
import threading
from datetime import datetime

from dash import Dash, html, dcc, callback, Output, Input, State, no_update, ctx
import dash_leaflet as dl
from flask import Flask

# pymavlink for MAVLink control/telemetry
from pymavlink import mavutil

# ----- Paths / log file -----
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(BASE_DIR, "logs")
os.makedirs(LOG_DIR, exist_ok=True)
GPS_LOG_FILENAME = os.path.join(
    LOG_DIR, f"gps_log_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv"
)

# Store filename for other tools (optional)
LAST_GPS_LOG_PATH = os.path.join(BASE_DIR, "HomeStation", "last_gps_log.txt")
os.makedirs(os.path.dirname(LAST_GPS_LOG_PATH), exist_ok=True)
with open(LAST_GPS_LOG_PATH, "w") as f:
    f.write(GPS_LOG_FILENAME)

# ----- Live state -----
latest = {
    "lat": None,
    "lon": None,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,           # degrees
    "groundspeed": 0.0,
    "alt": 0.0,           # relative if GLOBAL_POSITION_INT.relative_alt; fallback AMSL via VFR_HUD
    "voltage": None,
    "device_status": {"gps": False, "mpu": False, "obd": False},
    "gps_quality": None,
    "gps_sats": None,
    "gps_hdop": None,
}
track = []  # list of (lat, lon)
lock = threading.Lock()

# Prefer MAVLink values when available
PREFER_MAVLINK = True

# Go-To line state
goto_target = None          # (lat, lon, alt_rel)
goto_lock = threading.Lock()

# ----- Small geo helpers -----
EARTH_R_M = 6371000.0

def haversine_m(lat1, lon1, lat2, lon2):
    # Returns distance in meters
    if None in (lat1, lon1, lat2, lon2):
        return None
    rlat1 = math.radians(lat1)
    rlon1 = math.radians(lon1)
    rlat2 = math.radians(lat2)
    rlon2 = math.radians(lon2)
    dlat = rlat2 - rlat1
    dlon = rlon2 - rlon1
    a = math.sin(dlat/2.0)**2 + math.cos(rlat1)*math.cos(rlat2)*math.sin(dlon/2.0)**2
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return EARTH_R_M * c

def project_forward(lat, lon, bearing_deg, dist_m):
    # Returns new (lat, lon) given start, bearing (deg, 0=north, positive clockwise), and distance meters
    if None in (lat, lon) or dist_m is None:
        return None, None
    br = math.radians(bearing_deg)
    rlat = math.radians(lat)
    rlon = math.radians(lon)
    dr = dist_m / EARTH_R_M
    nlat = math.asin(math.sin(rlat)*math.cos(dr) + math.cos(rlat)*math.sin(dr)*math.cos(br))
    nlon = rlon + math.atan2(math.sin(br)*math.sin(dr)*math.cos(rlat),
                             math.cos(dr) - math.sin(rlat)*math.sin(nlat))
    return math.degrees(nlat), math.degrees(nlon)

def clamp01(x):
    try:
        return max(0.0, min(1.0, float(x)))
    except Exception:
        return 0.0

def dist_color_hex(dist_m, max_m=500.0):
    # Red when far, green when close. Map 0..max_m to green..red
    if dist_m is None:
        return "#ff0000"
    t = clamp01(dist_m / max_m)  # 0 close, 1 far
    # invert for green near: g = 255*(1-t), r = 255*t
    r = int(255 * t)
    g = int(255 * (1.0 - t))
    b = 0
    return f"#{r:02x}{g:02x}{b:02x}"

def append_csv(ts_iso, lat, lon):
    # append a line to the CSV log
    with open(GPS_LOG_FILENAME, "a", newline="") as f:
        csv.writer(f).writerow([ts_iso, lat, lon])

# ----- UDP listener -----
UDP_PORT = 9002

def _valid_num(x):
    try:
        xf = float(x)
        return math.isfinite(xf)
    except Exception:
        return False

def udp_listener():
    # UDP listener for external telemetry JSON
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", UDP_PORT))
    print(f"[UDP] listening on :{UDP_PORT}")

    while True:
        msg, _ = sock.recvfrom(4096)
        try:
            data = json.loads(msg.decode("utf-8"))
        except Exception:
            continue

        with lock:
            for k in (
                "lat",
                "lon",
                "roll",
                "pitch",
                "yaw",
                "groundspeed",
                "alt",
                "voltage",
                "gps_quality",
                "gps_sats",
                "gps_hdop",
            ):
                if k in data and data[k] is not None:
                    # Do not overwrite MAVLink with UDP if PREFER_MAVLINK is True and MAVLink already filled it
                    if PREFER_MAVLINK and k in ("lat", "lon", "alt", "groundspeed"):
                        if latest.get(k) not in (None, 0.0):
                            continue
                    # Ignore bogus zeros for lat/lon
                    if k in ("lat", "lon") and (not _valid_num(data[k]) or float(data[k]) == 0.0):
                        continue
                    latest[k] = data[k]

            if "device_status" in data and isinstance(data["device_status"], dict):
                latest["device_status"].update(data["device_status"])

            lat = latest["lat"]
            lon = latest["lon"]
            if (
                isinstance(lat, (int, float))
                and isinstance(lon, (int, float))
                and lat != 0.0
                and lon != 0.0
            ):
                if not track or (abs(lat - track[-1][0]) > 1e-9 or abs(lon - track[-1][1]) > 1e-9):
                    track.append((lat, lon))
                    append_csv(datetime.now().isoformat(), lat, lon)

# ----- MAVLink (control) -----
# You can override by env:
#   MAVLINK_DEVICE=COM4 (Windows) or /dev/ttyACM0 (Linux)
#   MAVLINK_BAUD=115200
MAVLINK_DEVICE = os.environ.get("MAVLINK_DEVICE", "COM4")
MAVLINK_BAUD = int(os.environ.get("MAVLINK_BAUD", "115200"))

mav = None
mav_lock = threading.Lock()
armed_state = False
flight_mode = "UNKNOWN"

# GUIDED keepalive
GUIDED_KEEPALIVE_HZ = 5.0
_guided_active = False
_guided_target = None  # (lat, lon, alt_rel)
_guided_lock = threading.Lock()

def _time_boot_ms():
    import time
    return int((time.time() % 1e6) * 1000)

def mav_connect():
    global mav, armed_state, flight_mode
    try:
        print(f"[MAV] Connecting {MAVLINK_DEVICE} @ {MAVLINK_BAUD}")
        m = mavutil.mavlink_connection(MAVLINK_DEVICE, baud=MAVLINK_BAUD)
        m.wait_heartbeat(timeout=7)
        print("[MAV] Heartbeat OK")

        # Request message intervals (best-effort)
        try:
            req = [
                (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,             10),
                (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,  10),
                (mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,               5),
                (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,            1),
                (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,           1),
            ]
            for mid, hz in req:
                m.mav.command_long_send(
                    m.target_system, m.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0, mid, int(1e6/float(hz)), 0,0,0,0,0
                )
        except Exception:
            pass

        with mav_lock:
            mav = m
    except Exception as e:
        print("[MAV] Connect failed:", e)
        with mav_lock:
            mav = None
        return

    # Combined RX loop for HEARTBEAT + telemetry
    def _rx_loop():
        nonlocal m
        import time
        last_pos = None  # (lat, lon) dedupe
        while True:
            try:
                msg = m.recv_match(blocking=True, timeout=0.2)
            except Exception:
                msg = None
            if not msg:
                continue

            t = msg.get_type()
            with lock:
                if t == "HEARTBEAT":
                    try:
                        base = msg.base_mode
                        globals()["armed_state"] = bool(base & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                        try:
                            inv = {v: k for k, v in m.mode_mapping().items()}
                            globals()["flight_mode"] = inv.get(msg.custom_mode, flight_mode)
                        except Exception:
                            pass
                    except Exception:
                        pass

                elif t == "GLOBAL_POSITION_INT":
                    try:
                        lat = msg.lat / 1e7
                        lon = msg.lon / 1e7
                        alt_rel = msg.relative_alt / 1000.0  # mm -> m
                        vx, vy = msg.vx/100.0, msg.vy/100.0  # cm/s -> m/s
                        gs = max(0.0, (vx*vx + vy*vy) ** 0.5)
                        latest["lat"] = lat
                        latest["lon"] = lon
                        latest["alt"] = alt_rel
                        latest["groundspeed"] = gs
                        latest["device_status"]["gps"] = True
                        if (
                            isinstance(lat, float) and isinstance(lon, float)
                            and (last_pos is None or abs(lat-last_pos[0])>1e-9 or abs(lon-last_pos[1])>1e-9)
                        ):
                            track.append((lat, lon))
                            append_csv(datetime.now().isoformat(), lat, lon)
                            last_pos = (lat, lon)
                    except Exception:
                        pass

                elif t == "VFR_HUD":
                    try:
                        latest["groundspeed"] = float(msg.groundspeed)
                        if latest.get("alt") in (None, 0.0):
                            latest["alt"] = float(msg.alt)  # AMSL fallback
                    except Exception:
                        pass

                elif t == "ATTITUDE":
                    try:
                        latest["roll"]  = math.degrees(float(msg.roll))
                        latest["pitch"] = math.degrees(float(msg.pitch))
                        latest["yaw"]   = math.degrees(float(msg.yaw))
                    except Exception:
                        pass

                elif t == "SYS_STATUS":
                    try:
                        vb = msg.voltage_battery / 1000.0  # mV -> V
                        if vb > 0:
                            latest["voltage"] = vb
                    except Exception:
                        pass

                elif t == "BATTERY_STATUS":
                    try:
                        if len(msg.voltages) > 0 and msg.voltages[0] > 0:
                            latest["voltage"] = msg.voltages[0] / 1000.0
                    except Exception:
                        pass

                elif t == "GPS_RAW_INT":
                    try:
                        fix_ok = int(getattr(msg, "fix_type", 0)) >= 3
                        latest["device_status"]["gps"] = fix_ok
                        latest["gps_sats"] = int(getattr(msg, "satellites_visible", 0))
                        eph = getattr(msg, "eph", None)  # HDOP*100 (ArduPilot)
                        if eph is not None and eph > 0:
                            latest["gps_hdop"] = float(eph) / 100.0
                    except Exception:
                        pass

            try:
                time.sleep(0.005)
            except Exception:
                pass

    threading.Thread(target=_rx_loop, daemon=True).start()

def set_mode(mode_name):
    with mav_lock:
        m = mav
    if not m:
        print("[MAV] Not connected")
        return False
    try:
        mode_id = m.mode_mapping()[mode_name]
        m.set_mode(mode_id)
        print(f"[MAV] Mode -> {mode_name}")
        return True
    except Exception as e:
        print(f"[MAV] set_mode {mode_name} failed:", e)
        return False

def _send_guided_position(lat, lon, alt_rel_m):
    with mav_lock:
        m = mav
    if not m:
        return
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
        m.mav.set_position_target_global_int_send(
            _time_boot_ms(),
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            type_mask,
            int(lat * 1e7), int(lon * 1e7), float(alt_rel_m),
            0,0,0, 0,0,0, 0.0, 0.0
        )
    except Exception as e:
        print("[MAV] set_position_target_global_int error:", e)

def _enable_guided_keepalive(lat, lon, alt_rel_m):
    global _guided_active, _guided_target
    with _guided_lock:
        _guided_target = (float(lat), float(lon), float(alt_rel_m))
        _guided_active = True

def stop_guided_hold():
    global _guided_active, _guided_target
    with _guided_lock:
        _guided_active = False
        _guided_target = None
    print("[GUIDED] keepalive stopped")

def guided_keepalive_loop():
    import time
    period = 1.0 / GUIDED_KEEPALIVE_HZ
    while True:
        tgt = None
        with _guided_lock:
            if _guided_active and _guided_target:
                tgt = _guided_target
        if tgt:
            _send_guided_position(*tgt)
        time.sleep(period)

def hold_here_current_alt():
    with lock:
        lat = latest.get("lat")
        lon = latest.get("lon")
        alt = latest.get("alt")
    if not isinstance(lat, (int, float)) or not isinstance(lon, (int, float)):
        print("[GUIDED] No valid GPS position yet.")
        return False
    if not isinstance(alt, (int, float)):
        alt = 10.0
    ok = set_mode("GUIDED")
    if not ok:
        return False
    for _ in range(5):
        _send_guided_position(lat, lon, alt)
    _enable_guided_keepalive(lat, lon, alt)
    print(f"[GUIDED] Holding here at alt_rel={alt:.2f} m (keepalive).")
    return True

def go_to_gps(lat, lon, alt_rel_m):
    # Remember target for UI line/marker even if FC rejects for now
    try:
        lat = float(lat); lon = float(lon); alt_rel_m = float(alt_rel_m)
    except Exception:
        print("[MAV] Go-To bad inputs")
        return False

    with goto_lock:
        globals()["goto_target"] = (lat, lon, alt_rel_m)

    ok = set_mode("GUIDED")
    if ok:
        import time
        for _ in range(5):
            _send_guided_position(lat, lon, alt_rel_m); time.sleep(0.05)
        t0 = time.time()
        while time.time() - t0 < 2.0:
            _send_guided_position(lat, lon, alt_rel_m); time.sleep(0.2)
        _enable_guided_keepalive(lat, lon, alt_rel_m)

    print(f"[GUIDED] Go-To lat={lat:.7f} lon={lon:.7f} alt_rel={alt_rel_m:.2f}")
    return True

def bump_alt_guided(delta_m):
    tgt = None
    with _guided_lock:
        if _guided_target:
            lat, lon, alt = _guided_target
            tgt = (lat, lon, alt + float(delta_m))
    if tgt is None:
        if not hold_here_current_alt():
            return False
        with _guided_lock:
            lat, lon, alt = _guided_target
            tgt = (lat, lon, alt + float(delta_m))
    lat, lon, new_alt = tgt
    for _ in range(5):
        _send_guided_position(lat, lon, new_alt)
    _enable_guided_keepalive(lat, lon, new_alt)
    print(f"[GUIDED] Target alt -> {new_alt:.2f} m")
    return True

# ----- Web app (Dash) -----
server = Flask(__name__)
app = Dash(
    __name__,
    server=server,
    url_base_pathname="/",
    suppress_callback_exceptions=True,
)
app.title = "Drone Map"

layer_urls = {
    "esri": "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
    "topo": "https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png",
    "osm": "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
    "google": "https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}",
}

def telemetry_chip(label, value_id, units, color):
    # small metric card
    return html.Div(
        [
            html.Div(
                label,
                style={
                    "fontWeight": "bold",
                    "fontSize": "0.95em",
                    "color": color,
                    "marginBottom": "4px",
                },
            ),
            html.Div(
                id=value_id,
                style={"fontSize": "1.8em", "fontWeight": "bold", "color": "#fff", "lineHeight": "1"},
            ),
            html.Div(units, style={"fontSize": "0.9em", "color": "#bbb"}),
        ],
        style={
            "background": "#23272f",
            "borderRadius": "14px",
            "padding": "10px 12px",
            "minWidth": "100px",
            "display": "flex",
            "flexDirection": "column",
            "alignItems": "center",
            "boxShadow": f"0 0 16px 0 {color}33",
        },
    )

def gps_text_chip():
    # small card that shows "lat, lon" as a single line of text
    return html.Div(
        [
            html.Div("GPS", style={"fontWeight": "bold", "fontSize": "0.95em", "color": "#38b6ff", "marginBottom": "4px"}),
            html.Div(id="val-gps", style={"fontSize": "1.1em", "fontWeight": "bold", "color": "#fff"}),
        ],
        style={
            "background": "#23272f",
            "borderRadius": "14px",
            "padding": "10px 12px",
            "minWidth": "180px",
            "display": "flex",
            "flexDirection": "column",
            "alignItems": "center",
            "boxShadow": "0 0 16px 0 #38b6ff33",
        },
    )

def status_chip():
    # container for status rows
    return html.Div(
        id="status-chip",
        style={
            "background": "#23272f",
            "borderRadius": "14px",
            "padding": "10px 12px",
            "display": "flex",
            "flexDirection": "column",
            "gap": "4px",
            "minWidth": "120px",
            "boxShadow": "0 0 16px 0 #00ff2233",
        },
    )

def mav_status_chip():
    # free-text chip for MAV/UDP status
    return html.Div(
        id="mav-status",
        style={
            "background": "#23272f",
            "borderRadius": "14px",
            "padding": "10px 12px",
            "color": "#fff",
            "minWidth": "140px",
            "boxShadow": "0 0 16px 0 #ffffff22",
            "fontWeight": "bold",
        },
        children="Waiting for telemetry...",
    )

def mav_chip():
    # small container for MAVLink status and controls
    return html.Div(
        [
            html.Div(id="mav-status-top", style={"color": "#fff", "fontWeight": "bold", "marginBottom": "6px"}),
            html.Div(
                [
                    dcc.Input(id="goto-lat", type="number", placeholder="lat", style={"width": "120px"}),
                    dcc.Input(id="goto-lon", type="number", placeholder="lon", style={"width": "120px", "marginLeft": "6px"}),
                    dcc.Input(id="goto-alt", type="number", placeholder="alt m", style={"width": "90px", "marginLeft": "6px"}),
                    html.Button("Go To", id="btn-goto", n_clicks=0, style={"marginLeft": "8px", "pointerEvents": "auto"}),
                    html.Button("Hold here", id="btn-hold", n_clicks=0, style={"marginLeft": "6px", "pointerEvents": "auto"}),
                    html.Button("+1m", id="btn-up1", n_clicks=0, style={"marginLeft": "6px", "pointerEvents": "auto"}),
                    html.Button("-1m", id="btn-dn1", n_clicks=0, style={"marginLeft": "6px", "pointerEvents": "auto"}),
                ],
                style={"display": "flex", "alignItems": "center", "gap": "4px"},
            ),
        ],
        style={
            "background": "#23272f",
            "borderRadius": "12px",
            "padding": "8px 10px",
            "pointerEvents": "auto",
            "boxShadow": "0 0 16px 0 #00e1ff33",
        },
    )

# ---- Layout with overlays above the map ----
app.layout = html.Div(
    [
        # Controls bar (top overlay)
        html.Div(
            [
                html.Button("Center", id="center-btn", style={"pointerEvents": "auto"}),
                html.Button("Fit route", id="fit-btn", style={"pointerEvents": "auto"}),
                dcc.RadioItems(
                    id="layer",
                    options=[
                        {"label": "ESRI", "value": "esri"},
                        {"label": "Topo", "value": "topo"},
                        {"label": "OSM", "value": "osm"},
                        {"label": "Google", "value": "google"},
                    ],
                    value="esri",
                    inputStyle={"marginRight": "6px"},
                    labelStyle={"display": "inline-flex", "alignItems": "center", "marginRight": "14px", "color": "#fff"},
                    style={
                        "display": "flex",
                        "flexWrap": "nowrap",
                        "gap": "0px",
                        "marginLeft": "12px",
                        "background": "#23272f",
                        "borderRadius": "8px",
                        "padding": "6px 8px",
                        "pointerEvents": "auto",
                    },
                ),
                dcc.Checklist(
                    id="follow",
                    options=[{"label": "Follow", "value": "on"}],
                    value=["on"],
                    inline=True,
                    style={"marginLeft": "12px", "pointerEvents": "auto"},
                ),
                html.Div(mav_chip(), style={"marginLeft": "12px"}),
            ],
            style={
                "position": "fixed",
                "top": "10px",
                "left": "50%",
                "transform": "translateX(-50%)",
                "zIndex": 10000,
                "display": "flex",
                "gap": "10px",
                "alignItems": "center",
                "background": "rgba(0,0,0,0.0)",
                "pointerEvents": "none",
            },
        ),

        # Telemetry strip (bottom overlay)
        html.Div(
            [
                telemetry_chip("Speed", "val-speed", "m/s", "#38b6ff"),
                telemetry_chip("Alt", "val-alt", "m", "#6cdb5a"),
                gps_text_chip(),
                telemetry_chip("Voltage", "val-vbat", "V", "#7c5fff"),
                telemetry_chip("Roll", "val-roll", "deg", "#f7a021"),
                telemetry_chip("Pitch", "val-pitch", "deg", "#f7a021"),
                telemetry_chip("Yaw", "val-yaw", "deg", "#f7a021"),
                status_chip(),
                mav_status_chip(),
            ],
            style={
                "position": "fixed",
                "bottom": "10px",
                "left": "50%",
                "transform": "translateX(-50%)",
                "zIndex": 10000,
                "display": "flex",
                "gap": "12px",
                "alignItems": "center",
                "pointerEvents": "none",
            },
        ),

        # Hidden store for manual center
        dcc.Store(id="manual-center", data=None),

        # Map container
        html.Div(
            [
                dl.Map(
                    id="map",
                    center=[32.08, 34.77],
                    zoom=17,
                    children=[
                        dl.TileLayer(id="tile"),
                        dl.Polyline(id="track", positions=[], color="blue", weight=6, opacity=0.85),
                        dl.Polyline(id="goto-line", positions=[], color="red", weight=4, opacity=0.95),
                        dl.Polyline(id="heading-line", positions=[], color="white", weight=3, opacity=0.9),
                        dl.Marker(id="veh", position=[32.08, 34.77]),
                        dl.Marker(
                            id="target-marker",
                            position=[32.08, 34.77],
                            opacity=0.0,  # hidden by default
                            children=[dl.Tooltip("Target")],
                        ),
                    ],
                    style={"width": "100vw", "height": "100vh", "zIndex": 0},
                )
            ],
            style={"margin": 0, "padding": 0},
        ),

        # Interval updates
        dcc.Interval(id="tick", interval=250, n_intervals=0),
    ],
    style={"margin": 0, "padding": 0, "background": "#111"},
)

# ---- Helpers to format ----
def _fmt(v, prec=2):
    try:
        if v is None:
            return "-"
        return str(round(float(v), prec))
    except Exception:
        return "-"

# ---- Callbacks ----
@callback(Output("tile", "url"), Input("layer", "value"))
def change_layer(layer):
    return layer_urls.get(layer, layer_urls["esri"])

@callback(
    Output("map", "center"),
    Output("map", "zoom"),
    Output("track", "positions"),
    Output("veh", "position"),
    Output("goto-line", "positions"),
    Output("goto-line", "color"),
    Output("target-marker", "position"),
    Output("target-marker", "opacity"),
    Output("heading-line", "positions"),
    Output("manual-center", "data"),
    Input("tick", "n_intervals"),
    Input("center-btn", "n_clicks"),
    Input("fit-btn", "n_clicks"),
    State("follow", "value"),
    State("manual-center", "data"),
)
def update_map(_n, c_clicks, f_clicks, follow_val, manual_center):
    trig_id = ctx.triggered_id
    with lock:
        pos = (latest["lat"], latest["lon"])
        yaw_deg = latest.get("yaw")  # 0 ~ north, positive clockwise to east (MAVLink)
        trk = list(track)

    # Build go-to info
    with goto_lock:
        tgt = goto_target

    have_pos = isinstance(pos[0], (int, float)) and isinstance(pos[1], (int, float))
    have_tgt = (tgt is not None) and isinstance(tgt[0], (int, float)) and isinstance(tgt[1], (int, float))

    goto_positions = []
    goto_color = "#ff0000"
    target_pos = [32.08, 34.77]
    target_opacity = 0.0

    # Distance-based color and auto-clear within 5 m
    if have_pos and have_tgt:
        goto_positions = [[pos[0], pos[1]], [tgt[0], tgt[1]]]
        d = haversine_m(pos[0], pos[1], tgt[0], tgt[1])
        goto_color = dist_color_hex(d, max_m=500.0)
        target_pos = [tgt[0], tgt[1]]
        target_opacity = 1.0
        if d is not None and d < 5.0:
            # Clear target and line
            with goto_lock:
                globals()["goto_target"] = None
            goto_positions = []
            target_opacity = 0.0

    # Heading arrow polyline (15 m forward)
    heading_positions = []
    if have_pos and isinstance(yaw_deg, (int, float)):
        fwd_lat, fwd_lon = project_forward(pos[0], pos[1], yaw_deg, 15.0)
        if fwd_lat is not None and fwd_lon is not None:
            heading_positions = [[pos[0], pos[1]], [fwd_lat, fwd_lon]]

    follow = ("on" in (follow_val or []))
    center = no_update
    zoom = no_update

    if trig_id == "fit-btn" and trk:
        lats = [p[0] for p in trk]
        lons = [p[1] for p in trk]
        clat = (min(lats) + max(lats)) / 2.0
        clon = (min(lons) + max(lons)) / 2.0
        center = [clat, clon]
        zoom = 15
        manual_center = center
    elif trig_id == "center-btn" and have_pos:
        center = [pos[0], pos[1]]
        zoom = 18
        manual_center = center
    elif follow and have_pos:
        center = [pos[0], pos[1]]
        zoom = 18
        manual_center = center

    veh_pos = [pos[0], pos[1]] if have_pos else no_update

    return (
        center, zoom, trk, veh_pos,
        goto_positions, goto_color,
        target_pos, target_opacity,
        heading_positions,
        manual_center
    )

@callback(
    Output("val-speed", "children"),
    Output("val-alt", "children"),
    Output("val-gps", "children"),
    Output("val-vbat", "children"),
    Output("val-roll", "children"),
    Output("val-pitch", "children"),
    Output("val-yaw", "children"),
    Output("status-chip", "children"),
    Output("mav-status", "children"),
    Output("mav-status-top", "children"),
    Input("tick", "n_intervals"),
)
def update_cards(_n):
    with lock:
        s = _fmt(latest["groundspeed"], 2)
        alt = _fmt(latest["alt"], 1)
        vb = _fmt(latest["voltage"], 1)
        r = _fmt(latest["roll"], 1)
        p = _fmt(latest["pitch"], 1)
        y = _fmt(latest["yaw"], 1)
        lat = latest.get("lat")
        lon = latest.get("lon")
        st = latest["device_status"].copy()
        sats = latest["gps_sats"]
        hdop = latest["gps_hdop"]

    def _fmt_ll(v):
        try:
            return f"{float(v):.6f}"
        except Exception:
            return "-"

    gps_text = f"{_fmt_ll(lat)}, {_fmt_ll(lon)}"

    def row(ok, label):
        color = "#24e07a" if ok else "#ff4444"
        prefix = "[OK]" if ok else "[X]"
        return html.Div([html.Span(prefix + " "), label],
                        style={"color": color, "fontWeight": "bold"})

    gps_has_fix = st.get("gps", False) and isinstance(lat, (int, float)) and isinstance(lon, (int, float))
    gps_color = "#24e07a" if gps_has_fix else ("#ff9900" if st.get("gps") else "#ff4444")
    gps_prefix = "[OK]" if gps_has_fix else ("[WARN]" if st.get("gps") else "[X]")
    gps_row = html.Div(
        [html.Span(gps_prefix + " "), f"GPS  sats={sats if sats is not None else '-'}  hdop={hdop if hdop is not None else '-'}"],
        style={"color": gps_color, "fontWeight": "bold"},
    )
    children = [gps_row, row(st.get("mpu", False), "MPU")]

    with mav_lock:
        m = mav
    if m:
        mav_line = f"MAV OK | Mode={flight_mode} | Armed={'YES' if armed_state else 'NO'} | Port={MAVLINK_DEVICE}"
    else:
        mav_line = f"MAV DISCONNECTED | Port={MAVLINK_DEVICE}"

    return s, alt, gps_text, vb, r, p, y, children, mav_line, mav_line

@callback(
    Output("btn-goto", "n_clicks"),
    Output("btn-hold", "n_clicks"),
    Output("btn-up1", "n_clicks"),
    Output("btn-dn1", "n_clicks"),
    Input("btn-goto", "n_clicks"),
    Input("btn-hold", "n_clicks"),
    Input("btn-up1", "n_clicks"),
    Input("btn-dn1", "n_clicks"),
    State("goto-lat", "value"),
    State("goto-lon", "value"),
    State("goto-alt", "value"),
    prevent_initial_call=True,
)
def mav_controls(n_goto, n_hold, n_up, n_dn, v_lat, v_lon, v_alt):
    which = ctx.triggered_id
    if which == "btn-goto":
        if v_lat is None or v_lon is None or v_alt is None:
            print("[MAV] Go-To missing lat/lon/alt.")
        else:
            go_to_gps(v_lat, v_lon, v_alt)
    elif which == "btn-hold":
        hold_here_current_alt()
    elif which == "btn-up1":
        bump_alt_guided(+1.0)
    elif which == "btn-dn1":
        bump_alt_guided(-1.0)

    # Reset click counters to avoid repeated triggers
    return 0, 0, 0, 0

# ---- Utilities ----
def _get_lan_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "0.0.0.0"

def main():
    # Start UDP listener
    threading.Thread(target=udp_listener, daemon=True).start()

    # Start MAVLink and keepalive
    threading.Thread(target=mav_connect, daemon=True).start()
    threading.Thread(target=guided_keepalive_loop, daemon=True).start()

    # Server printout similar to Flask dev server style
    host = "0.0.0.0"
    port = 8050
    lan = _get_lan_ip()

    print(f"* Running on all addresses ({host})")
    print(f"* Running on http://127.0.0.1:{port}")
    print(f"* Running on http://{lan}:{port}")
    print("[MAP] Dash listening (press CTRL+C to quit)")

    app.run(host=host, port=port, debug=False)

if __name__ == "__main__":
    main()
