#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ASCII-only, english comments

Dash GUI for map and control:
- Receives MAVLink bytes on UDP 14550 and parses basic telemetry.
- Receives YOLO bbox on UDP 9103 (optional).
- Sends JSON commands over UDP 9104 to the tracker/proxy.
- Includes Takeoff, Set Home, Disable Fence, Mode, Arm/Disarm, Go-To, Hold, Alt bump.
"""

import sys
import math
import json
import time
import socket
import threading
from datetime import datetime

from dash import Dash, html, dcc, callback, Output, Input, State, no_update, ctx
import dash_leaflet as dl
from flask import Flask
from pymavlink import mavutil

try:
    from pymavlink.dialects.v20 import ardupilotmega as mavlink
except Exception:
    from pymavlink.dialects.v20 import common as mavlink

# ---------------- Communication ----------------
UDP_IP = "127.0.0.1"
UDP_PORT_TEL  = 14550  # tracker -> GUI (MAVLink raw bytes)
UDP_PORT_BBOX = 9103   # tracker -> GUI (bbox json)
UDP_PORT_CMD  = 9104   # GUI -> tracker (commands json)

UDP_CMD_SOCK = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_command_to_tracker(cmd_type, data=None):
    """Send a JSON command to the tracker on UDP 9104."""
    if data is None:
        data = {}
    msg = json.dumps({"command": cmd_type, "data": data})
    UDP_CMD_SOCK.sendto(msg.encode("utf-8"), (UDP_IP, UDP_PORT_CMD))
    return f"[{datetime.now().strftime('%H:%M:%S')}] CMD {cmd_type} {data}"

# ---------------- Global telemetry state ----------------
global_mav_data = {
    "lat": 31.9272203818, "lon": 34.7913557651, "alt": 0.0,
    "heading": 0.0, "roll": 0.0, "pitch": 0.0,
    "vfr_hud_gs": 0.0, "flight_mode": "UNKNOWN",
    "volt": 0.0, "current": 0.0,
    "target_lat": None, "target_lon": None, "target_alt": None,
    "bbox": None
}

# ---------------- UDP listeners ----------------
def udp_listener_tel():
    """Listen for MAVLink bytes and update global state."""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((UDP_IP, UDP_PORT_TEL))
        print(f"[UDP-TEL] Listening on {UDP_IP}:{UDP_PORT_TEL}")
        parser = mavlink.MAVLink(file=None, srcSystem=1, srcComponent=1)
        while True:
            raw, _ = sock.recvfrom(2048)
            msgs = parser.parse_buffer(raw)
            if not msgs:
                continue
            for msg in msgs:
                t = msg.get_type()
                if t == "VFR_HUD":
                    global_mav_data["alt"] = float(msg.alt)
                    global_mav_data["vfr_hud_gs"] = float(msg.groundspeed)
                elif t == "GLOBAL_POSITION_INT":
                    global_mav_data["lat"] = msg.lat / 1e7
                    global_mav_data["lon"] = msg.lon / 1e7
                    global_mav_data["heading"] = msg.hdg / 100.0
                elif t == "ATTITUDE":
                    global_mav_data["roll"] = math.degrees(msg.roll)
                    global_mav_data["pitch"] = math.degrees(msg.pitch)
                elif t == "SYS_STATUS":
                    global_mav_data["volt"] = msg.voltage_battery / 1000.0
                    global_mav_data["current"] = msg.current_battery / 100.0
                elif t == "HEARTBEAT":
                    global_mav_data["flight_mode"] = mavutil.mode_string_v10(msg)
    except Exception as e:
        print(f"[UDP-TEL] Error: {e}")

def udp_listener_bbox():
    """Listen for bbox JSON messages and update state."""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((UDP_IP, UDP_PORT_BBOX))
        print(f"[UDP-BBOX] Listening on {UDP_IP}:{UDP_PORT_BBOX}")
        while True:
            data, _ = sock.recvfrom(1024)
            try:
                obj = json.loads(data.decode("utf-8"))
                global_mav_data["bbox"] = obj.get("bbox", None)
            except Exception:
                pass
    except Exception as e:
        print(f"[UDP-BBOX] Error: {e}")

# ---------------- Helpers ----------------
def calculate_bearing(lat1, lon1, lat2, lon2):
    """Return initial bearing from point 1 to point 2 in degrees."""
    lat1_rad = math.radians(lat1); lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2); lon2_rad = math.radians(lon2)
    dlon = lon2_rad - lon1_rad
    y = math.sin(dlon) * math.cos(lat2_rad)
    x = math.cos(lat1_rad)*math.sin(lat2_rad) - math.sin(lat1_rad)*math.cos(lat2_rad)*math.cos(dlon)
    brg = (math.degrees(math.atan2(y, x)) + 360.0) % 360.0
    return brg

# ---------------- Styles ----------------
STYLES = {
    "main_container": {"display": "flex", "flexDirection": "column", "height": "100vh", "margin": "0"},
    "header": {"padding": "12px", "backgroundColor": "#333", "color": "white",
               "textAlign": "center", "fontSize": "24px", "fontWeight": "bold"},
    "map_container": {"flexGrow": "1", "position": "relative"},
    "panel": {"position": "absolute", "top": "10px", "left": "10px", "zIndex": "1000",
              "backgroundColor": "rgba(255,255,255,0.95)", "padding": "14px",
              "borderRadius": "8px", "boxShadow": "0 4px 12px rgba(0,0,0,0.3)", "maxWidth": "340px"},
    "strip": {"padding": "10px", "backgroundColor": "#1E90FF", "color": "white",
              "textAlign": "center", "fontSize": "16px", "fontWeight": "bold"},
    "section_title": {"marginTop": "10px", "fontSize": "16px", "borderBottom": "1px solid #ccc", "paddingBottom": "5px"},
}

# ---------------- App ----------------
server = Flask(__name__)
app = Dash(__name__, server=server)

app.layout = html.Div(style=STYLES["main_container"], children=[
    dcc.Interval(id="interval-1s", interval=1000, n_intervals=0),
    dcc.Interval(id="interval-300ms", interval=300, n_intervals=0),

    html.Div("Map and Control GUI", style=STYLES["header"]),

    html.Div(style=STYLES["map_container"], children=[
        dl.Map(
            id="map",
            center=[global_mav_data["lat"], global_mav_data["lon"]],
            zoom=16,
            children=[
                dl.TileLayer(url="https://{s}.tile.osm.org/{z}/{x}/{y}.png"),
                dl.Marker(id="drone-marker",
                          position=[global_mav_data["lat"], global_mav_data["lon"]],
                          children=[dl.Tooltip(id="drone-tooltip")]),
                dl.Polyline(id="heading-line", positions=[[0, 0], [0, 0]], weight=3),
                dl.Polyline(id="goto-line", positions=[[0, 0], [0, 0]], color="red", weight=4),
            ],
            style={"width": "100%", "height": "100%"}
        ),

        html.Div(style=STYLES["panel"], children=[
            html.H4("Control", style={"marginTop": "0", "borderBottom": "1px solid #ccc", "paddingBottom": "5px"}),

            # Go-To
            html.Div([
                html.B("Go-To GPS:"),
                dcc.Input(id="input-gps-full", type="text", placeholder="lat, lon",
                          style={"width": "100%", "marginBottom": "6px"}, value="32.09, 34.81"),
                dcc.Input(id="input-alt", type="text", placeholder="Alt (m) [empty=current]",
                          style={"width": "45%", "marginRight": "5px"}),
                html.Button("Go-To", id="btn-goto", n_clicks=0,
                            style={"width": "45%", "backgroundColor": "#28a745", "color": "white"}),
            ], style={"marginBottom": "10px"}),

            # Hold + Alt bumps
            html.Div([
                html.Button("Hold Here", id="btn-hold", n_clicks=0,
                            style={"width": "48%", "marginRight": "4%", "backgroundColor": "#007bff", "color": "white"}),
                html.Button("+1m", id="btn-up1", n_clicks=0,
                            style={"width": "22%", "backgroundColor": "#ffc107"}),
                html.Button("-1m", id="btn-dn1", n_clicks=0,
                            style={"width": "22%", "marginLeft": "4%", "backgroundColor": "#ffc107"}),
            ], style={"marginBottom": "10px"}),

            html.H4("Flight", style=STYLES["section_title"]),
            html.Div([
                html.Button("ARM", id="btn-arm", n_clicks=0,
                            style={"width": "32%", "marginRight": "2%", "backgroundColor": "#008000",
                                   "color": "white", "fontWeight": "bold"}),
                html.Button("DISARM", id="btn-disarm", n_clicks=0,
                            style={"width": "32%", "marginRight": "2%", "backgroundColor": "#FFA500",
                                   "color": "black", "fontWeight": "bold"}),
                html.Button("E-STOP", id="btn-emergency-stop", n_clicks=0,
                            style={"width": "32%", "backgroundColor": "#FF0000",
                                   "color": "white", "fontWeight": "bold"}),
            ], style={"marginBottom": "10px"}),

            html.Div([
                dcc.Dropdown(
                    id="dropdown-flight-mode",
                    options=[
                        {"label": "Stabilize", "value": "STABILIZE"},
                        {"label": "Loiter", "value": "LOITER"},
                        {"label": "Alt Hold", "value": "ALT_HOLD"},
                        {"label": "Guided", "value": "GUIDED"},
                        {"label": "RTL", "value": "RTL"},
                        {"label": "Land", "value": "LAND"},
                    ],
                    value="LOITER",
                    placeholder="Select flight mode",
                    style={"width": "100%"},
                ),
                html.Button("Set Mode", id="btn-set-mode", n_clicks=0,
                            style={"width": "100%", "marginTop": "6px", "backgroundColor": "#4CAF50", "color": "white"}),
            ], style={"marginBottom": "10px"}),

            # Takeoff
            html.Div([
                html.B("Takeoff Alt (m):"),
                dcc.Input(id="takeoff-alt", type="number", placeholder="e.g. 20",
                          style={"width": "45%", "marginRight": "5px"}),
                html.Button("Takeoff", id="btn-takeoff", n_clicks=0,
                            style={"width": "45%", "backgroundColor": "#6c757d", "color": "white"}),
            ], style={"marginBottom": "10px"}),

            # Set Home + Disable Fence
            html.Div([
                html.B("Set Home:"),
                dcc.Input(id="home-lat", type="text", placeholder="lat", style={"width": "30%", "marginRight": "3px"}),
                dcc.Input(id="home-lon", type="text", placeholder="lon", style={"width": "30%", "marginRight": "3px"}),
                dcc.Input(id="home-alt", type="text", placeholder="alt", style={"width": "30%"}),
                html.Button("Set Home", id="btn-set-home", n_clicks=0,
                            style={"width": "100%", "marginTop": "6px", "backgroundColor": "#17a2b8", "color": "white"}),
                html.Button("Disable Fence", id="btn-disable-fence", n_clicks=0,
                            style={"width": "100%", "marginTop": "6px", "backgroundColor": "#dc3545", "color": "white"}),
            ], style={"marginBottom": "10px"}),

            html.H4("Detection (optional)", style=STYLES["section_title"]),
            html.Div([
                html.Button("Start YOLO", id="btn-start-yolo", n_clicks=0,
                            style={"width": "48%", "marginRight": "4%", "backgroundColor": "#dc3545", "color": "white"}),
                html.Button("Stop YOLO", id="btn-stop-yolo", n_clicks=0,
                            style={"width": "48%", "backgroundColor": "#ffc107"}),
            ], style={"marginBottom": "10px"}),

            html.Div([
                html.B("Class filter:"),
                dcc.Checklist(
                    id="checklist-predefined-classes",
                    options=[{"label": "person (0)", "value": "0"}, {"label": "tv/monitor (62)", "value": "62"}],
                    value=["0"]
                ),
                dcc.Input(id="input-custom-classes", type="text", placeholder="2,17,39",
                          style={"width": "100%", "marginTop": "6px"}),
                html.Button("Apply classes", id="btn-apply-classes", n_clicks=0,
                            style={"width": "100%", "marginTop": "6px", "backgroundColor": "#007bff", "color": "white"}),
            ]),

            html.Hr(),

            # Target/BBox panels EXIST to match callback outputs
            html.Div(id="target-info", style={"fontSize": "14px", "fontWeight": "bold"}),
            html.Pre(id="bbox-data-display",
                     style={"fontSize": "13px", "whiteSpace": "pre-wrap", "fontFamily": "monospace",
                            "marginTop": "5px", "maxHeight": "140px", "overflowY": "auto"}),

            html.Hr(),

            html.Pre(id="cmd-status-log",
                     style={"fontSize": "12px", "whiteSpace": "pre-wrap", "fontFamily": "monospace",
                            "maxHeight": "140px", "overflowY": "auto", "border": "1px solid #ddd", "padding": "6px"}),
        ])
    ]),

    html.Div(id="info-strip", children="Waiting for MAVLink...", style=STYLES["strip"]),
])

# ---------------- Callbacks ----------------
@callback(
    Output("drone-marker", "position"),
    Output("drone-tooltip", "children"),
    Output("info-strip", "children"),
    Output("heading-line", "positions"),
    Output("goto-line", "positions"),
    Output("goto-line", "color"),
    Output("target-info", "children"),
    Output("bbox-data-display", "children"),
    Input("interval-1s", "n_intervals"),
)
def update_telemetry_and_map(_n):
    d = global_mav_data
    lat, lon = d["lat"], d["lon"]

    if lat is None or lon is None or (lat == 0.0 and lon == 0.0):
        info_text = f"Mode: {d['flight_mode']} | Alt: {d['alt']:.1f}m | Waiting for GPS..."
        return no_update, no_update, info_text, [[0, 0], [0, 0]], [[0, 0], [0, 0]], "red", "No target", "No detection"

    # positions
    marker_pos = [lat, lon]
    tooltip = f"Alt: {d['alt']:.1f} m | GS: {d['vfr_hud_gs']:.1f} m/s"
    strip = (
        f"Mode: {d['flight_mode']} | Lat/Lon: {lat:.5f}, {lon:.5f} | "
        f"Alt: {d['alt']:.1f} m | GS: {d['vfr_hud_gs']:.1f} m/s | "
        f"Hdg: {d['heading']:.0f} deg | Roll/Pitch: {d['roll']:.1f}/{d['pitch']:.1f} deg"
    )

    # heading line
    end_lat = lat + 0.0001 * math.cos(math.radians(d["heading"]))
    end_lon = lon + 0.0001 * math.sin(math.radians(d["heading"]))
    heading_positions = [[lat, lon], [end_lat, end_lon]]

    # target info
    goto_positions = [[0, 0], [0, 0]]
    goto_color = "red"
    target_info_text = "No target set"
    if d["target_lat"] is not None and d["target_lon"] is not None:
        tlat, tlon = d["target_lat"], d["target_lon"]
        bearing = calculate_bearing(lat, lon, tlat, tlon)
        # distance is optional; keep simple dummy for now
        dist_m = 10.0
        target_info_text = (
            f"Target: {tlat:.5f}, {tlon:.5f} (Alt: {d['target_alt']})\n"
            f"Bearing: {bearing:.1f} deg | Distance: {dist_m:.1f} m"
        )
        goto_positions = [[lat, lon], [tlat, tlon]]
        goto_color = "green" if dist_m < 5.0 else "red"

    # bbox panel
    if d["bbox"]:
        try:
            x1, y1, x2, y2, tid = d["bbox"]
            bbox_txt = f"DETECTION\nTrack: {int(tid)}\n({x1:.0f},{y1:.0f}) -> ({x2:.0f},{y2:.0f})"
        except Exception:
            bbox_txt = f"DETECTION\n{d['bbox']}"
    else:
        bbox_txt = "No detection"

    return marker_pos, tooltip, strip, heading_positions, goto_positions, goto_color, target_info_text, bbox_txt


@callback(
    Output("map", "center"),
    Output("cmd-status-log", "children"),
    Input("btn-goto", "n_clicks"),
    Input("btn-hold", "n_clicks"),
    Input("btn-up1", "n_clicks"),
    Input("btn-dn1", "n_clicks"),
    Input("btn-start-yolo", "n_clicks"),
    Input("btn-stop-yolo", "n_clicks"),
    Input("btn-apply-classes", "n_clicks"),
    Input("btn-set-mode", "n_clicks"),
    Input("btn-arm", "n_clicks"),
    Input("btn-disarm", "n_clicks"),
    Input("btn-emergency-stop", "n_clicks"),
    Input("btn-takeoff", "n_clicks"),
    Input("btn-set-home", "n_clicks"),
    Input("btn-disable-fence", "n_clicks"),
    State("input-gps-full", "value"),
    State("input-alt", "value"),
    State("checklist-predefined-classes", "value"),
    State("input-custom-classes", "value"),
    State("dropdown-flight-mode", "value"),
    State("takeoff-alt", "value"),
    State("home-lat", "value"),
    State("home-lon", "value"),
    State("home-alt", "value"),
    State("cmd-status-log", "children"),
    prevent_initial_call=True,
)
def handle_controls(goto_n, hold_n, up_n, dn_n, start_n, stop_n, apply_n,
                    setmode_n, arm_n, disarm_n, estop_n, takeoff_n, sethome_n, fence_n,
                    v_gps, v_alt, v_checked, v_custom, v_mode, v_tko_alt,
                    v_home_lat, v_home_lon, v_home_alt, log_text):

    which = ctx.triggered_id
    logs = log_text or ""
    def append_log(line):
        nonlocal logs
        logs = (logs + "\n" + line).strip()

    # default center: keep map where it is
    new_center = no_update

    if which == "btn-goto":
        lat = lon = None
        alt = None
        try:
            parts = [p.strip() for p in (v_gps or "").split(",")]
            if len(parts) == 2:
                lat = float(parts[0]); lon = float(parts[1])
        except Exception:
            append_log("[ERR] GPS parse failed. Use 'lat, lon'.")
        if v_alt is not None and str(v_alt).strip() != "":
            try:
                alt = float(v_alt)
            except Exception:
                append_log("[WARN] Alt not numeric; using current.")
        if alt is None:
            alt = global_mav_data["alt"] if global_mav_data["alt"] is not None else 5.0
        if lat is None or lon is None:
            append_log("[ERR] Missing lat/lon for Go-To.")
        else:
            line = send_command_to_tracker("GOTO_GPS", {"lat": lat, "lon": lon, "alt": float(alt)})
            append_log(line)
            global_mav_data["target_lat"] = lat
            global_mav_data["target_lon"] = lon
            global_mav_data["target_alt"] = float(alt)
            new_center = [lat, lon]

    elif which == "btn-hold":
        append_log(send_command_to_tracker("HOLD_HERE"))
        global_mav_data["target_lat"] = None
        global_mav_data["target_lon"] = None
        global_mav_data["target_alt"] = None

    elif which == "btn-up1":
        append_log(send_command_to_tracker("ALT_BUMP", {"delta": 1.0}))
    elif which == "btn-dn1":
        append_log(send_command_to_tracker("ALT_BUMP", {"delta": -1.0}))

    elif which == "btn-start-yolo":
        append_log(send_command_to_tracker("START_YOLO"))
    elif which == "btn-stop-yolo":
        append_log(send_command_to_tracker("STOP_YOLO"))
    elif which == "btn-apply-classes":
        all_ids = []
        if v_checked:
            all_ids.extend(v_checked)
        if v_custom and v_custom.strip():
            all_ids.extend([c.strip() for c in v_custom.split(",") if c.strip()])
        # keep numeric only
        num_ids = []
        for s in all_ids:
            try:
                num_ids.append(int(s))
            except Exception:
                pass
        uniq = sorted(set(num_ids))
        append_log(send_command_to_tracker("SET_CLASSES", {"classes": ",".join(str(x) for x in uniq)}))

    elif which == "btn-set-mode":
        if v_mode:
            append_log(send_command_to_tracker("SET_MODE", {"mode": v_mode}))

    elif which == "btn-arm":
        append_log(send_command_to_tracker("ARM"))
    elif which == "btn-disarm":
        append_log(send_command_to_tracker("DISARM"))
    elif which == "btn-emergency-stop":
        append_log(send_command_to_tracker("EMERGENCY_STOP"))

    elif which == "btn-takeoff":
        try:
            alt = float(v_tko_alt) if v_tko_alt is not None else 20.0
        except Exception:
            alt = 20.0
        append_log(send_command_to_tracker("TAKEOFF", {"alt": float(alt)}))

    elif which == "btn-set-home":
        try:
            hlat = float(v_home_lat)
            hlon = float(v_home_lon)
            halt = float(v_home_alt) if v_home_alt not in (None, "") else 10.0
            append_log(send_command_to_tracker("SET_HOME", {"lat": hlat, "lon": hlon, "alt": halt}))
        except Exception:
            append_log("[ERR] Set Home needs numeric lat/lon (and optional alt).")

    elif which == "btn-disable-fence":
        append_log(send_command_to_tracker("DISABLE_FENCE"))

    return new_center, logs

# ---------------- Boot ----------------
if __name__ == "__main__":
    threading.Thread(target=udp_listener_tel, daemon=True).start()
    threading.Thread(target=udp_listener_bbox, daemon=True).start()
    print("Dash listening at http://127.0.0.1:8050/")
    app.run(debug=True, port=8050, use_reloader=False)
