#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ASCII-only comments.

Dash map for drone telemetry:
- Receives UDP JSON on port 9002: {lat, lon, roll, pitch, yaw, groundspeed, alt, voltage, device_status{gps,mpu}}
- Logs track to CSV 'logs/gps_log_YYYY-mm-dd_HH-MM-SS.csv'
- Shows live marker + blue polyline + bottom telemetry strip + follow/fit controls
- Overlay toolbars stay above the map (z-index) without blocking map gestures (pointer-events).
- No OBD. Values are taken from your GUI's UDP broadcast.

Dependencies:
  pip install dash dash-leaflet
"""

import os
import csv
import json
import socket
import threading
from datetime import datetime

from dash import Dash, html, dcc, callback, Output, Input, State, no_update, ctx
import dash_leaflet as dl
from flask import Flask

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
    "yaw": 0.0,
    "groundspeed": 0.0,
    "alt": 0.0,
    "voltage": None,
    "device_status": {"gps": False, "mpu": False, "obd": False},
    "gps_quality": None,
    "gps_sats": None,
    "gps_hdop": None,
}
track = []  # list of (lat, lon)
lock = threading.Lock()


def append_csv(ts_iso, lat, lon):
    # append a line to the CSV log
    with open(GPS_LOG_FILENAME, "a", newline="") as f:
        csv.writer(f).writerow([ts_iso, lat, lon])


# ----- UDP listener -----
UDP_PORT = 9002


def udp_listener():
    # UDP listener for telemetry JSON
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
            # copy known fields if present
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
                    latest[k] = data[k]

            # device status dict
            if "device_status" in data and isinstance(data["device_status"], dict):
                latest["device_status"].update(data["device_status"])

            # track update + CSV append (dedupe tiny repeats)
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
                    inline=True,
                    inputStyle={"marginRight": "6px"},
                    style={
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
            ],
            style={
                "position": "fixed",
                "top": "10px",
                "left": "50%",
                "transform": "translateX(-50%)",
                "zIndex": 10000,                 # ensure above the map
                "display": "flex",
                "gap": "10px",
                "alignItems": "center",
                "background": "rgba(0,0,0,0.0)",
                "pointerEvents": "none",          # container ignores map gestures
            },
        ),

        # Telemetry strip (bottom overlay)
        html.Div(
            [
                telemetry_chip("Speed", "val-speed", "m/s", "#38b6ff"),
                telemetry_chip("Alt", "val-alt", "m", "#6cdb5a"),
                telemetry_chip("Voltage", "val-vbat", "V", "#7c5fff"),
                telemetry_chip("Roll", "val-roll", "deg", "#f7a021"),
                telemetry_chip("Pitch", "val-pitch", "deg", "#f7a021"),
                telemetry_chip("Yaw", "val-yaw", "deg", "#f7a021"),
                status_chip(),
            ],
            style={
                "position": "fixed",
                "bottom": "10px",
                "left": "50%",
                "transform": "translateX(-50%)",
                "zIndex": 10000,                 # ensure above the map
                "display": "flex",
                "gap": "12px",
                "alignItems": "center",
                "pointerEvents": "none",          # overlay does not block map gestures
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
                        dl.Marker(id="veh", position=[32.08, 34.77]),
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
        trk = list(track)

    follow = ("on" in (follow_val or []))
    center = no_update
    zoom = no_update

    have_pos = all(isinstance(x, (int, float)) for x in pos)

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
    return center, zoom, trk, veh_pos, manual_center


@callback(
    Output("val-speed", "children"),
    Output("val-alt", "children"),
    Output("val-vbat", "children"),
    Output("val-roll", "children"),
    Output("val-pitch", "children"),
    Output("val-yaw", "children"),
    Output("status-chip", "children"),
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
        st = latest["device_status"].copy()
        sats = latest["gps_sats"]
        hdop = latest["gps_hdop"]

    def row(ok, label):
        color = "#24e07a" if ok else "#ff4444"
        prefix = "[OK]" if ok else "[X]"
        return html.Div(
            [html.Span(prefix + " "), label],
            style={"color": color, "fontWeight": "bold"},
        )

    gps_has_fix = st.get("gps", False) and isinstance(latest.get("lat"), (int, float)) and isinstance(
        latest.get("lon"), (int, float)
    )
    gps_color = "#24e07a" if gps_has_fix else ("#ff9900" if st.get("gps") else "#ff4444")
    gps_prefix = "[OK]" if gps_has_fix else ("[WARN]" if st.get("gps") else "[X]")
    gps_row = html.Div(
        [
            html.Span(gps_prefix + " "),
            f"GPS  sats={sats if sats is not None else '-'}  hdop={hdop if hdop is not None else '-'}",
        ],
        style={"color": gps_color, "fontWeight": "bold"},
    )

    children = [gps_row, row(st.get("mpu", False), "MPU")]
    return s, alt, vb, r, p, y, children


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

    # Server printout similar to Flask dev server style
    host = "0.0.0.0"
    port = 8050
    lan = _get_lan_ip()

    print(f"* Running on all addresses ({host})")
    print(f"* Running on http://127.0.0.1:{port}")
    print(f"* Running on http://{lan}:{port}")
    print("[MAP] Dash listening (press CTRL+C to quit)")

    # Run Dash via app.run (Dash >=2.9)
    app.run(host=host, port=port, debug=False)


if __name__ == "__main__":
    main()
