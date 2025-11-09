import sys
import os
import csv
import json
import math
import socket
import threading
from datetime import datetime
import time

from dash import Dash, html, dcc, callback, Output, Input, State, no_update, ctx
import dash_leaflet as dl
from flask import Flask

# ייבוא MAVLink לניתוח הודעות (לא לשליטה ישירה)
from pymavlink import mavutil

try:
    from pymavlink.dialects.v20 import ardupilotmega as mavlink
except ImportError:
    from pymavlink.dialects.v20 import common as mavlink

# --- Global Telemetry State ---
global_mav_data = {
    'lat': 31.92722038182233, 'lon':  34.79135576510038, 'alt': 50.0, 'heading': 0, 'roll': 0, 'pitch': 0,
    'vfr_hud_gs': 0.0, 'sat_fix': 0, 'flight_mode': 'UNKNOWN', 'volt': 0.0, 'current': 0.0,
    'target_lat': None, 'target_lon': None, 'target_alt': None,
    'bbox': None
}

# --- Communication Settings ---
UDP_IP = '127.0.0.1'
UDP_PORT_TEL = 14550  # קבלת טלמטריה מהטרקר
UDP_PORT_BBOX = 9103  # קבלת BBox מהטרקר
UDP_PORT_CMD = 9104  # שליחת פקודות לטרקר

# --- Styles ---
STYLES = {
    'main_container': {'display': 'flex', 'flexDirection': 'column', 'height': '100vh', 'margin': '0'},
    'header': {'padding': '12px', 'backgroundColor': '#333', 'color': 'white', 'textAlign': 'center',
               'fontSize': '24px', 'fontWeight': 'bold', 'zIndex': '1001'},
    'map_container': {'flexGrow': '1', 'position': 'relative'},
    'telemetry_strip': {'padding': '10px', 'backgroundColor': '#1E90FF', 'color': 'white',
                        'textAlign': 'center', 'fontSize': '16px', 'fontWeight': 'bold'},
    'overlay_controls': {
        'position': 'absolute', 'top': '10px', 'left': '10px', 'zIndex': '1000',
        'backgroundColor': 'rgba(255, 255, 255, 0.95)', 'padding': '15px', 'borderRadius': '8px',
        'boxShadow': '0 4px 12px rgba(0,0,0,0.3)', 'maxWidth': '300px'
    }
}

# --- UDP Command Sender ---
UDP_CMD_SOCK = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def send_command_to_tracker(cmd_type, data=None):
    """שולח פקודת JSON לטרקר דרך UDP 9104."""
    if data is None: data = {}
    message = json.dumps({"command": cmd_type, "data": data})
    try:
        UDP_CMD_SOCK.sendto(message.encode('utf-8'), (UDP_IP, UDP_PORT_CMD))
        print(f"[CMD] Sent: {cmd_type} with data: {data}")
        return True
    except Exception as e:
        print(f"[CMD] Failed to send {cmd_type}: {e}")
        return False


# ================= UDP Listener Threads =================

def udp_listener_tel():
    # ... (כפי שהיה)
    # קבלת טלמטריה מ-MAVLink והעברתה ל-global_mav_data
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((UDP_IP, UDP_PORT_TEL))
        print(f"[UDP-TEL] Listening for Telemetry on {UDP_IP}:{UDP_PORT_TEL}")
        mav_parser = mavlink.MAVLink(file=None, srcSystem=1, srcComponent=1)
        while True:
            raw_bytes, addr = sock.recvfrom(2048)
            msgs = mav_parser.parse_buffer(raw_bytes)
            if msgs:
                for msg in msgs:
                    msg_type = msg.get_type()

                    if msg_type == 'VFR_HUD':
                        global_mav_data['alt'] = msg.alt
                        global_mav_data['vfr_hud_gs'] = msg.groundspeed
                    elif msg_type == 'GLOBAL_POSITION_INT':
                        global_mav_data['lat'] = msg.lat / 1e7
                        global_mav_data['lon'] = msg.lon / 1e7
                        global_mav_data['heading'] = msg.hdg / 100
                    elif msg_type == 'ATTITUDE':
                        global_mav_data['roll'] = math.degrees(msg.roll)
                        global_mav_data['pitch'] = math.degrees(msg.pitch)
                    elif msg_type == 'SYS_STATUS':
                        global_mav_data['volt'] = msg.voltage_battery / 1000.0
                        global_mav_data['current'] = msg.current_battery / 100.0
                    elif msg_type == 'HEARTBEAT':
                        global_mav_data['flight_mode'] = mavutil.mode_string_v10(msg)

    except Exception as e:
        print(f"[UDP-TEL] Error in listener: {e}")


def udp_listener_bbox():
    # ... (כפי שהיה)
    # קבלת נתוני BBox - מטפל ב-null כשאובייקט אובד
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((UDP_IP, UDP_PORT_BBOX))
        print(f"[UDP-BBOX] Listening for BBox on {UDP_IP}:{UDP_PORT_BBOX}")
        while True:
            data, addr = sock.recvfrom(1024)
            message = data.decode('utf-8')
            bbox_obj = json.loads(message)
            if 'bbox' in bbox_obj:
                global_mav_data['bbox'] = bbox_obj['bbox']
    except Exception as e:
        print(f"[UDP-BBOX] Error in listener: {e}")


# ================= UI Helpers =================

def calculate_bearing(lat1, lon1, lat2, lon2):
    # ... (כפי שהיה)
    """מחשב זווית (Bearing) מנקודה 1 לנקודה 2 במעלות."""
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)

    dLon = lon2_rad - lon1_rad

    y = math.sin(dLon) * math.cos(lat2_rad)
    x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dLon)

    bearing = math.degrees(math.atan2(y, x))
    return (bearing + 360) % 360


# ================= Dash Layout (מעודכן) =================

server = Flask(__name__)
app = Dash(__name__, server=server)

app.layout = html.Div(style=STYLES['main_container'], children=[

    dcc.Interval(id="interval-1s", interval=1000, n_intervals=0),
    dcc.Interval(id="interval-300ms", interval=300, n_intervals=0),

    html.Div(style=STYLES['header'], children="ממשק מפה ושליטה (COM4 -> UDP)"),

    html.Div(style=STYLES['map_container'], children=[

        # 2a. המפה
        dl.Map(
            id="map",
            center=[global_mav_data['lat'], global_mav_data['lon']],
            zoom=16,
            children=[
                dl.TileLayer(url="https://{s}.tile.osm.org/{z}/{x}/{y}.png"),

                # מארקר הרחפן
                dl.Marker(id="drone-marker", position=[global_mav_data['lat'], global_mav_data['lon']], children=[
                    dl.Tooltip(id="drone-tooltip")
                ]),
                # קו כיוון הרחפן
                dl.Polyline(id="heading-line", positions=[[0, 0], [0, 0]], color="orange", weight=3),

                # קו Go-To (יעד)
                dl.Polyline(id="goto-line", positions=[[0, 0], [0, 0]], color="red", weight=4),
            ],
            style={'width': '100%', 'height': '100%'}
        ),

        # 2b. פאנל שליטה צף
        html.Div(style=STYLES['overlay_controls'], children=[
            html.H4("שליטה ויעדים (UDP)",
                    style={'marginTop': '0', 'borderBottom': '1px solid #ccc', 'paddingBottom': '5px'}),

            # --- הזנת יעד (Go-To) ---
            html.Div([
                html.B("יעד GPS:"),
                dcc.Input(
                    id="input-gps-full",
                    type="text",
                    placeholder="Latitude, Longitude",
                    style={'width': '100%', 'marginBottom': '5px'},
                    value="32.09, 34.81"
                ),
                dcc.Input(id="input-alt", type="text", placeholder="Alt (m) [ריק = גובה נוכחי]",
                          style={'width': '45%', 'marginRight': '5px'}),
                html.Button("Go-To", id="btn-goto", n_clicks=0,
                            style={'width': '45%', 'backgroundColor': '#28a745', 'color': 'white'}),
            ], style={'marginBottom': '10px'}),

            # --- כפתורי שליטה (Alt/Hold) ---
            html.Div([
                html.Button("Hold Here", id="btn-hold", n_clicks=0,
                            style={'width': '48%', 'marginRight': '4%', 'backgroundColor': '#007bff',
                                   'color': 'white'}),
                html.Button("+1m", id="btn-up1", n_clicks=0,
                            style={'width': '22%', 'backgroundColor': '#ffc107', 'color': 'black'}),
                html.Button("-1m", id="btn-dn1", n_clicks=0,
                            style={'width': '22%', 'marginLeft': '4%', 'backgroundColor': '#ffc107', 'color': 'black'}),
            ], style={'marginBottom': '10px'}),

            html.Hr(),

            # --- בקרת מצב טיסה וחימוש (חדש) ---
            html.H4("בקרת טיסה וחירום",
                    style={'marginTop': '10px', 'fontSize': '16px', 'borderBottom': '1px solid #ccc',
                           'paddingBottom': '5px'}),

            # Arm/Disarm/E-Stop Buttons
            html.Div([
                html.Button("ARM (חימוש)", id="btn-arm", n_clicks=0,
                            style={'width': '32%', 'marginRight': '2%', 'backgroundColor': '#008000', 'color': 'white',
                                   'fontWeight': 'bold'}),
                html.Button("DISARM (נטרול)", id="btn-disarm", n_clicks=0,
                            style={'width': '32%', 'marginRight': '2%', 'backgroundColor': '#FFA500', 'color': 'black',
                                   'fontWeight': 'bold'}),
                html.Button("E-STOP (הרג)", id="btn-emergency-stop", n_clicks=0,
                            style={'width': '32%', 'backgroundColor': '#FF0000', 'color': 'white',
                                   'fontWeight': 'bold'}),
            ], style={'marginBottom': '10px'}),

            # Mode Change Dropdown
            html.Div([
                dcc.Dropdown(
                    id='dropdown-flight-mode',
                    options=[
                        {'label': 'Stabilize', 'value': 'STABILIZE'},
                        {'label': 'Loiter', 'value': 'LOITER'},
                        {'label': 'Alt Hold', 'value': 'ALT_HOLD'},
                        {'label': 'Guided', 'value': 'GUIDED'},
                        {'label': 'RTL (חזרה לבית)', 'value': 'RTL'},
                        {'label': 'Land (נחיתה)', 'value': 'LAND'}
                    ],
                    value='LOITER',
                    placeholder="בחר מצב טיסה",
                    style={'width': '100%'}
                ),
                html.Button("החל מצב", id="btn-set-mode", n_clicks=0,
                            style={'width': '100%', 'backgroundColor': '#4CAF50', 'color': 'white', 'padding': '6px',
                                   'marginTop': '5px'}),
            ], style={'marginBottom': '10px'}),

            html.Hr(),

            # --- כפתורי YOLO (כפי שהיו) ---
            html.Div([
                html.Button("התחל זיהוי (YOLO)", id="btn-start-yolo", n_clicks=0,
                            style={'width': '48%', 'marginRight': '4%', 'backgroundColor': '#dc3545',
                                   'color': 'white'}),
                html.Button("עצור זיהוי", id="btn-stop-yolo", n_clicks=0,
                            style={'width': '48%', 'backgroundColor': '#ffc107', 'color': 'black'}),
            ], style={'marginBottom': '10px'}),

            html.Hr(),

            # --- קלט סינון מחלקות (Class Filter) ---
            html.Div([
                html.B("סינון מחלקות:", style={'display': 'block', 'marginBottom': '5px'}),

                # Checkboxes
                dcc.Checklist(
                    id="checklist-predefined-classes",
                    options=[
                        {'label': 'אדם (0)', 'value': '0'},
                        {'label': 'טלוויזיה/מוניטור (62)', 'value': '62'}
                    ],
                    value=['0'],  # ברירת מחדל: אדם
                    inline=False,
                    style={'marginBottom': '5px'}
                ),

                # Custom Text Input
                html.B("הזנת IDs נוספים (מופרדים בפסיקים):",
                       style={'display': 'block', 'marginBottom': '3px', 'fontSize': '12px', 'marginTop': '10px'}),
                dcc.Input(
                    id="input-custom-classes",
                    type="text",
                    placeholder="2, 17, 39...",
                    style={'width': '100%'},
                    value=""
                ),

                html.Button("החל פילטר", id="btn-apply-classes", n_clicks=0,
                            style={'width': '100%', 'backgroundColor': '#007bff', 'color': 'white', 'padding': '6px',
                                   'marginTop': '10px'}),
            ], style={'marginBottom': '10px'}),

            html.Hr(),

            # --- נתוני יעד / BBox ---
            html.Div(id="target-info", style={'fontSize': '14px', 'fontWeight': 'bold'}),
            html.Div(id="bbox-data-display",
                     style={'fontSize': '14px', 'whiteSpace': 'pre-wrap', 'fontFamily': 'monospace',
                            'marginTop': '5px'}),
        ])
    ]),

    # 3. רצועת טלמטריה תחתונה
    html.Div(id="info-strip", children="ממתין לנתוני MAVLink...", style=STYLES['telemetry_strip']),
])


# ================= Callbacks (מעודכן) =================

# --- 1. עדכון מפה וטלמטריה (ללא שינוי) ---
@callback(
    Output("drone-marker", "position"),
    Output("drone-tooltip", "children"),
    Output("info-strip", "children"),
    Output("heading-line", "positions"),
    Output("goto-line", "positions"),
    Output("goto-line", "color"),
    Output("target-info", "children"),
    Output("bbox-data-display", "children"),
    Input("interval-1s", "n_intervals")
)
def update_telemetry_and_map(n):
    data = global_mav_data
    lat, lon = data['lat'], data['lon']

    # --- עדכון מיקום ואינפו ---
    if lat is None or lon is None or (lat == 0.0 and lon == 0.0):
        info_text = f"Alt: {data['alt']:.2f}m | Mode: {data['flight_mode']} | ממתין ל-GPS..."
        return no_update, no_update, info_text, [[0, 0], [0, 0]], [[0, 0],
                                                                   [0, 0]], "red", "אין יעד מוגדר", "אין איתור כרגע"

    # --- טלמטריה ---
    position = [lat, lon]
    tooltip_text = f"Alt: {data['alt']:.1f}m | GS: {data['vfr_hud_gs']:.1f}m/s"
    info_strip_text = (
        f"**Mode:** {data['flight_mode']} | **Lat/Lon:** {lat:.5f}, {lon:.5f} | "
        f"**Alt:** {data['alt']:.1f}m | **GS:** {data['vfr_hud_gs']:.1f}m/s | **Heading:** {data['heading']}° | "
        f"**Roll/Pitch:** {data['roll']:.1f}°/{data['pitch']:.1f}° | **Batt:** {data['volt']:.1f}V / {data['current']:.1f}A"
    )

    # --- קו כיוון (Heading) ---
    end_lat = lat + 0.0001 * math.cos(math.radians(data['heading']))
    end_lon = lon + 0.0001 * math.sin(math.radians(data['heading']))
    heading_positions = [[lat, lon], [end_lat, end_lon]]

    # --- נתוני יעד וקו Go-To ---
    goto_positions = [[0, 0], [0, 0]]
    goto_color = "red"
    target_info_text = "אין יעד מוגדר"

    if data['target_lat'] is not None and data['target_lon'] is not None:
        target_lat, target_lon = data['target_lat'], data['target_lon']

        bearing_to_target = calculate_bearing(lat, lon, target_lat, target_lon)
        distance_m = 10  # ערך דמה

        target_info_text = (
            f"**Target:** {target_lat:.5f}, {target_lon:.5f} (Alt: {data['target_alt']}m)\n"
            f"**Bearing:** {bearing_to_target:.1f}° | **Distance (Dummy):** {distance_m:.1f}m"
        )

        goto_positions = [[lat, lon], [target_lat, target_lon]]
        goto_color = "green" if distance_m < 5 else "red"

        # --- BBox טקסטואלי ---
    if data['bbox']:
        # BBox format: x1, y1, x2, y2, track_id
        x1, y1, x2, y2, track_id = data['bbox']

        bbox_display = f"**!! אובייקט זוהה !!**\nTrack ID: {int(track_id)}\nx1:{x1:.0f}, y1:{y1:.0f}\nx2:{x2:.0f}, y2:{y2:.0f}"
    else:
        # data['bbox'] הוא None (כאשר הטרקר שלח null)
        bbox_display = "אין איתור כרגע (אובייקט אבד)"

    return position, tooltip_text, info_strip_text, heading_positions, goto_positions, goto_color, target_info_text, bbox_display


# --- 2. טיפול בפקודות שליטה (מעודכן לחימוש ומצבים) ---
@callback(
    Output("map", "center"),
    # פקדי Go-To / Hold / Alt
    Input("btn-goto", "n_clicks"),
    Input("btn-hold", "n_clicks"),
    Input("btn-up1", "n_clicks"),
    Input("btn-dn1", "n_clicks"),
    # פקדי YOLO
    Input("btn-start-yolo", "n_clicks"),
    Input("btn-stop-yolo", "n_clicks"),
    Input("btn-apply-classes", "n_clicks"),
    # פקדי טיסה (חדש)
    Input("btn-set-mode", "n_clicks"),
    Input("btn-arm", "n_clicks"),
    Input("btn-disarm", "n_clicks"),
    Input("btn-emergency-stop", "n_clicks"),

    # States
    State("input-gps-full", "value"),
    State("input-alt", "value"),
    State("checklist-predefined-classes", "value"),
    State("input-custom-classes", "value"),
    State("dropdown-flight-mode", "value"),  # חדש
    prevent_initial_call=True
)
def handle_control_buttons(goto_n, hold_n, up_n, dn_n, start_n, stop_n, apply_n,
                           set_mode_n, arm_n, disarm_n, emergency_stop_n,
                           v_gps_full, v_alt, v_checked_classes, v_custom_classes, v_flight_mode):
    which = ctx.triggered_id

    if which == "btn-goto":
        v_lat = None
        v_lon = None
        target_alt = None

        # 1. ניתוח GPS
        try:
            parts = [p.strip() for p in v_gps_full.split(',')]
            if len(parts) == 2:
                v_lat = float(parts[0])
                v_lon = float(parts[1])
        except Exception:
            print("[CMD] GPS parsing FAILED. Format must be 'Lat, Lon'.")

        # 2. קביעת גובה יעד (משתמש בגובה נוכחי אם לא הוזן)
        if v_alt is not None and str(v_alt).strip() != '':
            try:
                target_alt = float(v_alt)
            except ValueError:
                print("[CMD] Altitude input is not a valid number. Ignoring.")

        if target_alt is None:
            if global_mav_data['alt'] is not None:
                target_alt = global_mav_data['alt']
                print(f"[CMD] Go-To: Altitude not specified/invalid. Using current drone altitude: {target_alt:.1f}m")
            else:
                print("[CMD] Go-To: Altitude not specified and current altitude is UNKNOWN. Cannot proceed.")

        # 3. ביצוע הפקודה
        if v_lat is None or v_lon is None or target_alt is None:
            print("[CMD] Go-To missing valid GPS or Alt.")
        else:
            if send_command_to_tracker("GOTO_GPS", {"lat": v_lat, "lon": v_lon, "alt": target_alt}):
                global_mav_data['target_lat'] = v_lat
                global_mav_data['target_lon'] = v_lon
                global_mav_data['target_alt'] = target_alt

    elif which == "btn-hold":
        send_command_to_tracker("HOLD_HERE")
        global_mav_data['target_lat'] = None
        global_mav_data['target_lon'] = None

    elif which == "btn-up1":
        send_command_to_tracker("ALT_BUMP", {"delta": 1.0})

    elif which == "btn-dn1":
        send_command_to_tracker("ALT_BUMP", {"delta": -1.0})

    elif which == "btn-start-yolo":
        send_command_to_tracker("START_YOLO")
    elif which == "btn-stop-yolo":
        send_command_to_tracker("STOP_YOLO")

    elif which == "btn-apply-classes":

        all_classes_list = []

        if v_checked_classes:
            all_classes_list.extend(v_checked_classes)

        if v_custom_classes and v_custom_classes.strip():
            custom_parts = [c.strip() for c in v_custom_classes.split(',') if c.strip()]
            all_classes_list.extend(custom_parts)

        try:
            unique_classes = sorted(list(set(all_classes_list)), key=int)
            final_class_string = ",".join(str(c) for c in unique_classes)
        except ValueError:
            print("[CMD] Class Filter: Invalid character detected in custom input. Using only valid numbers.")
            final_class_string = ",".join(sorted(list(set([c for c in all_classes_list if c.isdigit()])), key=int))

        if final_class_string:
            send_command_to_tracker("SET_CLASSES", {"classes": final_class_string})
        else:
            send_command_to_tracker("SET_CLASSES", {"classes": ""})
            print("[CMD] Class Filter is empty. Sending an empty list.")

    # --- פקודות חדשות ---
    elif which == "btn-set-mode":
        if v_flight_mode:
            send_command_to_tracker("SET_MODE", {"mode": v_flight_mode})

    elif which == "btn-arm":
        send_command_to_tracker("ARM")

    elif which == "btn-disarm":
        send_command_to_tracker("DISARM")

    elif which == "btn-emergency-stop":
        send_command_to_tracker("EMERGENCY_STOP")
    # -------------------

    return no_update


# ------------------------------------------------

if __name__ == "__main__":
    # Start UDP listeners for data coming from the Tracker/Proxy
    threading.Thread(target=udp_listener_tel, daemon=True).start()
    threading.Thread(target=udp_listener_bbox, daemon=True).start()

    print("---" * 20)
    print(f"Flask/Dash Map started. Check http://127.0.0.1:8050/")

    app.run(debug=True, port=8050, use_reloader=False)