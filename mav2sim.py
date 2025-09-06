# Control GUI -> Isaac Sim (MAVLink over UDP)
# Replaces RC/COM14 with SET_POSITION_TARGET_LOCAL_NED to udpout:127.0.0.1:14550
# No telemetry. Optional YOLO tracking maps to position setpoints.

from pymavlink import mavutil
import tkinter as tk
from tkinter import ttk
import threading, time

# (optional) comment these two lines if you don't want YOLO
import cv2
from ultralytics import YOLO

# ===================== Config =====================
MAVLINK_URL = "udpout:127.0.0.1:14550"  # matches Isaac listener "udp:0.0.0.0:14550"
SYS_ID = 1
COMP_ID = 191

# Position setpoint limits/steps (in meters)
POS_STEP = 0.5
ALT_STEP = 0.3
ALT_MIN, ALT_MAX = 0.0, 6.0

# Initial target (NED: x=north, y=east, z=down; we send altitude as -z)
cur_x, cur_y, cur_alt = 0.0, 0.0, 2.0

# Map GUI throttle [1000..2000] -> altitude [ALT_MIN..ALT_MAX]
def throttle_to_alt(val):
    return ALT_MIN + (float(val) - 1000.0) / 1000.0 * (ALT_MAX - ALT_MIN)
# ==================================================

# -------------- MAVLink connection ---------------
master = mavutil.mavlink_connection(MAVLINK_URL, source_system=SYS_ID, source_component=COMP_ID)
print("✅ Sending to:", MAVLINK_URL)

def heartbeat_loop():
    while True:
        try:
            master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, mavutil.mavlink.MAV_STATE_ACTIVE
            )
        except Exception as e:
            print("HB error:", e)
        time.sleep(1.0)

# למעלה בקובץ, פעם אחת:
_t0 = time.monotonic()

def send_setpoint(x, y, alt_up):
    z_ned = -float(alt_up)
    type_mask = ((1<<3)|(1<<4)|(1<<5)|(1<<6)|(1<<7)|(1<<8)|(1<<9)|(1<<10)|(1<<11))
    t_ms = int((time.monotonic() - _t0) * 1000) & 0xFFFFFFFF  # millis since "boot"
    master.mav.set_position_target_local_ned_send(
        t_ms,
        SYS_ID, COMP_ID,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        float(x), float(y), z_ned,
        0,0,0, 0,0,0, 0,0
    )
    try:
        master.mav.set_position_target_local_ned_send(
            int(time.time() * 1000),
            SYS_ID, COMP_ID,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            float(x), float(y), z_ned,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )
    except Exception as e:
        print("Send error:", e)

# start heartbeat
threading.Thread(target=heartbeat_loop, daemon=True).start()
time.sleep(0.2)
# --------------------------------------------------

# ---------------------- GUI -----------------------
root = tk.Tk()
root.title("Drone Control → Isaac Sim (MAVLink UDP)")

# Altitude slider
def on_throttle(val):
    global cur_alt
    cur_alt = throttle_to_alt(val)
    send_setpoint(cur_x, cur_y, cur_alt)

throttle_slider = ttk.Scale(root, from_=2000, to=1000, orient="vertical",
                            command=on_throttle, length=220)  # top=high alt
throttle_slider.set(1000)
throttle_slider.grid(row=1, column=0, rowspan=4, padx=8, pady=8)

# Movement buttons (absolute setpoint updates)
def move(direction):
    global cur_x, cur_y
    if direction == "forward":  # north
        cur_x += POS_STEP
    elif direction == "back":   # south
        cur_x -= POS_STEP
    elif direction == "left":   # west -> y decreases
        cur_y -= POS_STEP
    elif direction == "right":  # east -> y increases
        cur_y += POS_STEP
    send_setpoint(cur_x, cur_y, cur_alt)

def change_altitude(direction):
    global cur_alt
    if direction == "up":
        cur_alt = min(ALT_MAX, cur_alt + ALT_STEP)
    elif direction == "down":
        cur_alt = max(ALT_MIN, cur_alt - ALT_STEP)
    throttle_slider.set(1000 + 1000 * (cur_alt - ALT_MIN) / (ALT_MAX - ALT_MIN))
    send_setpoint(cur_x, cur_y, cur_alt)

# Yaw buttons (listener ignores yaw; kept for UI completeness)
def yaw(direction):
    print("Yaw command pressed (ignored by sim listener)")

# Arm/Disarm placeholders (no-op for sim listener)
def arm():
    print("🚀 Arm (sim listener ignores)")
def disarm():
    print("🛑 Disarm (sim listener ignores)")

# Layout
ttk.Button(root, text="⬆️ קדימה", command=lambda: move("forward")).grid(row=1, column=1, columnspan=2, sticky="ew", pady=3)
ttk.Button(root, text="⬅️ שמאלה", command=lambda: move("left")).grid(row=2, column=1, sticky="ew", padx=3, pady=3)
ttk.Button(root, text="➡️ ימינה", command=lambda: move("right")).grid(row=2, column=2, sticky="ew", padx=3, pady=3)
ttk.Button(root, text="⬇️ אחורה", command=lambda: move("back")).grid(row=3, column=1, columnspan=2, sticky="ew", pady=3)

ttk.Button(root, text="↺ Yaw Left", command=lambda: yaw("left")).grid(row=5, column=1, sticky="ew", padx=3, pady=3)
ttk.Button(root, text="↻ Yaw Right", command=lambda: yaw("right")).grid(row=5, column=2, sticky="ew", padx=3, pady=3)

ttk.Button(root, text="⬆️ Alt+", command=lambda: change_altitude("up")).grid(row=6, column=1, sticky="ew", padx=3, pady=3)
ttk.Button(root, text="⬇️ Alt-", command=lambda: change_altitude("down")).grid(row=6, column=2, sticky="ew", padx=3, pady=3)

ttk.Button(root, text="🚀 Arm", command=arm).grid(row=7, column=1, sticky="ew", padx=3, pady=3)
ttk.Button(root, text="🛑 Disarm", command=disarm).grid(row=7, column=2, sticky="ew", padx=3, pady=3)

# ---------------- YOLO Tracking (optional) ----------------
tracking_enabled = False
def start_tracking():
    global tracking_enabled
    tracking_enabled = True
    print("✅ Tracking enabled")
def stop_tracking():
    global tracking_enabled
    tracking_enabled = False
    print("⛔ Tracking disabled")

ttk.Button(root, text="▶️ Start Tracking", command=start_tracking).grid(row=8, column=1, sticky="ew", padx=3, pady=3)
ttk.Button(root, text="⏹ Stop Tracking",  command=stop_tracking).grid(row=8, column=2, sticky="ew", padx=3, pady=3)

def yolo_tracking_loop():
    global cur_x, cur_y, cur_alt, tracking_enabled
    try:
        model = YOLO("yolov8n.pt")
    except Exception as e:
        print("⚠️ YOLO load error:", e); return

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("⚠️ No camera found"); return

    Kp_xy = 0.0025     # pixels -> meters per frame (tune)
    Kp_alt = 0.005     # pixels -> meters per frame (tune), positive raises altitude
    target_box_h = 200 # desired person size (pixels)

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        annotated = frame.copy()
        h, w = frame.shape[:2]

        if tracking_enabled:
            results = model(frame, verbose=False)
            for r in results:
                for box in r.boxes:
                    if int(box.cls[0].item()) == 0:  # class 0 = person
                        x1, y1, x2, y2 = box.xyxy[0]
                        cx = float((x1 + x2) / 2)
                        cy = float((y1 + y2) / 2)
                        box_h = float(y2 - y1)

                        dx = cx - (w / 2)     # +right
                        dy = cy - (h / 2)     # +down

                        # Map image error to small incremental setpoint changes
                        cur_y += Kp_xy * dx   # right in image -> +y (east)
                        cur_x -= Kp_xy * dy   # down in image -> -x (south); up -> +x (north)
                        cur_alt += Kp_alt * (target_box_h - box_h)
                        cur_alt = max(ALT_MIN, min(ALT_MAX, cur_alt))

                        send_setpoint(cur_x, cur_y, cur_alt)
                        annotated = results[0].plot()
                        break

        cv2.imshow("YOLO Tracking", annotated)
        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

# comment next line if you don't want YOLO
threading.Thread(target=yolo_tracking_loop, daemon=True).start()

# -------------- Init & Mainloop --------------
# Send initial setpoint so sim knows our starting target
send_setpoint(cur_x, cur_y, cur_alt)
root.mainloop()
