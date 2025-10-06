from pymavlink import mavutil
import tkinter as tk
from tkinter import ttk
import threading, time
import cv2
from ultralytics import YOLO

# --- MAVLink Connection ---
master = mavutil.mavlink_connection('COM14', baud=115200)
master.wait_heartbeat()
print("✅ Connected:", master.target_system, master.target_component)

# RC ערכי בסיס
rc_roll = 1500
rc_pitch = 1500
rc_yaw = 1500
rc_throttle = 1000

tracking_enabled = False

def send_rc():
    """שולח ערכי RC לבקר"""
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        rc_roll, rc_pitch, rc_throttle, rc_yaw,
        0, 0, 0, 0
    )

def arm():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    print("🚀 Armed")

def disarm():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print("🛑 Disarmed")

# --- GUI ---
root = tk.Tk()
root.title("Drone Control GUI")

# סליידר גובה
def on_throttle(val):
    global rc_throttle
    rc_throttle = int(float(val))
    send_rc()

throttle_slider = ttk.Scale(root, from_=1000, to=2000, orient="vertical",
                            command=on_throttle, length=200)
throttle_slider.set(1000)
throttle_slider.grid(row=1, column=0, rowspan=3)

# --- פונקציות שליטה ---
def move(direction):
    global rc_roll, rc_pitch
    if direction == "forward":
        rc_pitch = 1600
    elif direction == "back":
        rc_pitch = 1400
    elif direction == "left":
        rc_roll = 1400
    elif direction == "right":
        rc_roll = 1600
    send_rc()
    root.after(300, reset_sticks)

def yaw(direction):
    global rc_yaw
    if direction == "left":
        rc_yaw = 1400
    elif direction == "right":
        rc_yaw = 1600
    send_rc()
    root.after(300, reset_sticks)

def change_altitude(direction):
    global rc_throttle
    if direction == "up":
        rc_throttle = min(2000, rc_throttle + 50)
    elif direction == "down":
        rc_throttle = max(1000, rc_throttle - 50)
    send_rc()

def reset_sticks():
    global rc_roll, rc_pitch, rc_yaw
    rc_roll, rc_pitch, rc_yaw = 1500, 1500, 1500
    send_rc()

# --- כפתורים ידניים ---
ttk.Button(root, text="⬆️ קדימה", command=lambda: move("forward")).grid(row=1, column=1)
ttk.Button(root, text="⬅️ שמאלה", command=lambda: move("left")).grid(row=2, column=0)
ttk.Button(root, text="➡️ ימינה", command=lambda: move("right")).grid(row=2, column=2)
ttk.Button(root, text="⬇️ אחורה", command=lambda: move("back")).grid(row=3, column=1)

ttk.Button(root, text="↺ Yaw Left", command=lambda: yaw("left")).grid(row=5, column=0, sticky="ew")
ttk.Button(root, text="↻ Yaw Right", command=lambda: yaw("right")).grid(row=5, column=2, sticky="ew")

ttk.Button(root, text="⬆️ Alt+", command=lambda: change_altitude("up")).grid(row=6, column=0, sticky="ew")
ttk.Button(root, text="⬇️ Alt-", command=lambda: change_altitude("down")).grid(row=6, column=2, sticky="ew")

ttk.Button(root, text="🚀 Arm", command=arm).grid(row=7, column=0, sticky="ew")
ttk.Button(root, text="🛑 Disarm", command=disarm).grid(row=7, column=2, sticky="ew")

# --- כפתורי מעקב ---
def start_tracking():
    global tracking_enabled
    tracking_enabled = True
    print("✅ Tracking enabled")

def stop_tracking():
    global tracking_enabled, rc_roll, rc_pitch, rc_yaw
    tracking_enabled = False
    rc_roll, rc_pitch, rc_yaw = 1500, 1500, 1500
    send_rc()
    print("⛔ Tracking disabled")

ttk.Button(root, text="▶️ Start Tracking", command=start_tracking).grid(row=8, column=0, sticky="ew")
ttk.Button(root, text="⏹ Stop Tracking", command=stop_tracking).grid(row=8, column=2, sticky="ew")

# --- Thread לקריאת טלמטריה ---
def telemetry_loop():
    while True:
        msg = master.recv_match(blocking=True)
        if not msg:
            continue
        t = msg.get_type()

        if t == "ATTITUDE":
            print(f"🌀 Attitude | Roll={msg.roll:.2f}, Pitch={msg.pitch:.2f}, Yaw={msg.yaw:.2f}")
        elif t == "GLOBAL_POSITION_INT":
            rel_alt = msg.relative_alt / 1000.0
            abs_alt = msg.alt / 1000.0
            print(f"🌍 Position | Lat={msg.lat/1e7:.6f}, Lon={msg.lon/1e7:.6f}, Alt={abs_alt:.1f} m, RelAlt={rel_alt:.1f} m")
        elif t == "BATTERY_STATUS":
            voltage = msg.voltages[0] / 1000.0
            current = msg.current_battery / 100.0 if msg.current_battery != -1 else 0
            print(f"🔋 Battery | Voltage={voltage:.2f} V, Current={current:.1f} A")
        elif t == "HEARTBEAT":
            mode = mavutil.mode_string_v10(msg)
            armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            print(f"❤️ Heartbeat | Mode={mode}, Armed={armed}")

        time.sleep(0.001)

threading.Thread(target=telemetry_loop, daemon=True).start()

# --- Thread ל-YOLO Tracking ---
def yolo_tracking_loop():
    global rc_roll, rc_pitch, rc_yaw, rc_throttle, tracking_enabled
    model = YOLO("yolov8n.pt")
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("⚠️ No camera found")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        h, w, _ = frame.shape
        annotated = frame.copy()

        if tracking_enabled:
            results = model(frame, verbose=False)
            for r in results:
                for box in r.boxes:
                    if int(box.cls[0].item()) == 0:  # Person
                        x1, y1, x2, y2 = box.xyxy[0]
                        cx = (x1 + x2) / 2
                        cy = (y1 + y2) / 2
                        box_h = (y2 - y1)

                        dx = cx - (w / 2)
                        dy = cy - (h / 2)

                        # מקדמי בקרה
                        Kp_yaw = 0.05
                        Kp_pitch = 0.05
                        Kp_throttle = 0.05

                        rc_yaw = int(1500 + Kp_yaw * dx)
                        rc_pitch = int(1500 - Kp_pitch * dy)
                        rc_throttle = int(1500 + Kp_throttle * (200 - box_h))
                        send_rc()

                        # ציור פעולות
                        actions = []
                        if dx > 50:
                            actions.append("➡️ Turn Right")
                            cv2.arrowedLine(annotated, (w // 2, h // 2), (w // 2 + 80, h // 2), (0, 0, 255), 3)
                        elif dx < -50:
                            actions.append("⬅️ Turn Left")
                            cv2.arrowedLine(annotated, (w // 2, h // 2), (w // 2 - 80, h // 2), (0, 0, 255), 3)

                        if dy > 50:
                            actions.append("⬇️ Go Down")
                            cv2.arrowedLine(annotated, (w // 2, h // 2), (w // 2, h // 2 + 80), (255, 0, 0), 3)
                        elif dy < -50:
                            actions.append("⬆️ Go Up")
                            cv2.arrowedLine(annotated, (w // 2, h // 2), (w // 2, h // 2 - 80), (255, 0, 0), 3)

                        if box_h < 150:
                            actions.append("↗️ Forward")
                            cv2.putText(annotated, "FORWARD", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        elif box_h > 250:
                            actions.append("↙️ Backward")
                            cv2.putText(annotated, "BACKWARD", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                        print(f"👤 Tracking Actions: {actions}")
                        annotated = results[0].plot()
                        break

        cv2.imshow("YOLO Tracking", annotated)
        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

threading.Thread(target=yolo_tracking_loop, daemon=True).start()

root.mainloop()
