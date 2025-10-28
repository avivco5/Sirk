# gui_mav_to_sim_pose.py
# GUI לשליטת מיקום בסימולציה (Isaac) + שמירה על Roll/Pitch/Yaw מהרחפן האמיתי (MAVLink).
# שולח UDP JSON: {"cmd":"setpose","x":..,"y":..,"alt":..,"roll_deg":..,"pitch_deg":..,"yaw_deg":..}
# כדי לעבוד, תריץ ב-Isaac את הסקריפט המאזין (isaac_listener_pose_pid.py) על UDP 6000.

import math, time, json, socket, threading
import tkinter as tk
from tkinter import ttk, messagebox

# ==== תצורה ====
MAV_PORT = "COM14"
MAV_BAUD = 115200

SIM_IP, SIM_PORT = "127.0.0.1", 6000
SEND_RATE_HZ = 20.0                 # קצב שליחה לסימולציה

# צעדים/רגישות GUI
POS_STEP = 0.5                      # מטר לכל לחיצה קדימה/ימינה/אחורה/שמאלה (במערכת הגוף)
ALT_STEP = 0.3                      # מטר לכל לחיצה Alt+/Alt-
# מיפוי YOLO (אם מדליקים)
ENABLE_YOLO = True
K_FWD_M_PER_PX   = 0.000           # פיקסלים → תזוזת "קדימה" (X בגוף)
K_SIDE_M_PER_PX  = 0.002            # פיקסלים → תזוזת "ימינה/שמאלה" (Y בגוף)
K_ALT_M_PER_PX   = 0.000            # פיקסלים → שינוי גובה
TARGET_BOX_H_PX  = 220              # גובה יעד של הבוקס (שולט על מרחק)
DB_PIX           = 40               # deadband סביב מרכז הפריים
NUDGE_CLAMP_M    = 0.7              # הגבלת דחיפה פר פריים
# =================

# ---- MAVLink ----
from pymavlink import mavutil

# ---- YOLO אופציונלי ----
if ENABLE_YOLO:
    try:
        import cv2
        from ultralytics import YOLO
    except Exception as e:
        print("⚠️ YOLO/Camera לא נטענו, מכבה YOLO:", e)
        ENABLE_YOLO = False

# ---- UDP → Isaac ----
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
def send_setpose(x, y, alt, roll_deg, pitch_deg, yaw_deg):
    msg = {
        "cmd":"setpose",
        "x": float(x), "y": float(y), "alt": float(alt),
        "roll_deg": float(roll_deg),
        "pitch_deg": float(pitch_deg),
        "yaw_deg": float(yaw_deg)
    }
    try:
        sock.sendto(json.dumps(msg).encode("utf-8"), (SIM_IP, SIM_PORT))
    except Exception as e:
        print("[UDP] send error:", e)

# ---- מצב משותף ----
state_lock = threading.Lock()

# טלמטריית ג׳יירו מהרחפן (רדיאנים)
roll_rad = 0.0
pitch_rad = 0.0
yaw_rad = 0.0

# סט-פוינט מיקום (עולם) שנשלט ע"י GUI/YOLO
cur_x = 0.0
cur_y = 0.0
cur_alt = 2.0

# דגלים
quit_flag = False
tracking_enabled = False

# ---- חיבור MAVLink ----
master = None
def connect_mav():
    global master
    print(f"[MAV] connecting {MAV_PORT} @ {MAV_BAUD} ...")
    master = mavutil.mavlink_connection(MAV_PORT, baud=MAV_BAUD)
    master.wait_heartbeat()
    print("✅ MAVLink connected:", "sys", master.target_system, "comp", master.target_component)

def mav_telemetry_loop():
    global roll_rad, pitch_rad, yaw_rad
    last_print = 0.0
    while not quit_flag:
        try:
            msg = master.recv_match(blocking=False, timeout=0.1)
        except Exception:
            time.sleep(0.01)
            continue
        if msg is None:
            continue
        t = msg.get_type()
        if t == "ATTITUDE":
            with state_lock:
                roll_rad  = float(msg.roll)
                pitch_rad = float(msg.pitch)
                yaw_rad   = float(msg.yaw)
        now = time.time()
        if now - last_print > 1.0:
            with state_lock:
                r, p, y = math.degrees(roll_rad), math.degrees(pitch_rad), math.degrees(yaw_rad)
            print(f"[MAV] R/P/Y = ({r:+5.1f},{p:+5.1f},{y:+5.1f})°")
            last_print = now

# ---- שולח תקופתי לסימולציה ----
def tx_loop():
    period = 1.0 / SEND_RATE_HZ
    while not quit_flag:
        t0 = time.time()
        with state_lock:
            x, y, alt = cur_x, cur_y, cur_alt
            r, p, yv = math.degrees(roll_rad), math.degrees(pitch_rad), math.degrees(yaw_rad)
        send_setpose(x, y, alt, r, p, yv)
        dt = time.time() - t0
        if dt < period:
            time.sleep(period - dt)

# ---- GUI ----
root = tk.Tk()
root.title("Drone GUI → Isaac (Position by GUI/YOLO, Attitude by MAVLink)")

status = tk.StringVar(value="Disconnected")
ttk.Label(root, textvariable=status, anchor="w").grid(row=0, column=0, columnspan=6, sticky="ew", padx=8, pady=(8,4))

# תצוגת ערכים
vals = tk.StringVar(value="x=0.00 y=0.00 alt=2.00 | RPY=0/0/0")
ttk.Label(root, textvariable=vals, anchor="w").grid(row=1, column=0, columnspan=6, sticky="ew", padx=8)

def update_label():
    with state_lock:
        s = f"x={cur_x:.2f} y={cur_y:.2f} alt={cur_alt:.2f} | RPY={math.degrees(roll_rad):.1f}/{math.degrees(pitch_rad):.1f}/{math.degrees(yaw_rad):.1f}°"
    vals.set(s)
    if not quit_flag:
        root.after(150, update_label)

# סליידר גובה
def on_alt(val):
    global cur_alt
    with state_lock:
        cur_alt = float(val)
    # השילוח מתבצע ע"י הלולאת TX בקצב קבוע

alt_slider = ttk.Scale(root, from_=6.0, to=0.0, orient="vertical", command=on_alt, length=240)
alt_slider.set(cur_alt)
alt_slider.grid(row=2, column=0, rowspan=6, padx=8, pady=8, sticky="ns")
ttk.Label(root, text="גובה [m]").grid(row=8, column=0)

# פונקציית דחיפה במערכת הגוף: dx (קדימה+), dy (ימינה+)
def nudge_body(dx_b, dy_b):
    global cur_x, cur_y
    with state_lock:
        yaw = yaw_rad
        c, s = math.cos(yaw), math.sin(yaw)
        dx_w = c*dx_b - s*dy_b
        dy_w = s*dx_b + c*dy_b
        cur_x += dx_w
        cur_y += dy_w

def move(dir):
    step = POS_STEP
    if dir == "forward": nudge_body(+step, 0.0)
    elif dir == "back":  nudge_body(-step, 0.0)
    elif dir == "left":  nudge_body(0.0, -step)
    elif dir == "right": nudge_body(0.0, +step)

def do_alt(dir):
    global cur_alt
    with state_lock:
        cur_alt += ALT_STEP if dir == "up" else -ALT_STEP
        cur_alt = max(0.0, cur_alt)
    alt_slider.set(cur_alt)

# כפתורים
ttk.Button(root, text="⬆️ קדימה", command=lambda: move("forward")).grid(row=2, column=2, columnspan=2, sticky="ew", pady=4)
ttk.Button(root, text="⬅️ שמאלה", command=lambda: move("left")).grid(row=3, column=2, sticky="ew", padx=4, pady=4)
ttk.Button(root, text="➡️ ימינה",  command=lambda: move("right")).grid(row=3, column=3, sticky="ew", padx=4, pady=4)
ttk.Button(root, text="⬇️ אחורה", command=lambda: move("back")).grid(row=4, column=2, columnspan=2, sticky="ew", pady=4)

ttk.Button(root, text="⬆️ Alt+", command=lambda: do_alt("up")).grid(row=5, column=2, sticky="ew", padx=4, pady=4)
ttk.Button(root, text="⬇️ Alt-", command=lambda: do_alt("down")).grid(row=5, column=3, sticky="ew", padx=4, pady=4)

# קיצורי מקלדת
def on_key(e):
    k = e.keysym.lower()
    if   k == "w": move("forward")
    elif k == "s": move("back")
    elif k == "a": move("left")
    elif k == "d": move("right")
    elif k == "e": do_alt("up")
    elif k == "q": do_alt("down")
root.bind("<Key>", on_key)

# ---- YOLO (מוסיף רק דחיפות מיקום; לא משנה R/P/Y) ----
def start_tracking():
    global tracking_enabled
    tracking_enabled = True
    print("✅ YOLO tracking ON")

def stop_tracking():
    global tracking_enabled
    tracking_enabled = False
    print("⛔ YOLO tracking OFF")

ttk.Button(root, text="▶️ Start Tracking", command=start_tracking).grid(row=6, column=2, sticky="ew", padx=4, pady=(8,4))
ttk.Button(root, text="⏹ Stop Tracking",  command=stop_tracking).grid(row=6, column=3, sticky="ew", padx=4, pady=(8,4))

def yolo_loop():
    if not ENABLE_YOLO:
        return
    try:
        model = YOLO("yolov8n.pt")
    except Exception as e:
        print("⚠️ YOLO load failed:", e); return
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("⚠️ No camera found"); return

    print("[YOLO] ready. ESC to close preview window.")
    while not quit_flag:
        ok, frame = cap.read()
        if not ok: break
        annotated = frame
        if tracking_enabled:
            h, w = frame.shape[:2]
            results = model(frame, verbose=False)
            best = None
            for r in results:
                for b in getattr(r, "boxes", []):
                    cls = int(b.cls[0].item())
                    conf = float(b.conf[0].item()) if hasattr(b, "conf") else 0.0
                    if cls == 0 and (best is None or conf > best[0]):   # Person
                        x1,y1,x2,y2 = map(float, b.xyxy[0])
                        best = (conf, x1,y1,x2,y2)
            if best is not None:
                _, x1,y1,x2,y2 = best
                cx, cy = 0.5*(x1+x2), 0.5*(y1+y2)
                box_h  = (y2-y1)

                dx_px = cx - (w*0.5)
                dy_px = cy - (h*0.5)
                dh_px = TARGET_BOX_H_PX - box_h

                # תרגום לפקודות BODY: קדימה, צד, גובה
                dx_b = K_FWD_M_PER_PX  * dh_px
                dy_b = K_SIDE_M_PER_PX * (dx_px if abs(dx_px) > DB_PIX else 0.0)
                dalt = K_ALT_M_PER_PX  * (-dy_px if abs(dy_px) > DB_PIX else 0.0)

                # הגבלות
                dx_b = max(-NUDGE_CLAMP_M, min(NUDGE_CLAMP_M, dx_b))
                dy_b = max(-NUDGE_CLAMP_M, min(NUDGE_CLAMP_M, dy_b))
                dalt = max(-NUDGE_CLAMP_M, min(NUDGE_CLAMP_M, dalt))

                if abs(dx_b) > 1e-3 or abs(dy_b) > 1e-3 or abs(dalt) > 1e-3:
                    # עדכון מטרות GUI (ה-TX ישלח בעצמו)
                    nudge_body(dx_b, dy_b)
                    with state_lock:
                        global cur_alt
                        cur_alt = max(0.0, cur_alt + dalt)
                    alt_slider.set(cur_alt)
                    print(f"[YOLO→GUI] dB=({dx_b:+.2f},{dy_b:+.2f}) dAlt={dalt:+.2f}")

                # ציור
                annotated = frame.copy()
                cv2.rectangle(annotated, (int(x1),int(y1)), (int(x2),int(y2)), (0,255,0), 2)
                cv2.circle(annotated, (int(cx),int(cy)), 5, (0,0,255), -1)
                cv2.line(annotated, (w//2, h//2), (int(cx), int(cy)), (255,0,0), 2)
                cv2.putText(annotated, f"dx={dx_px:.0f} dy={dy_px:.0f} boxH={box_h:.0f}",
                            (20,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,(0,255,0),2)

        try:
            cv2.imshow("YOLO Tracking (position only)", annotated)
            if cv2.waitKey(1) & 0xFF == 27:
                break
        except Exception:
            pass

    try: cap.release(); cv2.destroyAllWindows()
    except: pass
    print("[YOLO] stopped")

# ---- סגירה נקייה ----
def on_close():
    global quit_flag
    if messagebox.askokcancel("Exit", "Close GUI and stop sending to sim?"):
        quit_flag = True
        root.destroy()

root.protocol("WM_DELETE_WINDOW", on_close)

# ---- הפעלה ----
def main():
    # חיבור MAVLink
    try:
        connect_mav()
        status.set("Connected to MAVLink")
    except Exception as e:
        status.set("MAVLink connection failed")
        print("❌ MAVLink connect failed:", e)

    # תהליכים
    threading.Thread(target=mav_telemetry_loop, daemon=True).start()
    threading.Thread(target=tx_loop,           daemon=True).start()
    if ENABLE_YOLO:
        threading.Thread(target=yolo_loop,     daemon=True).start()

    update_label()
    root.mainloop()

if __name__ == "__main__":
    main()
