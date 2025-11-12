# gui_mav_to_sim_pose.py
# GUI for position control in a simulator (Isaac) + real attitude from MAVLink.
# Adds ARM / FORCE-ARM / DISARM buttons to actually arm the real motors.
# Sends UDP JSON to the sim:
#   {"cmd":"setpose","x":..,"y":..,"alt":..,"roll_deg":..,"pitch_deg":..,"yaw_deg":..}
#   {"cmd":"spin","rpm":..}   # visual rotors in sim (not real ESC control)

import math, time, json, socket, threading
import tkinter as tk
from tkinter import ttk, messagebox

# ==== Configuration ====
MAV_PORT = "COM14"
MAV_BAUD = 115200

SIM_IP, SIM_PORT = "127.0.0.1", 6000
SEND_RATE_HZ = 20.0

# Visual prop spin in simulator only (not real):
SPIN_DEFAULT_RPM   = 10000.0
SPIN_MAX_RPM       = 20000.0
SPIN_HEARTBEAT_HZ  = 2.0

# GUI steps
POS_STEP = 0.5
ALT_STEP = 0.3

# YOLO mapping (optional)
ENABLE_YOLO = True
K_FWD_M_PER_PX   = 0.000
K_SIDE_M_PER_PX  = 0.002
K_ALT_M_PER_PX   = 0.000
TARGET_BOX_H_PX  = 220
DB_PIX           = 40
NUDGE_CLAMP_M    = 0.7
# =======================

# ---- MAVLink ----
from pymavlink import mavutil

# ---- Optional YOLO ----
if ENABLE_YOLO:
    try:
        import cv2
        from ultralytics import YOLO
    except Exception as e:
        print("YOLO/Camera not loaded, disabling YOLO:", e)
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

def send_spin(rpm):
    msg = {"cmd":"spin","rpm": float(rpm)}
    try:
        sock.sendto(json.dumps(msg).encode("utf-8"), (SIM_IP, SIM_PORT))
    except Exception as e:
        print("[UDP] spin send error:", e)

# ---- Shared state ----
state_lock = threading.Lock()

# Real attitude (radians) from MAVLink
roll_rad = 0.0
pitch_rad = 0.0
yaw_rad = 0.0

# Position setpoint (world) controlled by GUI/YOLO
cur_x = 0.0
cur_y = 0.0
cur_alt = 2.0

# Visual rotor state (sim only)
rotors_on = False
spin_rpm_cmd = SPIN_DEFAULT_RPM

# Flags
quit_flag = False
tracking_enabled = False

# ---- MAVLink connection and helpers ----
master = None

def connect_mav():
    global master
    print(f"[MAV] connecting {MAV_PORT} @ {MAV_BAUD} ...")
    master = mavutil.mavlink_connection(MAV_PORT, baud=MAV_BAUD)
    master.wait_heartbeat()
    print("[MAV] connected: sys", master.target_system, "comp", master.target_component)

def is_armed():
    try:
        return master.motors_armed()
    except Exception:
        return False

def set_mode(mode_name="STABILIZE", timeout=5.0):
    try:
        mapping = master.mode_mapping()
        if not mapping or mode_name not in mapping:
            print("[MODE] mapping unavailable or mode not found:", mode_name)
            return False
        target_id = mapping[mode_name]
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            target_id,
        )
        t0 = time.time()
        while time.time() - t0 < timeout:
            hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
            if hb and hasattr(hb, "custom_mode") and hb.custom_mode == target_id:
                print("[MODE] set:", mode_name)
                return True
        print("[MODE] timeout waiting for", mode_name)
        return False
    except Exception as e:
        print("[MODE] error:", e)
        return False

def arm(force=False, timeout=8.0):
    """
    Arms motors using MAV_CMD_COMPONENT_ARM_DISARM.
    force=True will try to bypass arming checks (bench only, props removed).
    """
    p2 = 21196 if force else 0
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, p2, 0, 0, 0, 0, 0
    )
    t0 = time.time()
    while time.time() - t0 < timeout:
        if is_armed():
            print("[ARM] ARMED")
            return True
        time.sleep(0.2)
    print("[ARM] failed (check PreArm checks: SD logging, GPS, sensors)")
    return False

def disarm(timeout=6.0):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    t0 = time.time()
    while time.time() - t0 < timeout:
        if not is_armed():
            print("[ARM] DISARMED")
            return True
        time.sleep(0.2)
    print("[ARM] disarm timeout")
    return False

def mav_telemetry_loop():
    global roll_rad, pitch_rad, yaw_rad
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
        elif t == "STATUSTEXT":
            # Helpful to see prearm reasons, etc.
            sev = getattr(msg, "severity", None)
            txt = getattr(msg, "text", "")
            print(f"[STATUSTEXT{' '+str(sev) if sev is not None else ''}] {txt}")

# ---- periodic TX to simulator ----
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

def spin_tx_loop():
    period = 1.0 / SPIN_HEARTBEAT_HZ
    while not quit_flag:
        with state_lock:
            on = rotors_on
            rpm = spin_rpm_cmd
        if on:
            send_spin(rpm)
        time.sleep(period)

# ---- GUI ----
root = tk.Tk()
root.title("Drone GUI → Isaac (Position by GUI/YOLO, Attitude by MAVLink)")

status = tk.StringVar(value="Disconnected")
ttk.Label(root, textvariable=status, anchor="w").grid(row=0, column=0, columnspan=8, sticky="ew", padx=8, pady=(8,4))

vals = tk.StringVar(value="x=0.00 y=0.00 alt=2.00 | RPY=0/0/0 | RPM=OFF | ARMED=NO")
ttk.Label(root, textvariable=vals, anchor="w").grid(row=1, column=0, columnspan=8, sticky="ew", padx=8)

def update_label():
    armed = is_armed() if master else False
    with state_lock:
        rpm_text = f"ON {int(spin_rpm_cmd)}" if rotors_on else "OFF"
        s = (f"x={cur_x:.2f} y={cur_y:.2f} alt={cur_alt:.2f} | "
             f"RPY={math.degrees(roll_rad):.1f}/{math.degrees(pitch_rad):.1f}/{math.degrees(yaw_rad):.1f}° | "
             f"RPM={rpm_text} | ARMED={'YES' if armed else 'NO'}")
    vals.set(s)
    if not quit_flag:
        root.after(150, update_label)

# Alt slider
def on_alt(val):
    global cur_alt
    with state_lock:
        cur_alt = float(val)

alt_slider = ttk.Scale(root, from_=6.0, to=0.0, orient="vertical", command=on_alt, length=240)
alt_slider.set(cur_alt)
alt_slider.grid(row=2, column=0, rowspan=7, padx=8, pady=8, sticky="ns")
ttk.Label(root, text="Altitude [m]").grid(row=9, column=0)

# Body-frame nudges
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

# Visual RPM controls (sim only)
def on_rpm_change(val):
    global spin_rpm_cmd
    with state_lock:
        spin_rpm_cmd = float(val)
        on = rotors_on
    if on:
        send_spin(spin_rpm_cmd)

def spin_on():
    global rotors_on
    with state_lock:
        rotors_on = True
        rpm = spin_rpm_cmd
    send_spin(rpm)
    print("[SPIN] ON ->", int(rpm), "RPM (sim only)")

def spin_off():
    global rotors_on
    with state_lock:
        rotors_on = False
    send_spin(0.0)
    print("[SPIN] OFF (sim only)")

# Movement buttons
ttk.Button(root, text="Forward", command=lambda: move("forward")).grid(row=2, column=2, columnspan=2, sticky="ew", pady=4)
ttk.Button(root, text="Left",    command=lambda: move("left")).grid(row=3, column=2, sticky="ew", padx=4, pady=4)
ttk.Button(root, text="Right",   command=lambda: move("right")).grid(row=3, column=3, sticky="ew", padx=4, pady=4)
ttk.Button(root, text="Back",    command=lambda: move("back")).grid(row=4, column=2, columnspan=2, sticky="ew", pady=4)

ttk.Button(root, text="Alt+", command=lambda: do_alt("up")).grid(row=5, column=2, sticky="ew", padx=4, pady=4)
ttk.Button(root, text="Alt-", command=lambda: do_alt("down")).grid(row=5, column=3, sticky="ew", padx=4, pady=4)

# Sim rotor + RPM slider
ttk.Button(root, text="Rotors ON (sim)",  command=spin_on ).grid(row=6, column=4, sticky="ew", padx=4, pady=(8,4))
ttk.Button(root, text="Rotors OFF (sim)", command=spin_off).grid(row=6, column=5, sticky="ew", padx=4, pady=(8,4))
ttk.Label(root, text="RPM (sim)").grid(row=7, column=4, sticky="w", padx=4)
rpm_slider = ttk.Scale(root, from_=0.0, to=SPIN_MAX_RPM, orient="horizontal",
                       command=on_rpm_change, length=220)
rpm_slider.set(SPIN_DEFAULT_RPM)
rpm_slider.grid(row=7, column=5, sticky="ew", padx=4, pady=(4,8))

# === ARM / DISARM controls (real) ===
def do_set_mode():
    ok = set_mode("STABILIZE")
    status.set("Mode STABILIZE set" if ok else "Mode set failed")

def do_arm():
    # Normal arming (requires all prearm checks to pass)
    ok = arm(force=False)
    status.set("ARMED" if ok else "Arm failed (see STATUSTEXT)")

def do_force_arm():
    # Force-arming: bench only, props removed. Bypasses checks like SD/GPS.
    ok = arm(force=True)
    status.set("FORCE-ARMED" if ok else "Force-arm failed (see STATUSTEXT)")

def do_disarm():
    ok = disarm()
    status.set("DISARMED" if ok else "Disarm failed")

ttk.Button(root, text="Set Mode: STABILIZE", command=do_set_mode).grid(row=2, column=4, sticky="ew", padx=4, pady=4)
ttk.Button(root, text="ARM",                 command=do_arm).grid(row=3, column=4, sticky="ew", padx=4, pady=4)
ttk.Button(root, text="Force-ARM (bench)",   command=do_force_arm).grid(row=3, column=5, sticky="ew", padx=4, pady=4)
ttk.Button(root, text="DISARM",              command=do_disarm).grid(row=4, column=4, sticky="ew", padx=4, pady=4)

# Keyboard shortcuts
def on_key(e):
    k = e.keysym.lower()
    if   k == "w": move("forward")
    elif k == "s": move("back")
    elif k == "a": move("left")
    elif k == "d": move("right")
    elif k == "e": do_alt("up")
    elif k == "q": do_alt("down")
    elif k == "r": do_arm()
    elif k == "f": do_disarm()
root.bind("<Key>", on_key)

# YOLO start/stop
def start_tracking():
    global tracking_enabled
    tracking_enabled = True
    print("YOLO tracking ON")

def stop_tracking():
    global tracking_enabled
    tracking_enabled = False
    print("YOLO tracking OFF")

ttk.Button(root, text="Start Tracking", command=start_tracking).grid(row=6, column=2, sticky="ew", padx=4, pady=(8,4))
ttk.Button(root, text="Stop Tracking",  command=stop_tracking).grid(row=6, column=3, sticky="ew", padx=4, pady=(8,4))

def yolo_loop():
    if not ENABLE_YOLO:
        return
    try:
        model = YOLO("../yolov8n.pt")
    except Exception as e:
        print("YOLO load failed:", e); return
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("No camera found"); return

    print("[YOLO] ready. Press ESC to close window.")
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

                dx_b = K_FWD_M_PER_PX  * dh_px
                dy_b = K_SIDE_M_PER_PX * (dx_px if abs(dx_px) > DB_PIX else 0.0)
                dalt = K_ALT_M_PER_PX  * (-dy_px if abs(dy_px) > DB_PIX else 0.0)

                dx_b = max(-NUDGE_CLAMP_M, min(NUDGE_CLAMP_M, dx_b))
                dy_b = max(-NUDGE_CLAMP_M, min(NUDGE_CLAMP_M, dy_b))
                dalt = max(-NUDGE_CLAMP_M, min(NUDGE_CLAMP_M, dalt))

                if abs(dx_b) > 1e-3 or abs(dy_b) > 1e-3 or abs(dalt) > 1e-3:
                    nudge_body(dx_b, dy_b)
                    with state_lock:
                        global cur_alt
                        cur_alt = max(0.0, cur_alt + dalt)
                    alt_slider.set(cur_alt)
                    print(f"[YOLO→GUI] dB=({dx_b:+.2f},{dy_b:+.2f}) dAlt={dalt:+.2f}")

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

# ---- clean shutdown ----
def on_close():
    global quit_flag
    try:
        # Try to disarm on exit for safety
        if master:
            disarm()
    except Exception:
        pass
    if messagebox.askokcancel("Exit", "Close GUI and stop sending to sim?"):
        quit_flag = True
        root.destroy()

root.protocol("WM_DELETE_WINDOW", on_close)

# ---- main ----
def main():
    # MAVLink
    try:
        connect_mav()
        status.set("Connected to MAVLink")
    except Exception as e:
        status.set("MAVLink connection failed")
        print("MAVLink connect failed:", e)

    # Threads
    threading.Thread(target=mav_telemetry_loop, daemon=True).start()
    threading.Thread(target=tx_loop,           daemon=True).start()
    threading.Thread(target=spin_tx_loop,      daemon=True).start()
    if ENABLE_YOLO:
        threading.Thread(target=yolo_loop,     daemon=True).start()

    update_label()
    root.mainloop()

if __name__ == "__main__":
    main()
