# gui_mav_to_sim_pose.py
# GUI to command position in Isaac Sim while keeping real drone attitude from MAVLink.
# Sends UDP JSON: {"cmd":"setpose","x":..,"y":..,"alt":..,"roll_deg":..,"pitch_deg":..,"yaw_deg":..}
# For sim side, run the listener (isaac_listener_pose_pid.py) on UDP 6000.
# Also supports sending MAVLink RC overrides to a real drone (ARM/DISARM + RC OUT with base throttle).
# SAFETY: Test in SITL first. For real hardware, remove props and keep a kill switch handy.

import math, time, json, socket, threading, os, argparse
from ipaddress import ip_network
import tkinter as tk
from tkinter import ttk, messagebox

# ==== Basic config (can be overridden by CLI/ENV) ====
MAV_PORT = "COM14"
MAV_BAUD = 115200

DEFAULT_SIM_IP   = os.getenv("SIM_IP", "127.0.0.1")  # IP or hostname
DEFAULT_SIM_PORT = int(os.getenv("SIM_PORT", "6000"))
DEFAULT_BCAST    = os.getenv("SIM_BROADCAST", "0") in ("1", "true", "True", "yes", "YES")
DEFAULT_NET_CIDR = os.getenv("SIM_NET")              # e.g. "192.168.1.0/24"
DEFAULT_BIND_IP  = os.getenv("SIM_BIND")             # force source interface if needed

SEND_RATE_HZ = 20.0  # UDP rate to sim

# GUI step sizes (body frame nudges)
POS_STEP = 0.5   # m per WASD press (X forward, Y right in body frame)
ALT_STEP = 0.3   # m per E/Q press

# YOLO mapping (optional)
ENABLE_YOLO = True
K_FWD_M_PER_PX   = 0.000   # pixels → forward/back nudge (X in body)
K_SIDE_M_PER_PX  = 0.002   # pixels → right/left nudge (Y in body)
K_ALT_M_PER_PX   = 0.000   # pixels → altitude change
TARGET_BOX_H_PX  = 220     # target person box height (controls distance)
DB_PIX           = 40      # deadband around image center
NUDGE_CLAMP_M    = 0.7     # max nudge per frame

# ---- MAVLink ----
from pymavlink import mavutil

# ---- Optional YOLO ----
if ENABLE_YOLO:
    try:
        import cv2
        from ultralytics import YOLO
    except Exception as e:
        print("⚠️ YOLO/Camera load failed, disabling YOLO:", e)
        ENABLE_YOLO = False

# ========= CLI / smart UDP target =========
def _get_local_ip_guess() -> str:
    """Try to infer local source IP by opening a dummy UDP 'connection'."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))  # not actually sending
        return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"
    finally:
        try: s.close()
        except: pass

def _resolve_target(target: str, port: int, use_broadcast: bool, net_cidr: str|None):
    """Return (dest_ip, dest_port, is_broadcast)."""
    if use_broadcast:
        # (1) Prefer explicit CIDR
        if net_cidr:
            try:
                net = ip_network(net_cidr, strict=False)
                bcast_ip = str(net.broadcast_address)
                return bcast_ip, port, True
            except Exception as e:
                print("⚠️ SIM_NET invalid, falling back:", e)
        # (2) If target already looks like a .255 broadcast
        try:
            parts = target.split(".")
            if len(parts) == 4 and parts[-1] == "255":
                return target, port, True
        except Exception:
            pass
        # (3) Automatic /24 based on local IP
        local_ip = _get_local_ip_guess()
        try:
            prefix = local_ip.rsplit(".", 1)[0]
            return f"{prefix}.255", port, True
        except Exception:
            # (4) Full broadcast fallback
            return "255.255.255.255", port, True
    else:
        # Unicast hostname/IP
        try:
            dest_ip = socket.gethostbyname(target)
        except Exception:
            dest_ip = target
        return dest_ip, port, False

def _build_socket(bind_ip: str|None, is_broadcast: bool):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if is_broadcast:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    if bind_ip:
        s.bind((bind_ip, 0))  # port 0 = auto
    return s

parser = argparse.ArgumentParser(description="GUI → Isaac pose sender (UDP, hostname/broadcast ready) + MAVLink RC OUT")
parser.add_argument("-t", "--target", default=DEFAULT_SIM_IP, help="UDP target: IP or hostname (default: %(default)s)")
parser.add_argument("-p", "--port",   default=DEFAULT_SIM_PORT, type=int, help="Sim UDP port (default: %(default)s)")
parser.add_argument("-b", "--broadcast", action="store_true", default=DEFAULT_BCAST, help="Send via broadcast")
parser.add_argument("--net",  default=DEFAULT_NET_CIDR, help="CIDR like 192.168.1.0/24 to compute broadcast")
parser.add_argument("--bind", default=DEFAULT_BIND_IP, help="Bind to a local IP (e.g., 192.168.1.197)")
_args = parser.parse_args()
DEST_IP, DEST_PORT, IS_BCAST = _resolve_target(_args.target, _args.port, _args.broadcast, _args.net)
sock = _build_socket(_args.bind, IS_BCAST)
print(f"[UDP] mode={'BROADCAST' if IS_BCAST else 'UNICAST'} → {DEST_IP}:{DEST_PORT}"
      + (f"  (bind={_args.bind})" if _args.bind else ""))

# ---- UDP → Isaac ----
def send_setpose(x, y, alt, roll_deg, pitch_deg, yaw_deg):
    msg = {
        "cmd":"setpose",
        "x": float(x), "y": float(y), "alt": float(alt),
        "roll_deg": float(roll_deg),
        "pitch_deg": float(pitch_deg),
        "yaw_deg": float(yaw_deg)
    }
    try:
        sock.sendto(json.dumps(msg).encode("utf-8"), (DEST_IP, DEST_PORT))
    except Exception as e:
        print("[UDP] send error:", e)

# ---- Shared state ----
state_lock = threading.Lock()

# Attitude (radians) from MAVLink
roll_rad  = 0.0
pitch_rad = 0.0
yaw_rad   = 0.0

# GUI/YOLO-controlled world setpoints
cur_x   = 0.0
cur_y   = 0.0
cur_alt = 2.0

# Flags
quit_flag = False
tracking_enabled = False

# ---- MAVLink IN (telemetry) ----
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

# ---- Periodic TX to sim ----
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

# ===================== MAV OUT (RC override) =====================
# RC loop at 50Hz; maps YOLO body nudges to roll/pitch/throttle PWM.
RC_RATE_HZ = 50
RC_MIN, RC_CENTER, RC_MAX = 1000, 1500, 2000

# gains: meters → PWM offset
K_ROLL_PWM_PER_M   = 400   # dy_b → roll
K_PITCH_PWM_PER_M  = 400   # dx_b → pitch
K_THR_PWM_PER_M    = 300   # dalt → throttle

# axis signs (flip if needed)
S_ROLL  = +1   # + right
S_PITCH = -1   # + forward (often inverted)
S_YAW   = +1

rc_out_enabled = False
armed = False
THROTTLE_BASE = 1000  # start at idle
rc_cmd = {"roll": RC_CENTER, "pitch": RC_CENTER, "yaw": RC_CENTER, "thr": RC_MIN}

# last YOLO nudges (body frame)
last_dx_b = 0.0
last_dy_b = 0.0
last_dalt = 0.0

def mav_set_mode(mode_name="GUIDED"):
    try:
        modes = master.mode_mapping()
        if mode_name in modes:
            master.set_mode(modes[mode_name])
            print(f"[MAV] set mode {mode_name}")
            return True
    except Exception as e:
        print("[MAV] mode mapping/set failed:", e)
    # Fallback (ArduPilot)
    try:
        mode_id = {"STABILIZE":0,"ACRO":1,"ALT_HOLD":2,"AUTO":3,"GUIDED":4,"LOITER":5}.get(mode_name, 4)
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            1, mode_id, 0, 0, 0, 0, 0
        )
        print(f"[MAV] DO_SET_MODE {mode_name} (fallback)")
        return True
    except Exception as e:
        print("[MAV] DO_SET_MODE failed:", e)
        return False

def mav_arm():
    global armed
    try:
        mav_set_mode("GUIDED")
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        armed = True
        print("✅ ARMED")
    except Exception as e:
        print("❌ ARM failed:", e)

def mav_disarm():
    global armed
    try:
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        armed = False
        print("⛔ DISARMED")
    except Exception as e:
        print("❌ DISARM failed:", e)

def send_rc_override(roll, pitch, thr, yaw):
    def clip(v): return max(RC_MIN, min(RC_MAX, int(v)))
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        clip(roll), clip(pitch), clip(thr), clip(yaw),
        0, 0, 0, 0
    )

def clear_rc_override():
    try:
        master.mav.rc_channels_override_send(
            master.target_system, master.target_component,
            0,0,0,0,0,0,0,0
        )
        print("[MAV] RC override cleared")
    except Exception as e:
        print("[MAV] clear override failed:", e)

def rc_out_loop():
    period = 1.0 / RC_RATE_HZ
    while not quit_flag:
        t0 = time.time()
        if rc_out_enabled and armed and (master is not None):
            roll  = RC_CENTER
            pitch = RC_CENTER
            yaw   = RC_CENTER
            thr   = THROTTLE_BASE

            try:
                r_off = S_ROLL  * (last_dy_b  * K_ROLL_PWM_PER_M)
                p_off = S_PITCH * (last_dx_b  * K_PITCH_PWM_PER_M)
                t_off =           (last_dalt * K_THR_PWM_PER_M)
            except Exception:
                r_off = p_off = t_off = 0

            roll  += r_off
            pitch += p_off
            thr   += t_off

            # gentle clamps
            def clamp(v, lo, hi): return max(lo, min(hi, v))
            roll  = clamp(roll,  RC_CENTER-400, RC_CENTER+400)
            pitch = clamp(pitch, RC_CENTER-400, RC_CENTER+400)
            thr   = clamp(thr,   RC_MIN,        min(1700, RC_MAX))  # soft safety ceiling

            send_rc_override(roll, pitch, thr, yaw)
            rc_cmd.update({"roll": int(roll), "pitch": int(pitch), "yaw": int(yaw), "thr": int(thr)})
        time.sleep(max(0.0, period - (time.time()-t0)))

# ---- GUI ----
root = tk.Tk()
root.title("Drone GUI → Isaac (Position by GUI/YOLO, Attitude by MAVLink)")

status = tk.StringVar(value="Disconnected")
ttk.Label(root, textvariable=status, anchor="w").grid(row=0, column=0, columnspan=6, sticky="ew", padx=8, pady=(8,4))

vals = tk.StringVar(value="x=0.00 y=0.00 alt=2.00 | RPY=0/0/0")
ttk.Label(root, textvariable=vals, anchor="w").grid(row=1, column=0, columnspan=6, sticky="ew", padx=8)

def update_label():
    with state_lock:
        s = f"x={cur_x:.2f} y={cur_y:.2f} alt={cur_alt:.2f} | RPY={math.degrees(roll_rad):.1f}/{math.degrees(pitch_rad):.1f}/{math.degrees(yaw_rad):.1f}°"
    s += f" | RC roll/pitch/yaw/thr = {rc_cmd['roll']}/{rc_cmd['pitch']}/{rc_cmd['yaw']}/{rc_cmd['thr']}"
    vals.set(s)
    if not quit_flag:
        root.after(150, update_label)

# Altitude slider
def on_alt(val):
    global cur_alt
    with state_lock:
        cur_alt = float(val)

alt_slider = ttk.Scale(root, from_=6.0, to=0.0, orient="vertical", command=on_alt, length=240)
alt_slider.set(cur_alt)
alt_slider.grid(row=2, column=0, rowspan=6, padx=8, pady=8, sticky="ns")
ttk.Label(root, text="Altitude [m]").grid(row=8, column=0)

# Body-frame nudge: dx (forward+), dy (right+)
def nudge_body(dx_b, dy_b):
    global cur_x, cur_y
    with state_lock:
        yaw = yaw_rad
        c, s = math.cos(yaw), math.sin(yaw)
        dx_w = c*dx_b - s*dy_b
        dy_w = s*dx_b + c*dy_b
        cur_x += dx_w
        cur_y += dy_w

def move(direction):
    step = POS_STEP
    if direction == "forward": nudge_body(+step, 0.0)
    elif direction == "back":  nudge_body(-step, 0.0)
    elif direction == "left":  nudge_body(0.0, -step)
    elif direction == "right": nudge_body(0.0, +step)

def do_alt(direction):
    global cur_alt
    with state_lock:
        cur_alt += ALT_STEP if direction == "up" else -ALT_STEP
        cur_alt = max(0.0, cur_alt)
    alt_slider.set(cur_alt)

# Motion buttons
ttk.Button(root, text="⬆️ קדימה", command=lambda: move("forward")).grid(row=2, column=2, columnspan=2, sticky="ew", pady=4)
ttk.Button(root, text="⬅️ שמאלה", command=lambda: move("left")).grid(row=3, column=2, sticky="ew", padx=4, pady=4)
ttk.Button(root, text="➡️ ימינה",  command=lambda: move("right")).grid(row=3, column=3, sticky="ew", padx=4, pady=4)
ttk.Button(root, text="⬇️ אחורה", command=lambda: move("back")).grid(row=4, column=2, columnspan=2, sticky="ew", pady=4)
ttk.Button(root, text="⬆️ Alt+", command=lambda: do_alt("up")).grid(row=5, column=2, sticky="ew", padx=4, pady=4)
ttk.Button(root, text="⬇️ Alt-", command=lambda: do_alt("down")).grid(row=5, column=3, sticky="ew", padx=4, pady=4)

# Keyboard
def on_key(e):
    k = e.keysym.lower()
    if   k == "w": move("forward")
    elif k == "s": move("back")
    elif k == "a": move("left")
    elif k == "d": move("right")
    elif k == "e": do_alt("up")
    elif k == "q": do_alt("down")
root.bind("<Key>", on_key)

# ---- YOLO (position nudges only; attitude stays from MAVLink) ----
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
    global last_dx_b, last_dy_b, last_dalt
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

                # body-frame commands
                dx_b = K_FWD_M_PER_PX  * dh_px
                dy_b = K_SIDE_M_PER_PX * (dx_px if abs(dx_px) > DB_PIX else 0.0)
                dalt = K_ALT_M_PER_PX  * (-dy_px if abs(dy_px) > DB_PIX else 0.0)

                # clamps
                dx_b = max(-NUDGE_CLAMP_M, min(NUDGE_CLAMP_M, dx_b))
                dy_b = max(-NUDGE_CLAMP_M, min(NUDGE_CLAMP_M, dy_b))
                dalt = max(-NUDGE_CLAMP_M, min(NUDGE_CLAMP_M, dalt))

                # apply nudges to GUI setpoints (sim side)
                if abs(dx_b) > 1e-3 or abs(dy_b) > 1e-3 or abs(dalt) > 1e-3:
                    nudge_body(dx_b, dy_b)
                    with state_lock:
                        global cur_alt
                        cur_alt = max(0.0, cur_alt + dalt)
                    alt_slider.set(cur_alt)
                    # store for RC OUT mapping
                    last_dx_b, last_dy_b, last_dalt = dx_b, dy_b, dalt
                    print(f"[YOLO→GUI] dB=({dx_b:+.2f},{dy_b:+.2f}) dAlt={dalt:+.2f}")

                # draw
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

# ---- ARM/DISARM + RC OUT controls ----
def do_arm():
    mav_arm()

def do_disarm():
    mav_disarm()
    try: clear_rc_override()
    except: pass

def toggle_rc():
    global rc_out_enabled
    rc_out_enabled = not rc_out_enabled
    print("RC OUT:", "ON" if rc_out_enabled else "OFF")
    if not rc_out_enabled:
        try: clear_rc_override()
        except: pass

def on_thr_base(val):
    global THROTTLE_BASE
    try:
        THROTTLE_BASE = int(float(val))
    except Exception:
        pass

ttk.Button(root, text="🟢 ARM",    command=do_arm).grid(row=7, column=2, sticky="ew", padx=4, pady=(8,4))
ttk.Button(root, text="🔴 DISARM", command=do_disarm).grid(row=7, column=3, sticky="ew", padx=4, pady=(8,4))
ttk.Button(root, text="RC OUT ON/OFF", command=toggle_rc).grid(row=8, column=2, columnspan=2, sticky="ew", padx=4, pady=(4,8))

thr_slider = ttk.Scale(root, from_=1000, to=1700, orient="horizontal", command=on_thr_base, length=240)
thr_slider.set(THROTTLE_BASE)
thr_slider.grid(row=9, column=2, columnspan=2, padx=4, pady=(0,8), sticky="ew")
ttk.Label(root, text="THROTTLE_BASE [PWM]").grid(row=10, column=2, columnspan=2)

# ---- Clean shutdown ----
def on_close():
    global quit_flag
    if messagebox.askokcancel("Exit", "Close GUI and stop sending to sim & RC?"):
        quit_flag = True
        try: clear_rc_override()
        except: pass
        try: mav_disarm()
        except: pass
        root.destroy()

root.protocol("WM_DELETE_WINDOW", on_close)

# ---- Main ----
def main():
    # MAVLink connect
    try:
        connect_mav()
        status.set("Connected to MAVLink")
    except Exception as e:
        status.set("MAVLink connection failed")
        print("❌ MAVLink connect failed:", e)

    # Threads
    threading.Thread(target=mav_telemetry_loop, daemon=True).start()
    threading.Thread(target=tx_loop,           daemon=True).start()
    threading.Thread(target=rc_out_loop,       daemon=True).start()
    if ENABLE_YOLO:
        threading.Thread(target=yolo_loop,     daemon=True).start()

    update_label()
    root.mainloop()

if __name__ == "__main__":
    main()
