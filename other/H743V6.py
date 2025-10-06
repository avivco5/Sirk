# mav2sim_bridge_attitude.py
# Mirror MAVLink telemetry → Isaac Sim UDP JSON (pose: XYZ + RPY)
# Isaac listener: isaac_listener_pose_pid.py on UDP 6000

import time, threading, math, socket, json
from pymavlink import mavutil

# ===== USER CONFIG =====
MAVLINK_DEVICE = "COM14"      # או: 'udpin:127.0.0.1:14550' אם חולקים פורט
MAV_BAUD       = 115200

SIM_IP   = "127.0.0.1"
SIM_PORT = 6000

SEND_RATE_HZ      = 20.0
PRINT_EVERY_SEC   = 1.0
ZERO_LOCAL_ORIGIN = True     # לאפס NED (x,y,z) לנקודת ההתחלה
# =======================

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
def send_json(obj: dict):
    sock.sendto(json.dumps(obj).encode("utf-8"), (SIM_IP, SIM_PORT))

def send_setpose(x, y, alt, roll_deg, pitch_deg, yaw_deg):
    send_json({
        "cmd":"setpose",
        "x": float(x), "y": float(y), "alt": float(alt),
        "roll_deg": float(roll_deg), "pitch_deg": float(pitch_deg), "yaw_deg": float(yaw_deg)
    })

state_lock = threading.Lock()
tele = {
    "yaw_rad": 0.0, "pitch_rad": 0.0, "roll_rad": 0.0,   # ATTITUDE (rad)
    "rel_alt_m": 0.0,                                    # GLOBAL_POSITION_INT (m)
    "has_local": False,
    "n": 0.0, "e": 0.0, "d": 0.0,
    "n0": None, "e0": None, "d0": None,
}

def mavlink_loop():
    print(f"[MAV] opening {MAVLINK_DEVICE} ...")
    if MAVLINK_DEVICE.lower().startswith("udp"):
        m = mavutil.mavlink_connection(MAVLINK_DEVICE)
    else:
        m = mavutil.mavlink_connection(MAVLINK_DEVICE, baud=MAV_BAUD)
    m.wait_heartbeat()
    print("✅ MAVLink connected:", "sys", m.target_system, "comp", m.target_component)

    last_p = 0.0
    while True:
        msg = m.recv_match(blocking=False, timeout=0.1)
        if msg is None:
            continue
        t = msg.get_type()
        with state_lock:
            if t == "ATTITUDE":
                tele["roll_rad"]  = float(msg.roll)
                tele["pitch_rad"] = float(msg.pitch)
                tele["yaw_rad"]   = float(msg.yaw)
            elif t == "GLOBAL_POSITION_INT":
                tele["rel_alt_m"] = float(msg.relative_alt)/1000.0
            elif t == "LOCAL_POSITION_NED":
                n, e, d = float(msg.x), float(msg.y), float(msg.z)
                if ZERO_LOCAL_ORIGIN and tele["n0"] is None:
                    tele["n0"], tele["e0"], tele["d0"] = n, e, d
                tele["n"], tele["e"], tele["d"]  = n, e, d
                tele["has_local"] = True

        now = time.time()
        if now - last_p >= PRINT_EVERY_SEC:
            with state_lock:
                r = math.degrees(tele["roll_rad"])
                p = math.degrees(tele["pitch_rad"])
                y = math.degrees(tele["yaw_rad"])
                alt = tele["rel_alt_m"]
                if tele["has_local"]:
                    xn = tele["n"] - (tele["n0"] if tele["n0"] is not None else 0.0)
                    ye = tele["e"] - (tele["e0"] if tele["e0"] is not None else 0.0)
                else:
                    xn = ye = 0.0
            print(f"[MAV] RPY=({r:+6.1f},{p:+6.1f},{y:+6.1f})°  alt={alt:4.1f}  XY=({xn:+6.1f},{ye:+6.1f})")
            last_p = now

def mirror_sender_loop():
    period = 1.0 / SEND_RATE_HZ
    while True:
        t0 = time.time()
        with state_lock:
            r = math.degrees(tele["roll_rad"])
            p = math.degrees(tele["pitch_rad"])
            y = math.degrees(tele["yaw_rad"])
            alt = tele["rel_alt_m"]
            if tele["has_local"]:
                xw = tele["n"] - (tele["n0"] if tele["n0"] is not None else 0.0)  # X~North
                yw = tele["e"] - (tele["e0"] if tele["e0"] is not None else 0.0)  # Y~East
            else:
                xw = 0.0; yw = 0.0
        send_setpose(xw, yw, alt, r, p, y)
        dt = time.time() - t0
        if dt < period:
            time.sleep(period - dt)

def main():
    threading.Thread(target=mavlink_loop,       daemon=True).start()
    threading.Thread(target=mirror_sender_loop, daemon=True).start()
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nBye.")

if __name__ == "__main__":
    main()
