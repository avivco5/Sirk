# mav2sim_bridge.py
# Mirror real drone telemetry (MAVLink) into Isaac Sim via UDP JSON (no GUI, no YOLO).
# Isaac listener (isaac_listener_yaw.py) must be running on UDP 0.0.0.0:6000.

import time, threading, math, socket, json

# ======= USER CONFIG =======
MAVLINK_DEVICE = "COM14"      # או 'udpin:127.0.0.1:14550' כשמריצים GUI במקביל
MAV_BAUD       = 115200

SIM_IP   = "127.0.0.1"
SIM_PORT = 6000

SEND_RATE_HZ      = 20.0      # קצב שליחת setpos (Hz)
PRINT_EVERY_SEC   = 1.0
ZERO_LOCAL_ORIGIN = True      # לאפס את NED לנקודת התחלה
# ===========================

# --- deps ---
from pymavlink import mavutil

# --- UDP sender ---
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
def send_json(obj: dict):
    data = json.dumps(obj).encode("utf-8")
    sock.sendto(data, (SIM_IP, SIM_PORT))

def send_setpos(x, y, alt, yaw_deg):
    send_json({"cmd": "setpos", "x": float(x), "y": float(y), "alt": float(alt), "yaw_deg": float(yaw_deg)})

# --- shared state ---
state_lock = threading.Lock()
tele = {
    "yaw_rad": 0.0,         # ATTITUDE.yaw (rad)
    "rel_alt_m": 0.0,       # GLOBAL_POSITION_INT.relative_alt (m)
    "has_local": False,     # LOCAL_POSITION_NED available
    "n": 0.0, "e": 0.0, "d": 0.0,
    "n0": None, "e0": None, "d0": None,
    "last_recv_ts": 0.0,
}

# --- MAVLink thread ---
def mavlink_loop():
    print(f"[MAV] opening {MAVLINK_DEVICE} @ {MAV_BAUD} ...")
    if MAVLINK_DEVICE.lower().startswith("udp"):
        m = mavutil.mavlink_connection(MAVLINK_DEVICE)
    else:
        m = mavutil.mavlink_connection(MAVLINK_DEVICE, baud=MAV_BAUD)
    m.wait_heartbeat()
    print("✅ MAVLink connected:", "sys", m.target_system, "comp", m.target_component)

    last_print = 0.0
    while True:
        msg = m.recv_match(blocking=False, timeout=0.1)
        now = time.time()
        if msg is None:
            continue

        t = msg.get_type()
        with state_lock:
            tele["last_recv_ts"] = now

            if t == "ATTITUDE":
                tele["yaw_rad"] = float(msg.yaw)

            elif t == "GLOBAL_POSITION_INT":
                tele["rel_alt_m"] = float(msg.relative_alt) / 1000.0  # mm→m

            elif t == "LOCAL_POSITION_NED":
                n, e, d = float(msg.x), float(msg.y), float(msg.z)
                if ZERO_LOCAL_ORIGIN and tele["n0"] is None:
                    tele["n0"], tele["e0"], tele["d0"] = n, e, d
                tele["n"], tele["e"], tele["d"] = n, e, d
                tele["has_local"] = True

        # periodic prints
        if now - last_print >= PRINT_EVERY_SEC:
            with state_lock:
                yaw_deg = math.degrees(tele["yaw_rad"])
                alt     = tele["rel_alt_m"]
                if tele["has_local"]:
                    xn = tele["n"] - (tele["n0"] if tele["n0"] is not None else 0.0)
                    ye = tele["e"] - (tele["e0"] if tele["e0"] is not None else 0.0)
                    zd = tele["d"] - (tele["d0"] if tele["d0"] is not None else 0.0)
                else:
                    xn = ye = 0.0
                    zd = -alt
            print(f"[MAV] yaw={yaw_deg:6.1f}° alt={alt:4.1f}m  "
                  f"N/E/D={xn:+6.1f}/{ye:+6.1f}/{zd:+6.1f}")
            last_print = now

# --- mirror to sim ---
def mirror_sender_loop():
    period = 1.0 / SEND_RATE_HZ
    while True:
        t0 = time.time()
        with state_lock:
            yaw_deg = math.degrees(tele["yaw_rad"])
            alt     = tele["rel_alt_m"]

            if tele["has_local"]:
                xw = tele["n"] - (tele["n0"] if tele["n0"] is not None else 0.0)  # world X ~ North
                yw = tele["e"] - (tele["e0"] if tele["e0"] is not None else 0.0)  # world Y ~ East
            else:
                # אין LOCAL_POSITION_NED — משדר רק גובה/יאו, XY=0
                xw = 0.0
                yw = 0.0

        send_setpos(xw, yw, alt, yaw_deg)
        dt = time.time() - t0
        if dt < period:
            time.sleep(period - dt)

# --- main ---
def main():
    threading.Thread(target=mavlink_loop,       daemon=True).start()
    threading.Thread(target=mirror_sender_loop, daemon=True).start()
    try:
        while True:
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nBye.")

if __name__ == "__main__":
    main()
