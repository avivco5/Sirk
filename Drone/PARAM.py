#!/usr/bin/env python3
# motor_unlimit.py – set wide motor ranges for bench
import time
from pymavlink import mavutil

DEVICE="COM14"; BAUD=115200

PARAMS = {
    # Choose ONE of the following lines; leave PWM=0 by default:
    "MOT_PWM_TYPE": 0,        # 0=PWM, 5=DShot300, 6=DShot600

    # Output mapping: Motor1..4 on SERVO1..4
    "SERVO1_FUNCTION": 33,
    "SERVO2_FUNCTION": 34,
    "SERVO3_FUNCTION": 35,
    "SERVO4_FUNCTION": 36,

    # Wide PWM limits (have effect when MOT_PWM_TYPE=0)
    "MOT_PWM_MIN": 1000,
    "MOT_PWM_MAX": 2000,

    # Also set per-channel min/max (good practice)
    "SERVO1_MIN": 1000, "SERVO1_MAX": 2000, "SERVO1_TRIM": 1500,
    "SERVO2_MIN": 1000, "SERVO2_MAX": 2000, "SERVO2_TRIM": 1500,
    "SERVO3_MIN": 1000, "SERVO3_MAX": 2000, "SERVO3_TRIM": 1500,
    "SERVO4_MIN": 1000, "SERVO4_MAX": 2000, "SERVO4_TRIM": 1500,

    # Spin thresholds (bench)
    "MOT_SPIN_ARMED": 0.10,
    "MOT_SPIN_MIN":   0.13,
    "MOT_SPIN_MAX":   1.00,
}

def setp(m, name, val):
    m.mav.param_set_send(m.target_system, m.target_component,
                         name.encode('ascii'), float(val),
                         mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    t0 = time.time()
    while time.time() - t0 < 2.0:
        msg = m.recv_match(type="PARAM_VALUE", blocking=False)
        if msg:
            pid = msg.param_id.decode('ascii','ignore').strip('\x00')
            if pid == name:
                print(f"[OK] {name} = {msg.param_value:.3f}")
                return
        time.sleep(0.05)
    print(f"[WARN] no ack for {name}")

def main():
    print(f"[MAV] connect {DEVICE} @ {BAUD}")
    m = mavutil.mavlink_connection(DEVICE, baud=BAUD)
    m.wait_heartbeat()
    print(f"[MAV] sys {m.target_system} comp {m.target_component}")

    for k,v in PARAMS.items():
        try: setp(m,k,v)
        except Exception as e: print(f"[ERR] {k}: {e}")

    print("\n[INFO] Done. For PWM ESCs, run ESC calibration if top speed still seems capped.")
    print("[INFO] Bench: STABILIZE → Force ARM → הזז Throttle; עקוב אחרי RCOUT ב-Tuning Graph.")

if __name__ == "__main__":
    main()
