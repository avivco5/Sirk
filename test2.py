# arm_spin_bypass_logging.py
# פותר "Arm: Logging not started" ע"י bypass זמני של בדיקות ARM, ואז ARM + ספין עדין.
# שימוש:
#   python arm_spin_bypass_logging.py --link COM14 --baud 115200 --mode STABILIZE --thr 1120 --secs 5 --bypass
# בלי --bypass: רק ניסיון רגיל (נדרש microSD תקין או LOG_BACKEND_TYPE מתאים).

import argparse, time
from pymavlink import mavutil

# תאימות לגירסאות ישנות
if not hasattr(mavutil.mavlink, "MAV_RESULT_CANCELLED"):
    mavutil.mavlink.MAV_RESULT_CANCELLED = 10

ACK_NAMES = {
    mavutil.mavlink.MAV_RESULT_ACCEPTED: "ACCEPTED",
    mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED: "TEMP_REJECTED",
    mavutil.mavlink.MAV_RESULT_DENIED: "DENIED",
    mavutil.mavlink.MAV_RESULT_UNSUPPORTED: "UNSUPPORTED",
    mavutil.mavlink.MAV_RESULT_FAILED: "FAILED",
    mavutil.mavlink.MAV_RESULT_IN_PROGRESS: "IN_PROGRESS",
    mavutil.mavlink.MAV_RESULT_CANCELLED: "CANCELLED",
}

def connect(link, baud, force_py_parser=False):
    print(f"[MAV] Connecting {link} @ {baud if not link.startswith(('udp:','tcp:')) else 'n/a'} ...")
    def _open(use_native):
        if link.startswith(("udp:","tcp:")):
            return mavutil.mavlink_connection(link, use_native=use_native)
        else:
            return mavutil.mavlink_connection(link, baud=baud, use_native=use_native)

    if force_py_parser:
        m = _open(False); m.wait_heartbeat()
        print(f"✅ Heartbeat: sys {m.target_system} comp {m.target_component} (py parser)")
        return m

    try:
        m = _open(True); m.wait_heartbeat()
        print(f"✅ Heartbeat: sys {m.target_system} comp {m.target_component}")
        return m
    except TypeError:
        print("⚠️ Native parser issue → retry with Python parser...")
        try: m.close()
        except Exception: pass
        m = _open(False); m.wait_heartbeat()
        print(f"✅ Heartbeat: sys {m.target_system} comp {m.target_component} (py parser)")
        return m

def request_streams(m, rate=5):
    try:
        for sid in (mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
                    mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                    mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
                    mavutil.mavlink.MAV_DATA_STREAM_EXTRA2):
            m.mav.request_data_stream_send(m.target_system, m.target_component, sid, rate, 1)
    except Exception:
        pass

def is_armed(hb): return bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

def mode_string(msg):
    try: return mavutil.mode_string_v10(msg)
    except Exception: return f"custom_mode={getattr(msg,'custom_mode','?')}"

def poll_status(m, secs=2.0):
    t0=time.time()
    while time.time()-t0 < secs:
        msg = m.recv_match(blocking=True, timeout=0.5)
        if not msg: continue
        t = msg.get_type()
        if t == "STATUSTEXT": print(f"[STATUSTEXT {getattr(msg,'severity',6)}] {msg.text}")
        elif t == "COMMAND_ACK":
            print(f"[ACK] cmd={msg.command} → {ACK_NAMES.get(msg.result, msg.result)}")
        elif t == "HEARTBEAT":
            print(f"[HEARTBEAT] {'ARMED' if is_armed(msg) else 'DISARMED'} | {mode_string(msg)}")
        elif t == "SYS_STATUS":
            v = getattr(msg,"voltage_battery",-1)/1000.0; rem=getattr(msg,"battery_remaining",-1)
            print(f"[SYS_STATUS] Vbat={v:.2f}V Rem={rem}%")
        elif t == "GPS_RAW_INT":
            print(f"[GPS] fix={getattr(msg,'fix_type',0)} sats={getattr(msg,'satellites_visible',0)}")

def _param_id_to_str(param_id):
    return param_id.decode('ascii','ignore').strip('\x00').upper() if isinstance(param_id, bytes) \
           else str(param_id).strip('\x00').upper()

def param_get(m, name, timeout=1.5):
    m.mav.param_request_read_send(m.target_system, m.target_component, name.encode('ascii'), -1)
    t0=time.time()
    while time.time()-t0 < timeout:
        msg = m.recv_match(type="PARAM_VALUE", blocking=False)
        if msg:
            if _param_id_to_str(msg.param_id) == name.upper():
                try: return float(msg.param_value)
                except: return None
        time.sleep(0.02)
    return None

def param_set(m, name, val, confirm=True):
    m.mav.param_set_send(m.target_system, m.target_component, name.encode('ascii'),
                         float(val), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    print(f"[PARAM] set {name}={val}")
    if not confirm: return True
    time.sleep(0.05)
    got = param_get(m, name, timeout=1.5)
    if got is None:
        print(f"[PARAM] {name} no response"); return False
    ok = abs(got-float(val)) <= 1e-3
    print(f"[PARAM] {name} now {got} → {'OK' if ok else 'MISMATCH'}")
    return ok

def set_mode(m, mode_name="STABILIZE"):
    try:
        modes = m.mode_mapping()
        if mode_name in modes:
            m.set_mode(modes[mode_name]); print(f"[MODE] {mode_name}"); return
    except Exception: pass
    mapping={"STABILIZE":0,"ACRO":1,"ALT_HOLD":2,"AUTO":3,"GUIDED":4,"LOITER":5,"LAND":9}
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_DO_SET_MODE,0, 1,mapping.get(mode_name,0),0,0,0,0,0)
    print(f"[MODE] {mode_name} (fallback)")

def arm(m):
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,1,0,0,0,0,0,0)
    print("➡️ ARM sent")

def disarm(m):
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,0,0,0,0,0,0,0)
    print("⛔ DISARM sent")

def wait_armed(m, timeout=6.0):
    t0=time.time()
    while time.time()-t0 < timeout:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if hb and is_armed(hb): print("✅ ARMED confirmed"); return True
        st = m.recv_match(type="STATUSTEXT", blocking=False)
        if st: print(f"[STATUSTEXT] {st.text}")
    print("❌ still DISARMED (timeout)"); return False

def rc_override(m, roll=None, pitch=None, thr=None, yaw=None):
    as_ch=lambda v: int(v) if v is not None else 0
    m.mav.rc_channels_override_send(m.target_system, m.target_component,
                                    as_ch(roll),as_ch(pitch),as_ch(thr),as_ch(yaw),
                                    0,0,0,0)

def spin_test(m, thr_pwm=1120, secs=5):
    RC_CTR=1500
    print(f"[TEST] throttle={thr_pwm} for {secs}s (RC override)")
    t0=time.time()
    while time.time()-t0 < secs:
        rc_override(m, roll=RC_CTR, pitch=RC_CTR, thr=thr_pwm, yaw=RC_CTR)
        time.sleep(0.05)
    rc_override(m,0,0,0,0); print("[TEST] override cleared")

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--link", default="COM14")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--mode", default="STABILIZE")
    ap.add_argument("--thr", type=int, default=1120)
    ap.add_argument("--secs", type=float, default=5.0)
    ap.add_argument("--py-parser", action="store_true", help="Force pymavlink use_native=False")
    ap.add_argument("--bypass", action="store_true", help="Temporarily disable arming checks (ARMING_CHECK=0)")
    ap.add_argument("--disable-logging", action="store_true", help="Set LOG_BACKEND_TYPE=0 (no logging)")
    args=ap.parse_args()

    m=connect(args.link, args.baud, force_py_parser=args.py_parser)
    request_streams(m, rate=5)

    # הדפסות מצב קצרות
    poll_status(m, secs=2.0)

    # BYPASS זמני אם ביקשת (מומלץ כשאין microSD)
    if args.bypass:
        ok = param_set(m, "ARMING_CHECK", 0)
        if not ok: print("⚠️ לא הצלחתי לכתוב ARMING_CHECK=0")
    if args.disable_logging:
        ok = param_set(m, "LOG_BACKEND_TYPE", 0)
        if not ok: print("⚠️ לא הצלחתי לכתוב LOG_BACKEND_TYPE=0 (ייתכן שידרוש Reboot)")

    # מצב טיסה (ללא GPS עדיף STABILIZE)
    set_mode(m, args.mode); time.sleep(0.5)

    # ARM → בדיקת אמת
    arm(m)
    if not wait_armed(m, timeout=6.0):
        print("❌ לא חמוש — אם קיבלת שוב 'Logging not started', תריץ עם --bypass או --disable-logging, או הכנס microSD.")
        return

    # ספין עדין
    spin_test(m, thr_pwm=args.thr, secs=args.secs)

    # DISARM וסיום
    disarm(m); poll_status(m, secs=1.0)
    print("✅ Done")

if __name__ == "__main__":
    main()
