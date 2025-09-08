# disable_logging_and_checks.py
from pymavlink import mavutil
import time

LINK, BAUD = "COM14", 115200

m = mavutil.mavlink_connection(LINK, baud=BAUD)
m.wait_heartbeat()
print("✅ HB:", m.target_system, m.target_component)

def setp(name, val):
    m.mav.param_set_send(m.target_system, m.target_component,
                         name.encode('ascii'), float(val),
                         mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    print(f"[PARAM] {name}={val}")

# מכבה לוגים (File), ומכבה בדיקות ARM (לבדיקות בלבד!)
setp("LOG_BACKEND_TYPE", 0)   # 0=None, 1=File(SD), 4=MAVLink ...
setp("ARMING_CHECK", 0)       # לכבות זמנית את כל הבדיקות

# ריבוט לבקר כדי להחיל
m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                        0, 1, 0,0,0,0,0,0)
print("🔁 Reboot sent. המתן לניתוק/התחברות מחדש.")
