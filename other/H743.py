from pymavlink import mavutil
import time

# התחברות לבקר (שנה את COM19/baud לפי המחשב שלך)
master = mavutil.mavlink_connection('COM14', baud=115200)

# ממתין ל-heartbeat מהבקר
master.wait_heartbeat()
print("✅ Connected to system:", master.target_system, "component:", master.target_component)

while True:
    # קבלת הודעת MAVLink
    msg = master.recv_match(blocking=True)
    if not msg:
        continue

    # הדפסה מסודרת לפי סוג ההודעה
    msg_type = msg.get_type()

    if msg_type == "ATTITUDE":
        print(f"🌀 Attitude | Roll={msg.roll:.2f}, Pitch={msg.pitch:.2f}, Yaw={msg.yaw:.2f}")

    elif msg_type == "VFR_HUD":
        print(f"✈️  HUD | Alt={msg.alt:.1f} m, Airspeed={msg.airspeed:.1f}, Groundspeed={msg.groundspeed:.1f}")

    elif msg_type == "BATTERY_STATUS":
        voltage = msg.voltages[0] / 1000.0
        print(f"🔋 Battery | Voltage={voltage:.2f} V, Current={msg.current_battery/100:.1f} A")

    elif msg_type == "HEARTBEAT":
        mode = mavutil.mode_string_v10(msg)
        print(f"❤️ Heartbeat | Mode={mode}, Armed={msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED != 0}")

    # אפשר להוסיף עוד הודעות לפי הצורך:
    # GPS_RAW_INT, SYS_STATUS, LOCAL_POSITION_NED, GLOBAL_POSITION_INT וכו'

    time.sleep(0.001)
