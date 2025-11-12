#!/usr/bin/env python3
from pymavlink import mavutil
import time

# התחברות לסימולטור (ברירת מחדל UDP על פורט 14550)
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()
print("Connected to system", master.target_system, "component", master.target_component)

def arm_and_takeoff(altitude):
    # ARM
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    master.motors_armed_wait()
    print("Armed!")

    # מצב GUIDED
    master.set_mode(mavutil.mavlink.MAV_MODE_GUIDED_ARMED)

    # המראה
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude
    )
    print(f"Takeoff to {altitude}m")
    time.sleep(10)

def goto(lat, lon, alt):
    master.mav.set_position_target_global_int_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,  # רק lat/lon/alt פעילים
        int(lat * 1e7), int(lon * 1e7), alt,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print(f"Goto {lat}, {lon}, {alt}m")
    time.sleep(15)

def land():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print("Landing...")

# ---------------------
arm_and_takeoff(10)  # המראה ל־10 מטר
goto(32.0852971, 34.9228633, 10)  # נקודה ראשונה
goto(32.085293,  34.922961,  10)  # נקודה שנייה
goto(32.085375,  34.922984,  10)  # נקודה שלישית
goto(32.085401,  34.922887,  10)  # נקודה רביעית
land()
