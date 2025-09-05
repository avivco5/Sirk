from pymavlink import mavutil
import tkinter as tk
from tkinter import ttk
import threading, time

# --- MAVLink Connection ---
master = mavutil.mavlink_connection('COM14', baud=115200)
master.wait_heartbeat()
print("✅ Connected:", master.target_system, master.target_component)

# RC ערכי בסיס
rc_roll = 1500
rc_pitch = 1500
rc_yaw = 1500
rc_throttle = 1000

def send_rc():
    """שולח ערכי RC לבקר"""
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        rc_roll, rc_pitch, rc_throttle, rc_yaw,
        0, 0, 0, 0
    )

def arm():
    """חימוש"""
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    print("🚀 Armed")

def disarm():
    """נטרול"""
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print("🛑 Disarmed")

# --- GUI ---
root = tk.Tk()
root.title("Drone Control GUI")

# סליידר גובה
def on_throttle(val):
    global rc_throttle
    rc_throttle = int(float(val))
    send_rc()

throttle_slider = ttk.Scale(root, from_=1000, to=2000, orient="vertical", command=on_throttle, length=200)
throttle_slider.set(1000)
throttle_slider.grid(row=1, column=0, rowspan=3)

# כפתורי שליטה
def move(direction):
    global rc_roll, rc_pitch
    if direction == "forward":
        rc_pitch = 1600
    elif direction == "back":
        rc_pitch = 1400
    elif direction == "left":
        rc_roll = 1400
    elif direction == "right":
        rc_roll = 1600
    send_rc()
    root.after(300, reset_sticks)

def reset_sticks():
    global rc_roll, rc_pitch
    rc_roll, rc_pitch = 1500, 1500
    send_rc()

ttk.Button(root, text="⬆️ קדימה", command=lambda: move("forward")).grid(row=1, column=1)
ttk.Button(root, text="⬅️ שמאלה", command=lambda: move("left")).grid(row=2, column=0)
ttk.Button(root, text="➡️ ימינה", command=lambda: move("right")).grid(row=2, column=2)
ttk.Button(root, text="⬇️ אחורה", command=lambda: move("back")).grid(row=3, column=1)

# כפתורי Arm/Disarm
ttk.Button(root, text="🚀 Arm", command=arm).grid(row=4, column=0, sticky="ew")
ttk.Button(root, text="🛑 Disarm", command=disarm).grid(row=4, column=2, sticky="ew")

# --- Thread לקריאת טלמטריה ---
def telemetry_loop():
    while True:
        msg = master.recv_match(blocking=True)
        if not msg:
            continue
        t = msg.get_type()

        if t == "ATTITUDE":
            print(f"🌀 Attitude | Roll={msg.roll:.2f}, Pitch={msg.pitch:.2f}, Yaw={msg.yaw:.2f}")

        elif t == "GLOBAL_POSITION_INT":
            rel_alt = msg.relative_alt / 1000.0
            abs_alt = msg.alt / 1000.0
            print(f"🌍 Position | Lat={msg.lat/1e7:.6f}, Lon={msg.lon/1e7:.6f}, Alt={abs_alt:.1f} m, RelAlt={rel_alt:.1f} m")

        elif t == "BATTERY_STATUS":
            voltage = msg.voltages[0] / 1000.0
            current = msg.current_battery / 100.0 if msg.current_battery != -1 else 0
            print(f"🔋 Battery | Voltage={voltage:.2f} V, Current={current:.1f} A")

        elif t == "HEARTBEAT":
            mode = mavutil.mode_string_v10(msg)
            armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            print(f"❤️ Heartbeat | Mode={mode}, Armed={armed}")

        time.sleep(0.001)

threading.Thread(target=telemetry_loop, daemon=True).start()

root.mainloop()
