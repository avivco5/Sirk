from pymavlink import mavutil
from pynput import keyboard
import time

# 📡 Connect from Windows host to SITL in Ubuntu VM
master = mavutil.mavlink_connection('udp:0.0.0.0:14550', source_system=255)

# Wait for heartbeat
print("⏳ Waiting for heartbeat...")
master.wait_heartbeat()
print("✅ Connected to system:", master.target_system, "component:", master.target_component)

# Initial RC values
roll = 1500
pitch = 1500
yaw = 1500
throttle = 1000

def send_rc():
    """Send RC override values to ArduPilot SITL"""
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        roll, pitch, throttle, yaw,
        0, 0, 0, 0
    )

def wait_ack():
    """📩 Wait for COMMAND_ACK and print result"""
    msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    if msg:
        print(f"📩 ACK received: command={msg.command}, result={msg.result}")
    else:
        print("⚠️ No ACK received")

def arm():
    """Arm the drone safely"""
    global throttle
    throttle = 1000
    send_rc()
    time.sleep(1)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    print("⚡ ARM command sent")
    wait_ack()

def disarm():
    """Disarm the drone"""
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print("🛑 DISARM command sent")
    wait_ack()

def on_press(key):
    """Handle key press events"""
    global roll, pitch, yaw, throttle
    try:
        if key.char == 'w':   # Increase throttle
            throttle = min(2000, throttle + 50)
        elif key.char == 's': # Decrease throttle
            throttle = max(1000, throttle - 50)
        elif key.char == 'a': # Yaw left
            yaw = max(1000, yaw - 50)
        elif key.char == 'd': # Yaw right
            yaw = min(2000, yaw + 50)
        elif key.char == 'j': # Roll left
            roll = max(1000, roll - 50)
        elif key.char == 'l': # Roll right
            roll = min(2000, roll + 50)
        elif key.char == 'i': # Pitch forward
            pitch = max(1000, pitch - 50)
        elif key.char == 'k': # Pitch backward
            pitch = min(2000, pitch + 50)
        elif key.char == 'o': # Arm
            arm()
        elif key.char == 'p': # Disarm
            disarm()

        send_rc()
        print(f"Roll={roll}, Pitch={pitch}, Yaw={yaw}, Throttle={throttle}")

    except AttributeError:
        pass

def on_release(key):
    """Stop program on ESC key"""
    if key == keyboard.Key.esc:
        disarm()
        return False

# Start listening to keyboard
with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    print("🎮 Keyboard control ready (W/S=Throttle, A/D=Yaw, I/K=Pitch, J/L=Roll, O=Arm, P=Disarm, ESC=Exit)")
    listener.join()
