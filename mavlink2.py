# arm_disarm_fixed.py
# Arms motors on COM14 @115200 for 3 seconds, then disarms.
# Make sure props are removed!

import time
from pymavlink import mavutil

# ==== Fixed config ====
PORT = "COM14"
BAUD = 115200
MODE_NAME = "STABILIZE"
ARM_TIME = 1.0
FORCE_ARM = True   # Change to True only for bench tests

# -----------------------

def connect_serial(port, baud):
    print(f"[MAV] Connecting {port} @ {baud} ...")
    m = mavutil.mavlink_connection(port, baud=baud)
    m.wait_heartbeat()
    print(f"✅ Heartbeat: sys {m.target_system} comp {m.target_component}")
    return m

def set_mode(master, mode_name="STABILIZE"):
    mapping = master.mode_mapping()
    if mapping and mode_name in mapping:
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mapping[mode_name],
        )
        t0 = time.time()
        while time.time() - t0 < 5:
            hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
            if hb and hasattr(hb, "custom_mode"):
                if hb.custom_mode == mapping[mode_name]:
                    print(f"✅ Mode: {mode_name}")
                    return True
    print("⚠️ Mode set timed out")
    return False

def is_armed(master):
    return master.motors_armed()

def arm(master, force=False):
    p2 = 21196 if force else 0
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, p2, 0, 0, 0, 0, 0
    )
    t0 = time.time()
    while time.time() - t0 < 8:
        if is_armed(master):
            print("✅ ARMED")
            return True
        time.sleep(0.2)
    print("❌ Failed to arm")
    return False

def disarm(master):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    t0 = time.time()
    while time.time() - t0 < 6:
        if not is_armed(master):
            print("✅ DISARMED")
            return True
        time.sleep(0.2)
    print("❌ Failed to disarm")
    return False

def motor_test(master, motor_id=1, throttle_type=0, throttle=20, timeout=2):
    """
    Perform a motor test.
    :param master: mavutil connection
    :param motor_id: Motor number (1=A, 2=B, 3=C, 4=D for quad)
    :param throttle_type: 0=PWM, 1=percentage, 2=pass-thru
    :param throttle: power value (depends on throttle_type)
    :param timeout: seconds to run motor
    """
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
        0,
        motor_id,         # param1: motor number
        throttle_type,    # param2: throttle type
        throttle,         # param3: throttle value
        timeout,          # param4: timeout (s)
        0, 0, 0
    )
    print(f"⚡ Motor {motor_id} test started for {timeout}s at {throttle} ({'%' if throttle_type==1 else 'PWM'})")


def main():
    master = connect_serial(PORT, BAUD)
    set_mode(master, MODE_NAME)
    if arm(master, force=FORCE_ARM):
        time.sleep(ARM_TIME)
        disarm(master)

if __name__ == "__main__":
    main()
