from pymavlink import mavutil
import time

# Connect to the flight controller (Windows COM11 at 115200 baud)
master = mavutil.mavlink_connection('COM11', baud=115200)

# Wait for the heartbeat from the FC
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Heartbeat received from system (system %u component %u)" % (master.target_system, master.target_component))

# Function to spin motor
def spin_motor(motor_number, pwm_value=1500, duration=2):
    """
    motor_number: 1-8 (ArduPilot motor outputs)
    pwm_value: microseconds (1000 = min, 2000 = max)
    duration: seconds to spin
    """
    print(f"Spinning motor {motor_number} at {pwm_value} for {duration} sec")

    # Send MAV_CMD_DO_MOTOR_TEST
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
        0,               # confirmation
        motor_number,    # param1: motor number (1-8)
        1,               # param2: test type (1 = PWM)
        pwm_value,       # param3: PWM value
        duration,        # param4: timeout
        0, 0, 0          # param5-7 unused
    )
    time.sleep(duration + 1)

# Example: test motors 1-4
for i in range(1, 5):
    spin_motor(i, 1500, 2)

print("Motor test complete.")
