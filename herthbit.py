from pymavlink import mavutil

master = mavutil.mavlink_connection('udp:0.0.0.0:14550', source_system=255)

print("⏳ Waiting for heartbeat...")
master.wait_heartbeat()
print("✅ Connected to system:", master.target_system, "component:", master.target_component)
