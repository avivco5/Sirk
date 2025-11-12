from pymavlink import mavutil, mavparm
import time
dev, baud = "COM40", 115200  # עדכן לפי הצורך

params = {
    "ARMING_CHECK": 0,
    "BRD_SAFETYENABLE": 0,
    "FS_THR_ENABLE": 0,
    "FS_GCS_ENABLE": 0,
    "FS_EKF_ACTION": 0,
    "FENCE_ENABLE": 0,
    "GPS_TYPE": 0,
    "AHRS_GPS_USE": 0,
    "BATT_MONITOR": 0,
    "BATT_ARM_VOLT": 0,
    "BATT_FS_LOW_ACT": 0,
    "BATT_FS_CRT_ACT": 0,
    # אופציונלי:
    # "COMPASS_USE": 0, "COMPASS_USE2": 0, "COMPASS_USE3": 0,
    # "PRX_TYPE": 0, "AVOID_ENABLE": 0, "TERRAIN_ENABLE": 0,
}

m = mavutil.mavlink_connection(dev, baud=baud)
m.wait_heartbeat()

def set_param(name, value):
    m.mav.param_set_send(m.target_system, m.target_component,
                         name.encode('ascii'), float(value),
                         mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    # המתן לאישור קצר
    for _ in range(40):
        msg = m.recv_match(type='PARAM_VALUE', blocking=False)
        if msg:
            pid = msg.param_id
            if isinstance(pid, bytes):
                pid = pid.decode('ascii', 'ignore')
            pid = pid.strip('\x00')
            if pid == name:
                break
        time.sleep(0.05)


for k,v in params.items():
    try:
        set_param(k, v)
        print("OK:", k, "=", v)
    except Exception as e:
        print("ERR:", k, e)
