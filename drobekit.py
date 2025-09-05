from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

# התחברות לבקר – תעדכן ל-COM בווינדוס או ל-ttyUSB ברספברי
# Windows: "COM18"
# Linux/RPi: "/dev/ttyAMA0" או "/dev/ttyUSB0"
vehicle = connect("COM19", wait_ready=True, baud=115200)

print("מחובר לבקר")

# Arm & Takeoff
def arm_and_takeoff(aTargetAltitude):
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" מנסה לחמש...")
        time.sleep(1)

    print("המראה")
    vehicle.simple_takeoff(aTargetAltitude)

    # מחכה להגיע לגובה
    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f"גובה: {alt:.2f} מטר")
        if alt >= aTargetAltitude * 0.95:
            print("הגענו לגובה יעד")
            break
        time.sleep(1)

# 🟢 הרצה
arm_and_takeoff(3)  # המראה לגובה 3 מטר

print("טיסה ישרה...")
vehicle.simple_goto(LocationGlobalRelative(vehicle.location.global_frame.lat + 0.00005,
                                           vehicle.location.global_frame.lon,
                                           vehicle.location.global_relative_frame.alt))
time.sleep(5)

print("סיבוב במקום...")
vehicle.condition_yaw(90)  # פנייה 90° ימינה
time.sleep(3)

print("הנמכה...")
vehicle.simple_goto(LocationGlobalRelative(vehicle.location.global_frame.lat,
                                           vehicle.location.global_frame.lon,
                                           1))
time.sleep(5)

print("נחיתה...")
vehicle.mode = VehicleMode("LAND")

# סיום
time.sleep(10)
vehicle.close()
