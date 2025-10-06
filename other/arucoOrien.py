import cv2
import cv2.aruco as aruco
import numpy as np
import socket
import time

# =======================
# UDP SETUP
# =======================
UDP_IP = "10.0.0.28"   # כתובת ה־VM (Ubuntu)
UDP_PORT = 5006
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send(cmd):
    sock.sendto(cmd.encode(), (UDP_IP, UDP_PORT))
    print(f"➡️ {cmd}")

# =======================
# INIT + ARM + TAKEOFF
# =======================
def init_and_arm():
    send("ROLL 1500")
    send("PITCH 1500")
    send("YAW 1500")
    send("THROTTLE 1000")
    time.sleep(1)

    send("ARM")
    time.sleep(2)

def takeoff(throttle=1600, duration=5):
    print("🛫 Takeoff...")
    for _ in range(duration):
        send(f"THROTTLE {throttle}")
        time.sleep(1)
    send("THROTTLE 1550")
    print("✅ Holding altitude")

# =======================
# ARUCO CONTROL LOOP
# =======================
def aruco_control():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("⚠️ Camera not found")
        return

    # ArUco dictionary
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()

    # ⚠️ צריך קליברציה אמיתית של מצלמה, פה ערכים משוערים
    camera_matrix = np.array([[800, 0, 320],
                              [0, 800, 240],
                              [0,   0,   1]], dtype=np.float32)
    dist_coeffs = np.zeros((5, 1))  # בלי עיוות

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        h, w, _ = frame.shape
        corners, ids, _ = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

        if ids is not None:
            # הערכת פוזה של המרקר
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs)
            rvec, tvec = rvecs[0], tvecs[0]

            # מטריצת רוטציה → זוויות אוילר
            R, _ = cv2.Rodrigues(rvec)
            sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)

            if sy > 1e-6:
                roll = np.arctan2(R[2, 1], R[2, 2])
                pitch = np.arctan2(-R[2, 0], sy)
                yaw = np.arctan2(R[1, 0], R[0, 0])
            else:
                roll = np.arctan2(-R[1, 2], R[1, 1])
                pitch = np.arctan2(-R[2, 0], sy)
                yaw = 0

            # המרה למעלות
            roll_deg, pitch_deg, yaw_deg = np.degrees([roll, pitch, yaw])

            # =========================
            # מיפוי זוויות לסטיקים
            # =========================
            yaw_stick = int(1500 + yaw_deg * 2)     # פנייה לפי זווית
            pitch_stick = int(1500 - pitch_deg * 2) # קדימה/אחורה לפי זווית
            roll_stick = int(1500 + roll_deg * 2)   # גלגול לפי זווית

            # תחום הערכים [1000, 2000]
            yaw_stick = max(1000, min(2000, yaw_stick))
            pitch_stick = max(1000, min(2000, pitch_stick))
            roll_stick = max(1000, min(2000, roll_stick))

            # שליחה
            send(f"YAW {yaw_stick}")
            send(f"PITCH {pitch_stick}")
            send(f"ROLL {roll_stick}")
            send("THROTTLE 1550")  # שמירה על גובה קבוע

            print(f"📌 Marker {ids[0]} | Roll={roll_deg:.1f}° | Pitch={pitch_deg:.1f}° | Yaw={yaw_deg:.1f}° "
                  f"→ ROLL={roll_stick}, PITCH={pitch_stick}, YAW={yaw_stick}")

            # ציור ציר על המרקר
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.05)
            frame = aruco.drawDetectedMarkers(frame, corners, ids)

        else:
            print("⛔ No marker – Hover")
            send("ROLL 1500")
            send("PITCH 1500")
            send("YAW 1500")
            send("THROTTLE 1550")

        cv2.imshow("ArUco Tracking", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

# =======================
# MAIN
# =======================
if __name__ == "__main__":
    init_and_arm()
    takeoff()
    aruco_control()
