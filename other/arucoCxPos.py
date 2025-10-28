import cv2
import cv2.aruco as aruco
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
    # שלב 1: RC Neutral
    send("ROLL 1500")
    send("PITCH 1500")
    send("YAW 1500")
    send("THROTTLE 1000")
    time.sleep(1)

    # שלב 2: ARM
    send("ARM")
    time.sleep(2)

def takeoff(throttle=1600, duration=5):
    print("🛫 Takeoff...")
    for _ in range(duration):
        send(f"THROTTLE {throttle}")
        time.sleep(1)
    send("THROTTLE 1550")  # החזקת גובה
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

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        h, w, _ = frame.shape
        corners, ids, _ = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

        if ids is not None:
            # מרקר ראשון שזוהה
            c = corners[0][0]
            cx = int((c[0][0] + c[2][0]) / 2)
            cy = int((c[0][1] + c[2][1]) / 2)

            # גודל משוער של המרקר (שטח מלבן)
            marker_size = abs(c[0][0] - c[1][0]) * abs(c[0][1] - c[2][1])
            frame_area = w * h

            # -----------------------
            # YAW: כיוון אופקי
            # -----------------------
            if cx < w/2 - 50:
                yaw = 1400
            elif cx > w/2 + 50:
                yaw = 1600
            else:
                yaw = 1500

            # -----------------------
            # PITCH: תנועה קדימה
            # -----------------------
            if marker_size < frame_area * 0.01:   # קטן מאוד → רחוק → קדימה
                pitch = 1600
            else:
                pitch = 1500

            # -----------------------
            # THROTTLE: שליטה בגובה
            # -----------------------
            if marker_size > frame_area * 0.05:   # גדול מדי → קרוב → תרומם
                throttle = 1600
            elif marker_size < frame_area * 0.005:  # קטן מדי → רחוק → תרד
                throttle = 1500
            else:
                throttle = 1550  # שמירה על גובה

            # שליחת פקודות
            send(f"YAW {yaw}")
            send(f"PITCH {pitch}")
            send(f"THROTTLE {throttle}")

            print(f"📌 Marker {ids[0]} | size={marker_size:.0f} | "
                  f"YAW={yaw} | PITCH={pitch} | THR={throttle}")

            # ציור תיבה
            frame = aruco.drawDetectedMarkers(frame, corners, ids)

        else:
            print("⛔ No marker – Hover")
            send("YAW 1500")
            send("PITCH 1500")
            send("THROTTLE 1550")

        cv2.imshow("ArUco Tracking", frame)
        if cv2.waitKey(1) & 0xFF == 27:  # ESC לסיום
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
