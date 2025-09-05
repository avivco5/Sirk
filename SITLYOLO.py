import socket
import time
import cv2
from ultralytics import YOLO

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
# INIT + ARM
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
    time.sleep(2)  # זמן לאישור חימוש

# =======================
# TAKEOFF (לפני YOLO)
# =======================
def takeoff(throttle=1600, duration=5):
    """מרים את הרחפן לגובה יציב לפני זיהוי"""
    print("🛫 Takeoff sequence...")
    send("ROLL 1500")
    send("PITCH 1500")
    send("YAW 1500")

    for _ in range(duration):
        send(f"THROTTLE {throttle}")
        time.sleep(1)

    # החזק טיסה יציבה
    send("THROTTLE 1500")
    print("✅ Takeoff complete – holding altitude")

# =======================
# YOLO LOOP
# =======================
def yolo_control():
    model = YOLO("yolov8n.pt")
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("⚠️ Camera not found")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        h, w, _ = frame.shape
        results = model(frame, verbose=False)

        detected = False
        for r in results:
            for box in r.boxes:
                if int(box.cls[0].item()) == 0:  # class 0 = person
                    detected = True
                    x1, y1, x2, y2 = box.xyxy[0]
                    cx = (x1 + x2) / 2
                    box_h = y2 - y1  # גובה התיבה

                    # ---- שליטה ב-YAW ----
                    center_error = cx - (w/2)
                    if abs(center_error) < 50:
                        yaw = 1500
                    elif center_error > 0:
                        yaw = 1600
                    else:
                        yaw = 1400
                    send(f"YAW {yaw}")

                    # ---- שליטה ב-PITCH לפי גודל האובייקט ----
                    if box_h < (h/2):  # אם האובייקט קטן מחצי מסך → להתקרב
                        pitch = 1600
                        print("👣 Object far → moving forward")
                    else:
                        pitch = 1500
                        print("🛑 Object close → stop forward movement")
                    send(f"PITCH {pitch}")

                    # ---- שמירה על גובה ----
                    send("THROTTLE 1500")
                    print(f"👤 Person detected | Yaw={yaw}, Pitch={pitch}, Throttle=1500")
                    break

        if not detected:
            print("⛔ No person – hovering neutral")
            send("YAW 1500")
            send("PITCH 1500")
            send("THROTTLE 1550")

        annotated = results[0].plot()
        cv2.imshow("YOLO Person Detection", annotated)
        if cv2.waitKey(1) & 0xFF == 27:  # ESC key
            break

    cap.release()
    cv2.destroyAllWindows()

# =======================
# MAIN
# =======================
if __name__ == "__main__":
    init_and_arm()    # ARM
    takeoff()         # המראה לגובה יציב
    yolo_control()    # הפעלת YOLO
