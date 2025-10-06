import serial
import struct
import time
import tkinter as tk
import cv2
from ultralytics import YOLO
import threading

# ------------------------ MSP helpers ------------------------

def make_msp(command, payload=b""):
    if isinstance(payload, list):
        payload = bytes(payload)
    length = len(payload)
    checksum = 0
    frame = b"$M<" + bytes([length]) + bytes([command]) + payload
    checksum ^= length
    checksum ^= command
    for b in payload:
        checksum ^= b
    frame += bytes([checksum])
    return frame

def set_motors(ser, motor_values):
    """Send MSP_SET_MOTOR (214)."""
    values = (motor_values + [0]*8)[:8]
    payload = b""
    for v in values:
        payload += struct.pack("<H", v)
    frame = make_msp(214, payload)
    ser.write(frame)

# ---------------------- GUI ----------------------

def build_gui(ser):
    root = tk.Tk()
    root.title("Betaflight Motor Test")

    sliders = []
    def send_motor_cmd(motor, value):
        arr = [s.get() for s in sliders]
        set_motors(ser, arr)

    for i in range(4):
        frame = tk.Frame(root)
        frame.pack()
        tk.Label(frame, text=f"Motor {i+1}").pack(side=tk.LEFT)
        s = tk.Scale(frame, from_=1000, to=2000, orient=tk.HORIZONTAL,
                     command=lambda val, m=i: send_motor_cmd(m, int(val)))
        s.set(1000)
        s.pack(side=tk.LEFT)
        sliders.append(s)

    root.mainloop()

# ---------------------- YOLO Detection ----------------------

def yolo_loop(ser):
    model = YOLO("yolov8n.pt")  # YOLOv8 Nano
    cap = cv2.VideoCapture(0)   # מצלמת מחשב
    if not cap.isOpened():
        print("⚠️ לא נמצאה מצלמה")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        results = model(frame, verbose=False)
        detected_person = False

        for r in results:
            for box in r.boxes:
                cls = int(box.cls[0].item())
                if cls == 0:  # class 0 = person
                    detected_person = True
                    break

        if detected_person:
            print("👤 זוהה אדם → מנועים ל-25%")
            set_motors(ser, [1250, 1250, 1250, 1250])
        else:
            print("⛔ אין אדם → מנועים כבויים")
            set_motors(ser, [1000, 1000, 1000, 1000])

        # הצגת פריים עם תוצאות
        annotated = results[0].plot()
        cv2.imshow("YOLO Person Detection", annotated)

        if cv2.waitKey(1) & 0xFF == 27:  # ESC לסגירה
            break

    cap.release()
    cv2.destroyAllWindows()

# ---------------------- Main ----------------------

if __name__ == "__main__":
    # ⚠️ שנה את הפורט לפי הצורך
    ser = serial.Serial("COM19", 115200, timeout=1)

    # להריץ YOLO בלולאה נפרדת
    threading.Thread(target=yolo_loop, args=(ser,), daemon=True).start()

    # להפעיל את ה-GUI
    build_gui(ser)
