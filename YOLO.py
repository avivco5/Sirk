import serial
import struct
import time
import tkinter as tk
import cv2
from ultralytics import YOLO
import threading
import atexit
import signal
import sys

# ==============================================================
# MSP FRAME HELPERS
# Functions to build and send MSP frames
# ==============================================================

def make_msp(command, payload=b""):
    """Build an MSP v1 frame"""
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

# ==============================================================
# MOTOR CONTROL (DIRECT + RC CHANNELS)
# ==============================================================

def set_motors(ser, motor_values):
    """
    MSP_SET_MOTOR (214) – direct ESC control.
    Works even if drone is NOT armed.
    Dangerous – use only for bench testing without props.
    """
    values = (motor_values + [0]*8)[:8]
    payload = b"".join(struct.pack("<H", v) for v in values)
    ser.write(make_msp(214, payload))

def set_rc_channels(ser, roll=1500, pitch=1500, yaw=1500, throttle=1000, aux=None):
    """
    MSP_SET_RAW_RC (200) – send RC stick values.
    Requires the drone to be armed to spin motors.
    Values: 1000–2000 (like PWM from radio).
    """
    channels = [roll, pitch, yaw, throttle]
    if aux:
        channels.extend(aux)
    else:
        channels.extend([1000] * 4)  # Default AUX
    payload = b"".join(struct.pack("<H", ch) for ch in channels)
    ser.write(make_msp(200, payload))

# ==============================================================
# ARM / DISARM FUNCTIONS
# ==============================================================

def arm_drone(ser):
    """
    Arm the drone by simulating RC stick command.
    Throttle = 1000, Yaw = 2000 (right).
    """
    print("⚡ Sending ARM command...")
    for _ in range(20):  # ~1 second
        set_rc_channels(ser, roll=1500, pitch=1500, yaw=2000, throttle=1000)
        time.sleep(0.05)

def disarm_drone(ser):
    """
    Disarm the drone by simulating RC stick command.
    Throttle = 1000, Yaw = 1000 (left).
    """
    print("🛑 Sending DISARM command...")
    for _ in range(20):  # ~1 second
        set_rc_channels(ser, roll=1500, pitch=1500, yaw=1000, throttle=1000)
        time.sleep(0.05)

# ==============================================================
# FAILSAFE – ENSURE MOTORS STOP ON EXIT
# ==============================================================

def shutdown_motors():
    """Ensure motors are stopped when program exits"""
    try:
        print("🛑 Failsafe: stopping motors...")
        set_motors(ser, [1000]*8)
        set_rc_channels(ser, 1500, 1500, 1500, 1000)
        time.sleep(0.1)
    except Exception as e:
        print("⚠️ Failed to send failsafe command:", e)

# ==============================================================
# YOLO OBJECT DETECTION LOOP
# Runs in background thread and controls drone when enabled
# ==============================================================

auto_mode_rc = False
auto_mode_direct = False

def yolo_loop(ser):
    global auto_mode_rc, auto_mode_direct
    model = YOLO("yolov8n.pt")
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("⚠️ No camera found")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        h, w, _ = frame.shape
        annotated = frame
        detected_person = False
        cx = None

        # Only run YOLO if RC or Direct mode is enabled
        if auto_mode_rc or auto_mode_direct:
            results = model(frame, verbose=False)
            for r in results:
                for box in r.boxes:
                    if int(box.cls[0].item()) == 0:  # class 0 = person
                        detected_person = True
                        x1, y1, x2, y2 = box.xyxy[0]
                        cx = (x1 + x2) / 2
                        break

            if detected_person:
                # Calculate yaw correction relative to frame center
                center_error = cx - (w/2)
                if abs(center_error) < 50:
                    yaw_cmd = 1500
                elif center_error > 0:
                    yaw_cmd = 1600
                else:
                    yaw_cmd = 1400

                print(f"👤 Person detected | Yaw={yaw_cmd}")

                if auto_mode_rc:
                    # RC mode: behaves like radio sticks
                    set_rc_channels(ser, roll=1500, pitch=1500,
                                    yaw=yaw_cmd, throttle=1050)
                elif auto_mode_direct:
                    # Direct mode: raw motor values
                    set_motors(ser, [1050, 1050, 1050, 1050])

            else:
                print("⛔ No person → motors off")
                set_rc_channels(ser, 1500, 1500, 1500, 1000)
                set_motors(ser, [1000]*8)

            annotated = results[0].plot()

        cv2.imshow("YOLO Person Detection", annotated)
        if cv2.waitKey(1) & 0xFF == 27:  # ESC key
            break

    cap.release()
    cv2.destroyAllWindows()

# ==============================================================
# TKINTER GUI – CONTROL PANEL
# ==============================================================

def build_gui():
    global auto_mode_rc, auto_mode_direct
    root = tk.Tk()
    root.title("Hybrid Drone Control")

    # Toggle RC Mode
    def toggle_rc():
        global auto_mode_rc, auto_mode_direct
        auto_mode_direct = False
        auto_mode_rc = not auto_mode_rc
        btn_rc["text"] = "Stop Object Detection (RC)" if auto_mode_rc else "Start Object Detection (RC)"
        print("✅ RC Mode ON" if auto_mode_rc else "⛔ RC Mode OFF")
        if not auto_mode_rc:
            shutdown_motors()

    # Toggle Direct Motor Mode
    def toggle_direct():
        global auto_mode_rc, auto_mode_direct
        auto_mode_rc = False
        auto_mode_direct = not auto_mode_direct
        btn_direct["text"] = "Stop Direct Motor Test" if auto_mode_direct else "Start Direct Motor Test"
        print("✅ Direct Mode ON" if auto_mode_direct else "⛔ Direct Mode OFF")
        if not auto_mode_direct:
            shutdown_motors()

    # GUI Buttons
    btn_arm = tk.Button(root, text="ARM", width=15, height=2,
                        command=lambda: arm_drone(ser))
    btn_arm.pack(pady=5)

    btn_disarm = tk.Button(root, text="DISARM", width=15, height=2,
                           command=lambda: disarm_drone(ser))
    btn_disarm.pack(pady=5)

    btn_rc = tk.Button(root, text="Start Object Detection (RC)",
                       width=30, height=3, command=toggle_rc)
    btn_rc.pack(pady=10)

    btn_direct = tk.Button(root, text="Start Direct Motor Test",
                           width=30, height=3, command=toggle_direct)
    btn_direct.pack(pady=10)

    root.protocol("WM_DELETE_WINDOW", lambda: (shutdown_motors(), root.destroy()))
    root.mainloop()

# ==============================================================
# MAIN ENTRY POINT
# ==============================================================

if __name__ == "__main__":
    # ⚠️ Update COM port if needed
    ser = serial.Serial("COM19", 115200, timeout=1)

    # Register failsafe shutdown
    atexit.register(shutdown_motors)
    signal.signal(signal.SIGINT, lambda s, f: sys.exit(0))
    signal.signal(signal.SIGTERM, lambda s, f: sys.exit(0))

    # Start YOLO loop in background
    threading.Thread(target=yolo_loop, args=(ser,), daemon=True).start()

    # Launch GUI
    build_gui()
