import sys
import time
import socket
import threading
import json
import math
import numpy as np

# --- ייבוא חובה לשדרוג ---
import cv2
from ultralytics import YOLO
from deep_sort_realtime.deepsort_tracker import DeepSort

from pymavlink import mavutil

# ייבוא MAVLink רגיל
try:
    from pymavlink.dialects.v20 import ardupilotmega as mavlink
except ImportError:
    from pymavlink.dialects.v20 import common as mavlink

# --- MAVLink Settings ---
MAVLINK_DEVICE = 'COM4'
MAVLINK_BAUD = 57600
MAVLINK_CONN = None
GUIDED_KEEPALIVE_PERIOD = 0.5

# --- Communication Ports ---
UDP_IP = '127.0.0.1'
UDP_PORT_TEL = 14550
UDP_PORT_BBOX = 9103
UDP_PORT_CMD = 9104

# --- YOLO / DeepSORT Settings (Global and Dynamic) ---
YOLO_CAM_INDEX = 0
YOLO_MODEL_NAME = 'yolov8n.pt'
YOLO_CONF = 0.25
YOLO_IOU = 0.50
# משתנה גלובלי שמעודכן ע"י ה-GUI
TARGET_CLASSES = [0]  # ברירת מחדל: [0] = אדם (person)
YOLO_SHOW_WINDOW = True

# --- Global State ---
yolo_runner_active = False
current_alt_guided = None
yolo_thread = None


# ================= MAVLink Helpers (פונקציות מתוקנות) =================

def set_mode(mode_name):
    """שולח פקודה לשינוי מצב טיסה לפי שם."""
    if MAVLINK_CONN is None: return False

    mode = MAVLINK_CONN.mode_mapping().get(mode_name)
    if mode is None:
        print(f"[MAV] Mode {mode_name} not recognized.")
        return False

    MAVLINK_CONN.set_mode(mode)
    print(f"[MAV] Changing mode to {mode_name} (ID: {mode})")
    return True


def arm_motors():
    """חימוש המנועים באמצעות שליחת COMMAND_LONG."""
    if MAVLINK_CONN is None: return False

    MAVLINK_CONN.mav.command_long_send(
        MAVLINK_CONN.target_system,
        MAVLINK_CONN.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    print("[MAV-CMD] Motors ARMED (sent COMMAND_LONG).")
    return True


def disarm_motors():
    """נטרול המנועים באמצעות שליחת COMMAND_LONG."""
    if MAVLINK_CONN is None: return False

    MAVLINK_CONN.mav.command_long_send(
        MAVLINK_CONN.target_system,
        MAVLINK_CONN.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print("[MAV-CMD] Motors DISARMED (sent COMMAND_LONG).")
    return True


def emergency_stop():
    """עצירת חירום: נטרול מנועים מיידי + ניסיון מעבר ל-LOITER/LAND."""
    if MAVLINK_CONN is None: return

    # 1. Kill switch (disarm) - משתמש בפונקציה המתוקנת
    disarm_motors()
    print("[MAV-CMD] !!! EMERGENCY STOP: Motors DISARMED !!!")

    # 2. Try to set to a safe mode (LOITER or LAND) for redundancy
    try:
        set_mode('LOITER')
    except Exception:
        try:
            set_mode('LAND')
        except Exception:
            pass

    print("[MAV-CMD] Emergency stop routine complete.")


# ----------------- הוספה: בקשת זרמי טלמטריה (תיקון VFR_HUD) -----------------
def request_telemetry_streams():
    """מבקש מהרחפן לשלוח את הזרמים הנדרשים (VFR_HUD, GPS) באופן קבוע."""
    global MAVLINK_CONN
    if MAVLINK_CONN is None:
        return

    # MAV_DATA_STREAM_POSITION = GPS (GLOBAL_POSITION_INT, GPS_RAW_INT)
    MAVLINK_CONN.mav.request_data_stream_send(
        MAVLINK_CONN.target_system,
        MAVLINK_CONN.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION,
        1,  # קצב: 1 הרץ
        1  # הפעלה
    )

    # MAV_DATA_STREAM_EXTRA1 = VFR_HUD (כולל גובה יחסי ומהירות)
    MAVLINK_CONN.mav.request_data_stream_send(
        MAVLINK_CONN.target_system,
        MAVLINK_CONN.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
        10,  # קצב: 10 הרץ
        1
    )
    print("[MAV] Requested VFR_HUD and GPS streams.")


# ----------------- go_to_gps (כפי שהיה, עם בדיקת None) -----------------
def go_to_gps(lat, lon, alt):
    """שולח פקודת Go-To למיקום GPS מוחלט (MAVLink_set_position_target_global_int_message מתוקן)."""
    global current_alt_guided
    if MAVLINK_CONN is None: return

    # --- התיקון הקריטי: ודא שכל הקואורדינטות והגובה אינם None ---
    if lat is None or lon is None or alt is None:
        # במקום לקרוס, אנחנו מדווחים ונמנעים משליחת הפקודה הלא-חוקית.
        print(f"[MAV-CMD] GOTO_GPS FAILED: Missing valid coordinate/alt (lat:{lat}, lon:{lon}, alt:{alt}).")
        return
    # -----------------------------------------------------------

    if MAVLINK_CONN.flightmode != 'GUIDED':
        if not set_mode('GUIDED'):
            return

    current_alt_guided = alt

    # הוספת YAW (0) ו-YAW_RATE (0) בסוף ההודעה כדי להתאים לגרסת הדיאלקט
    MAVLINK_CONN.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(
        int(time.time() * 1000),
        MAVLINK_CONN.target_system,
        MAVLINK_CONN.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,
        int(lat * 1e7),
        int(lon * 1e7),
        alt,
        0, 0, 0,  # Velocity components (Vx, Vy, Vz)
        0, 0, 0,  # Acceleration components (Ax, Ay, Az)
        0, 0  # YAW (0) and YAW_RATE (0)
    ))
    print(f"[MAV-CMD] GOTO Lat:{lat:.5f}, Lon:{lon:.5f}, Alt:{alt:.1f}m")


# ----------------- hold_here_current_alt (מעודכן: משתמש בגובה בטוח) -----------------
def hold_here_current_alt():
    """מורה לרחפן להישאר במיקום הנוכחי, באמצעות גובה עדכני או גובה בטוח."""
    global current_alt_guided
    if MAVLINK_CONN is None:
        print("[MAV-CMD] HOLD FAILED: No MAVLink connection.")
        return

    # 1. קבלת נתוני GPS עדכניים (Global Position)
    # נותנים לו 1.0 שניה לחפש את ההודעה כדי להבטיח את הנתון הכי עדכני
    msg = MAVLINK_CONN.recv_match(type=['GLOBAL_POSITION_INT'], blocking=True, timeout=1.0)

    # אם אין GPS Fix, lat ו-lon יהיו None (מה שיחסום את go_to_gps)
    lat, lon = (msg.lat / 1e7, msg.lon / 1e7) if msg else (None, None)

    # 2. קבלת גובה מ-VFR_HUD או שימוש בגובה בטוח
    alt = None
    alt_msg = MAVLINK_CONN.recv_match(type='VFR_HUD', blocking=True, timeout=0.1)

    if alt_msg:
        alt = alt_msg.alt
        print(f"[MAV] Altitude received via VFR_HUD: {alt:.1f}m")
    else:
        # אם VFR_HUD נכשל, משתמשים בגובה בטוח (5 מטר)
        alt = 5.0
        print(f"[MAV-CMD] HOLD WARNING: VFR_HUD failed. Using safe default altitude of {alt:.1f}m.")

    # 3. בדיקה סופית: אם אין GPS Fix, go_to_gps ייכשל, לכן נצא מראש עם לוג מפורש
    if lat is None or lon is None:
        print("[MAV-CMD] HOLD FAILED: Cannot execute HOLD, no current GPS Fix available (lat/lon are None).")
        return

    if MAVLINK_CONN.flightmode != 'GUIDED':
        if not set_mode('GUIDED'):
            return

    current_alt_guided = alt

    # קורא לפונקציה go_to_gps הבטוחה
    go_to_gps(lat, lon, alt)
    print(f"[MAV-CMD] HOLD @ Lat:{lat:.5f}, Lon:{lon:.5f}, Alt:{alt:.1f}m")


def bump_alt_guided(delta_alt):
    """משנה את גובה היעד במצב GUIDED."""
    global current_alt_guided
    if MAVLINK_CONN is None: return

    if current_alt_guided is None:
        print("[MAV-CMD] ALT_BUMP failed: current_alt_guided is not set. Use HOLD first.")
        return

    new_alt = max(5.0, current_alt_guided + delta_alt)

    msg = MAVLINK_CONN.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1.0)
    # אם אין GPS, lat/lon יהיו None
    lat, lon = (msg.lat / 1e7, msg.lon / 1e7) if msg else (None, None)

    if lat is None or lon is None:
        print("[MAV-CMD] ALT_BUMP failed: No recent GPS data available.")
        return

    go_to_gps(lat, lon, new_alt)
    print(f"[MAV-CMD] ALT_BUMP: New alt target: {new_alt:.1f}m")


# ----------------- mav_connect (מעודכן: קורא ל-request_telemetry_streams) -----------------
def mav_connect():
    global MAVLINK_CONN
    print(f"[MAV] Connecting to {MAVLINK_DEVICE} @ {MAVLINK_BAUD}...")
    try:
        MAVLINK_CONN = mavutil.mavlink_connection(MAVLINK_DEVICE, baud=MAVLINK_BAUD)
        MAVLINK_CONN.wait_heartbeat(timeout=10)
        print(f"[MAV] Heartbeat received from sys {MAVLINK_CONN.target_system}, comp {MAVLINK_CONN.target_component}")

        # --- הוספת הבקשה לזרמי טלמטריה! ---
        request_telemetry_streams()
        # -----------------------------------

        return True
    except Exception as e:
        print(f"[MAV] Connection FAILED: {e}")
        return False


def mav_forwarding_loop():
    """מקבל MAVLink מ-COM4 ומשדר הכל ל-UDP 14550."""
    if MAVLINK_CONN is None: return

    print("[MAV] Starting telemetry forwarding loop...")

    try:
        UDP_SOCK_TEL = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    except Exception as e:
        print(f"[UDP-TEL] Failed to initialize TEL UDP socket: {e}")
        return

    period = 1.0 / 50.0

    while True:
        start_time = time.time()
        try:
            msg = MAVLINK_CONN.recv_match(blocking=False)

            if msg:
                raw_bytes = msg.get_msgbuf()

                if raw_bytes:
                    UDP_SOCK_TEL.sendto(raw_bytes, (UDP_IP, UDP_PORT_TEL))

            elapsed = time.time() - start_time
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        except Exception as e:
            if "Bad file descriptor" not in str(e):
                print(f"[MAV-TX] Error in forwarding loop: {e}")
            time.sleep(0.1)


# ================= YOLO Control Functions (ללא שינוי) =================

def start_yolo_detection():
    # ... (קוד YOLO נשאר ללא שינוי)
    global yolo_runner_active, yolo_thread
    if yolo_runner_active:
        print("[YOLO] Detection is already active.")
        return

    def yolo_execution_target():
        global yolo_runner_active, TARGET_CLASSES
        print(f"[YOLO] Loading model, camera {YOLO_CAM_INDEX}, and DeepSORT...")
        print(f"[YOLO] Initial filter classes: {TARGET_CLASSES}")

        # --- אתחול חומרה ומודלים ---
        try:
            # 1. מצלמה
            cap = cv2.VideoCapture(YOLO_CAM_INDEX)
            if not cap.isOpened():
                raise IOError(f"Cannot open camera index {YOLO_CAM_INDEX}")

            # 2. YOLO
            model = YOLO('yolov8n.pt')

            # 3. DeepSORT
            tracker = DeepSort(max_age=5, nms_max_overlap=0.7, embedder='mobilenet', half=True)
            UDP_SOCK_BBOX = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            yolo_runner_active = True
            print("[YOLO] All modules initialized. Starting tracking loop.")

        except Exception as e:
            print(f"[YOLO-INIT] FAILED to initialize components: {e}")
            yolo_runner_active = False
            return

            # --- לולאת זיהוי ומעקב ---
        while yolo_runner_active:
            start_time = time.time()
            ret, frame = cap.read()

            if not ret:
                print("[YOLO] Failed to read frame from camera. Retrying...")
                time.sleep(0.1)
                continue

            # 1. זיהוי YOLO
            results = model.predict(source=frame, conf=YOLO_CONF, iou=YOLO_IOU,
                                    classes=TARGET_CLASSES,
                                    verbose=False)

            detections = []
            for r in results:
                for box in r.boxes:
                    x1, y1, x2, y2 = [int(i) for i in box.xyxy[0]]
                    confidence = float(box.conf[0])
                    class_id = int(box.cls[0])
                    detections.append(([x1, y1, x2 - x1, y2 - y1], confidence, class_id))

            # 2. מעקב DeepSORT
            tracks = tracker.update_tracks(detections, frame=frame)

            # 3. מציאת היעד הראשי ושידור BBox
            found_target = False
            if tracks:
                primary_track = tracks[0]

                if primary_track.is_confirmed():
                    ltrb = primary_track.to_ltrb()  # Left, Top, Right, Bottom
                    track_id = primary_track.track_id

                    target_bbox = [ltrb[0], ltrb[1], ltrb[2], ltrb[3], track_id]

                    message = json.dumps({"bbox": target_bbox})
                    UDP_SOCK_BBOX.sendto(message.encode('utf-8'), (UDP_IP, UDP_PORT_BBOX))

                    found_target = True

                    # 4. ציור על המסך (אם מופעל)
                    if YOLO_SHOW_WINDOW:
                        x1_int, y1_int, x2_int, y2_int = [int(p) for p in ltrb]
                        cv2.rectangle(frame, (x1_int, y1_int), (x2_int, y2_int), (0, 255, 0), 2)
                        cv2.putText(frame, f"ID: {track_id}", (x1_int, y1_int - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

            # שליחת הודעת אובדן (NULL)
            if not found_target:
                message = json.dumps({"bbox": None})
                UDP_SOCK_BBOX.sendto(message.encode('utf-8'), (UDP_IP, UDP_PORT_BBOX))

            # 5. הצגת חלון
            if YOLO_SHOW_WINDOW:
                cv2.putText(frame, f"FPS: {1 / (time.time() - start_time):.1f}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.imshow("YOLO/DeepSORT Tracker", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    yolo_runner_active = False

            time.sleep(0.001)

            # --- ניקוי ---
        cap.release()
        cv2.destroyAllWindows()
        yolo_runner_active = False
        print("[YOLO] Detection loop finished and cleaned up.")

    # יצירה והפעלה של ה-Thread
    yolo_thread = threading.Thread(target=yolo_execution_target, daemon=True)
    yolo_thread.start()
    print("[YOLO] Started detection thread.")


def stop_yolo_detection():
    # ... (קוד YOLO נשאר ללא שינוי)
    global yolo_runner_active, yolo_thread
    if not yolo_runner_active:
        print("[YOLO] Detection is already inactive.")
        return

    yolo_runner_active = False
    print("[YOLO] Sent stop signal to detection thread...")


# ================= MAVLink Keepalive (מעודכן - עם בדיקת GPS Fix למניעת הצפת לוגים) =================

def guided_keepalive_loop():
    """שולח פקודת Go-To מחזורית למיקום האחרון במצב GUIDED, כדי למנוע יציאה אוטומטית."""
    global current_alt_guided
    last_gps_warning_time = 0

    while True:
        if MAVLINK_CONN and MAVLINK_CONN.flightmode == 'GUIDED' and current_alt_guided is not None:

            # 1. נסה לקבל מיקום GPS עדכני
            msg = MAVLINK_CONN.recv_match(type='GLOBAL_POSITION_INT', blocking=False)

            # אם אין GPS (msg is None), lat ו-lon יהיו None.
            lat, lon = (msg.lat / 1e7, msg.lon / 1e7) if msg else (None, None)
            alt = current_alt_guided

            # 2. אם יש GPS Fix, שלח פקודה. אם לא, הדפס אזהרה שקטה.
            if lat is not None and lon is not None:
                go_to_gps(lat, lon, alt)
            else:
                current_time = time.time()
                # הדפס אזהרה רק פעם אחת כל 5 שניות כדי למנוע הצפה של המסך
                if current_time - last_gps_warning_time > 5.0:
                    print("[MAV-CMD] WARNING: Skipping GUIDED keepalive, no GPS Fix available.")
                    last_gps_warning_time = current_time

        time.sleep(GUIDED_KEEPALIVE_PERIOD)


# ================= Command Listener (ללא שינוי בפקודות) =================

def command_listener_loop():
    """מאזין לפורט 9104 לפקודות שליטה מה-GUI ומבצע אותן."""
    global TARGET_CLASSES
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((UDP_IP, UDP_PORT_CMD))
        print(f"[CMD] Listening for commands on {UDP_IP}:{UDP_PORT_CMD}")
    except Exception as e:
        print(f"[CMD] Failed to bind command listener: {e}")
        return

    while True:
        try:
            data, addr = sock.recvfrom(1024)
            cmd_obj = json.loads(data.decode('utf-8'))
            cmd = cmd_obj.get("command")
            cmd_data = cmd_obj.get("data", {})

            if cmd == "GOTO_GPS":
                # משתמש בפונקציה go_to_gps המתוקנת
                go_to_gps(cmd_data.get("lat"), cmd_data.get("lon"), cmd_data.get("alt"))
            elif cmd == "HOLD_HERE":
                hold_here_current_alt()
            elif cmd == "ALT_BUMP":
                bump_alt_guided(cmd_data.get("delta"))
            elif cmd == "START_YOLO":
                start_yolo_detection()
            elif cmd == "STOP_YOLO":
                stop_yolo_detection()

            elif cmd == "SET_CLASSES":
                classes_str = cmd_data.get("classes", "")

                try:
                    new_classes = [int(c.strip()) for c in classes_str.split(',') if c.strip().isdigit()]

                    if new_classes:
                        TARGET_CLASSES = new_classes
                        print(f"[CMD] Updated TARGET_CLASSES to: {TARGET_CLASSES}")
                    else:
                        TARGET_CLASSES = []
                        print("[CMD] SET_CLASSES: Received empty or invalid filter. Detection list is empty.")

                except Exception as e:
                    print(f"[CMD] SET_CLASSES parsing error: {e}. Keeping old classes: {TARGET_CLASSES}")

            elif cmd == "SET_MODE":
                set_mode(cmd_data.get("mode"))

            # --- פקודות חימוש וחירום ---
            elif cmd == "ARM":
                arm_motors()
            elif cmd == "DISARM":
                disarm_motors()
            elif cmd == "EMERGENCY_STOP":
                emergency_stop()
            # ---------------------------------

            else:
                print(f"[CMD] Unknown command received: {cmd}")

        except Exception as e:
            # ה-Error parsing or executing command: argument out of range
            # מופיע כאן כאשר ה-GUI שולח פקודה עם פרמטרים חסרים/לא תקינים.
            # זה לא קריטי, כיוון שה-go_to_gps חוסם, אך כדי לתקן את ההודעה עצמה יש לבדוק
            # את הקוד של ה-GUI (כנראה '5070FlaskGuiGimini.py').
            print(f"[CMD] Error parsing or executing command: {e}")


# ================= Main Execution =================

if __name__ == "__main__":

    if not mav_connect():
        print("Starting without MAVLink connection.")

    # הפעלת ה-Threads הבסיסיים
    threading.Thread(target=mav_forwarding_loop, daemon=True).start()
    threading.Thread(target=command_listener_loop, daemon=True).start()
    threading.Thread(target=guided_keepalive_loop, daemon=True).start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Tracker shutdown initiated.")

    print("Tracker shutdown complete.")