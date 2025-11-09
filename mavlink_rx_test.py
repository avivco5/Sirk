import sys
import socket
import time
from pymavlink import mavutil

# [!!! התיקון הקריטי: ייבוא הדיאלקט הספציפי של MAVLink !!!]
# זה מאפשר להשתמש במחלקת ה-MAVLink הטהורה (parse_buffer)
try:
    from pymavlink.dialects.v20 import ardupilotmega as mavlink
except ImportError:
    # אם ardupilotmega לא נטען (מסיבה כלשהי), נשתמש בדיאלקט הנפוץ
    from pymavlink.dialects.v20 import common as mavlink

# --- הגדרות ---
UDP_IP = "127.0.0.1"  # ה-IP עליו מאזינים (localhost)
UDP_PORT = 14550  # הפורט ממנו מקבלים את הטלמטריה
MASTER_SYSTEM = 1

# --- גלובלי ---
udp_sock = None
mav = None


def setup_receiver():
    global udp_sock, mav

    try:
        # 1. יצירת סוקט UDP להאזנה
        udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_sock.bind((UDP_IP, UDP_PORT))

        # 2. יצירת אובייקט MAVLink טהור (לפענוח)
        mav = mavlink.MAVLink(
            file=None,
            srcSystem=MASTER_SYSTEM,
            srcComponent=1
        )

        print(f"[RX] Listening for MAVLink packets on UDP {UDP_IP}:{UDP_PORT}")
        return True
    except Exception as e:
        print(f"[RX] Failed to set up receiver: {e}")
        return False


def telemetry_receive_loop():
    if not udp_sock or not mav:
        print("[RX] Cannot start receive loop: setup failed.")
        return

    last_print = time.time()

    while True:
        try:
            # 1. קבלת בייטים גולמיים מה-UDP
            raw_bytes, addr = udp_sock.recvfrom(2048)

            # 2. שימוש ב-parse_buffer של אובייקט MAVLink טהור
            msgs = mav.parse_buffer(raw_bytes)

            # 3. בדיקת הודעות מפוענחות
            if msgs:
                for msg in msgs:
                    # הדפסת סטאטוס כל 2 שניות
                    if (time.time() - last_print) > 2.0:
                        if msg.get_type() == 'VFR_HUD':
                            print(f"[RX] VFR_HUD: Alt={msg.alt:.2f}m, GS={msg.groundspeed:.1f}m/s")
                            last_print = time.time()

        except Exception as e:
            print(f"[RX] Error in receive loop: {e}")
            time.sleep(0.01)


if __name__ == "__main__":
    if setup_receiver():
        try:
            telemetry_receive_loop()
        except KeyboardInterrupt:
            print("[RX] Exiting...")
    else:
        sys.exit(1)