import sys
import time
import socket
from pymavlink import mavutil  # מייבא רק את מה שצריך
import threading

# --- הגדרות ---
SERIAL_PORT = 'COM4'  # היציאה הטורית הפיזית
BAUD_RATE = 57600
UDP_IP = "127.0.0.1"  # כתובת ה-IP המקומית
UDP_PORT = 14550  # הפורט אליו נשלחת הטלמטריה
SEND_RATE_HZ = 50.0  # קצב שליחת ההודעות (50 הרץ)

# --- גלובלי ---
the_connection = None
target_addr = (UDP_IP, UDP_PORT)
udp_sock = None


def mav_connect():
    global the_connection
    print(f"[TX] Connecting to MAVLink on {SERIAL_PORT} @ {BAUD_RATE}...")
    try:
        the_connection = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE)
        the_connection.wait_heartbeat(timeout=5)
        print(
            f"[TX] Heartbeat received from sys {the_connection.target_system}, comp {the_connection.target_component}")
        return True
    except Exception as e:
        print(f"[TX] Connection failed: {e}")
        return False


def setup_udp():
    global udp_sock
    try:
        udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print(f"[TX] UDP socket ready, sending to {UDP_IP}:{UDP_PORT}")
        return True
    except Exception as e:
        print(f"[TX] Failed to set up UDP socket: {e}")
        return False


def telemetry_transmit_loop():
    global the_connection
    if not the_connection or not udp_sock:
        print("[TX] Cannot start transmit loop: connection or UDP not ready.")
        return

    print("[TX] Starting telemetry forward loop...")

    period = 1.0 / SEND_RATE_HZ

    while True:
        start_time = time.time()
        try:
            msg = the_connection.recv_match(blocking=False)

            if msg:
                raw_bytes = msg.get_msgbuf()

                if raw_bytes:
                    udp_sock.sendto(raw_bytes, target_addr)

            elapsed = time.time() - start_time
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        except Exception as e:
            print(f"[TX] Error in transmit loop: {e}")
            time.sleep(1)


if __name__ == "__main__":
    if mav_connect() and setup_udp():
        try:
            telemetry_transmit_loop()
        except KeyboardInterrupt:
            print("[TX] Exiting...")
    else:
        sys.exit(1)