import socket
import json
import time

# --- הגדרות תקשורת ---
UDP_IP = "127.0.0.1"
UDP_PORT_CMD = 9104  # הפורט ש-tracker_yolo_main2.py מאזין לו לפקודות שליטה

# יצירת סוקט UDP
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def send_command(cmd_name, params=None, delay=1.0):
    """שולח פקודת JSON ומחכה."""
    command_data = {
        "cmd": cmd_name,
        "params": params if params is not None else {}
    }

    message = json.dumps(command_data).encode('utf-8')
    sock.sendto(message, (UDP_IP, UDP_PORT_CMD))

    print(f"[SENT] {cmd_name} - {params if params else ''}")
    time.sleep(delay)  # המתן כדי לאפשר לרחפן לבצע את הפקודה


# ==============================================================
# רצף הפקודות
# ==============================================================

print("--- מתחיל רצף פקודות לרחפן (דרך UDP 9104) ---")

# 1. שינוי מצב ל-GUIDED (דרוש להמראה/ניווט)
send_command("SET_MODE", {"mode": "GUIDED"}, delay=2.0)

# 2. דריכת מנועים (ARM)
send_command("ARM", delay=3.0)

# 3. המראה לגובה 5 מטרים
send_command("TAKEOFF", {"alt": 5.0}, delay=10.0)  # המתן זמן רב יותר להשלמת ההמראה

# 4. שינוי מצב ל-HOLD (שמירת מיקום נוכחי באוויר)
send_command("SET_MODE", {"mode": "HOLD"}, delay=3.0)

# 5. עצירת חירום (ניתוק מנועים - DISARM)
# send_command("DISARM", delay=3.0) # אם נדרש לנחות, השתמש ב-RTL או LAND

print("--- סיום רצף הפקודות. ---")

# ==============================================================
# דוגמה נוספת: שליחת הרחפן לנקודת GPS ספציפית
# (החלף את הערכים במיקומי LAT/LON רצויים)
# ==============================================================

# print("\n--- שליחת הרחפן לנקודת ציון ---")
# send_command("GOTO_GPS", {"lat": 32.08530, "lon": 34.78177, "alt": 15.0}, delay=20.0)