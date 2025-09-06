# udp_setpos_client.py — שולח סט-פוינטים ב-UDP JSON ל-Isaac
# Usage: python udp_setpos_client.py
import socket, json, time

SIM_IP = "127.0.0.1"  # אם Isaac על מחשב אחר, שים את ה-IP שלו
SIM_PORT = 6000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_setpos(x, y, alt):
    msg = {"cmd":"setpos", "x": float(x), "y": float(y), "alt": float(alt)}
    data = (json.dumps(msg)).encode("utf-8")
    sock.sendto(data, (SIM_IP, SIM_PORT))
    print(f"[TX] setpos -> x={x:.2f} y={y:.2f} alt={alt:.2f}")

def send_nudge(dx=0.0, dy=0.0, dalt=0.0):
    msg = {"cmd":"nudge", "dx": float(dx), "dy": float(dy), "dalt": float(dalt)}
    data = (json.dumps(msg)).encode("utf-8")
    sock.sendto(data, (SIM_IP, SIM_PORT))
    print(f"[TX] nudge  -> dx={dx:.2f} dy={dy:.2f} dalt={dalt:.2f}")

# ---- בדיקת רצף כמו אצלך בלוג ----
seq = [
    (0.00, -3.00, 2.00),
    (0.00, 3.50, 4.00),
    (1.00, -4.00, 1.00),
    (3.50, -6.00, 9.00),
    (0.00, 4.00, 2.00),
    (-2.50, 2.00, 4.00),
    (2.00, -4.00, 2.00),
]
for (x,y,alt) in seq:
    send_setpos(x,y,alt)
    time.sleep(5.2)

# דוגמה לשינוי יחסי:
# send_nudge(dx=0.5)      # קדימה 0.5m
# send_nudge(dy=-0.5)     # שמאלה 0.5m
# send_nudge(dalt=0.2)    # גובה +0.2m

