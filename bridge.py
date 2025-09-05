import socket
import time

UDP_IP = "10.0.0.28"   # כתובת ה־VM (Ubuntu)
UDP_PORT = 5006

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# שלב 1: ודא שכל הערוצים ניטרליים
sock.sendto(b"ROLL 1500", (UDP_IP, UDP_PORT))
sock.sendto(b"PITCH 1500", (UDP_IP, UDP_PORT))
sock.sendto(b"YAW 1500", (UDP_IP, UDP_PORT))
sock.sendto(b"THROTTLE 1000", (UDP_IP, UDP_PORT))
time.sleep(1)  # תן לסימולטור זמן לקלוט

# שלב 2: חימוש
sock.sendto(b"ARM", (UDP_IP, UDP_PORT))
time.sleep(2)

# שלב 3: העלאת מצערת והטסה
sock.sendto(b"THROTTLE 1200", (UDP_IP, UDP_PORT))
time.sleep(1)
sock.sendto(b"PITCH 1600", (UDP_IP, UDP_PORT))
