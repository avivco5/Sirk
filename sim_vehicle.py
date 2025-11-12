import socket
import json

# הגדרות תקשורת (כפי שהוגדרו ב-tracker_yolo_main.py)
UDP_IP = '127.0.0.1'
UDP_PORT_CMD = 9104

# יצירת פקודת START_YOLO
cmd_obj = {"command": "START_YOLO", "data": {}}
message = json.dumps(cmd_obj).encode('utf-8')

# שליחת הפקודה
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(message, (UDP_IP, UDP_PORT_CMD))
print(f"Sent command: {cmd_obj}")