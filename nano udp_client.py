import socket

UDP_IP = "10.0.0.28"   # כתובת ה־VM שלך
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
message = "Hello from Windows!".encode()

sock.sendto(message, (UDP_IP, UDP_PORT))
print("✅ Message sent")
