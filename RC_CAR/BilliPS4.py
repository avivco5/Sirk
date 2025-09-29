import pygame
import serial
import time

PORT = "COM26"   # שנה ל-COM של הטינזי שלך
BAUD = 9600

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No controller found!")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()
print("Connected to:", joystick.get_name())

def send_command(cmd, speed=0):
    if cmd in ["F", "B", "L", "R"]:
        line = f"{cmd} {speed}\n"
    elif cmd == "S":
        line = "S\n"
    else:
        return
    ser.write(line.encode())
    print("Sent:", line.strip())

try:
    while True:
        pygame.event.pump()

        lx = joystick.get_axis(0)   # שמאלה/ימינה
        ly = joystick.get_axis(1)   # קדימה/אחורה

        # Deadzone קטן
        if abs(lx) < 0.2: lx = 0
        if abs(ly) < 0.2: ly = 0

        # מהירות 0–500 (כדי לנצל את טווח ה-PWM בטינזי)
        speed_y = int(abs(ly) * 500)
        speed_x = int(abs(lx) * 500)

        if ly < 0:  # קדימה
            send_command("F", speed_y)
        elif ly > 0:  # אחורה
            send_command("B", speed_y)
        elif lx < 0:  # שמאלה במקום
            send_command("L", speed_x)
        elif lx > 0:  # ימינה במקום
            send_command("R", speed_x)
        else:
            send_command("S")

        time.sleep(0.05)

except KeyboardInterrupt:
    send_command("S")
    print("Stopped by user")
