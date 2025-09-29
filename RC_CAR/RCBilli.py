import serial
import time
from pynput import keyboard

# ===== הגדרות =====
PORT = "COM26"      # החלף ל-COM הנכון אצלך (/dev/ttyACM0 בלינוקס)
BAUD = 9600
SPEED = 200        # מהירות ברירת מחדל (0-255)

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)  # לחכות ל-Teensy

def send_command(cmd, speed=0):
    if cmd in ["F", "B", "L", "R"]:
        command_str = f"{cmd} {speed}\n"
    elif cmd == "S":
        command_str = "S\n"
    else:
        return
    ser.write(command_str.encode())
    print("Sent:", command_str.strip())

def on_press(key):
    try:
        if key == keyboard.Key.up:
            send_command("F", SPEED)
        elif key == keyboard.Key.down:
            send_command("B", SPEED)
        elif key == keyboard.Key.left:
            send_command("L", SPEED)
        elif key == keyboard.Key.right:
            send_command("R", SPEED)
        elif key == keyboard.Key.space:
            send_command("S")
    except Exception as e:
        print("Error:", e)

def on_release(key):
    # כשמרפים מהחץ -> עצירה
    if key in [keyboard.Key.up, keyboard.Key.down, keyboard.Key.left, keyboard.Key.right]:
        send_command("S")

    # ESC = יציאה מהתוכנית
    if key == keyboard.Key.esc:
        return False

print("Use arrow keys to drive, SPACE to stop, ESC to exit.")
with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()
