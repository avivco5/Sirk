from pymultiwii import MultiWii
import time

# שים את הפורט הנכון (במקרה שלך COM18)
board = MultiWii("COM18")

def send_rc(roll=1500, pitch=1500, throttle=1000, yaw=1500,
            aux1=1000, aux2=1000, aux3=1000, aux4=1000):
    rc_channels = [roll, pitch, throttle, yaw, aux1, aux2, aux3, aux4]
    # H = unsigned short (16bit). יש 8 ערוצים → "H"*8
    board.sendCMD(16, MultiWii.SET_RAW_RC, rc_channels, "H"*8)

print("Arming...")
send_rc(throttle=1000, aux1=2000)   # חימוש
time.sleep(2)

print("Takeoff...")
for t in range(1000, 1600, 100):
    send_rc(throttle=t, aux1=2000)
    time.sleep(0.5)

print("Disarm...")
send_rc(throttle=1000, aux1=1000)   # נטרול
