import tkinter as tk
from tkinter import ttk
import socket
import time
import threading

# ---- UDP ----
PI_IP = "192.168.1.200"
PI_PORT = 6000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

SEND_HZ = 20.0
running = True

# ערכים
steer = 0.0
throttle = 0.0
b_states = [1, 1, 1]  # התחלה ב-1500 (מצב אמצע)

def send_loop():
    dt = 1.0 / SEND_HZ
    next_time = time.time()
    while running:
        msg = f"s:{steer:.3f} t:{throttle:.3f} b1:{b_states[0]} b2:{b_states[1]} b3:{b_states[2]}"
        sock.sendto(msg.encode("ascii"), (PI_IP, PI_PORT))
        #print("Sent:", msg)
        now = time.time()
        sleep_time = next_time - now
        if sleep_time > 0:
            time.sleep(sleep_time)
        next_time += dt

def on_steer(val):
    global steer
    steer = float(val)

def on_throttle(val):
    global throttle
    throttle = float(val)

def cycle_button(idx, label):
    # 0 -> 1 -> 2 -> 0
    b_states[idx] = (b_states[idx] + 1) % 3
    label.config(text=f"Button {idx+1}: {b_states[idx]}")

# ---- GUI ----
root = tk.Tk()
root.title("RC UDP Controller")

frm = ttk.Frame(root, padding=10)
frm.grid()

ttk.Label(frm, text="Steer [-1..1]").grid(column=0, row=0)
steer_slider = ttk.Scale(frm, from_=-1, to=1, orient="horizontal", command=on_steer)
steer_slider.set(0.0)
steer_slider.grid(column=1, row=0)

ttk.Label(frm, text="Throttle [-1..1]").grid(column=0, row=1)
throttle_slider = ttk.Scale(frm, from_=-1, to=1, orient="horizontal", command=on_throttle)
throttle_slider.set(0.0)
throttle_slider.grid(column=1, row=1)

btn_labels = []
for i in range(3):
    lbl = ttk.Label(frm, text=f"Button {i+1}: {b_states[i]}")
    lbl.grid(column=0, row=2+i)
    btn = ttk.Button(frm, text=f"Cycle B{i+1}", command=lambda idx=i, l=lbl: cycle_button(idx, l))
    btn.grid(column=1, row=2+i)
    btn_labels.append(lbl)

# ---- Thread ----
threading.Thread(target=send_loop, daemon=True).start()

root.protocol("WM_DELETE_WINDOW", lambda: (globals().update(running=False), root.destroy()))
root.mainloop()
