# -*- coding: utf-8 -*-
import tkinter as tk
from tkinter import ttk
import socket
import time

# ==== Config ====
#PI_IP = "192.168.1.200"   # set to your Raspberry Pi IP
PI_IP = "10.0.0.30"
PI_PORT = 6000
SEND_RATE_HZ = 20.0
STEP = 0.1  # keyboard step size

class RCControllerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("RC Controller GUI with Keyboard (Step Control)")

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # variables
        self.steer_var = tk.DoubleVar(value=0.0)
        self.throttle_var = tk.DoubleVar(value=0.0)
        self.b1_state = 0
        self.b2_state = 0
        self.b3_state = 0

        # sliders frame
        frame = ttk.Frame(root, padding=10)
        frame.grid(row=0, column=0)

        # Steer slider
        ttk.Label(frame, text="Steer [-1..1]").grid(row=0, column=0, sticky="w")
        self.steer_slider = ttk.Scale(
            frame, from_=-1.0, to=1.0, orient="horizontal",
            variable=self.steer_var, command=lambda e: self.send_packet()
        )
        self.steer_slider.grid(row=1, column=0, sticky="ew")
        self.steer_slider.bind("<ButtonRelease-1>", lambda e: self.reset_steer())

        # Throttle slider
        ttk.Label(frame, text="Throttle [-1..1]").grid(row=2, column=0, sticky="w")
        self.throttle_slider = ttk.Scale(
            frame, from_=-1.0, to=1.0, orient="horizontal",
            variable=self.throttle_var, command=lambda e: self.send_packet()
        )
        self.throttle_slider.grid(row=3, column=0, sticky="ew")
        self.throttle_slider.bind("<ButtonRelease-1>", lambda e: self.reset_throttle())

        # buttons for switches
        self.b1_btn = ttk.Button(frame, text="B1: 0", command=self.toggle_b1)
        self.b1_btn.grid(row=4, column=0, pady=5, sticky="ew")

        self.b2_btn = ttk.Button(frame, text="B2: 0", command=self.toggle_b2)
        self.b2_btn.grid(row=5, column=0, pady=5, sticky="ew")

        self.b3_btn = ttk.Button(frame, text="B3: 0", command=self.toggle_b3)
        self.b3_btn.grid(row=6, column=0, pady=5, sticky="ew")

        # neutral button
        self.neutral_btn = ttk.Button(frame, text="Neutral", command=self.set_neutral)
        self.neutral_btn.grid(row=7, column=0, pady=5, sticky="ew")

        # keyboard bindings (step control)
        root.bind("<Left>",  lambda e: self.steer_left())
        root.bind("<Right>", lambda e: self.steer_right())
        root.bind("<Up>",    lambda e: self.throttle_up())
        root.bind("<Down>",  lambda e: self.throttle_down())
        root.bind("<space>", lambda e: self.set_neutral())
        root.bind("1",       lambda e: self.toggle_b1())
        root.bind("2",       lambda e: self.toggle_b2())
        root.bind("3",       lambda e: self.toggle_b3())

        # start send loop
        self.last_send = 0
        self.update_loop()

    # === Steer actions ===
    def steer_left(self):
        v = self.steer_var.get() - STEP
        if v < -1.0:
            v = -1.0
        self.steer_var.set(v)
        self.send_packet()

    def steer_right(self):
        v = self.steer_var.get() + STEP
        if v > 1.0:
            v = 1.0
        self.steer_var.set(v)
        self.send_packet()

    def reset_steer(self):
        self.steer_var.set(0.0)
        self.send_packet()

    # === Throttle actions ===
    def throttle_up(self):
        v = self.throttle_var.get() + STEP
        if v > 1.0:
            v = 1.0
        self.throttle_var.set(v)
        self.send_packet()

    def throttle_down(self):
        v = self.throttle_var.get() - STEP
        if v < -1.0:
            v = -1.0
        self.throttle_var.set(v)
        self.send_packet()

    def reset_throttle(self):
        self.throttle_var.set(0.0)
        self.send_packet()

    # === Neutral ===
    def set_neutral(self):
        self.steer_var.set(0.0)
        self.throttle_var.set(0.0)
        self.send_packet()

    # === Switches ===
    def toggle_b1(self):
        self.b1_state = (self.b1_state + 1) % 3
        self.b1_btn.config(text=f"B1: {self.b1_state}")
        self.send_packet()

    def toggle_b2(self):
        self.b2_state = (self.b2_state + 1) % 3
        self.b2_btn.config(text=f"B2: {self.b2_state}")
        self.send_packet()

    def toggle_b3(self):
        self.b3_state = (self.b3_state + 1) % 3
        self.b3_btn.config(text=f"B3: {self.b3_state}")
        self.send_packet()

    # === UDP Send ===
    def build_packet(self):
        s = self.steer_var.get()
        t = self.throttle_var.get()
        b1 = self.b1_state
        b2 = self.b2_state
        b3 = self.b3_state
        return f"s:{s:.2f} t:{t:.2f} b1:{b1} b2:{b2} b3:{b3}"

    def send_packet(self):
        now = time.time()
        if now - self.last_send >= 1.0 / SEND_RATE_HZ:
            data = self.build_packet().encode("ascii")
            self.sock.sendto(data, (PI_IP, PI_PORT))
            self.last_send = now

    def update_loop(self):
        self.send_packet()
        self.root.after(int(1000 / SEND_RATE_HZ), self.update_loop)


if __name__ == "__main__":
    root = tk.Tk()
    app = RCControllerGUI(root)
    root.mainloop()
