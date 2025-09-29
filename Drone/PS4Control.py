#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Fake RC Sticks GUI + PS4 Joystick (COM14 @115200)
- GUI סליידרים + שלט PS4
- Throttle (CH3): חצי עליון בלבד + Deadzone
- שליטה גם על CH5 ו-CH7 (סרוואים)
- ARM/DISARM גם דרך כפתורי PS4 (Cross=ARM, Circle=DISARM)
- מציג מתח/זרם/אחוז סוללה ב-GUI
- מנסה להתחבר מחדש לשלט PS4 אם לא מחובר או מתנתק
"""

import time, threading, tkinter as tk
from tkinter import ttk, messagebox
from pymavlink import mavutil
import pygame

DEVICE = "COM14"
BAUD   = 115200
RC_MIN, RC_MAX, RC_MID = 1000, 2000, 1500
SEND_HZ = 20.0
DEADZONE = 0.1

def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v
def apply_deadzone(val, dz=DEADZONE): return 0.0 if abs(val) < dz else val

class FakeSticks:
    def __init__(self, root):
        self.root = root
        self.root.title("Fake RC Sticks + PS4 (COM14 @115200)")

        # Connect MAVLink
        try:
            self.m = mavutil.mavlink_connection(DEVICE, baud=BAUD)
            self.m.wait_heartbeat(timeout=5)
            ttk.Label(root, text=f"Connected: sys "
                      f"{self.m.target_system}, comp {self.m.target_component}").pack()
        except Exception as e:
            messagebox.showerror("MAVLink", f"Connect failed: {e}")
            root.destroy(); return

        # Init pygame
        pygame.init()
        pygame.joystick.init()
        self.js = None
        self._try_connect_joystick()

        # RC sticks
        self.roll  = tk.IntVar(value=RC_MID)
        self.pitch = tk.IntVar(value=RC_MID)
        self.thr   = tk.IntVar(value=RC_MIN)
        self.yaw   = tk.IntVar(value=RC_MID)

        # Servo vars
        self.servo5 = tk.IntVar(value=1500)
        self.servo7 = tk.IntVar(value=1500)

        # Battery vars
        self.batt_v   = tk.StringVar(value="--.- V")
        self.batt_a   = tk.StringVar(value="--.- A")
        self.batt_pct = tk.StringVar(value="-- %")

        # RC sliders
        lf = ttk.LabelFrame(root, text="RC Sticks", padding=8)
        lf.pack(fill="x", padx=8, pady=6)
        self._mk_slider(lf,"Roll (CH1)", self.roll, RC_MID)
        self._mk_slider(lf,"Pitch (CH2)",self.pitch,RC_MID)
        self._mk_slider(lf,"Throttle (CH3)",self.thr,RC_MIN)
        self._mk_slider(lf,"Yaw (CH4)",   self.yaw,RC_MID)

        # Servo sliders
        ls = ttk.LabelFrame(root, text="Servos (CH5 & CH7)", padding=8)
        ls.pack(fill="x", padx=8, pady=6)
        self._mk_slider(ls,"Servo CH5", self.servo5, 1500)
        self._mk_slider(ls,"Servo CH7", self.servo7, 1500)

        # Battery info
        lb = ttk.LabelFrame(root, text="Battery", padding=8)
        lb.pack(fill="x", padx=8, pady=6)
        ttk.Label(lb, text="Voltage:").grid(row=0,column=0,sticky="w")
        ttk.Label(lb, textvariable=self.batt_v).grid(row=0,column=1,sticky="w")
        ttk.Label(lb, text="Current:").grid(row=1,column=0,sticky="w")
        ttk.Label(lb, textvariable=self.batt_a).grid(row=1,column=1,sticky="w")
        ttk.Label(lb, text="Remaining:").grid(row=2,column=0,sticky="w")
        ttk.Label(lb, textvariable=self.batt_pct).grid(row=2,column=1,sticky="w")

        # Buttons
        btns = ttk.Frame(root, padding=8); btns.pack(fill="x")
        ttk.Button(btns,text="Force ARM", command=self.force_arm).pack(
            side="left",expand=True,fill="x",padx=4)
        ttk.Button(btns,text="DISARM",    command=self.disarm).pack(
            side="left",expand=True,fill="x",padx=4)
        ttk.Button(btns,text="Reset All", command=self.reset_all).pack(
            side="left",expand=True,fill="x",padx=4)
        ttk.Button(btns,text="Exit",      command=self.on_close).pack(
            side="left",expand=True,fill="x",padx=4)

        # Thread
        self.running = True
        self.sender = threading.Thread(target=self._send_loop, daemon=True)
        self.sender.start()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.geometry("640x560")

    def _mk_slider(self,parent,label,var,reset_val):
        row=ttk.Frame(parent); row.pack(fill="x", pady=4)
        ttk.Label(row,text=label,width=12).pack(side="left")
        s=ttk.Scale(row,from_=RC_MIN,to=RC_MAX,orient="horizontal",variable=var)
        s.pack(side="left",fill="x",expand=True,padx=8)
        ttk.Label(row,textvariable=var,width=5).pack(side="left")
        ttk.Button(row,text="Reset",
                   command=lambda v=var,rv=reset_val: v.set(rv)).pack(side="left",padx=4)

    def _try_connect_joystick(self):
        pygame.joystick.quit()
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            self.js = pygame.joystick.Joystick(0)
            self.js.init()
            print(f"[PS4] Connected: {self.js.get_name()}")
        else:
            self.js = None
            print("[PS4] No joystick detected")

    def _send_override(self):
        try:
            self.m.mav.rc_channels_override_send(
                self.m.target_system, 1,
                int(self.roll.get()),   # CH1
                int(self.pitch.get()),  # CH2
                int(self.thr.get()),    # CH3
                int(self.yaw.get()),    # CH4
                int(self.servo5.get()), # CH5
                65535,                  # CH6
                int(self.servo7.get()), # CH7
                65535                   # CH8
            )
        except Exception as e:
            print("[RC_OVERRIDE] error:", e)

    def _poll_battery(self):
        msg = self.m.recv_match(
            type=["SYS_STATUS","BATTERY_STATUS"], blocking=False)
        if not msg: return
        if msg.get_type() == "SYS_STATUS":
            voltage = msg.voltage_battery / 1000.0
            current = msg.current_battery / 100.0
            remaining = msg.battery_remaining
            self.batt_v.set(f"{voltage:.1f} V")
            if current > -9000:
                self.batt_a.set(f"{current:.1f} A")
            if remaining >= 0:
                self.batt_pct.set(f"{remaining} %")
        elif msg.get_type() == "BATTERY_STATUS":
            if len(msg.voltages) > 0 and msg.voltages[0] > 0:
                self.batt_v.set(f"{msg.voltages[0]/1000.0:.1f} V")

    def _send_loop(self):
        per=1.0/SEND_HZ
        last_js_check = time.time()
        while self.running:
            # נסה להתחבר שוב לשלט כל 5 שניות אם לא מחובר
            if self.js is None and time.time()-last_js_check > 5:
                self._try_connect_joystick()
                last_js_check = time.time()

            if self.js:
                try:
                    pygame.event.pump()
                    axis_roll  = apply_deadzone(self.js.get_axis(0))
                    axis_pitch = apply_deadzone(-self.js.get_axis(1))
                    axis_yaw   = apply_deadzone(self.js.get_axis(2))
                    axis_thr   = -self.js.get_axis(3)

                    r = int(RC_MID + axis_roll  * 500)
                    p = int(RC_MID + axis_pitch * 500)
                    y = int(RC_MID + axis_yaw   * 500)

                    if axis_thr <= 0:
                        t = RC_MIN
                    else:
                        t = int(RC_MIN + axis_thr * (RC_MAX - RC_MIN))

                    self.roll.set(clamp(r, RC_MIN, RC_MAX))
                    self.pitch.set(clamp(p, RC_MIN, RC_MAX))
                    self.yaw.set(clamp(y, RC_MIN, RC_MAX))
                    self.thr.set(clamp(t, RC_MIN, RC_MAX))

                    if self.js.get_button(0): self.force_arm()
                    if self.js.get_button(1): self.disarm()
                except Exception as e:
                    print("[PS4] joystick lost:", e)
                    self.js = None
                    last_js_check = time.time()

            self._send_override()
            self._poll_battery()
            time.sleep(per)

    def force_arm(self):
        self.m.mav.command_long_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 21196, 0,0,0,0,0
        )
        print("[FORCE ARM]")

    def disarm(self):
        self.m.mav.command_long_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0,0,0,0,0,0
        )
        print("[DISARM]")

    def reset_all(self):
        self.roll.set(RC_MID)
        self.pitch.set(RC_MID)
        self.thr.set(RC_MIN)
        self.yaw.set(RC_MID)
        self.servo5.set(1500)
        self.servo7.set(1500)

    def on_close(self):
        self.running=False
        time.sleep(0.05)
        self.root.destroy()

def main():
    root=tk.Tk()
    FakeSticks(root)
    root.mainloop()

if __name__=="__main__":
    main()
