#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Fake RC Sticks GUI for ArduPilot (COM14 @115200)
- Sliders: Roll (CH1), Pitch (CH2), Throttle (CH3), Yaw (CH4)
- Forced ARM / DISARM
- Reset buttons to neutralize sticks
- Sends RC_OVERRIDE continuously
- Safe for bench: בלי פרופים, אפשר גם בלי ESCים
"""

import time
import threading
import tkinter as tk
from tkinter import ttk, messagebox
from pymavlink import mavutil

DEVICE = "COM14"
BAUD   = 115200
RC_MIN, RC_MAX, RC_MID = 1000, 2000, 1500
SEND_HZ = 20.0

def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v

class FakeSticks:
    def __init__(self, root):
        self.root = root
        self.root.title("Fake RC Sticks (COM14 @115200)")

        # Connect
        try:
            self.m = mavutil.mavlink_connection(DEVICE, baud=BAUD)
            self.m.wait_heartbeat(timeout=5)
            ttk.Label(root, text=f"Connected: sys {self.m.target_system}, comp {self.m.target_component}").pack()
        except Exception as e:
            messagebox.showerror("MAVLink", f"Connect failed: {e}")
            root.destroy(); return

        # Stick vars
        self.roll = tk.IntVar(value=RC_MID)
        self.pitch= tk.IntVar(value=RC_MID)
        self.thr  = tk.IntVar(value=RC_MIN)
        self.yaw  = tk.IntVar(value=RC_MID)

        # Sliders
        lf = ttk.LabelFrame(root, text="Fake RC Sticks", padding=8)
        lf.pack(fill="x", padx=8, pady=6)
        self._mk_slider(lf,"Roll (CH1)", self.roll, RC_MID)
        self._mk_slider(lf,"Pitch (CH2)",self.pitch,RC_MID)
        self._mk_slider(lf,"Throttle (CH3)",self.thr,RC_MIN)
        self._mk_slider(lf,"Yaw (CH4)",   self.yaw,RC_MID)

        # Buttons
        btns = ttk.Frame(root, padding=8); btns.pack(fill="x")
        ttk.Button(btns,text="Force ARM", command=self.force_arm).pack(side="left",expand=True,fill="x",padx=4)
        ttk.Button(btns,text="DISARM",    command=self.disarm).pack(side="left",expand=True,fill="x",padx=4)
        ttk.Button(btns,text="Reset All", command=self.reset_all).pack(side="left",expand=True,fill="x",padx=4)
        ttk.Button(btns,text="Exit",      command=self.on_close).pack(side="left",expand=True,fill="x",padx=4)

        # Start thread
        self.running = True
        self.sender = threading.Thread(target=self._send_loop, daemon=True)
        self.sender.start()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.geometry("520x360")

    def _mk_slider(self,parent,label,var,reset_val):
        row=ttk.Frame(parent); row.pack(fill="x", pady=4)
        ttk.Label(row,text=label,width=12).pack(side="left")
        s=ttk.Scale(row,from_=RC_MIN,to=RC_MAX,orient="horizontal",variable=var)
        s.pack(side="left",fill="x",expand=True,padx=8)
        ttk.Label(row,textvariable=var,width=5).pack(side="left")
        ttk.Button(row,text="Reset",command=lambda v=var,rv=reset_val: v.set(rv)).pack(side="left",padx=4)

    def _send_override(self,ch1,ch2,ch3,ch4):
        try:
            self.m.mav.rc_channels_override_send(
                self.m.target_system, 1,
                int(ch1),int(ch2),int(ch3),int(ch4),
                65535,65535,65535,65535
            )
        except: pass

    def _send_loop(self):
        per=1.0/SEND_HZ
        while self.running:
            r=clamp(self.roll.get(),RC_MIN,RC_MAX)
            p=clamp(self.pitch.get(),RC_MIN,RC_MAX)
            t=clamp(self.thr.get(),  RC_MIN,RC_MAX)
            y=clamp(self.yaw.get(),  RC_MIN,RC_MAX)
            self._send_override(r,p,t,y)
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
