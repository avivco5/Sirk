#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MAVLink RC override GUI (auto-maps to RCMAP_*).
- COM14 @ 115200 (hard-coded)
- Sliders: Roll, Pitch, Throttle, Yaw
- Reverse per channel + Swap RP / TY
- ARM/DISARM, Center, Panic, modes
- Sends overrides at 20 Hz to the actual mapped RC channels.
"""
import time, threading, tkinter as tk
from tkinter import ttk, messagebox
from pymavlink import mavutil

DEVICE = "COM14"
BAUD   = 115200
RC_MIN, RC_MAX, RC_MID = 1000, 2000, 1500
SEND_RATE_HZ = 20.0

def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v

def get_param_int(m, names, default):
    for name in names:
        try:
            m.mav.param_request_read_send(m.target_system, m.target_component, name.encode('ascii'), -1)
            t0 = time.time()
            while time.time() - t0 < 1.0:
                msg = m.recv_match(type="PARAM_VALUE", blocking=False)
                if msg:
                    pid = msg.param_id.decode('ascii','ignore').strip('\x00')
                    if pid == name:
                        return int(msg.param_value)
                time.sleep(0.02)
        except: pass
    return default

class App:
    def __init__(self, root):
        self.root = root
        self.root.title("MAVLink Sliders (Auto-map)")
        # Connect
        try:
            self.m = mavutil.mavlink_connection(DEVICE, baud=BAUD)
            self.m.wait_heartbeat(timeout=5)
        except Exception as e:
            messagebox.showerror("MAVLink", f"Connect failed: {e}")
            root.destroy(); return

        top = ttk.Frame(root, padding=8); top.pack(fill="x")
        ttk.Label(top, text=f"Connected: sys {self.m.target_system}, comp {self.m.target_component}").pack(side="left")

        # Read mapping from FC
        self.map_roll  = get_param_int(self.m, ["RCMAP_ROLL","RC_MAP_ROLL"], 1)
        self.map_pitch = get_param_int(self.m, ["RCMAP_PITCH","RC_MAP_PITCH"], 2)
        self.map_thr   = get_param_int(self.m, ["RCMAP_THROTTLE","RC_MAP_THROTTLE"], 3)
        self.map_yaw   = get_param_int(self.m, ["RCMAP_YAW","RC_MAP_YAW"], 4)
        maptxt = f"RCMAP: R={self.map_roll} P={self.map_pitch} T={self.map_thr} Y={self.map_yaw}"
        ttk.Label(top, text=maptxt).pack(side="right")

        # Vars
        self.roll = tk.IntVar(value=RC_MID)
        self.pitch = tk.IntVar(value=RC_MID)
        self.thr = tk.IntVar(value=RC_MIN)
        self.yaw = tk.IntVar(value=RC_MID)
        self.rev_r = tk.BooleanVar(value=False)
        self.rev_p = tk.BooleanVar(value=False)
        self.rev_t = tk.BooleanVar(value=False)
        self.rev_y = tk.BooleanVar(value=False)
        self.swap_rp = tk.BooleanVar(value=False)
        self.swap_ty = tk.BooleanVar(value=False)

        # UI
        lf = ttk.LabelFrame(root, text="Channels (1000..2000)", padding=8); lf.pack(fill="x", padx=8, pady=6)
        self._mk_slider(lf, "Roll", self.roll, self.rev_r)
        self._mk_slider(lf, "Pitch", self.pitch, self.rev_p)
        self._mk_slider(lf, "Throttle", self.thr, self.rev_t)
        self._mk_slider(lf, "Yaw", self.yaw, self.rev_y)

        swap = ttk.Frame(root, padding=(8,0)); swap.pack(fill="x")
        ttk.Checkbutton(swap, text="Swap Roll ↔ Pitch", variable=self.swap_rp).pack(side="left", padx=4)
        ttk.Checkbutton(swap, text="Swap Throttle ↔ Yaw", variable=self.swap_ty).pack(side="left", padx=4)

        btns = ttk.Frame(root, padding=8); btns.pack(fill="x")
        ttk.Button(btns, text="ARM", command=lambda: self._arm(True)).grid(row=0, column=0, padx=4, pady=4, sticky="ew")
        ttk.Button(btns, text="DISARM", command=lambda: self._arm(False)).grid(row=0, column=1, padx=4, pady=4, sticky="ew")
        ttk.Button(btns, text="Center R/P/Y", command=self._center).grid(row=0, column=2, padx=4, pady=4, sticky="ew")
        ttk.Button(btns, text="Panic (Thr→MIN)", command=self._panic).grid(row=0, column=3, padx=4, pady=4, sticky="ew")
        ttk.Button(btns, text="GUIDED", command=lambda: self._mode("GUIDED")).grid(row=1, column=0, padx=4, pady=4, sticky="ew")
        ttk.Button(btns, text="STABILIZE", command=lambda: self._mode("STABILIZE")).grid(row=1, column=1, padx=4, pady=4, sticky="ew")
        ttk.Button(btns, text="RTL", command=lambda: self._mode("RTL")).grid(row=1, column=2, padx=4, pady=4, sticky="ew")
        ttk.Button(btns, text="Exit", command=self.on_close).grid(row=1, column=3, padx=4, pady=4, sticky="ew")
        for c in range(4): btns.grid_columnconfigure(c, weight=1)

        self.running = True
        self.sender = threading.Thread(target=self._send_loop, daemon=True); self.sender.start()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.geometry("660x360")

    def _mk_slider(self, parent, label, var, revflag):
        row = ttk.Frame(parent); row.pack(fill="x", pady=4)
        ttk.Label(row, text=label, width=12).pack(side="left")
        s = ttk.Scale(row, from_=RC_MIN, to=RC_MAX, orient="horizontal", variable=var)
        s.pack(side="left", fill="x", expand=True, padx=8)
        ttk.Label(row, textvariable=var, width=5).pack(side="left", padx=(6,2))
        ttk.Checkbutton(row, text="Reverse", variable=revflag).pack(side="right")

    def _apply_rev(self, v, rev): return RC_MIN + (RC_MAX - v) if rev else v

    def _mode(self, name):
        try:
            m = self.m.mode_mapping()
            if not m or name not in m: print(f"[MODE] '{name}' not available"); return
            self.m.mav.set_mode_send(self.m.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, m[name])
            print(f"[MODE] request {name}")
        except Exception as e:
            print(f"[MODE] error: {e}")

    def _arm(self, arm=True):
        try:
            self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                         1 if arm else 0, 0,0,0,0,0,0)
            print("[ARM]" if arm else "[DISARM]")
        except Exception as e:
            print(f"[ARM] error: {e}")

    def _center(self):
        self.roll.set(RC_MID); self.pitch.set(RC_MID); self.yaw.set(RC_MID)

    def _panic(self): self.thr.set(RC_MIN)

    def _send_loop(self):
        per = 1.0 / SEND_RATE_HZ
        while self.running:
            r = clamp(self.roll.get(), RC_MIN, RC_MAX)
            p = clamp(self.pitch.get(), RC_MIN, RC_MAX)
            t = clamp(self.thr.get(), RC_MIN, RC_MAX)
            y = clamp(self.yaw.get(), RC_MIN, RC_MAX)

            # Reverse per axis
            r = self._apply_rev(r, self.rev_r.get())
            p = self._apply_rev(p, self.rev_p.get())
            t = self._apply_rev(t, self.rev_t.get())
            y = self._apply_rev(y, self.rev_y.get())

            # Optional “stick” swaps in software
            if self.swap_rp.get(): r, p = p, r
            if self.swap_ty.get(): t, y = y, t

            # Build override array for channels 1..8
            vals = [65535]*8
            def put(ch, val):
                if 1 <= ch <= 8: vals[ch-1] = int(val)

            put(self.map_roll,  r)
            put(self.map_pitch, p)
            put(self.map_thr,   t)
            put(self.map_yaw,   y)

            try:
                # target component = autopilot (1) is safest if default comp id causes issues
                self.m.mav.rc_channels_override_send(self.m.target_system, 1, *vals)
            except Exception as e:
                print(f"[RC] error: {e}")
            time.sleep(per)

    def _release(self):
        try:
            # neutral push
            vals = [65535]*8
            def put(ch, val):
                if 1 <= ch <= 8: vals[ch-1] = int(val)
            put(self.map_roll,  RC_MID)
            put(self.map_pitch, RC_MID)
            put(self.map_thr,   RC_MIN)
            put(self.map_yaw,   RC_MID)
            self.m.mav.rc_channels_override_send(self.m.target_system, 1, *vals)
            time.sleep(0.1)
            self.m.mav.rc_channels_override_send(self.m.target_system, 1, *([65535]*8))
        except Exception as e:
            print(f"[EXIT] release error: {e}")

    def on_close(self):
        if not self.running:
            self.root.destroy(); return
        self.running = False
        self._release()
        self.root.after(50, self.root.destroy)

def main():
    root = tk.Tk()
    App(root)
    root.mainloop()

if __name__ == "__main__":
    main()
