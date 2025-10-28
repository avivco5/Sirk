#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Fake RC Sticks GUI + PS4 + Servos + Battery + Mission (COM14 @115200)

- CH1..CH4: Roll/Pitch/Throttle/Yaw (GUI + PS4)
- CH5, CH7: Servos (GUI)
- Force ARM / DISARM
- Battery: Voltage / Current / Remaining %
- Execute Mission: uploads TAKEOFF -> WPs -> RTL -> LAND and starts mission
- PS4 connects once at startup (no reconnect loop), like your working example
"""

import time
import threading
import tkinter as tk
from tkinter import ttk, messagebox
from pymavlink import mavutil
import pygame

# ---------------- Config ----------------
DEVICE = "COM14"
BAUD   = 115200

RC_MIN, RC_MAX, RC_MID = 1000, 2000, 1500
SEND_HZ    = 20.0
DEADZONE   = 0.10
TAKEOFF_ALT_M = 10  # מטר
# Waypoints (lat, lon) in degrees
DEFAULT_WPS = [
    (32.0852971, 34.9228633),
    (32.085293,  34.922961),
    (32.085375,  34.922984),
    (32.085401,  34.922887),
]

# ------------- Helpers ------------------
def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v
def apply_deadzone(val, dz=DEADZONE): return 0.0 if abs(val) < dz else val

# --------- Main GUI App -----------------
class FakeSticksApp:
    def __init__(self, root):
        self.root = root
        self.root.title("RC GUI + PS4 + Servos + Battery + Mission (COM14 @115200)")

        # Connect MAVLink
        try:
            self.m = mavutil.mavlink_connection(DEVICE, baud=BAUD)
            self.m.wait_heartbeat(timeout=5)
            ttk.Label(root, text=f"Connected: sys {self.m.target_system}, comp {self.m.target_component}").pack()
        except Exception as e:
            messagebox.showerror("MAVLink", f"Connect failed: {e}")
            root.destroy(); return

        # Init PS4 (single check at startup, like your working code)
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            self.js = pygame.joystick.Joystick(0)
            self.js.init()
            print(f"[PS4] Connected: {self.js.get_name()}")
            self.ps4_status = tk.StringVar(value="PS4: Connected")
        else:
            self.js = None
            print("[PS4] No joystick detected")
            self.ps4_status = tk.StringVar(value="PS4: Not detected")
        ttk.Label(root, textvariable=self.ps4_status).pack(pady=(2,0))

        # RC stick vars
        self.roll  = tk.IntVar(value=RC_MID)
        self.pitch = tk.IntVar(value=RC_MID)
        self.thr   = tk.IntVar(value=RC_MIN)
        self.yaw   = tk.IntVar(value=RC_MID)

        # Servo vars (CH5 & CH7)
        self.servo5 = tk.IntVar(value=1500)
        self.servo7 = tk.IntVar(value=1500)

        # Battery vars
        self.batt_v   = tk.StringVar(value="--.- V")
        self.batt_a   = tk.StringVar(value="--.- A")
        self.batt_pct = tk.StringVar(value="-- %")

        # RC sliders UI
        lf = ttk.LabelFrame(root, text="RC Sticks (CH1..CH4)", padding=8)
        lf.pack(fill="x", padx=8, pady=6)
        self._mk_slider(lf, "Roll (CH1)",     self.roll,  RC_MID)
        self._mk_slider(lf, "Pitch (CH2)",    self.pitch, RC_MID)
        self._mk_slider(lf, "Throttle (CH3)", self.thr,   RC_MIN)
        self._mk_slider(lf, "Yaw (CH4)",      self.yaw,   RC_MID)

        # Servos UI
        ls = ttk.LabelFrame(root, text="Servos (CH5 & CH7)", padding=8)
        ls.pack(fill="x", padx=8, pady=6)
        self._mk_slider(ls, "Servo CH5", self.servo5, 1500)
        self._mk_slider(ls, "Servo CH7", self.servo7, 1500)

        # Battery UI
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
        ttk.Button(btns, text="Force ARM",      command=self.force_arm)\
            .pack(side="left", expand=True, fill="x", padx=4)
        ttk.Button(btns, text="DISARM",         command=self.disarm)\
            .pack(side="left", expand=True, fill="x", padx=4)
        ttk.Button(btns, text="Execute Mission",command=self.execute_mission)\
            .pack(side="left", expand=True, fill="x", padx=4)
        ttk.Button(btns, text="Reset All",      command=self.reset_all)\
            .pack(side="left", expand=True, fill="x", padx=4)
        ttk.Button(btns, text="Exit",           command=self.on_close)\
            .pack(side="left", expand=True, fill="x", padx=4)

        # Worker thread
        self.running = True
        self.worker = threading.Thread(target=self._loop, daemon=True)
        self.worker.start()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.geometry("720x640")

    # ---------- UI helpers ----------
    def _mk_slider(self, parent, label, var, reset_val):
        row = ttk.Frame(parent); row.pack(fill="x", pady=4)
        ttk.Label(row, text=label, width=14).pack(side="left")
        s = ttk.Scale(row, from_=RC_MIN, to=RC_MAX, orient="horizontal", variable=var)
        s.pack(side="left", fill="x", expand=True, padx=8)
        ttk.Label(row, textvariable=var, width=5).pack(side="left")
        ttk.Button(row, text="Reset", command=lambda v=var, rv=reset_val: v.set(rv)).pack(side="left", padx=4)

    # ---------- Core I/O ----------
    def _send_override_all(self):
        try:
            self.m.mav.rc_channels_override_send(
                self.m.target_system, 1,
                int(self.roll.get()),   # CH1
                int(self.pitch.get()),  # CH2
                int(self.thr.get()),    # CH3
                int(self.yaw.get()),    # CH4
                int(self.servo5.get()), # CH5
                65535,                  # CH6 (unused)
                int(self.servo7.get()), # CH7
                65535                   # CH8 (unused)
            )
        except Exception as e:
            print("[RC_OVERRIDE] error:", e)

    def _poll_battery(self):
        msg = self.m.recv_match(type=["SYS_STATUS","BATTERY_STATUS"], blocking=False)
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
            # display first-cell voltage if present
            try:
                if len(msg.voltages) > 0 and msg.voltages[0] > 0:
                    self.batt_v.set(f"{msg.voltages[0]/1000.0:.1f} V")
            except:  # some pymavlink versions store as array/list
                pass

    # ---------- PS4 mapping ----------
    def _read_ps4_into_vars(self):
        if not self.js: return
        pygame.event.pump()
        # Typical DS4 mapping (adjust if needed):
        axis_roll  = apply_deadzone(self.js.get_axis(0))     # left stick X
        axis_pitch = apply_deadzone(-self.js.get_axis(1))    # left stick Y (invert)
        axis_yaw   = apply_deadzone(self.js.get_axis(2))     # right stick X
        axis_thr   = -self.js.get_axis(3)                    # right stick Y (invert)

        # Map sticks: -1..+1 => 1000..2000 (center at 1500)
        r = int(RC_MID + axis_roll  * 500)
        p = int(RC_MID + axis_pitch * 500)
        y = int(RC_MID + axis_yaw   * 500)

        # Throttle: use only upper half (center-return stick)
        if axis_thr <= 0:
            t = RC_MIN
        else:
            t = int(RC_MIN + axis_thr * (RC_MAX - RC_MIN))

        self.roll.set(clamp(r, RC_MIN, RC_MAX))
        self.pitch.set(clamp(p, RC_MIN, RC_MAX))
        self.yaw.set(clamp(y, RC_MIN, RC_MAX))
        self.thr.set(clamp(t, RC_MIN, RC_MAX))

        # Buttons
        if self.js.get_button(0):  # Cross
            self.force_arm()
        if self.js.get_button(1):  # Circle
            self.disarm()

    # ---------- Flight mode / arming ----------
    def _set_mode(self, mode_name: str):
        try:
            mapping = self.m.mode_mapping()
            if not mapping or mode_name not in mapping:
                print(f"[MODE] '{mode_name}' not in mapping; skipping")
                return False
            mode_id = mapping[mode_name]
            self.m.mav.set_mode_send(
                self.m.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )
            print(f"[MODE] requested {mode_name}")
            return True
        except Exception as e:
            print(f"[MODE] error: {e}")
            return False

    def force_arm(self):
        try:
            # param2=21196 → force arm (bypass some prearm checks)
            self.m.mav.command_long_send(
                self.m.target_system, self.m.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 21196, 0,0,0,0,0
            )
            print("[FORCE ARM]")
        except Exception as e:
            print("[ARM] error:", e)

    def disarm(self):
        try:
            self.m.mav.command_long_send(
                self.m.target_system, self.m.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 0, 0,0,0,0,0,0
            )
            print("[DISARM]")
        except Exception as e:
            print("[DISARM] error:", e)

    # ---------- Mission upload / start ----------
    def execute_mission(self, wps=DEFAULT_WPS, alt_m=TAKEOFF_ALT_M):
        try:
            print("[MISSION] building mission...")
            items = self._build_mission_items(wps, alt_m)

            # Switch to AUTO
            self._set_mode("AUTO")

            # Try to arm (normal arm first; if fails on bench, you can click Force ARM)
            try:
                self.m.arducopter_arm()
                self.m.motors_armed_wait()
                print("[MISSION] Armed (normal)")
            except Exception as _:
                print("[MISSION] normal arm failed; try Force ARM if needed")

            # Clear existing mission
            self.m.waypoint_clear_all_send()
            time.sleep(0.2)

            # Upload with handshake
            if self._upload_mission(items):
                print("[MISSION] Upload OK")
            else:
                print("[MISSION] Upload failed"); return

            # Start mission
            self.m.mav.command_long_send(
                self.m.target_system, self.m.target_component,
                mavutil.mavlink.MAV_CMD_MISSION_START,
                0, 0,0,0,0,0,0,0
            )
            print("[MISSION] Started")

            # Lift throttle a bit so you'll see motor mix moving on bench
            self.thr.set(RC_MIN + 100)

        except Exception as e:
            print("[MISSION] error:", e)

    def _build_mission_items(self, wps, alt_m):
        """
        Return list of dict items to be sent via MISSION_ITEM_INT
        """
        mi = []

        # TAKEOFF at first WP coordinate
        lat0, lon0 = wps[0]
        mi.append(dict(
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            current=0, autocontinue=1,
            param1=0, param2=0, param3=0, param4=0,
            x=int(lat0 * 1e7), y=int(lon0 * 1e7), z=float(alt_m)
        ))

        # Waypoints
        for (lat, lon) in wps:
            mi.append(dict(
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                current=0, autocontinue=1,
                param1=0, param2=0, param3=0, param4=0,
                x=int(lat * 1e7), y=int(lon * 1e7), z=float(alt_m)
            ))

        # RTL
        mi.append(dict(
            frame=mavutil.mavlink.MAV_FRAME_MISSION,  # frame ignored for RTL
            command=mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            current=0, autocontinue=1,
            param1=0, param2=0, param3=0, param4=0,
            x=0, y=0, z=0
        ))

        # LAND
        mi.append(dict(
            frame=mavutil.mavlink.MAV_FRAME_MISSION,
            command=mavutil.mavlink.MAV_CMD_NAV_LAND,
            current=0, autocontinue=1,
            param1=0, param2=0, param3=0, param4=0,
            x=0, y=0, z=0
        ))

        return mi

    def _upload_mission(self, items):
        """
        Robust mission upload using COUNT + REQUEST(_INT) handshake,
        sending MISSION_ITEM_INT with lat/lon*1e7 when requested.
        """
        self.m.mav.mission_count_send(self.m.target_system, self.m.target_component, len(items))
        sent = 0
        t_start = time.time()
        while True:
            # Timeout safeguard
            if time.time() - t_start > 20:
                print("[MISSION] timeout"); return False
            msg = self.m.recv_match(type=["MISSION_REQUEST","MISSION_REQUEST_INT","MISSION_ACK"], blocking=False)
            if not msg:
                time.sleep(0.05)
                continue

            mtype = msg.get_type()
            if mtype in ("MISSION_REQUEST", "MISSION_REQUEST_INT"):
                seq = msg.seq
                if seq >= len(items):
                    print(f"[MISSION] unexpected seq {seq}"); return False
                it = items[seq]
                # Send INT form (preferred)
                self.m.mav.mission_item_int_send(
                    self.m.target_system, self.m.target_component,
                    seq,
                    it["frame"], it["command"],
                    it["current"], it["autocontinue"],
                    it["param1"], it["param2"], it["param3"], it["param4"],
                    it["x"], it["y"], it["z"]
                )
                sent += 1
            elif mtype == "MISSION_ACK":
                print(f"[MISSION] ACK received (sent {sent}/{len(items)})")
                return True

    # ---------- Loop / Close ----------
    def _loop(self):
        per = 1.0 / SEND_HZ
        while self.running:
            # Read PS4 (if connected)
            try:
                if self.js:
                    self._read_ps4_into_vars()
                    self.ps4_status.set("PS4: Connected")
                else:
                    self.ps4_status.set("PS4: Not detected")
            except Exception as e:
                print("[PS4] error:", e)
                self.js = None
                self.ps4_status.set("PS4: Not detected")

            # Send RC override + poll battery
            self._send_override_all()
            self._poll_battery()

            time.sleep(per)

    def reset_all(self):
        self.roll.set(RC_MID)
        self.pitch.set(RC_MID)
        self.thr.set(RC_MIN)
        self.yaw.set(RC_MID)
        self.servo5.set(1500)
        self.servo7.set(1500)

    def on_close(self):
        self.running = False
        time.sleep(0.05)
        self.root.destroy()

# -------------- main ---------------
def main():
    root = tk.Tk()
    FakeSticksApp(root)
    root.mainloop()

if __name__ == "__main__":
    main()
