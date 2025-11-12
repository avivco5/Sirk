#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ASCII-only, English comments

Minimal Tkinter RC GUI for ArduPilot SITL over TCP.
- Hard-coded WSL IP/port inside the script
- RC sliders (CH1..CH4) sent at 20 Hz via RC_OVERRIDE
- Buttons for ARM/DISARM and mode changes (STABILIZE, ALT_HOLD, GUIDED, LOITER, RTL)
- Buttons for TAKEOFF (10 m) and LAND
- Status area: heartbeat sys/comp, current mode, simple battery readout
"""

import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
from pymavlink import mavutil

# ------------------- connection settings (edit these) -------------------
MAV_IP   = "172.20.186.151"   # WSL IP
MAV_PORT = 5770               # provided by MAVProxy --out
MASTER   = f"tcp:{MAV_IP}:{MAV_PORT}"

# ------------------- RC bounds -------------------
RC_MIN, RC_MID, RC_MAX = 1000, 1500, 2000
SEND_HZ = 20.0

class SITLGui:
    def __init__(self, root):
        self.root = root
        self.root.title("SITL RC GUI (TCP)")

        # connect
        self.m = self._connect_mavlink(MASTER)

        # state
        self.running = True
        self.rc_enable = tk.BooleanVar(value=True)
        self.armed = False

        # status strings
        self.t_status = tk.StringVar(value="Status: --")
        self.t_mode   = tk.StringVar(value="Mode: --")
        self.t_batt   = tk.StringVar(value="Battery: -- V")

        # build ui
        self._build_ui()

        # set useful message rates
        self._set_msg_rates()

        # threads
        threading.Thread(target=self._rx_loop, daemon=True).start()
        threading.Thread(target=self._tx_loop, daemon=True).start()

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------------- MAVLink ----------------
    def _connect_mavlink(self, master):
        print(f"[MAVLINK] Connecting to {master} ...")
        m = mavutil.mavlink_connection(master)
        hb = m.wait_heartbeat(timeout=10)
        if not hb:
            messagebox.showerror("MAVLink", "No heartbeat from SITL")
            raise SystemExit(1)
        print(f"[HEARTBEAT] sys={m.target_system} comp={m.target_component}")
        return m

    def _set_msg_rates(self):
        # set a few message intervals so UI feels responsive
        def set_rate(msgid, hz):
            try:
                interval = int(1e6/float(hz)) if hz > 0 else 0
                self.m.mav.command_long_send(
                    self.m.target_system, self.m.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0, msgid, interval, 0,0,0,0,0
                )
            except Exception as e:
                print(f"[MSG_RATE] id={msgid} failed: {e}")

        set_rate(mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT,            1)
        set_rate(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,           1)
        set_rate(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,            10)
        set_rate(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,              5)
        set_rate(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10)

    # ---------------- UI ----------------
    def _build_ui(self):
        # status bar
        sb = ttk.Frame(self.root, padding=6)
        sb.pack(fill="x")
        ttk.Label(sb, textvariable=self.t_status).pack(side="left", padx=6)
        ttk.Label(sb, textvariable=self.t_mode).pack(side="left", padx=12)
        ttk.Label(sb, textvariable=self.t_batt).pack(side="left", padx=12)
        ttk.Checkbutton(sb, text="Send RC", variable=self.rc_enable).pack(side="right")

        # rc sliders
        lf = ttk.LabelFrame(self.root, text="RC Sticks (1000..2000)", padding=8)
        lf.pack(fill="x", padx=8, pady=6)

        self.rc1 = tk.IntVar(value=RC_MID)  # roll
        self.rc2 = tk.IntVar(value=RC_MID)  # pitch
        self.rc3 = tk.IntVar(value=RC_MIN)  # throttle
        self.rc4 = tk.IntVar(value=RC_MID)  # yaw

        self._mk_slider(lf, "CH1 Roll",     self.rc1, RC_MIN, RC_MAX, RC_MID)
        self._mk_slider(lf, "CH2 Pitch",    self.rc2, RC_MIN, RC_MAX, RC_MID)
        self._mk_slider(lf, "CH3 Throttle", self.rc3, RC_MIN, RC_MAX, RC_MIN)
        self._mk_slider(lf, "CH4 Yaw",      self.rc4, RC_MIN, RC_MAX, RC_MID)

        # buttons
        bf = ttk.LabelFrame(self.root, text="Flight", padding=8)
        bf.pack(fill="x", padx=8, pady=6)

        ttk.Button(bf, text="ARM",     command=self.btn_arm).pack(side="left", expand=True, fill="x", padx=4)
        ttk.Button(bf, text="DISARM",  command=self.btn_disarm).pack(side="left", expand=True, fill="x", padx=4)
        ttk.Button(bf, text="STABILIZE", command=lambda: self.set_mode("STABILIZE")).pack(side="left", expand=True, fill="x", padx=4)
        ttk.Button(bf, text="ALT_HOLD",  command=lambda: self.set_mode("ALT_HOLD")).pack(side="left", expand=True, fill="x", padx=4)
        ttk.Button(bf, text="GUIDED",    command=lambda: self.set_mode("GUIDED")).pack(side="left", expand=True, fill="x", padx=4)
        ttk.Button(bf, text="LOITER",    command=lambda: self.set_mode("LOITER")).pack(side="left", expand=True, fill="x", padx=4)
        ttk.Button(bf, text="RTL",       command=lambda: self.set_mode("RTL")).pack(side="left", expand=True, fill="x", padx=4)

        af = ttk.LabelFrame(self.root, text="Automation", padding=8)
        af.pack(fill="x", padx=8, pady=6)
        ttk.Button(af, text="TAKEOFF 10 m", command=lambda: self.takeoff(10)).pack(side="left", expand=True, fill="x", padx=4)
        ttk.Button(af, text="LAND",         command=self.land).pack(side="left", expand=True, fill="x", padx=4)

    def _mk_slider(self, parent, title, var, vmin, vmax, reset):
        row = ttk.Frame(parent); row.pack(fill="x", pady=3)
        ttk.Label(row, text=title, width=12).pack(side="left")
        s = ttk.Scale(row, from_=vmin, to=vmax, orient="horizontal", variable=var)
        s.pack(side="left", fill="x", expand=True, padx=6)
        ttk.Label(row, textvariable=var, width=6).pack(side="left")
        ttk.Button(row, text="Reset", command=lambda v=var, r=reset: v.set(r)).pack(side="left", padx=6)

    # ---------------- TX/RX loops ----------------
    def _tx_loop(self):
        period = 1.0 / SEND_HZ
        while self.running:
            if self.rc_enable.get():
                try:
                    rc1 = int(self.rc1.get())
                    rc2 = int(self.rc2.get())
                    rc3 = int(self.rc3.get())
                    rc4 = int(self.rc4.get())
                    self.m.mav.rc_channels_override_send(
                        self.m.target_system, self.m.target_component,
                        rc1, rc2, rc3, rc4, 65535, 65535, 65535, 65535
                    )
                except Exception as e:
                    print("[RC_OVERRIDE] err:", e)
            time.sleep(period)

    def _rx_loop(self):
        # also poll current mode periodically
        last_mode_print = 0
        while self.running:
            msg = self.m.recv_match(blocking=False)
            if not msg:
                time.sleep(0.01); continue
            t = msg.get_type()

            if t == "HEARTBEAT":
                # armed state
                armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.armed = armed
                self.t_status.set(f"Status: HEARTBEAT (armed={armed})")

                # decode mode name
                try:
                    mode = mavutil.mode_string_v10(msg)
                    self.t_mode.set(f"Mode: {mode}")
                    now = time.time()
                    if now - last_mode_print > 1.0:
                        print(f"[MODE] {mode}")
                        last_mode_print = now
                except Exception:
                    pass

            elif t == "SYS_STATUS":
                try:
                    vbat = float(msg.voltage_battery) / 1000.0
                    self.t_batt.set(f"Battery: {vbat:.1f} V")
                except Exception:
                    pass

    # ---------------- Commands ----------------
    def set_mode(self, mode_name):
        try:
            mode_id = self.m.mode_mapping()[mode_name]
            self.m.set_mode(mode_id)
            print(f"[MODE] {mode_name}")
        except Exception as e:
            print(f"[MODE] set failed: {e}")

    def btn_arm(self):
        try:
            self.m.mav.command_long_send(
                self.m.target_system, self.m.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 21196, 0,0,0,0,0
            )
            print("[ARM]")
        except Exception as e:
            print("[ARM] failed:", e)

    def btn_disarm(self):
        try:
            self.m.mav.command_long_send(
                self.m.target_system, self.m.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 0, 0,0,0,0,0,0
            )
            print("[DISARM]")
        except Exception as e:
            print("[DISARM] failed:", e)

    def takeoff(self, alt_m):
        try:
            # must be in GUIDED
            self.set_mode("GUIDED")
            time.sleep(0.2)
            self.m.mav.command_long_send(
                self.m.target_system, self.m.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0, 0,0,0,0, 0,0, float(alt_m)
            )
            print(f"[TAKEOFF] {alt_m}m")
        except Exception as e:
            print("[TAKEOFF] failed:", e)

    def land(self):
        try:
            self.m.mav.command_long_send(
                self.m.target_system, self.m.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
                0,0,0,0, 0,0, 0
            )
            print("[LAND]")
        except Exception as e:
            print("[LAND] failed:", e)

    # ---------------- Close ----------------
    def _on_close(self):
        if self.armed:
            if not messagebox.askyesno("Exit", "Vehicle is ARMED. Disarm and exit?"):
                return
            self.btn_disarm()
            time.sleep(0.3)
        self.running = False
        try: self.m.close()
        except Exception: pass
        self.root.destroy()

def main():
    root = tk.Tk()
    app = SITLGui(root)
    root.mainloop()

if __name__ == "__main__":
    main()
