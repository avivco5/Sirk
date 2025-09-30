#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Fake RC Sticks GUI + PS4 Joystick (COM14 @115200)
- GUI סליידרים + שלט PS4
- Throttle (CH3): חצי עליון בלבד + Deadzone
- שליטה גם על CH5 ו-CH7 (סרוואים)
- ARM/DISARM גם דרך כפתורי PS4 (Cross=ARM, Circle=DISARM)
- מציג סוללה + Telemetry: Alt, Yaw, Dist-to-WP, Dist-to-Home, GroundSpeed, 3D Speed
- YOLO Person tracking → שומר אדם במרכז ע"י שינוי YAW בלבד (CH4) + Throttle מינימלי למיקס
- ALT_HOLD (ברומטר) + כפתורי ▲ +0.1m / ▼ -0.1m לשינוי גובה יעד
"""

import math, time, threading, tkinter as tk
from tkinter import ttk, messagebox
from pymavlink import mavutil
import pygame

# ==== YOLO (אופציונלי) ====
YOLO_OK = True
try:
    import cv2
    from ultralytics import YOLO as _YOLO
except Exception as _e:
    YOLO_OK = False
    _YOLO = None
    cv2 = None
    print("⚠️ YOLO/Camera unavailable:", _e)

# ==== תצורה כללית ====
DEVICE = "COM14"
BAUD   = 115200
RC_MIN, RC_MAX, RC_MID = 1000, 2000, 1500
SEND_HZ = 20.0
DEADZONE = 0.1

# ==== תצורת YOLO (YAW בלבד) ====
YOLO_DB_PIX = 40
YOLO_YAW_GAIN = 0.6
YOLO_THR_MIN = 1350
YOLO_SHOW_WINDOW = True
YOLO_CAM_INDEX = 0
YOLO_MODEL_NAME = "yolov8n.pt"
YOLO_PERSON_CLS = 0

# ==== קפיצות גובה ב-ALT_HOLD ====
ALT_BUMP_DEFAULT = 0.1
ALT_BUMP_FRAC = 0.30
ALT_BUMP_MIN_T = 0.08
ALT_BUMP_MAX_T = 1.00
FALLBACK_SPEED_UP = 1.5
FALLBACK_SPEED_DN = 1.0

def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v
def apply_deadzone(val, dz=DEADZONE): return 0.0 if abs(val) < dz else val

def haversine_m(lat1, lon1, lat2, lon2):
    """distance meters בין שתי נקודות GPS בדגריז"""
    R = 6371000.0
    a1, b1 = math.radians(lat1), math.radians(lon1)
    a2, b2 = math.radians(lat2), math.radians(lon2)
    da, db = a2-a1, b2-b1
    h = math.sin(da/2)**2 + math.cos(a1)*math.cos(a2)*math.sin(db/2)**2
    return 2*R*math.asin(math.sqrt(h))

class FakeSticks:
    def __init__(self, root):
        self.root = root
        self.root.title("Fake RC Sticks + PS4 (COM14 @115200)")

        # Connect MAVLink
        try:
            self.m = mavutil.mavlink_connection(DEVICE, baud=BAUD)
            self.m.wait_heartbeat(timeout=5)
            ttk.Label(root, text=f"Connected: sys {self.m.target_system}, comp {self.m.target_component}").pack()
        except Exception as e:
            messagebox.showerror("MAVLink", f"Connect failed: {e}")
            root.destroy(); return

        # Init pygame joystick – רק בהתחלה
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            self.js = pygame.joystick.Joystick(0); self.js.init()
            print(f"[PS4] Connected: {self.js.get_name()}")
        else:
            self.js = None; print("[PS4] No joystick detected")

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

        # Telemetry vars (מציגים כמו במיסשן פלנר)
        self.t_alt     = tk.StringVar(value="0.00")
        self.t_yaw     = tk.StringVar(value="0.00")
        self.t_wpdist  = tk.StringVar(value="0.00")
        self.t_disthome= tk.StringVar(value="0.00")
        self.t_gspeed  = tk.StringVar(value="0.00")
        self.t_speed3d = tk.StringVar(value="0.00")

        # מצב GPS להערכת מרחקים
        self._cur_latlon = None
        self._home_latlon = None

        # RC sliders
        lf = ttk.LabelFrame(root, text="RC Sticks", padding=8); lf.pack(fill="x", padx=8, pady=6)
        self._mk_slider(lf,"Roll (CH1)", self.roll, RC_MID)
        self._mk_slider(lf,"Pitch (CH2)",self.pitch,RC_MID)
        self._mk_slider(lf,"Throttle (CH3)",self.thr,RC_MIN)
        self._mk_slider(lf,"Yaw (CH4)",   self.yaw,RC_MID)

        # Servo sliders
        ls = ttk.LabelFrame(root, text="Servos (CH5 & CH7)", padding=8); ls.pack(fill="x", padx=8, pady=6)
        self._mk_slider(ls,"Servo CH5", self.servo5, 1500)
        self._mk_slider(ls,"Servo CH7", self.servo7, 1500)

        # Battery info
        lb = ttk.LabelFrame(root, text="Battery", padding=8); lb.pack(fill="x", padx=8, pady=6)
        ttk.Label(lb, text="Voltage:").grid(row=0,column=0,sticky="w"); ttk.Label(lb, textvariable=self.batt_v).grid(row=0,column=1,sticky="w")
        ttk.Label(lb, text="Current:").grid(row=1,column=0,sticky="w"); ttk.Label(lb, textvariable=self.batt_a).grid(row=1,column=1,sticky="w")
        ttk.Label(lb, text="Remaining:").grid(row=2,column=0,sticky="w"); ttk.Label(lb, textvariable=self.batt_pct).grid(row=2,column=1,sticky="w")

        # Telemetry panel (big digits)
        tele = ttk.LabelFrame(root, text="Telemetry", padding=8); tele.pack(fill="x", padx=8, pady=6)
        self._mk_big(tele, 0, 0, "Altitude (m)",  self.t_alt)
        self._mk_big(tele, 0, 1, "GroundSpeed (m/s)", self.t_gspeed)
        self._mk_big(tele, 1, 0, "Dist to WP (m)", self.t_wpdist)
        self._mk_big(tele, 1, 1, "Yaw (deg)",      self.t_yaw)
        self._mk_big(tele, 2, 0, "3D Speed (m/s)", self.t_speed3d)
        self._mk_big(tele, 2, 1, "DistToHome (m)", self.t_disthome)

        # YOLO controls
        ly = ttk.LabelFrame(root, text="YOLO Yaw Tracking", padding=8); ly.pack(fill="x", padx=8, pady=6)
        self.yolo_enabled = False; self.yolo_thread = None
        self.yolo_status  = tk.StringVar(value="YOLO: ready" if YOLO_OK else "YOLO: unavailable")
        ttk.Label(ly, textvariable=self.yolo_status).pack(side="left", padx=4)
        ttk.Button(ly, text="▶ Start", command=self.start_yolo).pack(side="left", padx=4)
        ttk.Button(ly, text="⏹ Stop",  command=self.stop_yolo).pack(side="left", padx=4)

        # ALT HOLD controls
        lm = ttk.LabelFrame(root, text="Altitude (Baro / ALT_HOLD)", padding=8); lm.pack(fill="x", padx=8, pady=6)
        ttk.Button(lm, text="Hold Alt (ALT_HOLD)", command=self.hold_alt).pack(side="left", expand=True, fill="x", padx=4)
        ttk.Button(lm, text="▲ +0.1 m", command=lambda: self.bump_alt(+ALT_BUMP_DEFAULT)).pack(side="left", expand=True, fill="x", padx=4)
        ttk.Button(lm, text="▼ -0.1 m", command=lambda: self.bump_alt(-ALT_BUMP_DEFAULT)).pack(side="left", expand=True, fill="x", padx=4)

        # Action buttons
        btns = ttk.Frame(root, padding=8); btns.pack(fill="x")
        ttk.Button(btns,text="Force ARM", command=self.force_arm).pack(side="left",expand=True,fill="x",padx=4)
        ttk.Button(btns,text="DISARM",    command=self.disarm).pack(side="left",expand=True,fill="x",padx=4)
        ttk.Button(btns,text="Reset All", command=self.reset_all).pack(side="left",expand=True,fill="x",padx=4)
        ttk.Button(btns,text="Exit",      command=self.on_close).pack(side="left",expand=True,fill="x",padx=4)

        # Threads
        self.running = True
        threading.Thread(target=self._send_loop,      daemon=True).start()
        threading.Thread(target=self._mav_telemetry,  daemon=True).start()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.geometry("840x780")

    # ---------- GUI helpers ----------
    def _mk_slider(self,parent,label,var,reset_val):
        row=ttk.Frame(parent); row.pack(fill="x", pady=4)
        ttk.Label(row,text=label,width=12).pack(side="left")
        s=ttk.Scale(row,from_=RC_MIN,to=RC_MAX,orient="horizontal",variable=var)
        s.pack(side="left",fill="x",expand=True,padx=8)
        ttk.Label(row,textvariable=var,width=5).pack(side="left")
        ttk.Button(row,text="Reset", command=lambda v=var,rv=reset_val: v.set(rv)).pack(side="left",padx=4)

    def _mk_big(self, parent, r, c, title, var):
        f = ttk.Frame(parent, padding=6); f.grid(row=r, column=c, sticky="nsew", padx=6, pady=6)
        for i in range(2): parent.grid_columnconfigure(i, weight=1); parent.grid_rowconfigure(r, weight=1)
        ttk.Label(f, text=title, anchor="center").pack(fill="x")
        lbl = ttk.Label(f, textvariable=var, anchor="center")
        lbl.pack(fill="both", expand=True, padx=4, pady=4)
        lbl.configure(font=("Segoe UI", 36, "bold"))

    # ---------- MAVLink helpers ----------
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

    def _poll_battery(self, msg):
        if msg.get_type() == "SYS_STATUS":
            voltage = msg.voltage_battery / 1000.0
            current = msg.current_battery / 100.0
            remaining = msg.battery_remaining
            self.batt_v.set(f"{voltage:.1f} V")
            if current > -9000: self.batt_a.set(f"{current:.1f} A")
            if remaining >= 0:  self.batt_pct.set(f"{remaining} %")
        elif msg.get_type() == "BATTERY_STATUS":
            if len(msg.voltages) > 0 and msg.voltages[0] > 0:
                self.batt_v.set(f"{msg.voltages[0]/1000.0:.1f} V")

    def set_mode(self, mode_name):
        try:
            mode_id = self.m.mode_mapping()[mode_name]
            self.m.set_mode(mode_id)
            print(f"[MODE] {mode_name}")
        except Exception as e:
            print(f"[MODE] Error setting {mode_name}:", e)

    def get_param(self, name, default=None, timeout=2.0):
        try:
            self.m.mav.param_request_read_send(self.m.target_system, self.m.target_component,
                                               name.encode('ascii'), -1)
            t0 = time.time()
            while time.time() - t0 < timeout:
                p = self.m.recv_match(type="PARAM_VALUE", blocking=False)
                if p:
                    pid = p.param_id.decode('ascii','ignore').strip('\x00')
                    if pid == name:
                        return float(p.param_value)
                time.sleep(0.05)
        except Exception as e:
            print(f"[PARAM] read {name} failed:", e)
        return default

    def _hover_rc(self):
        hover = self.get_param("MOT_THST_HOVER", default=0.5)
        hover = clamp(hover if hover is not None else 0.5, 0.0, 1.0)
        rc3_hover = int(RC_MIN + hover * (RC_MAX - RC_MIN))
        return rc3_hover, hover

    # ---------- ALT HOLD ----------
    def hold_alt(self):
        self.set_mode("ALT_HOLD")
        rc3_hover, hover = self._hover_rc()
        steps, cur = 10, self.thr.get()
        for i in range(1, steps+1):
            self.thr.set(int(cur + (rc3_hover - cur) * i/steps))
            self._send_override()
            time.sleep(0.03)
        print(f"[ALT_HOLD] Hover={hover:.2f} → RC3={rc3_hover}")

    def bump_alt(self, delta_m):
        up = self.get_param("PILOT_SPEED_UP", default=None)
        dn = self.get_param("PILOT_SPEED_DN", default=None)
        def _as_mps(val, fallback): return (val/100.0 if (val and val>20) else (val if val else fallback))
        v_up, v_dn = _as_mps(up, FALLBACK_SPEED_UP), _as_mps(dn, FALLBACK_SPEED_DN)
        rc3_hover, _ = self._hover_rc()
        sign = 1 if delta_m >= 0 else -1
        rate = v_up if sign>0 else v_dn
        frac = ALT_BUMP_FRAC
        dur = abs(delta_m) / max(rate*frac, 1e-3)
        dur = clamp(dur, ALT_BUMP_MIN_T, ALT_BUMP_MAX_T)
        rc_pulse = clamp(rc3_hover + sign*int(500*frac), RC_MIN, RC_MAX)
        def _do():
            self.set_mode("ALT_HOLD")
            prev = self.thr.get()
            self.thr.set(rc_pulse)
            t_end = time.time()+dur
            while time.time()<t_end and self.running:
                self._send_override(); time.sleep(0.03)
            self.thr.set(rc3_hover); self._send_override()
            print(f"[ALT_BUMP] Δh={delta_m:+.2f}m dur={dur:.2f}s RC3={rc_pulse}")
        threading.Thread(target=_do, daemon=True).start()

    # ---------- Main TX loop ----------
    def _send_loop(self):
        per=1.0/SEND_HZ
        while self.running:
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
                    t = RC_MIN if axis_thr <= 0 else int(RC_MIN + axis_thr * (RC_MAX - RC_MIN))
                    if not self.yolo_enabled:
                        self.yaw.set(clamp(y, RC_MIN, RC_MAX))
                        self.thr.set(clamp(t, RC_MIN, RC_MAX))
                    self.roll.set(clamp(r, RC_MIN, RC_MAX))
                    self.pitch.set(clamp(p, RC_MIN, RC_MAX))
                    if self.js.get_button(0): self.force_arm()
                    if self.js.get_button(1): self.disarm()
                except Exception as e:
                    print("[PS4] joystick error:", e); self.js = None
            self._send_override()
            time.sleep(per)

    # ---------- MAV Telemetry reader ----------
    def _mav_telemetry(self):
        last_att_yaw = None
        while self.running:
            try:
                msg = self.m.recv_match(blocking=False, timeout=0.1)
            except Exception:
                time.sleep(0.01); continue
            if not msg: continue
            t = msg.get_type()

            # Battery
            if t in ("SYS_STATUS","BATTERY_STATUS"):
                self._poll_battery(msg)

            # VFR_HUD: alt, groundspeed, heading
            if t == "VFR_HUD":
                try:
                    self.t_alt.set(f"{float(msg.alt):.2f}")
                except Exception: pass
                try:
                    self.t_gspeed.set(f"{float(msg.groundspeed):.2f}")
                except Exception: pass
                try:
                    self.t_yaw.set(f"{float(msg.heading):.2f}")
                except Exception: pass

            # ATTITUDE fallback yaw
            if t == "ATTITUDE":
                try:
                    last_att_yaw = math.degrees(float(msg.yaw))
                    if self.t_yaw.get() == "0.00":  # אם אין ערך מ-VFR_HUD
                        self.t_yaw.set(f"{last_att_yaw:.2f}")
                except Exception: pass

            # GLOBAL_POSITION_INT: מהירותים, lat/lon, alt_rel
            if t == "GLOBAL_POSITION_INT":
                try:
                    vx, vy, vz = float(msg.vx)/100.0, float(msg.vy)/100.0, float(msg.vz)/100.0  # m/s
                    spd3d = math.sqrt(vx*vx + vy*vy + vz*vz)
                    self.t_speed3d.set(f"{spd3d:.2f}")
                except Exception: pass
                try:
                    self._cur_latlon = (msg.lat/1e7, msg.lon/1e7)
                    if self._home_latlon and self._cur_latlon[0]!=0 and self._cur_latlon[1]!=0:
                        d = haversine_m(self._home_latlon[0], self._home_latlon[1],
                                        self._cur_latlon[0], self._cur_latlon[1])
                        self.t_disthome.set(f"{d:.2f}")
                except Exception: pass

            # HOME_POSITION: lat/lon בית
            if t == "HOME_POSITION":
                try:
                    self._home_latlon = (msg.latitude/1e7, msg.longitude/1e7)
                except Exception: pass

            # NAV_CONTROLLER_OUTPUT: מרחק ל-WP
            if t == "NAV_CONTROLLER_OUTPUT":
                try:
                    self.t_wpdist.set(f"{float(msg.wp_dist):.2f}")
                except Exception: pass

    # ---------- YOLO Yaw Tracking ----------
    def start_yolo(self):
        if not YOLO_OK:
            messagebox.showwarning("YOLO", "YOLO/Camera not available (missing packages?)"); return
        if self.yolo_enabled: return
        self.yolo_enabled = True
        self.yolo_status.set("YOLO: running (yaw only)")
        threading.Thread(target=self._yolo_loop, daemon=True).start()

    def stop_yolo(self):
        if not self.yolo_enabled: return
        self.yolo_enabled = False
        self.yolo_status.set("YOLO: stopped")
        self.yaw.set(RC_MID)

    def _yolo_loop(self):
        try:
            model = _YOLO(YOLO_MODEL_NAME)
        except Exception as e:
            print("⚠️ YOLO load failed:", e); self.yolo_status.set("YOLO: load failed"); self.yolo_enabled=False; return
        cap = cv2.VideoCapture(YOLO_CAM_INDEX)
        if not cap.isOpened():
            print("⚠️ No camera found"); self.yolo_status.set("YOLO: no camera"); self.yolo_enabled=False; return

        print("[YOLO] tracking person by YAW only. ESC/Q closes preview.")
        while self.running and self.yolo_enabled:
            ok, frame = cap.read()
            if not ok: time.sleep(0.01); continue
            h, w = frame.shape[:2]; cx_ref = w*0.5
            rc4_to_send = None
            try:
                results = model(frame, verbose=False)
                best = None
                for r in results:
                    if not hasattr(r, "boxes"): continue
                    for b in r.boxes:
                        cls = int(b.cls[0].item())
                        if cls != YOLO_PERSON_CLS: continue
                        conf = float(b.conf[0].item()) if hasattr(b,"conf") else 0.0
                        if best is None or conf > best[0]:
                            x1,y1,x2,y2 = map(float, b.xyxy[0])
                            best = (conf, x1,y1,x2,y2)
                if best is not None:
                    _, x1,y1,x2,y2 = best
                    cx = 0.5*(x1+x2); dx_px = cx - cx_ref
                    if abs(dx_px) > YOLO_DB_PIX:
                        norm = clamp(dx_px/(w*0.5), -1.0, 1.0)
                        yaw_cmd = clamp(YOLO_YAW_GAIN * norm, -1.0, 1.0)
                        rc4_to_send = int(RC_MID + yaw_cmd*500)
                    else:
                        rc4_to_send = RC_MID
                    if self.thr.get() < YOLO_THR_MIN: self.thr.set(YOLO_THR_MIN)
                    if YOLO_SHOW_WINDOW:
                        annotated = frame.copy()
                        cv2.rectangle(annotated,(int(x1),int(y1)),(int(x2),int(y2)),(0,255,0),2)
                        cv2.line(annotated,(int(cx_ref),0),(int(cx_ref),h),(255,0,0),2)
                        cv2.imshow("YOLO Yaw Tracking", annotated)
                else:
                    if YOLO_SHOW_WINDOW: cv2.imshow("YOLO Yaw Tracking", frame)
            except Exception as e:
                print("[YOLO] inference error:", e)

            if rc4_to_send is not None:
                self.yaw.set(clamp(rc4_to_send, RC_MIN, RC_MAX))

            if YOLO_SHOW_WINDOW:
                k = cv2.waitKey(1) & 0xFF
                if k in (27, ord('q')): self.stop_yolo(); break

        try:
            cap.release()
            if YOLO_SHOW_WINDOW: cv2.destroyAllWindows()
        except Exception: pass
        print("[YOLO] stopped")

    # ---------- ARM/DISARM/Reset/Close ----------
    def force_arm(self):
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0,0,0,0,0)
        print("[FORCE ARM]")

    def disarm(self):
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0,0,0,0,0,0)
        print("[DISARM]")

    def reset_all(self):
        self.roll.set(RC_MID); self.pitch.set(RC_MID); self.thr.set(RC_MIN); self.yaw.set(RC_MID)
        self.servo5.set(1500); self.servo7.set(1500)

    def on_close(self):
        if messagebox.askokcancel("Exit", "Close GUI?"):
            self.running=False; self.stop_yolo(); time.sleep(0.05); self.root.destroy()

def main():
    root=tk.Tk()
    FakeSticks(root)
    root.mainloop()

if __name__=="__main__":
    main()
