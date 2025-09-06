# udp_setpos_gui_yaw.py — GUI לשליחת סט-פוינטים/דחיפות ל-Isaac Sim ב-UDP JSON, כולל YAW
# Usage: python udp_setpos_gui_yaw.py

import socket, json, tkinter as tk
from tkinter import ttk

# ===================== הגדרות =====================
SIM_IP   = "127.0.0.1"   # אם Isaac על מחשב אחר—שים כאן את ה-IP שלו
SIM_PORT = 6000

MODE_SET = "setpos"
MODE_NDG = "nudge"
mode = MODE_SET

cur_x, cur_y, cur_alt = 0.0, 0.0, 2.0
cur_yaw_deg = 0.0

POS_STEP_DEFAULT  = 0.5   # m
ALT_STEP_DEFAULT  = 0.3   # m
YAW_STEP_DEFAULT  = 10.0  # deg
# ==================================================

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def _send_json(obj: dict):
    data = (json.dumps(obj)).encode("utf-8")
    sock.sendto(data, (SIM_IP, SIM_PORT))

def send_setpos(x, y, alt, yaw_deg=None):
    msg = {"cmd": "setpos", "x": float(x), "y": float(y), "alt": float(alt)}
    if yaw_deg is not None:
        msg["yaw_deg"] = float(yaw_deg)
    _send_json(msg)
    print(f"[TX] setpos -> x={x:.2f} y={y:.2f} alt={alt:.2f}" + (f" yaw={yaw_deg:.1f}°" if yaw_deg is not None else ""), flush=True)

def send_nudge(dx=0.0, dy=0.0, dalt=0.0, dyaw_deg=0.0):
    msg = {"cmd": "nudge", "dx": float(dx), "dy": float(dy), "dalt": float(dalt), "dyaw_deg": float(dyaw_deg)}
    _send_json(msg)
    print(f"[TX] nudge  -> dx={dx:.2f} dy={dy:.2f} dalt={dalt:.2f} dyaw={dyaw_deg:.1f}°", flush=True)

def send_setyaw(yaw_deg):
    msg = {"cmd": "setyaw", "yaw_deg": float(yaw_deg)}
    _send_json(msg)
    print(f"[TX] setyaw -> yaw={yaw_deg:.1f}°", flush=True)

# ===================== GUI =====================
root = tk.Tk()
root.title("UDP Drone Control → Isaac (XY/Alt/Yaw)")

status_var = tk.StringVar(value=f"Mode: {mode} | x={cur_x:.2f}  y={cur_y:.2f}  alt={cur_alt:.2f}  yaw={cur_yaw_deg:.1f}°")
status_lbl = ttk.Label(root, textvariable=status_var, anchor="w")
status_lbl.grid(row=0, column=0, columnspan=8, sticky="ew", padx=8, pady=(8,4))

# בחירת מצב: setpos / nudge
mode_var = tk.StringVar(value=MODE_SET)
def _refresh_status():
    status_var.set(f"Mode: {mode_var.get()} | x={cur_x:.2f}  y={cur_y:.2f}  alt={cur_alt:.2f}  yaw={cur_yaw_deg:.1f}°")

def on_mode_change():
    global mode
    mode = mode_var.get()
    _refresh_status()

mode_frame = ttk.LabelFrame(root, text="מצב שליחה")
mode_frame.grid(row=1, column=0, columnspan=8, sticky="ew", padx=8, pady=4)
ttk.Radiobutton(mode_frame, text="Set Position (מוחלט)", variable=mode_var, value=MODE_SET, command=on_mode_change).grid(row=0, column=0, padx=6, pady=4)
ttk.Radiobutton(mode_frame, text="Nudge (יחסי)",        variable=mode_var, value=MODE_NDG, command=on_mode_change).grid(row=0, column=1, padx=6, pady=4)

# גדלי צעדים
steps_frame = ttk.LabelFrame(root, text="גדלי צעדים")
steps_frame.grid(row=2, column=0, columnspan=8, sticky="ew", padx=8, pady=4)
ttk.Label(steps_frame, text="ΔXY [m]:").grid(row=0, column=0, padx=4)
pos_step_var = tk.DoubleVar(value=POS_STEP_DEFAULT)
ttk.Entry(steps_frame, textvariable=pos_step_var, width=7).grid(row=0, column=1)

ttk.Label(steps_frame, text="ΔAlt [m]:").grid(row=0, column=2, padx=8)
alt_step_var = tk.DoubleVar(value=ALT_STEP_DEFAULT)
ttk.Entry(steps_frame, textvariable=alt_step_var, width=7).grid(row=0, column=3)

ttk.Label(steps_frame, text="ΔYaw [°]:").grid(row=0, column=4, padx=8)
yaw_step_var = tk.DoubleVar(value=YAW_STEP_DEFAULT)
ttk.Entry(steps_frame, textvariable=yaw_step_var, width=7).grid(row=0, column=5)

# סליידר גובה (0..6m)
def on_alt_slider(val):
    global cur_alt
    cur_alt = float(val)
    send_setpos(cur_x, cur_y, cur_alt, cur_yaw_deg)
    _refresh_status()

alt_slider = ttk.Scale(root, from_=6.0, to=0.0, orient="vertical", command=on_alt_slider, length=220)
alt_slider.set(cur_alt)
alt_slider.grid(row=3, column=0, rowspan=7, padx=8, pady=8, sticky="ns")
ttk.Label(root, text="גובה [m]").grid(row=10, column=0)

# כפתורי XY
def do_move(direction):
    global cur_x, cur_y
    step = float(pos_step_var.get())
    dx = dy = 0.0
    if direction == "forward":   dx = +step
    elif direction == "back":    dx = -step
    elif direction == "left":    dy = -step
    elif direction == "right":   dy = +step

    if mode == MODE_SET:
        cur_x += dx; cur_y += dy
        send_setpos(cur_x, cur_y, cur_alt, cur_yaw_deg)
    else:
        send_nudge(dx=dx, dy=dy, dalt=0.0, dyaw_deg=0.0)
        cur_x += dx; cur_y += dy
    _refresh_status()

# כפתורי גובה
def do_alt(direction):
    global cur_alt
    step = float(alt_step_var.get())
    da = step if direction == "up" else -step
    if mode == MODE_SET:
        cur_alt += da
        alt_slider.set(cur_alt)
        send_setpos(cur_x, cur_y, cur_alt, cur_yaw_deg)
    else:
        send_nudge(dalt=da)
        cur_alt += da
        alt_slider.set(cur_alt)
    _refresh_status()

# כפתורי Yaw
def do_yaw(direction):
    global cur_yaw_deg
    step = float(yaw_step_var.get())
    dyaw = +step if direction == "right" else -step
    if mode == MODE_SET:
        cur_yaw_deg = (cur_yaw_deg + dyaw + 180.0) % 360.0 - 180.0
        send_setpos(cur_x, cur_y, cur_alt, cur_yaw_deg)
    else:
        send_nudge(dyaw_deg=dyaw)
        cur_yaw_deg = (cur_yaw_deg + dyaw + 180.0) % 360.0 - 180.0
    _refresh_status()

# Go To (מוחלט)
goto_frame = ttk.LabelFrame(root, text="Go To (מוחלט)")
goto_frame.grid(row=9, column=1, columnspan=7, sticky="ew", padx=8, pady=6)

goto_x = tk.DoubleVar(value=cur_x)
goto_y = tk.DoubleVar(value=cur_y)
goto_a = tk.DoubleVar(value=cur_alt)
goto_yaw = tk.DoubleVar(value=cur_yaw_deg)

ttk.Label(goto_frame, text="X:").grid(row=0, column=0, padx=4)
ttk.Entry(goto_frame, textvariable=goto_x, width=8).grid(row=0, column=1)
ttk.Label(goto_frame, text="Y:").grid(row=0, column=2, padx=8)
ttk.Entry(goto_frame, textvariable=goto_y, width=8).grid(row=0, column=3)
ttk.Label(goto_frame, text="Alt:").grid(row=0, column=4, padx=8)
ttk.Entry(goto_frame, textvariable=goto_a, width=8).grid(row=0, column=5)
ttk.Label(goto_frame, text="Yaw°:").grid(row=0, column=6, padx=8)
ttk.Entry(goto_frame, textvariable=goto_yaw, width=8).grid(row=0, column=7)

def do_goto():
    global cur_x, cur_y, cur_alt, cur_yaw_deg, mode
    cur_x = float(goto_x.get())
    cur_y = float(goto_y.get())
    cur_alt = float(goto_a.get())
    cur_yaw_deg = float(goto_yaw.get())
    alt_slider.set(cur_alt)
    send_setpos(cur_x, cur_y, cur_alt, cur_yaw_deg)
    if mode != MODE_SET:
        mode = MODE_SET; mode_var.set(MODE_SET)
    _refresh_status()

ttk.Button(goto_frame, text="Go!", command=do_goto).grid(row=0, column=8, padx=8)

# פריסת כפתורים
ttk.Button(root, text="⬆️ קדימה", command=lambda: do_move("forward")).grid(row=3, column=2, columnspan=2, sticky="ew", pady=4)
ttk.Button(root, text="⬅️ שמאלה", command=lambda: do_move("left")).grid(row=4, column=2, sticky="ew", padx=4, pady=4)
ttk.Button(root, text="➡️ ימינה", command=lambda: do_move("right")).grid(row=4, column=3, sticky="ew", padx=4, pady=4)
ttk.Button(root, text="⬇️ אחורה", command=lambda: do_move("back")).grid(row=5, column=2, columnspan=2, sticky="ew", pady=4)

ttk.Button(root, text="↺ Yaw Left",  command=lambda: do_yaw("left")).grid(row=6, column=2, sticky="ew", padx=4, pady=4)
ttk.Button(root, text="↻ Yaw Right", command=lambda: do_yaw("right")).grid(row=6, column=3, sticky="ew", padx=4, pady=4)

ttk.Button(root, text="⬆️ Alt+", command=lambda: do_alt("up")).grid(row=7, column=2, sticky="ew", padx=4, pady=4)
ttk.Button(root, text="⬇️ Alt-", command=lambda: do_alt("down")).grid(row=7, column=3, sticky="ew", padx=4, pady=4)

def _reset_xy():
    global cur_x, cur_y
    cur_x, cur_y = 0.0, 0.0
    send_setpos(cur_x, cur_y, cur_alt, cur_yaw_deg)
    _refresh_status()

ttk.Button(root, text="🔁 אפס XY", command=_reset_xy).grid(row=6, column=5, columnspan=2, sticky="ew", padx=8, pady=4)

# קיצורי מקלדת: WASD, Q/E גובה, Z/C YAW
def on_key(event):
    k = event.keysym.lower()
    if k == "w": do_move("forward")
    elif k == "s": do_move("back")
    elif k == "a": do_move("left")
    elif k == "d": do_move("right")
    elif k == "q": do_alt("down")
    elif k == "e": do_alt("up")
    elif k == "z": do_yaw("left")
    elif k == "c": do_yaw("right")

root.bind("<Key>", on_key)

# שליחת סט-פוינט פתיחה
send_setpos(cur_x, cur_y, cur_alt, cur_yaw_deg)
_refresh_status()

root.mainloop()
