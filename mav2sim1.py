# udp_setpos_gui.py — GUI לשליחת סט-פוינטים/דחיפות ל-Isaac Sim ב-UDP JSON
# עובד עם המאזין שנתנו לך בסימולציה (UDP JSON על פורט 6000).
# שימוש: python udp_setpos_gui.py

import socket, json, time, tkinter as tk
from tkinter import ttk

# ===================== הגדרות =====================
SIM_IP   = "127.0.0.1"   # אם Isaac על מחשב אחר—שים כאן את ה-IP שלו
SIM_PORT = 6000

# מצב ברירת מחדל: שליחת סט-פוינט מוחלט (setpos)
MODE_SET = "setpos"
MODE_NDG = "nudge"
mode = MODE_SET

# סט-פוינט מקומי שנשמור ב-GUI (לא טלמטריה)
cur_x, cur_y, cur_alt = 0.0, 0.0, 2.0

# גדלי צעדים (אפשר לשנות ב-GUI)
POS_STEP_DEFAULT = 0.5    # מטר
ALT_STEP_DEFAULT = 0.3    # מטר
# ==================================================

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def _send_json(obj: dict):
    data = (json.dumps(obj)).encode("utf-8")
    sock.sendto(data, (SIM_IP, SIM_PORT))

def send_setpos(x, y, alt):
    msg = {"cmd": "setpos", "x": float(x), "y": float(y), "alt": float(alt)}
    _send_json(msg)
    print(f"[TX] setpos -> x={x:.2f} y={y:.2f} alt={alt:.2f}", flush=True)

def send_nudge(dx=0.0, dy=0.0, dalt=0.0):
    msg = {"cmd": "nudge", "dx": float(dx), "dy": float(dy), "dalt": float(dalt)}
    _send_json(msg)
    print(f"[TX] nudge  -> dx={dx:.2f} dy={dy:.2f} dalt={dalt:.2f}", flush=True)

# ===================== GUI =====================
root = tk.Tk()
root.title("UDP Drone Control → Isaac Sim (JSON)")

# סטטוס
status_var = tk.StringVar(value=f"Mode: setpos | x={cur_x:.2f}  y={cur_y:.2f}  alt={cur_alt:.2f}")
status_lbl = ttk.Label(root, textvariable=status_var, anchor="w")
status_lbl.grid(row=0, column=0, columnspan=6, sticky="ew", padx=8, pady=(8,4))

# בחירת מצב: setpos / nudge
mode_var = tk.StringVar(value=MODE_SET)
def on_mode_change():
    global mode
    mode = mode_var.get()
    status_var.set(f"Mode: {mode} | x={cur_x:.2f}  y={cur_y:.2f}  alt={cur_alt:.2f}")

mode_frame = ttk.LabelFrame(root, text="מצב שליחה")
mode_frame.grid(row=1, column=0, columnspan=6, sticky="ew", padx=8, pady=4)
ttk.Radiobutton(mode_frame, text="Set Position (מוחלט)", variable=mode_var, value=MODE_SET,
                command=on_mode_change).grid(row=0, column=0, padx=6, pady=4)
ttk.Radiobutton(mode_frame, text="Nudge (יחסי)", variable=mode_var, value=MODE_NDG,
                command=on_mode_change).grid(row=0, column=1, padx=6, pady=4)

# גדלי צעדים
steps_frame = ttk.LabelFrame(root, text="גדלי צעדים")
steps_frame.grid(row=2, column=0, columnspan=6, sticky="ew", padx=8, pady=4)
ttk.Label(steps_frame, text="ΔXY [m]:").grid(row=0, column=0, padx=4)
pos_step_var = tk.DoubleVar(value=POS_STEP_DEFAULT)
ttk.Entry(steps_frame, textvariable=pos_step_var, width=7).grid(row=0, column=1)

ttk.Label(steps_frame, text="ΔAlt [m]:").grid(row=0, column=2, padx=8)
alt_step_var = tk.DoubleVar(value=ALT_STEP_DEFAULT)
ttk.Entry(steps_frame, textvariable=alt_step_var, width=7).grid(row=0, column=3)

# סליידר גובה (מטרים, 0..6)
def on_alt_slider(val):
    global cur_alt
    cur_alt = float(val)
    if mode == MODE_SET:
        send_setpos(cur_x, cur_y, cur_alt)
    else:
        # במצב Nudge: הזזת סליידר תשלח setpos בכל זאת, כדי לא "לברוח" עם דחיפות קטנות
        send_setpos(cur_x, cur_y, cur_alt)
    status_var.set(f"Mode: {mode} | x={cur_x:.2f}  y={cur_y:.2f}  alt={cur_alt:.2f}")

alt_slider = ttk.Scale(root, from_=6.0, to=0.0, orient="vertical", command=on_alt_slider, length=220)
alt_slider.set(cur_alt)
alt_slider.grid(row=3, column=0, rowspan=5, padx=8, pady=8, sticky="ns")

ttk.Label(root, text="גובה [m]").grid(row=8, column=0)

# כפתורי תנועה
def do_move(direction):
    """קדימה/אחורה/שמאלה/ימינה"""
    global cur_x, cur_y
    step = float(pos_step_var.get())
    dx = dy = 0.0
    if direction == "forward":   # +X
        dx = step
    elif direction == "back":    # -X
        dx = -step
    elif direction == "left":    # -Y
        dy = step
    elif direction == "right":   # +Y
        dy = -step

    if mode == MODE_SET:
        cur_x += dx
        cur_y += dy
        send_setpos(cur_x, cur_y, cur_alt)
    else:
        send_nudge(dx=dx, dy=dy, dalt=0.0)
        # עדכון הסטטוס המקומי (הערכה)
        cur_x += dx
        cur_y += dy

    status_var.set(f"Mode: {mode} | x={cur_x:.2f}  y={cur_y:.2f}  alt={cur_alt:.2f}")

def do_alt(direction):
    """גובה +/−"""
    global cur_alt
    step = float(alt_step_var.get())
    da = step if direction == "up" else -step
    if mode == MODE_SET:
        cur_alt += da
        alt_slider.set(cur_alt)
        send_setpos(cur_x, cur_y, cur_alt)
    else:
        send_nudge(dalt=da)
        cur_alt += da
        alt_slider.set(cur_alt)
    status_var.set(f"Mode: {mode} | x={cur_x:.2f}  y={cur_y:.2f}  alt={cur_alt:.2f}")

# גריד הכפתורים
ttk.Button(root, text="⬆️ קדימה", command=lambda: do_move("forward")).grid(row=3, column=2, columnspan=2, sticky="ew", pady=4)
ttk.Button(root, text="⬅️ שמאלה", command=lambda: do_move("left")).grid(row=4, column=2, sticky="ew", padx=4, pady=4)
ttk.Button(root, text="➡️ ימינה", command=lambda: do_move("right")).grid(row=4, column=3, sticky="ew", padx=4, pady=4)
ttk.Button(root, text="⬇️ אחורה", command=lambda: do_move("back")).grid(row=5, column=2, columnspan=2, sticky="ew", pady=4)

ttk.Button(root, text="⬆️ Alt+", command=lambda: do_alt("up")).grid(row=6, column=2, sticky="ew", padx=4, pady=4)
ttk.Button(root, text="⬇️ Alt-", command=lambda: do_alt("down")).grid(row=6, column=3, sticky="ew", padx=4, pady=4)

# Go To (סט-פוינט מוחלט לפי שדות)
goto_frame = ttk.LabelFrame(root, text="Go To (מוחלט)")
goto_frame.grid(row=7, column=1, columnspan=5, sticky="ew", padx=8, pady=6)

goto_x = tk.DoubleVar(value=cur_x)
goto_y = tk.DoubleVar(value=cur_y)
goto_a = tk.DoubleVar(value=cur_alt)

ttk.Label(goto_frame, text="X:").grid(row=0, column=0, padx=4)
ttk.Entry(goto_frame, textvariable=goto_x, width=8).grid(row=0, column=1)
ttk.Label(goto_frame, text="Y:").grid(row=0, column=2, padx=8)
ttk.Entry(goto_frame, textvariable=goto_y, width=8).grid(row=0, column=3)
ttk.Label(goto_frame, text="Alt:").grid(row=0, column=4, padx=8)
ttk.Entry(goto_frame, textvariable=goto_a, width=8).grid(row=0, column=5)

def do_goto():
    global cur_x, cur_y, cur_alt, mode
    x = float(goto_x.get())
    y = float(goto_y.get())
    a = float(goto_a.get())
    cur_x, cur_y, cur_alt = x, y, a
    alt_slider.set(cur_alt)
    send_setpos(cur_x, cur_y, cur_alt)
    if mode != MODE_SET:
        mode = MODE_SET
        mode_var.set(MODE_SET)
    status_var.set(f"Mode: {mode} | x={cur_x:.2f}  y={cur_y:.2f}  alt={cur_alt:.2f}")

ttk.Button(goto_frame, text="Go!", command=do_goto).grid(row=0, column=6, padx=8)

# אפס XY
def reset_xy():
    global cur_x, cur_y
    cur_x, cur_y = 0.0, 0.0
    if mode == MODE_SET:
        send_setpos(cur_x, cur_y, cur_alt)
    else:
        # גם כאן עדיף ליישר את היעד בעזרת setpos כדי להימנע מסחיבה מצטברת
        send_setpos(cur_x, cur_y, cur_alt)
    status_var.set(f"Mode: {mode} | x={cur_x:.2f}  y={cur_y:.2f}  alt={cur_alt:.2f}")

ttk.Button(root, text="🔁 אפס XY", command=reset_xy).grid(row=6, column=4, columnspan=2, sticky="ew", padx=8, pady=4)

# קיצורי מקלדת
def on_key(event):
    k = event.keysym.lower()
    if k == "w": do_move("forward")
    elif k == "s": do_move("back")
    elif k == "a": do_move("left")
    elif k == "d": do_move("right")
    elif k == "q": do_alt("down")
    elif k == "e": do_alt("up")

root.bind("<Key>", on_key)

# שליחת סט-פוינט התחלתי כדי "ליישר קו" עם הסימולטור
send_setpos(cur_x, cur_y, cur_alt)

root.mainloop()
