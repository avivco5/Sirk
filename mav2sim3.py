# udp_setpos_gui_yaw.py — GUI לשליחת סט-פוינטים/דחיפות ל-Isaac Sim (UDP JSON) עם Yaw
# תנועה קדימה/אחורה/ימינה/שמאלה ⇒ NUDGE במערכת הגוף.
# Go To / סליידר גובה ⇒ SETPOS (עולם).
import socket, json, tkinter as tk
from tkinter import ttk

SIM_IP, SIM_PORT = "127.0.0.1", 6000

cur_x, cur_y, cur_alt = 0.0, 0.0, 2.0    # סט-פוינט לוגי בצד ה-GUI (לא טלמטריה)
cur_yaw_deg = 0.0

POS_STEP_DEFAULT  = 0.5   # m
ALT_STEP_DEFAULT  = 0.3   # m
YAW_STEP_DEFAULT  = 10.0  # deg

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
def _send(obj): sock.sendto(json.dumps(obj).encode("utf-8"), (SIM_IP, SIM_PORT))

def send_setpos(x,y,alt,yaw_deg=None):
    msg={"cmd":"setpos","x":float(x),"y":float(y),"alt":float(alt)}
    if yaw_deg is not None: msg["yaw_deg"]=float(yaw_deg)
    _send(msg); print(f"[TX] setpos -> x={x:.2f} y={y:.2f} alt={alt:.2f}" + (f" yaw={yaw_deg:.1f}°" if yaw_deg is not None else ""))

def send_nudge(dx=0.0,dy=0.0,dalt=0.0,dyaw_deg=0.0):
    msg={"cmd":"nudge","dx":float(dx),"dy":float(dy),"dalt":float(dalt),"dyaw_deg":float(dyaw_deg)}
    _send(msg); print(f"[TX] nudge  -> dB=({dx:.2f},{dy:.2f}) dAlt={dalt:.2f} dYaw={dyaw_deg:.1f}°")

root=tk.Tk(); root.title("UDP Drone Control → Isaac (Body Nudge + Yaw)")
status=tk.StringVar(value=f"x={cur_x:.2f} y={cur_y:.2f} alt={cur_alt:.2f} yaw={cur_yaw_deg:.1f}°")
ttk.Label(root,textvariable=status,anchor="w").grid(row=0,column=0,columnspan=8,sticky="ew",padx=8,pady=(8,4))

# צעדים
frm_steps=ttk.LabelFrame(root,text="גדלי צעדים"); frm_steps.grid(row=1,column=0,columnspan=8,sticky="ew",padx=8,pady=4)
pos_step=tk.DoubleVar(value=POS_STEP_DEFAULT); alt_step=tk.DoubleVar(value=ALT_STEP_DEFAULT); yaw_step=tk.DoubleVar(value=YAW_STEP_DEFAULT)
ttk.Label(frm_steps,text="ΔXY [m]:").grid(row=0,column=0); ttk.Entry(frm_steps,textvariable=pos_step,width=7).grid(row=0,column=1)
ttk.Label(frm_steps,text="ΔAlt [m]:").grid(row=0,column=2,padx=8); ttk.Entry(frm_steps,textvariable=alt_step,width=7).grid(row=0,column=3)
ttk.Label(frm_steps,text="ΔYaw [°]:").grid(row=0,column=4,padx=8); ttk.Entry(frm_steps,textvariable=yaw_step,width=7).grid(row=0,column=5)

# סליידר גובה ( שולח SETPOS כדי להצמיד לגובה מוחלט )
def on_alt(val):
    global cur_alt
    cur_alt=float(val)
    send_setpos(cur_x,cur_y,cur_alt,cur_yaw_deg)
    status.set(f"x={cur_x:.2f} y={cur_y:.2f} alt={cur_alt:.2f} yaw={cur_yaw_deg:.1f}°")

alt_slider=ttk.Scale(root,from_=6.0,to=0.0,orient="vertical",command=on_alt,length=220)
alt_slider.set(cur_alt); alt_slider.grid(row=2,column=0,rowspan=7,padx=8,pady=8,sticky="ns")
ttk.Label(root,text="גובה [m]").grid(row=9,column=0)

# תנועה (NUDGE בגוף)
def move(dir):
    step=float(pos_step.get())
    dx=dy=0.0
    if dir=="f": dx=+step          # forward = +X(body)
    elif dir=="b": dx=-step
    elif dir=="l": dy=-step        # left = -Y(body)
    elif dir=="r": dy=+step
    send_nudge(dx=dx,dy=dy)
    # עדכון לוגי בלבד (הסימולטור עושה רוטציה בפועל)
    status.set(f"x={cur_x:.2f} y={cur_y:.2f} alt={cur_alt:.2f} yaw={cur_yaw_deg:.1f}°")

def do_alt(dir):
    global cur_alt
    step=float(alt_step.get()); da= step if dir=="up" else -step
    cur_alt+=da; alt_slider.set(cur_alt)
    send_setpos(cur_x,cur_y,cur_alt,cur_yaw_deg)
    status.set(f"x={cur_x:.2f} y={cur_y:.2f} alt={cur_alt:.2f} yaw={cur_yaw_deg:.1f}°")

def do_yaw(dir):
    global cur_yaw_deg
    step=float(yaw_step.get()); dyaw= +step if dir=="right" else -step
    cur_yaw_deg = (cur_yaw_deg + dyaw + 180.0)%360.0 - 180.0
    send_nudge(dyaw_deg=dyaw)
    status.set(f"x={cur_x:.2f} y={cur_y:.2f} alt={cur_alt:.2f} yaw={cur_yaw_deg:.1f}°")

# Go To (SETPOS מוחלט)
frm_go=ttk.LabelFrame(root,text="Go To (מוחלט)")
frm_go.grid(row=8,column=1,columnspan=7,sticky="ew",padx=8,pady=6)
gx=tk.DoubleVar(value=cur_x); gy=tk.DoubleVar(value=cur_y); ga=tk.DoubleVar(value=cur_alt); gyaw=tk.DoubleVar(value=cur_yaw_deg)
for i,(lbl,var) in enumerate([("X:",gx),("Y:",gy),("Alt:",ga),("Yaw°:",gyaw)]):
    ttk.Label(frm_go,text=lbl).grid(row=0,column=2*i,padx=4)
    ttk.Entry(frm_go,textvariable=var,width=8).grid(row=0,column=2*i+1)
def go_to():
    global cur_x,cur_y,cur_alt,cur_yaw_deg
    cur_x=float(gx.get()); cur_y=float(gy.get()); cur_alt=float(ga.get()); cur_yaw_deg=float(gyaw.get())
    alt_slider.set(cur_alt)
    send_setpos(cur_x,cur_y,cur_alt,cur_yaw_deg)
    status.set(f"x={cur_x:.2f} y={cur_y:.2f} alt={cur_alt:.2f} yaw={cur_yaw_deg:.1f}°")
ttk.Button(frm_go,text="Go!",command=go_to).grid(row=0,column=8,padx=8)

# פריסה
ttk.Button(root,text="⬆️ קדימה", command=lambda: move("f")).grid(row=2,column=2,columnspan=2,sticky="ew",pady=4)
ttk.Button(root,text="⬅️ שמאלה", command=lambda: move("l")).grid(row=3,column=2,sticky="ew",padx=4,pady=4)
ttk.Button(root,text="➡️ ימינה",  command=lambda: move("r")).grid(row=3,column=3,sticky="ew",padx=4,pady=4)
ttk.Button(root,text="⬇️ אחורה", command=lambda: move("b")).grid(row=4,column=2,columnspan=2,sticky="ew",pady=4)

ttk.Button(root,text="↺ Yaw Left",  command=lambda: do_yaw("left")).grid(row=5,column=2,sticky="ew",padx=4,pady=4)
ttk.Button(root,text="↻ Yaw Right", command=lambda: do_yaw("right")).grid(row=5,column=3,sticky="ew",padx=4,pady=4)

ttk.Button(root,text="⬆️ Alt+", command=lambda: do_alt("up")).grid(row=6,column=2,sticky="ew",padx=4,pady=4)
ttk.Button(root,text="⬇️ Alt-", command=lambda: do_alt("down")).grid(row=6,column=3,sticky="ew",padx=4,pady=4)

# קיצורי מקלדת: WASD לתנועה יחסית גוף, Q/E לגובה, Z/C ל-YAW
def on_key(e):
    k=e.keysym.lower()
    if k=="w": move("f")
    elif k=="s": move("b")
    elif k=="a": move("l")
    elif k=="d": move("r")
    elif k=="q": do_alt("down")
    elif k=="e": do_alt("up")
    elif k=="z": do_yaw("left")
    elif k=="c": do_yaw("right")
root.bind("<Key>", on_key)

# סט-פוינט פתיחה להצמדה
send_setpos(cur_x,cur_y,cur_alt,cur_yaw_deg)
root.mainloop()
