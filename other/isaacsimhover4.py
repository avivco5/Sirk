# Isaac Sim 5.0 — Script Editor
# Iris existing + UDP JSON listener (XY, Alt, Yaw) + hover control.
# NUDGE IS BODY-FRAME: dx/dy interpreted in drone frame (x=forward, y=right).
# UDP JSON on 0.0.0.0:6000:
#   {"cmd":"setpos","x":<m>,"y":<m>,"alt":<m>,"yaw_deg":<deg-optional>}   # absolute world
#   {"cmd":"nudge","dx":<m>,"dy":<m>,"dalt":<m>,"dyaw_deg":<deg-optional>} # BODY-FRAME by default
#   {"cmd":"setyaw","yaw_deg":<deg>}

from pxr import Usd, UsdPhysics, PhysxSchema, Gf
import carb, omni, math, json, socket, threading, time
import omni.usd, omni.timeline, omni.kit.app
from omni.isaac.dynamic_control import _dynamic_control

# ===================== USER SETTINGS =====================
IRIS_ROOT_PATH = "/World/iris"
BODY_CANDIDATES = ["body","base_link"]

UDP_HOST, UDP_PORT = "0.0.0.0", 6000
VERBOSE_RX, RX_PRINT_PERIOD = True, 0.25
APPLY_BODY_CONVEX_FIX = True
# =========================================================

# ===== Targets & gains =====
TARGET_POS_XY = Gf.Vec2d(0.0,0.0)   # world meters
TARGET_ALT_Z  = 2.0                 # world Z up
TARGET_YAW    = 0.0                 # radians (about world Z)

# PD (linear)
KP_Z,KD_Z     = 1.8,3.4
KP_XY,KD_XY   = 1.2,0.8
VEL_CLAMP_Z   = 0.7
VEL_CLAMP_XY  = 1.2
Z_ERR_DB,Z_VEL_DB   = 0.03,0.05
XY_ERR_DB,XY_VEL_DB = 0.02,0.05
EMA_VZ,EMA_VXY      = 0.25,0.25
SMOOTH_Z,SMOOTH_XY  = 0.5,0.5

# Yaw via torque (robust in PhysX)
KPY_TAU, KDY_TAU = 6.0, 1.8
TAU_Z_CLAMP      = 8.0     # N*m
YAW_ERR_DB, YAW_RATE_DB = math.radians(1.0), math.radians(2.0)

ANG_DAMPING, LIN_DAMPING = 2.0, 0.4
MAX_ANG_VEL = 12.0
# ===========================================

# ---------- Internals ----------
dc = _dynamic_control.acquire_dynamic_control_interface()
_stage = omni.usd.get_context().get_stage()
_app = omni.kit.app.get_app()
_timeline = omni.timeline.get_timeline_interface()
_update_sub = None
_body_path = None
_body_handle = None

_vx_meas_f=_vy_meas_f=_vz_meas_f=0.0
_vx_cmd_s=_vy_cmd_s=_vz_cmd_s=0.0
_measured_yaw = 0.0
_last_yaw_dbg = 0.0

# UDP state
_udp_sock=None; _udp_thread=None; _udp_stop=False; _last_rx_print=0.0

# ---------- Helpers ----------
def _get_prim(path):
    p = _stage.GetPrimAtPath(path)
    return p if p and p.IsValid() else None

def _find_body_prim(root_prim):
    for name in BODY_CANDIDATES:
        cand = _stage.GetPrimAtPath(f"{root_prim.GetPath().pathString}/{name}")
        if cand and cand.IsValid(): return cand
    for p in Usd.PrimRange(root_prim):
        try:
            if p.HasAPI(UsdPhysics.RigidBodyAPI): return p
        except: pass
    return root_prim

def _apply_physx_on_body(prim):
    if not prim.HasAPI(UsdPhysics.RigidBodyAPI): UsdPhysics.RigidBodyAPI.Apply(prim)
    if not prim.HasAPI(UsdPhysics.CollisionAPI): UsdPhysics.CollisionAPI.Apply(prim)
    try: UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(False)
    except: pass
    rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    rb.CreateAngularDampingAttr(ANG_DAMPING)
    rb.CreateLinearDampingAttr(LIN_DAMPING)
    try: rb.CreateMaxAngularVelocityAttr(MAX_ANG_VEL)
    except: pass
    try: rb.CreateEnableGyroscopicForcesAttr(True)
    except: pass

def _optional_fix_body_convex(prim):
    try:
        if prim.GetTypeName()=="Mesh":
            if not prim.HasAPI(UsdPhysics.CollisionAPI): UsdPhysics.CollisionAPI.Apply(prim)
            m = UsdPhysics.MeshCollisionAPI.Apply(prim)
            m.CreateApproximationAttr("convexDecomposition").Set("convexDecomposition")
            print("[Fix] Body mesh set to convexDecomposition", flush=True)
    except Exception as e:
        print("[Fix] WARN:", e, flush=True)

def _wrap_pi(a): return (a+math.pi)%(2.0*math.pi)-math.pi
def _clamp(x,a,b): return b if x>b else a if x<a else x

def _get_pose_dc(h):
    pose = dc.get_rigid_body_pose(h)
    lin  = dc.get_rigid_body_linear_velocity(h)
    ang  = dc.get_rigid_body_angular_velocity(h)
    p = (pose.p.x, pose.p.y, pose.p.z)
    v = (lin.x, lin.y, lin.z)
    w = (ang.x, ang.y, ang.z)
    qx,qy,qz,qw = pose.r.x, pose.r.y, pose.r.z, pose.r.w
    siny_cosp = 2.0*(qw*qz + qx*qy)
    cosy_cosp = 1.0 - 2.0*(qy*qy + qz*qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return p,v,w,yaw

# ---------- Per-frame controller ----------
def _on_update(e):
    global _body_handle,_vx_meas_f,_vy_meas_f,_vz_meas_f,_vx_cmd_s,_vy_cmd_s,_vz_cmd_s,_measured_yaw,_last_yaw_dbg
    if not _timeline.is_playing() or _body_path is None: return
    if _body_handle is None:
        try: _body_handle = dc.get_rigid_body(_body_path)
        except: _body_handle=None
        if _body_handle is None: return

    try: dc.wake_up_rigid_body(_body_handle)
    except: pass

    p,v,w,yaw = _get_pose_dc(_body_handle)
    _measured_yaw = yaw  # <<< store for body-frame nudges

    # Yaw control via torque
    wz = float(w[2])
    eyaw = _wrap_pi(TARGET_YAW - yaw)
    if abs(eyaw)<YAW_ERR_DB and abs(wz)<YAW_RATE_DB:
        tau_z = 0.0
    else:
        tau_z = _clamp(KPY_TAU*eyaw + KDY_TAU*(-wz), -TAU_Z_CLAMP, TAU_Z_CLAMP)
    dc.apply_body_torque(_body_handle, carb.Float3(0.0, 0.0, tau_z), True)

    now=time.time()
    if now-_last_yaw_dbg>0.5:
        print(f"[Yaw] tgt={math.degrees(TARGET_YAW):.1f}° cur={math.degrees(yaw):.1f}° err={math.degrees(eyaw):.1f}° τz={tau_z:.2f}", flush=True)
        _last_yaw_dbg=now

    # Vertical Z (PD on velocity)
    _vz_meas_f = (1.0-EMA_VZ)*_vz_meas_f + EMA_VZ*v[2]
    ez, evz = TARGET_ALT_Z - p[2], -_vz_meas_f
    vz_raw = 0.0 if (abs(ez)<Z_ERR_DB and abs(_vz_meas_f)<Z_VEL_DB) else (KP_Z*ez + KD_Z*evz)
    _vz_cmd_s = (1.0-SMOOTH_Z)*_vz_cmd_s + SMOOTH_Z* _clamp(vz_raw,-VEL_CLAMP_Z,VEL_CLAMP_Z)

    # Planar XY
    _vx_meas_f = (1.0-EMA_VXY)*_vx_meas_f + EMA_VXY*v[0]
    _vy_meas_f = (1.0-EMA_VXY)*_vy_meas_f + EMA_VXY*v[1]
    ex,ey = TARGET_POS_XY[0]-p[0], TARGET_POS_XY[1]-p[1]
    evx,evy = -_vx_meas_f, -_vy_meas_f
    vx_raw = 0.0 if (abs(ex)<XY_ERR_DB and abs(_vx_meas_f)<XY_VEL_DB) else (KP_XY*ex + KD_XY*evx)
    vy_raw = 0.0 if (abs(ey)<XY_ERR_DB and abs(_vy_meas_f)<XY_VEL_DB) else (KP_XY*ey + KD_XY*evy)
    _vx_cmd_s = (1.0-SMOOTH_XY)*_vx_cmd_s + SMOOTH_XY* _clamp(vx_raw,-VEL_CLAMP_XY,VEL_CLAMP_XY)
    _vy_cmd_s = (1.0-SMOOTH_XY)*_vy_cmd_s + SMOOTH_XY* _clamp(vy_raw,-VEL_CLAMP_XY,VEL_CLAMP_XY)

    dc.set_rigid_body_linear_velocity(_body_handle, carb.Float3(_vx_cmd_s,_vy_cmd_s,_vz_cmd_s))

# ---------- UDP ----------
def _rx_print(s):
    global _last_rx_print
    if not VERBOSE_RX: return
    now=time.time()
    if now-_last_rx_print>=RX_PRINT_PERIOD:
        print(f"[UDP RX] {s}", flush=True)
        _last_rx_print=now

def _consume_json(msg:dict):
    global TARGET_POS_XY, TARGET_ALT_Z, TARGET_YAW, _measured_yaw
    cmd = str(msg.get("cmd","")).lower()

    def yaw_deg_to_rad(k):
        return _wrap_pi(math.radians(float(msg[k]))) if k in msg else None

    if cmd=="setpos":
        x=float(msg.get("x",TARGET_POS_XY[0])); y=float(msg.get("y",TARGET_POS_XY[1])); alt=float(msg.get("alt",TARGET_ALT_Z))
        yawr=yaw_deg_to_rad("yaw_deg")
        TARGET_POS_XY=Gf.Vec2d(x,y); TARGET_ALT_Z=alt
        if yawr is not None: TARGET_YAW=yawr
        _rx_print(f"setpos → x={x:.2f} y={y:.2f} alt={alt:.2f}" + (f" yaw={math.degrees(TARGET_YAW):.1f}°" if yawr is not None else ""))

    elif cmd=="nudge":
        # BODY-FRAME nudge → rotate by current yaw into world
        dx_b=float(msg.get("dx",0.0)); dy_b=float(msg.get("dy",0.0)); da=float(msg.get("dalt",0.0))
        dyaw_deg = float(msg.get("dyaw_deg",0.0))
        c,s = math.cos(_measured_yaw), math.sin(_measured_yaw)
        dx_w = c*dx_b - s*dy_b
        dy_w = s*dx_b + c*dy_b
        TARGET_POS_XY = Gf.Vec2d(TARGET_POS_XY[0] + dx_w, TARGET_POS_XY[1] + dy_w)
        TARGET_ALT_Z  = TARGET_ALT_Z + da
        if abs(dyaw_deg)>1e-6: TARGET_YAW = _wrap_pi(TARGET_YAW + math.radians(dyaw_deg))
        _rx_print(f"nudge (body) → dB=({dx_b:.2f},{dy_b:.2f}) dW=({dx_w:.2f},{dy_w:.2f}) ⇒ x={TARGET_POS_XY[0]:.2f} y={TARGET_POS_XY[1]:.2f} alt={TARGET_ALT_Z:.2f} yaw={math.degrees(TARGET_YAW):.1f}°")

    elif cmd=="setyaw":
        yawd=float(msg.get("yaw_deg",math.degrees(TARGET_YAW))); TARGET_YAW=_wrap_pi(math.radians(yawd))
        _rx_print(f"setyaw → yaw={yawd:.1f}°")
    else:
        _rx_print(f"unknown cmd: {cmd}")

def _udp_loop():
    print(f"[UDP] Listening on {UDP_HOST}:{UDP_PORT} ...", flush=True)
    while not _udp_stop:
        try:
            data,_= _udp_sock.recvfrom(8192)
        except socket.timeout: continue
        except Exception as e:
            if not _udp_stop: print("[UDP] recv error:",e, flush=True)
            continue
        try:
            _consume_json(json.loads(data.decode("utf-8","ignore")))
        except Exception:
            for line in data.splitlines():
                try: _consume_json(json.loads(line.decode("utf-8","ignore") if isinstance(line,bytes) else line))
                except: pass

def start_udp():
    global _udp_sock,_udp_thread,_udp_stop
    if _udp_thread is not None: print("[UDP] already running"); return
    _udp_stop=False
    _udp_sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    _udp_sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
    _udp_sock.bind((UDP_HOST,UDP_PORT))
    _udp_sock.settimeout(0.1)
    _udp_thread=threading.Thread(target=_udp_loop,daemon=True); _udp_thread.start()

def stop_udp():
    global _udp_sock,_udp_thread,_udp_stop
    _udp_stop=True
    if _udp_sock:
        try:_udp_sock.close()
        except: pass
        _udp_sock=None
    if _udp_thread:
        _udp_thread.join(timeout=1.0); _udp_thread=None
    print("[UDP] stopped", flush=True)

# ---------- lifecycle ----------
def start():
    global _body_path,_body_handle,_update_sub
    iris_root = _get_prim(IRIS_ROOT_PATH)
    if not iris_root: print(f"[Iris] Prim not found at {IRIS_ROOT_PATH}"); return
    body_prim = _find_body_prim(iris_root)
    if not body_prim: print("[Iris] No RigidBody under Iris"); return
    _body_path = body_prim.GetPath().pathString; _body_handle=None
    if APPLY_BODY_CONVEX_FIX: _optional_fix_body_convex(body_prim)
    _apply_physx_on_body(body_prim)
    if _update_sub is None:
        _update_sub = _app.get_update_event_stream().create_subscription_to_pop(_on_update, name="IrisHover_UDP_BodyNudge")
    start_udp()
    if not _timeline.is_playing(): _timeline.play()
    print(f"[Iris] Hover attached to {_body_path}. UDP on {UDP_HOST}:{UDP_PORT}.", flush=True)

def stop():
    global _update_sub,_body_handle
    if _update_sub: _update_sub.unsubscribe(); _update_sub=None
    _body_handle=None
    stop_udp()
    print("[Iris] Hover controller stopped.", flush=True)

start()
# Isaac Sim 5.0 — Script Editor
# Iris existing + UDP JSON listener (XY, Alt, Yaw) + hover control.
# NUDGE IS BODY-FRAME: dx/dy interpreted in drone frame (x=forward, y=right).
# UDP JSON on 0.0.0.0:6000:
#   {"cmd":"setpos","x":<m>,"y":<m>,"alt":<m>,"yaw_deg":<deg-optional>}   # absolute world
#   {"cmd":"nudge","dx":<m>,"dy":<m>,"dalt":<m>,"dyaw_deg":<deg-optional>} # BODY-FRAME by default
#   {"cmd":"setyaw","yaw_deg":<deg>}

from pxr import Usd, UsdPhysics, PhysxSchema, Gf
import carb, omni, math, json, socket, threading, time
import omni.usd, omni.timeline, omni.kit.app
from omni.isaac.dynamic_control import _dynamic_control

# ===================== USER SETTINGS =====================
IRIS_ROOT_PATH = "/World/iris"
BODY_CANDIDATES = ["body","base_link"]

UDP_HOST, UDP_PORT = "0.0.0.0", 6000
VERBOSE_RX, RX_PRINT_PERIOD = True, 0.25
APPLY_BODY_CONVEX_FIX = True
# =========================================================

# ===== Targets & gains =====
TARGET_POS_XY = Gf.Vec2d(0.0,0.0)   # world meters
TARGET_ALT_Z  = 2.0                 # world Z up
TARGET_YAW    = 0.0                 # radians (about world Z)

# PD (linear)
KP_Z,KD_Z     = 1.8,3.4
KP_XY,KD_XY   = 1.2,0.8
VEL_CLAMP_Z   = 0.7
VEL_CLAMP_XY  = 1.2
Z_ERR_DB,Z_VEL_DB   = 0.03,0.05
XY_ERR_DB,XY_VEL_DB = 0.02,0.05
EMA_VZ,EMA_VXY      = 0.25,0.25
SMOOTH_Z,SMOOTH_XY  = 0.5,0.5

# Yaw via torque (robust in PhysX)
KPY_TAU, KDY_TAU = 6.0, 1.8
TAU_Z_CLAMP      = 8.0     # N*m
YAW_ERR_DB, YAW_RATE_DB = math.radians(1.0), math.radians(2.0)

ANG_DAMPING, LIN_DAMPING = 2.0, 0.4
MAX_ANG_VEL = 12.0
# ===========================================

# ---------- Internals ----------
dc = _dynamic_control.acquire_dynamic_control_interface()
_stage = omni.usd.get_context().get_stage()
_app = omni.kit.app.get_app()
_timeline = omni.timeline.get_timeline_interface()
_update_sub = None
_body_path = None
_body_handle = None

_vx_meas_f=_vy_meas_f=_vz_meas_f=0.0
_vx_cmd_s=_vy_cmd_s=_vz_cmd_s=0.0
_measured_yaw = 0.0
_last_yaw_dbg = 0.0

# UDP state
_udp_sock=None; _udp_thread=None; _udp_stop=False; _last_rx_print=0.0

# ---------- Helpers ----------
def _get_prim(path):
    p = _stage.GetPrimAtPath(path)
    return p if p and p.IsValid() else None

def _find_body_prim(root_prim):
    for name in BODY_CANDIDATES:
        cand = _stage.GetPrimAtPath(f"{root_prim.GetPath().pathString}/{name}")
        if cand and cand.IsValid(): return cand
    for p in Usd.PrimRange(root_prim):
        try:
            if p.HasAPI(UsdPhysics.RigidBodyAPI): return p
        except: pass
    return root_prim

def _apply_physx_on_body(prim):
    if not prim.HasAPI(UsdPhysics.RigidBodyAPI): UsdPhysics.RigidBodyAPI.Apply(prim)
    if not prim.HasAPI(UsdPhysics.CollisionAPI): UsdPhysics.CollisionAPI.Apply(prim)
    try: UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(False)
    except: pass
    rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    rb.CreateAngularDampingAttr(ANG_DAMPING)
    rb.CreateLinearDampingAttr(LIN_DAMPING)
    try: rb.CreateMaxAngularVelocityAttr(MAX_ANG_VEL)
    except: pass
    try: rb.CreateEnableGyroscopicForcesAttr(True)
    except: pass

def _optional_fix_body_convex(prim):
    try:
        if prim.GetTypeName()=="Mesh":
            if not prim.HasAPI(UsdPhysics.CollisionAPI): UsdPhysics.CollisionAPI.Apply(prim)
            m = UsdPhysics.MeshCollisionAPI.Apply(prim)
            m.CreateApproximationAttr("convexDecomposition").Set("convexDecomposition")
            print("[Fix] Body mesh set to convexDecomposition", flush=True)
    except Exception as e:
        print("[Fix] WARN:", e, flush=True)

def _wrap_pi(a): return (a+math.pi)%(2.0*math.pi)-math.pi
def _clamp(x,a,b): return b if x>b else a if x<a else x

def _get_pose_dc(h):
    pose = dc.get_rigid_body_pose(h)
    lin  = dc.get_rigid_body_linear_velocity(h)
    ang  = dc.get_rigid_body_angular_velocity(h)
    p = (pose.p.x, pose.p.y, pose.p.z)
    v = (lin.x, lin.y, lin.z)
    w = (ang.x, ang.y, ang.z)
    qx,qy,qz,qw = pose.r.x, pose.r.y, pose.r.z, pose.r.w
    siny_cosp = 2.0*(qw*qz + qx*qy)
    cosy_cosp = 1.0 - 2.0*(qy*qy + qz*qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return p,v,w,yaw

# ---------- Per-frame controller ----------
def _on_update(e):
    global _body_handle,_vx_meas_f,_vy_meas_f,_vz_meas_f,_vx_cmd_s,_vy_cmd_s,_vz_cmd_s,_measured_yaw,_last_yaw_dbg
    if not _timeline.is_playing() or _body_path is None: return
    if _body_handle is None:
        try: _body_handle = dc.get_rigid_body(_body_path)
        except: _body_handle=None
        if _body_handle is None: return

    try: dc.wake_up_rigid_body(_body_handle)
    except: pass

    p,v,w,yaw = _get_pose_dc(_body_handle)
    _measured_yaw = yaw  # <<< store for body-frame nudges

    # Yaw control via torque
    wz = float(w[2])
    eyaw = _wrap_pi(TARGET_YAW - yaw)
    if abs(eyaw)<YAW_ERR_DB and abs(wz)<YAW_RATE_DB:
        tau_z = 0.0
    else:
        tau_z = _clamp(KPY_TAU*eyaw + KDY_TAU*(-wz), -TAU_Z_CLAMP, TAU_Z_CLAMP)
    dc.apply_body_torque(_body_handle, carb.Float3(0.0, 0.0, tau_z), True)

    now=time.time()
    if now-_last_yaw_dbg>0.5:
        print(f"[Yaw] tgt={math.degrees(TARGET_YAW):.1f}° cur={math.degrees(yaw):.1f}° err={math.degrees(eyaw):.1f}° τz={tau_z:.2f}", flush=True)
        _last_yaw_dbg=now

    # Vertical Z (PD on velocity)
    _vz_meas_f = (1.0-EMA_VZ)*_vz_meas_f + EMA_VZ*v[2]
    ez, evz = TARGET_ALT_Z - p[2], -_vz_meas_f
    vz_raw = 0.0 if (abs(ez)<Z_ERR_DB and abs(_vz_meas_f)<Z_VEL_DB) else (KP_Z*ez + KD_Z*evz)
    _vz_cmd_s = (1.0-SMOOTH_Z)*_vz_cmd_s + SMOOTH_Z* _clamp(vz_raw,-VEL_CLAMP_Z,VEL_CLAMP_Z)

    # Planar XY
    _vx_meas_f = (1.0-EMA_VXY)*_vx_meas_f + EMA_VXY*v[0]
    _vy_meas_f = (1.0-EMA_VXY)*_vy_meas_f + EMA_VXY*v[1]
    ex,ey = TARGET_POS_XY[0]-p[0], TARGET_POS_XY[1]-p[1]
    evx,evy = -_vx_meas_f, -_vy_meas_f
    vx_raw = 0.0 if (abs(ex)<XY_ERR_DB and abs(_vx_meas_f)<XY_VEL_DB) else (KP_XY*ex + KD_XY*evx)
    vy_raw = 0.0 if (abs(ey)<XY_ERR_DB and abs(_vy_meas_f)<XY_VEL_DB) else (KP_XY*ey + KD_XY*evy)
    _vx_cmd_s = (1.0-SMOOTH_XY)*_vx_cmd_s + SMOOTH_XY* _clamp(vx_raw,-VEL_CLAMP_XY,VEL_CLAMP_XY)
    _vy_cmd_s = (1.0-SMOOTH_XY)*_vy_cmd_s + SMOOTH_XY* _clamp(vy_raw,-VEL_CLAMP_XY,VEL_CLAMP_XY)

    dc.set_rigid_body_linear_velocity(_body_handle, carb.Float3(_vx_cmd_s,_vy_cmd_s,_vz_cmd_s))

# ---------- UDP ----------
def _rx_print(s):
    global _last_rx_print
    if not VERBOSE_RX: return
    now=time.time()
    if now-_last_rx_print>=RX_PRINT_PERIOD:
        print(f"[UDP RX] {s}", flush=True)
        _last_rx_print=now

def _consume_json(msg:dict):
    global TARGET_POS_XY, TARGET_ALT_Z, TARGET_YAW, _measured_yaw
    cmd = str(msg.get("cmd","")).lower()

    def yaw_deg_to_rad(k):
        return _wrap_pi(math.radians(float(msg[k]))) if k in msg else None

    if cmd=="setpos":
        x=float(msg.get("x",TARGET_POS_XY[0])); y=float(msg.get("y",TARGET_POS_XY[1])); alt=float(msg.get("alt",TARGET_ALT_Z))
        yawr=yaw_deg_to_rad("yaw_deg")
        TARGET_POS_XY=Gf.Vec2d(x,y); TARGET_ALT_Z=alt
        if yawr is not None: TARGET_YAW=yawr
        _rx_print(f"setpos → x={x:.2f} y={y:.2f} alt={alt:.2f}" + (f" yaw={math.degrees(TARGET_YAW):.1f}°" if yawr is not None else ""))

    elif cmd=="nudge":
        # BODY-FRAME nudge → rotate by current yaw into world
        dx_b=float(msg.get("dx",0.0)); dy_b=float(msg.get("dy",0.0)); da=float(msg.get("dalt",0.0))
        dyaw_deg = float(msg.get("dyaw_deg",0.0))
        c,s = math.cos(_measured_yaw), math.sin(_measured_yaw)
        dx_w = c*dx_b - s*dy_b
        dy_w = s*dx_b + c*dy_b
        TARGET_POS_XY = Gf.Vec2d(TARGET_POS_XY[0] + dx_w, TARGET_POS_XY[1] + dy_w)
        TARGET_ALT_Z  = TARGET_ALT_Z + da
        if abs(dyaw_deg)>1e-6: TARGET_YAW = _wrap_pi(TARGET_YAW + math.radians(dyaw_deg))
        _rx_print(f"nudge (body) → dB=({dx_b:.2f},{dy_b:.2f}) dW=({dx_w:.2f},{dy_w:.2f}) ⇒ x={TARGET_POS_XY[0]:.2f} y={TARGET_POS_XY[1]:.2f} alt={TARGET_ALT_Z:.2f} yaw={math.degrees(TARGET_YAW):.1f}°")

    elif cmd=="setyaw":
        yawd=float(msg.get("yaw_deg",math.degrees(TARGET_YAW))); TARGET_YAW=_wrap_pi(math.radians(yawd))
        _rx_print(f"setyaw → yaw={yawd:.1f}°")
    else:
        _rx_print(f"unknown cmd: {cmd}")

def _udp_loop():
    print(f"[UDP] Listening on {UDP_HOST}:{UDP_PORT} ...", flush=True)
    while not _udp_stop:
        try:
            data,_= _udp_sock.recvfrom(8192)
        except socket.timeout: continue
        except Exception as e:
            if not _udp_stop: print("[UDP] recv error:",e, flush=True)
            continue
        try:
            _consume_json(json.loads(data.decode("utf-8","ignore")))
        except Exception:
            for line in data.splitlines():
                try: _consume_json(json.loads(line.decode("utf-8","ignore") if isinstance(line,bytes) else line))
                except: pass

def start_udp():
    global _udp_sock,_udp_thread,_udp_stop
    if _udp_thread is not None: print("[UDP] already running"); return
    _udp_stop=False
    _udp_sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    _udp_sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
    _udp_sock.bind((UDP_HOST,UDP_PORT))
    _udp_sock.settimeout(0.1)
    _udp_thread=threading.Thread(target=_udp_loop,daemon=True); _udp_thread.start()

def stop_udp():
    global _udp_sock,_udp_thread,_udp_stop
    _udp_stop=True
    if _udp_sock:
        try:_udp_sock.close()
        except: pass
        _udp_sock=None
    if _udp_thread:
        _udp_thread.join(timeout=1.0); _udp_thread=None
    print("[UDP] stopped", flush=True)

# ---------- lifecycle ----------
def start():
    global _body_path,_body_handle,_update_sub
    iris_root = _get_prim(IRIS_ROOT_PATH)
    if not iris_root: print(f"[Iris] Prim not found at {IRIS_ROOT_PATH}"); return
    body_prim = _find_body_prim(iris_root)
    if not body_prim: print("[Iris] No RigidBody under Iris"); return
    _body_path = body_prim.GetPath().pathString; _body_handle=None
    if APPLY_BODY_CONVEX_FIX: _optional_fix_body_convex(body_prim)
    _apply_physx_on_body(body_prim)
    if _update_sub is None:
        _update_sub = _app.get_update_event_stream().create_subscription_to_pop(_on_update, name="IrisHover_UDP_BodyNudge")
    start_udp()
    if not _timeline.is_playing(): _timeline.play()
    print(f"[Iris] Hover attached to {_body_path}. UDP on {UDP_HOST}:{UDP_PORT}.", flush=True)

def stop():
    global _update_sub,_body_handle
    if _update_sub: _update_sub.unsubscribe(); _update_sub=None
    _body_handle=None
    stop_udp()
    print("[Iris] Hover controller stopped.", flush=True)

start()





"""
# isaac_listener_pose_visual_pivot_auto.py  (Isaac Sim 5.0)
# מציב Pose לפי UDP + מסובב פרופים ויזואלית סביב הציר הנכון עם pivot נכון.
# אוטו-זיהוי פרופים לפי גאומטריה; נפילה ל-Override ידני אם צריך.

import json, socket, threading, time, atexit
from pxr import Usd, UsdGeom, Gf
import omni.usd, omni.kit.app

# ===== הגדרות כלליות =====
UDP_PORT           = 6000
DRONE_ROOT         = "/World/iris"    # ⬅️ עדכן אם שם הפרים שלך שונה
AUTO_FIND_ON_MISS  = True
PRINT_EVERY_SEC    = 2.0

# סיבוב ויזואלי
VISUAL_SPIN        = True
SPIN_DEG_PER_SEC   = 3000.0

# חיפוש פרופים
# 1) זיהוי בשם (אם מכיל אחד מהמחרוזות הבאות):
PROP_NAME_HINTS    = ("prop", "rotor", "blade", "propeller")
# 2) זיהוי גאומטרי: Mesh "דק" (ציר סיבוב הוא הציר עם הממד הקטן ביותר)
GEOM_THIN_RATIO    = 6.0   # max/min >= 6 ייחשב "דק"
GEOM_MAX_COUNT     = 32    # לאסוף עד N פרימים "דקים" לכל היותר

# Override ידני (אם צריך—הכנס נתיבי פרופים מדויקים)
PROP_PATHS_MANUAL  = [
    # דוגמאות:
    # "/World/iris/prop_0",
    # "/World/iris/prop_1",
    # "/World/iris/prop_2",
    # "/World/iris/prop_3",
]

# ===== מצב משותף =====
_state_lock   = threading.Lock()
_target_pose  = {"x":0.0, "y":0.0, "z":2.0, "roll_deg":0.0, "pitch_deg":0.0, "yaw_deg":0.0}
_last_rx_time = 0.0
_quit_flag    = False

# ===== עזרי USD =====
def stage(): return omni.usd.get_context().get_stage()

def prim_exists(path):
    s = stage()
    if s is None: return False
    p = s.GetPrimAtPath(path)
    return bool(p) and p.IsValid()

def wait_world(timeout=10.0):
    t0 = time.perf_counter()
    while time.perf_counter()-t0 < timeout:
        s = stage()
        if s:
            w = s.GetPrimAtPath("/World")
            if w and w.IsValid(): return True
        time.sleep(0.1)
    return False

def auto_find_root():
    w = stage().GetPrimAtPath("/World")
    for p in Usd.PrimRange(w):
        if "iris" in p.GetName().lower() and p.IsValid():
            return p.GetPath().pathString
    return None

def ensure_pose_ops(xf: UsdGeom.Xformable):
    ops = xf.GetOrderedXformOps()
    t_op = r_op = None
    for op in ops:
        on = op.GetOpName()
        if on.startswith("xformOp:translate") and t_op is None: t_op = op
        elif on.startswith("xformOp:rotateXYZ") and r_op is None: r_op = op
    if t_op is None: t_op = xf.AddTranslateOp()
    if r_op is None: r_op = xf.AddRotateXYZOp()
    xf.SetXformOpOrder([t_op, r_op])
    return t_op, r_op

def set_pose_deg(prim_path, x,y,z, r_deg,p_deg,y_deg):
    p = stage().GetPrimAtPath(prim_path)
    if not p or not p.IsValid(): return
    xf = UsdGeom.Xformable(p)
    t_op, rot_op = ensure_pose_ops(xf)
    t_op.Set(Gf.Vec3d(x,y,z))
    rot_op.Set(Gf.Vec3f(r_deg,p_deg,y_deg))

# ===== סיבוב פרופים (עם pivot וציר אוטומטי) =====
_prop_infos = []  # [{path, angle, pre, rot, post, axis_char}]

def bbox_world(prim):
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
                              useExtentsHint=False)
    return cache.ComputeWorldBound(prim).GetBox()

def to_local(prim, pt_world: Gf.Vec3d):
    xcache = UsdGeom.XformCache(Usd.TimeCode.Default())
    M = xcache.GetLocalToWorldTransform(prim)
    return M.GetInverse().Transform(pt_world)

def ensure_pivot_stack(xf: UsdGeom.Xformable, pivot_local: Gf.Vec3d, axis_char: str):
    # מוסיפים שלישייה: translate(-pivot) -> rotateAxis -> translate(+pivot)
    ops = xf.GetOrderedXformOps()
    t_pre  = xf.AddTranslateOp()
    if axis_char == "X": r_op = xf.AddRotateXOp()
    elif axis_char == "Y": r_op = xf.AddRotateYOp()
    else: r_op = xf.AddRotateZOp()
    t_post = xf.AddTranslateOp()
    xf.SetXformOpOrder(ops + [t_pre, r_op, t_post])
    t_pre.Set(Gf.Vec3d(-pivot_local[0], -pivot_local[1], -pivot_local[2]))
    t_post.Set(Gf.Vec3d(+pivot_local[0], +pivot_local[1], +pivot_local[2]))
    return t_pre, r_op, t_post

def guess_axis_from_bbox(ext):
    # הציר עם הממד הקטן ביותר → כנראה נורמל ללהב
    dims = [abs(ext[0]), abs(ext[1]), abs(ext[2])]
    axis_idx = int(dims.index(min(dims)))
    return "XYZ"[axis_idx]

def find_props_by_name(root_prim):
    out = []
    for p in Usd.PrimRange(root_prim):
        if any(h in p.GetName().lower() for h in PROP_NAME_HINTS):
            out.append(p)
    return out

def find_props_by_geometry(root_prim):
    # מחפש Mesh-ים "דקים"
    out = []
    for p in Usd.PrimRange(root_prim):
        if p.GetTypeName() != "Mesh": continue
        b = bbox_world(p); ext = b.GetSize()  # world extents
        dims = [max(1e-6, abs(ext[0])), max(1e-6, abs(ext[1])), max(1e-6, abs(ext[2]))]
        if max(dims) / min(dims) >= GEOM_THIN_RATIO:
            out.append(p)
            if len(out) >= GEOM_MAX_COUNT: break
    return out

def prepare_prop_spinners(root_path):
    global _prop_infos
    _prop_infos = []
    s = stage()
    root = s.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        print(f"⚠️ prop-scan: invalid root '{root_path}'"); return

    candidates = []

    # 0) override ידני
    if PROP_PATHS_MANUAL:
        for path in PROP_PATHS_MANUAL:
            p = s.GetPrimAtPath(path)
            if p and p.IsValid():
                candidates.append(p)

    # 1) לפי שם
    if not candidates:
        candidates = find_props_by_name(root)

    # 2) לפי גאומטריה
    if not candidates:
        candidates = find_props_by_geometry(root)

    # הכנה
    for p in candidates:
        try:
            xf = UsdGeom.Xformable(p)
            if not xf: continue
            center_w = bbox_world(p).GetCenter()
            center_l = to_local(p, center_w)
            ext = bbox_world(p).GetSize()
            axis = guess_axis_from_bbox(ext)
            t_pre, r_op, t_post = ensure_pivot_stack(xf, center_l, axis)
            _prop_infos.append({
                "path": p.GetPath().pathString,
                "angle": 0.0,
                "pre": t_pre, "rot": r_op, "post": t_post,
                "axis_char": axis
            })
        except Exception as e:
            print(f"⚠️ prop prep failed for {p.GetPath()}: {e}")

    _prop_infos.sort(key=lambda d: d["path"])
    print(f"🔎 Prepared {len(_prop_infos)} prop prims:")
    for i, info in enumerate(_prop_infos):
        print(f"   • [{i}] {info['path']}  axis={info['axis_char']}")

def spin_props(dt):
    if not VISUAL_SPIN or not _prop_infos: return
    for i, info in enumerate(_prop_infos):
        sgn = +1.0 if (i % 2 == 0) else -1.0   # זוגות בכיוונים מנוגדים
        info["angle"] = (info["angle"] + sgn * SPIN_DEG_PER_SEC * dt) % 360.0
        info["rot"].Set(float(info["angle"]))

# ===== UDP =====
def udp_thread():
    global _last_rx_time
    print(f"📡 UDP listening on 0.0.0.0:{UDP_PORT} …")
    sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sk.bind(("0.0.0.0", UDP_PORT))
    sk.settimeout(0.25)
    while not _quit_flag:
        try:
            data, _ = sk.recvfrom(4096)
        except socket.timeout:
            continue
        except Exception as e:
            print("UDP error:", e); time.sleep(0.1); continue
        try:
            msg = json.loads(data.decode("utf-8", errors="ignore"))
        except Exception:
            continue
        if not isinstance(msg, dict) or msg.get("cmd") != "setpose":
            continue
        try:
            x  = float(msg.get("x", 0.0))
            y  = float(msg.get("y", 0.0))
            z  = float(msg.get("alt", msg.get("z", 0.0)))
            rd = float(msg.get("roll_deg", 0.0))
            pd = float(msg.get("pitch_deg", 0.0))
            yd = float(msg.get("yaw_deg", 0.0))
        except Exception:
            continue
        with _state_lock:
            _target_pose.update({"x":x,"y":y,"z":z,"roll_deg":rd,"pitch_deg":pd,"yaw_deg":yd})
            _last_rx_time = time.perf_counter()
    try: sk.close()
    except: pass

# ===== עדכון פר פריים =====
_app = omni.kit.app.get_app()
_last_t = time.perf_counter()
_last_log = time.perf_counter()
_DRONE_ROOT = DRONE_ROOT

def on_update(_e):
    global _last_t, _last_log
    now = time.perf_counter()
    dt  = max(1e-4, now - _last_t)
    _last_t = now

    with _state_lock:
        pose = dict(_target_pose)
        last_rx = _last_rx_time

    set_pose_deg(_DRONE_ROOT, pose["x"], pose["y"], pose["z"],
                 pose["roll_deg"], pose["pitch_deg"], pose["yaw_deg"])
    spin_props(dt)

    if now - _last_log > PRINT_EVERY_SEC:
        age = (now - last_rx) if last_rx else float("inf")
        print(f"[Pose] x={pose['x']:.2f} y={pose['y']:.2f} z={pose['z']:.2f} | "
              f"RPY=({pose['roll_deg']:.1f},{pose['pitch_deg']:.1f},{pose['yaw_deg']:.1f})° | "
              f"rx_age={age:.2f}s | props={len(_prop_infos)}")
        _last_log = now

# ===== Start =====
def start():
    global _DRONE_ROOT
    print("⏳ Waiting for USD stage …")
    if not wait_world(10.0):
        raise RuntimeError("No /World. Load your scene first.")

    if not prim_exists(_DRONE_ROOT):
        if AUTO_FIND_ON_MISS:
            guess = auto_find_root()
            if guess and prim_exists(guess):
                print(f"ℹ️ Using auto-detected drone prim: {guess}")
                _DRONE_ROOT = guess
            else:
                raise RuntimeError("Drone prim not found. Set DRONE_ROOT correctly.")
        else:
            raise RuntimeError(f"Drone prim '{_DRONE_ROOT}' not found.")

    # ודא טרנספורמים לרחפן
    p = stage().GetPrimAtPath(_DRONE_ROOT)
    ensure_pose_ops(UsdGeom.Xformable(p))

    # הכן ספינרים
    if VISUAL_SPIN:
        # לכתוב לשכבת session כדי לא לשנות קובץ USD
        stage().SetEditTarget(stage().GetSessionLayer())
        prepare_prop_spinners(_DRONE_ROOT)

    # UDP
    threading.Thread(target=udp_thread, daemon=True).start()

    # subscribe
    sub = _app.get_update_event_stream().create_subscription_to_pop(on_update)

    def cleanup():
        global _quit_flag
        _quit_flag = True
        try: sub.unsubscribe()
        except: pass
        print("👋 listener stopped")
    atexit.register(cleanup)

    print(f"✅ Visual listener ready. Drone prim: '{_DRONE_ROOT}'. UDP on 0.0.0.0:{UDP_PORT}")

start()


"""
