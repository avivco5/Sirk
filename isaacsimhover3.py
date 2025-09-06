# Isaac Sim 5.0 — UDP JSON hover + YAW (torque around world Z)
from pxr import Usd, UsdPhysics, PhysxSchema, Gf
import carb, omni, math, json, socket, threading, time
import omni.usd, omni.timeline, omni.kit.app
from omni.isaac.dynamic_control import _dynamic_control

# ===== User =====
IRIS_ROOT_PATH = "/World/iris"
BODY_CANDIDATES = ["body","base_link"]
UDP_HOST, UDP_PORT = "0.0.0.0", 6000
VERBOSE_RX, RX_PRINT_PERIOD = True, 0.25
APPLY_BODY_CONVEX_FIX = True

# ===== Targets & gains =====
TARGET_POS_XY = Gf.Vec2d(0.0,0.0)
TARGET_ALT_Z  = 2.0
TARGET_YAW    = 0.0         # rad (world Z)

KP_Z,KD_Z   = 1.8,3.4
KP_XY,KD_XY = 1.2,0.8
VEL_CLAMP_Z,VEL_CLAMP_XY = 0.7,1.2
Z_ERR_DB,Z_VEL_DB = 0.03,0.05
XY_ERR_DB,XY_VEL_DB = 0.02,0.05
EMA_VZ,EMA_VXY = 0.25,0.25
SMOOTH_Z,SMOOTH_XY = 0.5,0.5

# Yaw via torque
KPY_TAU, KDY_TAU = 6.0, 1.8          # מומנט = Kp*eyaw + Kd*(-wz)
TAU_Z_CLAMP = 8.0                    # N*m (התאם לפי המסה/אינרציה)
YAW_ERR_DB, YAW_RATE_DB = math.radians(1.0), math.radians(2.0)

ANG_DAMPING, LIN_DAMPING = 2.0, 0.4  # פחות דעיכה כדי לאפשר סיבוב
MAX_ANG_VEL = 12.0

# ===== Internals =====
dc = _dynamic_control.acquire_dynamic_control_interface()
_stage = omni.usd.get_context().get_stage()
_app = omni.kit.app.get_app()
_timeline = omni.timeline.get_timeline_interface()
_update_sub = None
_body_path = None
_body_handle = None

_vx_meas_f=_vy_meas_f=_vz_meas_f=0.0
_vx_cmd_s=_vy_cmd_s=_vz_cmd_s=0.0

_udp_sock=None; _udp_thread=None; _udp_stop=False; _last_rx_print=0.0
_last_yaw_dbg = 0.0

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

def _on_update(e):
    global _body_handle, _vx_meas_f,_vy_meas_f,_vz_meas_f, _vx_cmd_s,_vy_cmd_s,_vz_cmd_s, _last_yaw_dbg
    if not _timeline.is_playing() or _body_path is None: return
    if _body_handle is None:
        try: _body_handle = dc.get_rigid_body(_body_path)
        except: _body_handle=None
        if _body_handle is None: return

    try: dc.wake_up_rigid_body(_body_handle)
    except: pass

    p,v,w,yaw = _get_pose_dc(_body_handle)

    # --- YAW via torque about world Z ---
    wz = float(w[2])
    eyaw = _wrap_pi(TARGET_YAW - yaw)
    if abs(eyaw)<YAW_ERR_DB and abs(wz)<YAW_RATE_DB:
        tau_z = 0.0
    else:
        tau_z = KPY_TAU*eyaw + KDY_TAU*(-wz)
        tau_z = _clamp(tau_z, -TAU_Z_CLAMP, TAU_Z_CLAMP)
    dc.apply_body_torque(_body_handle, carb.Float3(0.0, 0.0, tau_z), True)

    # Yaw debug (0.5s)
    now=time.time()
    if now-_last_yaw_dbg>0.5:
        print(f"[Yaw] tgt={math.degrees(TARGET_YAW):.1f}° cur={math.degrees(yaw):.1f}° err={math.degrees(eyaw):.1f}° wz={wz:.2f} rad/s τz={tau_z:.2f} N·m", flush=True)
        _last_yaw_dbg=now

    # --- Vertical (Z) ---
    _vz_meas_f = (1.0-EMA_VZ)*_vz_meas_f + EMA_VZ*v[2]
    ez, evz = TARGET_ALT_Z - p[2], -_vz_meas_f
    vz_raw = 0.0 if (abs(ez)<Z_ERR_DB and abs(_vz_meas_f)<Z_VEL_DB) else (KP_Z*ez + KD_Z*evz)
    vz_raw = _clamp(vz_raw, -VEL_CLAMP_Z, VEL_CLAMP_Z)
    _vz_cmd_s = (1.0-SMOOTH_Z)*_vz_cmd_s + SMOOTH_Z*vz_raw

    # --- Planar (X,Y) ---
    _vx_meas_f = (1.0-EMA_VXY)*_vx_meas_f + EMA_VXY*v[0]
    _vy_meas_f = (1.0-EMA_VXY)*_vy_meas_f + EMA_VXY*v[1]
    ex,ey = TARGET_POS_XY[0]-p[0], TARGET_POS_XY[1]-p[1]
    evx,evy = -_vx_meas_f, -_vy_meas_f
    vx_raw = 0.0 if (abs(ex)<XY_ERR_DB and abs(_vx_meas_f)<XY_VEL_DB) else (KP_XY*ex + KD_XY*evx)
    vy_raw = 0.0 if (abs(ey)<XY_ERR_DB and abs(_vy_meas_f)<XY_VEL_DB) else (KP_XY*ey + KD_XY*evy)
    vx_raw = _clamp(vx_raw, -VEL_CLAMP_XY, VEL_CLAMP_XY)
    vy_raw = _clamp(vy_raw, -VEL_CLAMP_XY, VEL_CLAMP_XY)
    _vx_cmd_s = (1.0-SMOOTH_XY)*_vx_cmd_s + SMOOTH_XY*vx_raw
    _vy_cmd_s = (1.0-SMOOTH_XY)*_vy_cmd_s + SMOOTH_XY*vy_raw
    dc.set_rigid_body_linear_velocity(_body_handle, carb.Float3(_vx_cmd_s,_vy_cmd_s,_vz_cmd_s))

# ---------- UDP ----------
_udp_sock=None; _udp_thread=None; _udp_stop=False; _last_rx_print=0.0
def _rx_print(s):
    global _last_rx_print
    if not VERBOSE_RX: return
    now=time.time()
    if now-_last_rx_print>=RX_PRINT_PERIOD:
        print(f"[UDP RX] {s}", flush=True)
        _last_rx_print=now

def _consume_json(msg:dict):
    global TARGET_POS_XY, TARGET_ALT_Z, TARGET_YAW
    cmd = str(msg.get("cmd","")).lower()
    def yaw_deg_to_rad(k):
        return _wrap_pi(math.radians(float(msg[k]))) if k in msg else None

    if cmd=="setpos":
        x=float(msg.get("x",TARGET_POS_XY[0])); y=float(msg.get("y",TARGET_POS_XY[1])); alt=float(msg.get("alt",TARGET_ALT_Z))
        yawr=yaw_deg_to_rad("yaw_deg")
        TARGET_POS_XY=Gf.Vec2d(x,y); TARGET_ALT_Z=alt
        if yawr is not None: TARGET_YAW=yawr
        _rx_print(f"setpos → x={x:.2f} y={y:.2f} alt={alt:.2f} yaw={math.degrees(TARGET_YAW):.1f}°" if yawr is not None else f"setpos → x={x:.2f} y={y:.2f} alt={alt:.2f}")
    elif cmd=="nudge":
        dx=float(msg.get("dx",0.0)); dy=float(msg.get("dy",0.0)); da=float(msg.get("dalt",0.0)); dyaw=float(msg.get("dyaw_deg",0.0))
        TARGET_POS_XY=Gf.Vec2d(TARGET_POS_XY[0]+dx, TARGET_POS_XY[1]+dy)
        TARGET_ALT_Z += da
        if abs(dyaw)>1e-6: TARGET_YAW = _wrap_pi(TARGET_YAW + math.radians(dyaw))
        _rx_print(f"nudge → x={TARGET_POS_XY[0]:.2f} y={TARGET_POS_XY[1]:.2f} alt={TARGET_ALT_Z:.2f} yaw={math.degrees(TARGET_YAW):.1f}°")
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
    _udp_sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); _udp_sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
    _udp_sock.bind((UDP_HOST,UDP_PORT)); _udp_sock.settimeout(0.1)
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
_stage = omni.usd.get_context().get_stage()
_app = omni.kit.app.get_app(); _timeline = omni.timeline.get_timeline_interface()

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
        _update_sub = _app.get_update_event_stream().create_subscription_to_pop(_on_update, name="IrisHover_YAW_Torque")
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
