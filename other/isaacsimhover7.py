# Isaac Sim 5.0 — Iris hover + UDP + Rotor spin + Camera-under-body (create-if-missing) + Track Target Cube + Camera Preview
# UDP on 0.0.0.0:6000:
#   {"cmd":"setpose"/"setpos","x":..,"y":..,"alt":..,"yaw_deg":..}
#   {"cmd":"nudge","dx":..,"dy":..,"dalt":..,"dyaw_deg":..}
#   {"cmd":"setyaw","yaw_deg":..}
#   {"cmd":"spin","rpm":..}
#   {"cmd":"track","enable":true/false}
#   {"cmd":"set_target","path":"/World/track_cube"}   # לשינוי יעד המעקב

from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf
import carb, omni, math, json, socket, threading, time
import omni.usd, omni.timeline, omni.kit.app
from omni.isaac.dynamic_control import _dynamic_control

# ===================== USER SETTINGS =====================
IRIS_ROOT_PATH = "/World/iris"
BODY_CANDIDATES = ["body", "base_link"]

UDP_HOST, UDP_PORT = "0.0.0.0", 6000
VERBOSE_RX, RX_PRINT_PERIOD = True, 0.25
APPLY_BODY_CONVEX_FIX = True

# --- Camera (child of body) ---
CAMERA_NAME         = "camera_front"
CAM_OFFSET_LOCAL    = Gf.Vec3d(0.18, 0.0, 0.06)    # קדימה/ימינה/מעלה יחסית לגוף
CAM_ROT_DEG_XYZ     = Gf.Vec3f(0.0, -90.0, 0.0)    # להביט קדימה לאורך +X של הגוף
CAM_SCALE           = 0.20                         # "מצלמה קטנה" (ויזואלי)
CAM_CREATE_ONLY     = True                         # אם קיימת מצלמה באותו נתיב — לא נוגעים בה

# --- Camera preview window (small) ---
PREVIEW_WINDOW_ENABLE = True
PREVIEW_SIZE = (480, 270)   # רוחב, גובה
PREVIEW_TITLE = "Camera Preview"

# --- Target cube to follow (kinematic, move with gizmo) ---
TARGET_CUBE_PATH    = "/World/track_cube"
CUBE_SIZE           = 0.22
CUBE_START_POS      = Gf.Vec3d(1.5, 0.0, 1.2)
CUBE_COLOR_RGB      = Gf.Vec3f(0.95, 0.25, 0.1)     # צבע בולט

# --- Follow behavior ---
TRACK_ENABLED       = True                          # מעקב מופעל כברירת מחדל
TARGET_PRIM_PATH    = TARGET_CUBE_PATH              # יעד ברירת מחדל: הקובייה
FOLLOW_RADIUS_XY    = 1.5                           # מרחק אופקי קבוע מהמטרה [m]
ALT_OFFSET          = 0.6                           # גובה מעל המטרה [m]
YAW_FACE_TARGET     = True                          # לפנות אל המטרה

# --- Rotor visual spin (about local Z via xformOp:orient) ---
VISUAL_SPIN         = True
ROTOR_NAME_PREFIX   = "rotor"        # Xforms like rotor0/1/2/3
SPIN_RPM_CMD        = 0.0
SPIN_RPM_MIN, SPIN_RPM_MAX = 0.0, 40000.0
# =========================================================

# ===== Targets & gains =====
TARGET_POS_XY = Gf.Vec2d(0.0, 0.0)  # world meters
TARGET_ALT_Z  = 2.0                 # world Z up
TARGET_YAW    = 0.0                 # radians (about world Z)

KP_Z, KD_Z     = 1.8, 3.4
KP_XY, KD_XY   = 1.2, 0.8
VEL_CLAMP_Z    = 0.7
VEL_CLAMP_XY   = 1.2
Z_ERR_DB, Z_VEL_DB   = 0.03, 0.05
XY_ERR_DB, XY_VEL_DB = 0.02, 0.05
EMA_VZ, EMA_VXY      = 0.25, 0.25
SMOOTH_Z, SMOOTH_XY  = 0.5, 0.5

# Yaw via torque (robust in PhysX)
KPY_TAU, KDY_TAU = 6.0, 1.8
TAU_Z_CLAMP      = 8.0
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

_vx_meas_f = _vy_meas_f = _vz_meas_f = 0.0
_vx_cmd_s  = _vy_cmd_s  = _vz_cmd_s  = 0.0
_measured_yaw = 0.0
_last_yaw_dbg = 0.0

# UDP state
_udp_sock=None; _udp_thread=None; _udp_stop=False; _last_rx_print=0.0

# Rotors
_rotors=[]
_last_t  = time.perf_counter()
_last_log= time.perf_counter()

# Follow target internal
_target_path = TARGET_PRIM_PATH

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

# ---------- Camera (create only if missing) ----------
def _ensure_camera_under_body(body_path: str):
    """יוצר מצלמה קטנה כ-child של ה-body רק אם לא קיימת. אם קיימת — לא משנה אותה."""
    cam_path = f"{body_path}/{CAMERA_NAME}"
    if _get_prim(cam_path) and CAM_CREATE_ONLY:
        print(f"[Camera] Using existing {cam_path} (unchanged)", flush=True)
        return cam_path

    cam = UsdGeom.Camera.Define(_stage, cam_path)
    xf = UsdGeom.XformCommonAPI(cam)
    xf.SetTranslate(CAM_OFFSET_LOCAL)
    xf.SetRotate(CAM_ROT_DEG_XYZ, UsdGeom.XformCommonAPI.RotationOrderXYZ)
    xf.SetScale(Gf.Vec3f(CAM_SCALE, CAM_SCALE, CAM_SCALE))  # ויזואלי בלבד
    try:
        cam.CreateFocalLengthAttr(20.0)
        cam.CreateHorizontalApertureAttr(20.955)
        cam.CreateVerticalApertureAttr(15.2908)
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 1e5))
    except: pass
    print(f"[Camera] Created {cam_path}", flush=True)
    return cam_path

# ---------- Camera Preview (Viewport) ----------
_preview_win = None
_preview_release_fn = None

def _open_camera_preview(cam_path: str, size=(480, 270), title="Camera Preview"):
    """Try new viewport utility API; fall back to legacy API if needed."""
    global _preview_win, _preview_release_fn
    # 1) New utility (Isaac 4.x/5.x)
    try:
        import omni.kit.viewport.utility as vp_utils
        win = vp_utils.create_viewport_window(title, width=size[0], height=size[1])
        vp = win.viewport_api
        try:
            vp.set_active_camera(cam_path)
        except Exception:
            try:
                vp.set_active_camera_path(cam_path)
            except Exception:
                pass
        try: vp.set_texture_resolution(size[0], size[1])
        except Exception: pass

        _preview_win = win
        _preview_release_fn = (lambda: win.destroy() if hasattr(win, "destroy") else None)
        print(f"[Preview] Opened via viewport.utility → '{cam_path}' {size}", flush=True)
        return True
    except Exception as e:
        print("[Preview] viewport.utility failed:", e, flush=True)

    # 2) Legacy viewport (fallback)
    try:
        from omni.kit.viewport_legacy import get_viewport_interface
        vp_if = get_viewport_interface()
        vpi = vp_if.create_instance()
        vp_if.set_window_title(vpi, title)
        vp_if.resize_window(vpi, size[0], size[1])
        try:
            vp_if.set_active_camera(vpi, cam_path)
        except Exception:
            pass
        _preview_win = vpi
        _preview_release_fn = (lambda: vp_if.destroy_instance(vpi))
        print(f"[Preview] Opened via viewport_legacy → '{cam_path}' {size}", flush=True)
        return True
    except Exception as e:
        print("[Preview] viewport_legacy failed:", e, flush=True)

    print("[Preview] Could not open preview window (fallback to selecting camera in main Viewport).", flush=True)
    return False

def _close_camera_preview():
    global _preview_win, _preview_release_fn
    try:
        if _preview_release_fn:
            _preview_release_fn()
    except Exception:
        pass
    _preview_win = None
    _preview_release_fn = None
    print("[Preview] Closed", flush=True)

# ---------- Target cube (create if missing) ----------
def _ensure_target_cube():
    """יוצר קובייה קינמטית לעקיבה אם לא קיימת."""
    if _get_prim(TARGET_CUBE_PATH):
        print(f"[Target] Using existing {TARGET_CUBE_PATH}", flush=True); return
    cube = UsdGeom.Cube.Define(_stage, TARGET_CUBE_PATH)
    cube.CreateSizeAttr(CUBE_SIZE)
    xf = UsdGeom.XformCommonAPI(cube)
    xf.SetTranslate(CUBE_START_POS)
    # צבע בולט
    try:
        gprim = UsdGeom.Gprim(cube.GetPrim())
        gprim.CreateDisplayColorAttr([CUBE_COLOR_RGB])
    except: pass
    # פיזיקה: קינטי + קוליז’ן כדי שייראה בעולם
    UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
    PhysxSchema.PhysxRigidBodyAPI.Apply(cube.GetPrim())
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    UsdPhysics.RigidBodyAPI(cube.GetPrim()).CreateKinematicEnabledAttr(True)
    print(f"[Target] Created {TARGET_CUBE_PATH} @ {list(CUBE_START_POS)} size={CUBE_SIZE}", flush=True)

def _get_world_pos_by_dc_or_xform(path):
    """מנסה להביא מיקום עולמי דרך DC; אם לא, דרך טרנספורם."""
    try:
        h = dc.get_rigid_body(path)
        if h:
            p,_,_,_ = _get_pose_dc(h)
            return Gf.Vec3d(p[0], p[1], p[2])
    except: pass
    prim = _get_prim(path)
    if not prim: return None
    wm = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = wm.ExtractTranslation()
    return Gf.Vec3d(t[0], t[1], t[2])

# ---------- Rotors via orient(Z) ----------
def _quatd_to_quatf(qd: Gf.Quatd) -> Gf.Quatf:
    return Gf.Quatf(float(qd.GetReal()), Gf.Vec3f(*map(float, qd.GetImaginary())))

def _get_or_add_orient_op(xf: UsdGeom.Xformable):
    ops = xf.GetOrderedXformOps()
    for op in ops:
        if op.GetOpName().startswith("xformOp:orient"):
            return op
    orient = xf.AddOrientOp()
    xf.SetXformOpOrder(ops + [orient])
    return orient

def _get_quatf_or_identity(orient_op):
    try:
        q = orient_op.Get()
        if isinstance(q, Gf.Quatf): return q
        if isinstance(q, Gf.Quatd): return _quatd_to_quatf(q)
    except: pass
    return Gf.Quatf(1.0, Gf.Vec3f(0,0,0))

def _prepare_rotors(root_path):
    global _rotors
    _rotors = []
    root = _stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        print(f"⚠️ rotor-scan: invalid root '{root_path}'"); return
    _stage.SetEditTarget(_stage.GetSessionLayer())
    found=[]
    for p in Usd.PrimRange(root):
        if p.GetTypeName()=="Xform" and p.GetName().lower().startswith(ROTOR_NAME_PREFIX):
            found.append(p)
    found.sort(key=lambda prm: prm.GetPath().pathString)
    for i, prm in enumerate(found):
        xf = UsdGeom.Xformable(prm)
        if not xf: continue
        orient = _get_or_add_orient_op(xf)
        base_q = _get_quatf_or_identity(orient)
        _rotors.append({"path":prm.GetPath().pathString,"orient_op":orient,"base_q":base_q,"angle_deg":0.0,"sign":(+1.0 if i%2==0 else -1.0)})
    print(f"🔎 Prepared {len(_rotors)} rotors", flush=True)

def _spin_rotors(dt):
    if not VISUAL_SPIN or not _rotors: return
    rpm = _clamp(SPIN_RPM_CMD, SPIN_RPM_MIN, SPIN_RPM_MAX)
    rate_dps = rpm * 6.0
    if rate_dps <= 0.0: return
    for r in _rotors:
        r["angle_deg"] = (r["angle_deg"] + r["sign"] * rate_dps * dt) % 360.0
        rotZ = Gf.Rotation(Gf.Vec3d(0,0,1), r["angle_deg"]).GetQuaternion()
        qZf  = _quatd_to_quatf(rotZ)
        r["orient_op"].Set(r["base_q"] * qZf)

# ---------- Follow logic ----------
def _follow_target(p_drone_xy: Gf.Vec2d, p_drone_z: float):
    global TARGET_POS_XY, TARGET_ALT_Z, TARGET_YAW
    tgt = _get_world_pos_by_dc_or_xform(_target_path)
    if tgt is None: return
    tgt_xy = Gf.Vec2d(tgt[0], tgt[1])
    # יעד במרחק FOLLOW_RADIUS_XY מהמטרה בכיוון הנוכחי מהמרכז אל הרחפן:
    vec = Gf.Vec2d(p_drone_xy[0]-tgt_xy[0], p_drone_xy[1]-tgt_xy[1])
    dist = max(1e-6, math.hypot(vec[0], vec[1]))
    unit = Gf.Vec2d(vec[0]/dist, vec[1]/dist)
    TARGET_POS_XY = Gf.Vec2d(tgt_xy[0] + unit[0]*FOLLOW_RADIUS_XY,
                             tgt_xy[1] + unit[1]*FOLLOW_RADIUS_XY)
    TARGET_ALT_Z  = float(tgt[2] + ALT_OFFSET)
    if YAW_FACE_TARGET:
        ey = tgt_xy[1] - p_drone_xy[1]
        ex = tgt_xy[0] - p_drone_xy[0]
        TARGET_YAW = _wrap_pi(math.atan2(ey, ex))

# ---------- Per-frame controller ----------
def _on_update(e):
    global _body_handle,_vx_meas_f,_vy_meas_f,_vz_meas_f,_vx_cmd_s,_vy_cmd_s,_vz_cmd_s,_measured_yaw,_last_yaw_dbg,_last_t,_last_log
    if not _timeline.is_playing(): return

    now = time.perf_counter()
    dt  = max(1e-4, now - _last_t); _last_t = now

    _spin_rotors(dt)

    if _body_path is None: return
    if _body_handle is None:
        try: _body_handle = dc.get_rigid_body(_body_path)
        except: _body_handle = None
        if _body_handle is None: return

    try: dc.wake_up_rigid_body(_body_handle)
    except: pass

    p,v,w,yaw = _get_pose_dc(_body_handle)
    drone_xy = Gf.Vec2d(p[0], p[1])
    _measured_yaw = yaw

    # Follow target
    if TRACK_ENABLED:
        _follow_target(drone_xy, p[2])

    # Yaw torque control
    wz = float(w[2])
    eyaw = _wrap_pi(TARGET_YAW - yaw)
    tau_z = 0.0 if (abs(eyaw)<YAW_ERR_DB and abs(wz)<YAW_RATE_DB) else _clamp(KPY_TAU*eyaw + KDY_TAU*(-wz), -TAU_Z_CLAMP, TAU_Z_CLAMP)
    dc.apply_body_torque(_body_handle, carb.Float3(0.0,0.0,tau_z), True)

    tnow = time.time()
    if tnow - _last_yaw_dbg > 0.5:
        print(f"[Yaw] tgt={math.degrees(TARGET_YAW):.1f}° cur={math.degrees(yaw):.1f}° err={math.degrees(eyaw):.1f}° τz={tau_z:.2f}", flush=True)
        _last_yaw_dbg = tnow

    # Z velocity PD
    _vz_meas_f = (1.0-EMA_VZ)*_vz_meas_f + EMA_VZ*v[2]
    ez, evz = TARGET_ALT_Z - p[2], -_vz_meas_f
    vz_raw = 0.0 if (abs(ez)<Z_ERR_DB and abs(_vz_meas_f)<Z_VEL_DB) else (KP_Z*ez + KD_Z*evz)
    _vz_cmd_s = (1.0-SMOOTH_Z)*_vz_cmd_s + SMOOTH_Z*_clamp(vz_raw,-VEL_CLAMP_Z,VEL_CLAMP_Z)

    # XY velocity PD
    _vx_meas_f = (1.0-EMA_VXY)*_vx_meas_f + EMA_VXY*v[0]
    _vy_meas_f = (1.0-EMA_VXY)*_vy_meas_f + EMA_VXY*v[1]
    ex,ey = TARGET_POS_XY[0]-p[0], TARGET_POS_XY[1]-p[1]
    evx,evy = -_vx_meas_f, -_vy_meas_f
    vx_raw = 0.0 if (abs(ex)<XY_ERR_DB and abs(_vx_meas_f)<XY_VEL_DB) else (KP_XY*ex + KD_XY*evx)
    vy_raw = 0.0 if (abs(ey)<XY_ERR_DB and abs(_vy_meas_f)<XY_VEL_DB) else (KP_XY*ey + KD_XY*evy)
    _vx_cmd_s = (1.0-SMOOTH_XY)*_vx_cmd_s + SMOOTH_XY*_clamp(vx_raw,-VEL_CLAMP_XY,VEL_CLAMP_XY)
    _vy_cmd_s = (1.0-SMOOTH_XY)*_vy_cmd_s + SMOOTH_XY*_clamp(vy_raw,-VEL_CLAMP_XY,VEL_CLAMP_XY)

    dc.set_rigid_body_linear_velocity(_body_handle, carb.Float3(_vx_cmd_s,_vy_cmd_s,_vz_cmd_s))

    if now - _last_log > 2.0:
        print(f"[Pose] x={p[0]:.2f} y={p[1]:.2f} z={p[2]:.2f} | track={TRACK_ENABLED} tgt='{_target_path}' rpm={SPIN_RPM_CMD:.0f}", flush=True)
        _last_log = now

# ---------- UDP ----------
def _rx_print(s):
    global _last_rx_print
    if not VERBOSE_RX: return
    now = time.time()
    if now - _last_rx_print >= RX_PRINT_PERIOD:
        print(f"[UDP RX] {s}", flush=True)
        _last_rx_print = now

def _consume_json(msg: dict):
    global TARGET_POS_XY, TARGET_ALT_Z, TARGET_YAW, SPIN_RPM_CMD, TRACK_ENABLED, _target_path
    cmd = str(msg.get("cmd", "")).lower()

    def yaw_deg_to_rad(k):
        return _wrap_pi(math.radians(float(msg[k]))) if k in msg else None

    if cmd in ("setpos", "setpose"):
        x = float(msg.get("x", TARGET_POS_XY[0]))
        y = float(msg.get("y", TARGET_POS_XY[1]))
        alt = float(msg.get("alt", TARGET_ALT_Z))
        yawr = yaw_deg_to_rad("yaw_deg")
        TARGET_POS_XY = Gf.Vec2d(x, y); TARGET_ALT_Z = alt
        if yawr is not None: TARGET_YAW = yawr
        _rx_print(f"setpose → x={x:.2f} y={y:.2f} alt={alt:.2f}" + (f" yaw={math.degrees(TARGET_YAW):.1f}°" if yawr is not None else ""))

    elif cmd == "nudge":
        dx_b = float(msg.get("dx", 0.0)); dy_b = float(msg.get("dy", 0.0)); da = float(msg.get("dalt", 0.0))
        dyaw_deg = float(msg.get("dyaw_deg", 0.0))
        c, s = math.cos(_measured_yaw), math.sin(_measured_yaw)
        dx_w = c*dx_b - s*dy_b; dy_w = s*dx_b + c*dy_b
        TARGET_POS_XY = Gf.Vec2d(TARGET_POS_XY[0] + dx_w, TARGET_POS_XY[1] + dy_w)
        TARGET_ALT_Z  = TARGET_ALT_Z + da
        if abs(dyaw_deg) > 1e-6: TARGET_YAW = _wrap_pi(TARGET_YAW + math.radians(dyaw_deg))
        _rx_print(f"nudge(body) → dW=({dx_w:.2f},{dy_w:.2f}) alt={TARGET_ALT_Z:.2f} yaw={math.degrees(TARGET_YAW):.1f}°")

    elif cmd == "setyaw":
        yawd = float(msg.get("yaw_deg", math.degrees(TARGET_YAW))); TARGET_YAW = _wrap_pi(math.radians(yawd))
        _rx_print(f"setyaw → yaw={yawd:.1f}°")

    elif cmd == "spin":
        try:    rpm = float(msg.get("rpm", 0.0))
        except: rpm = 0.0
        SPIN_RPM_CMD = _clamp(rpm, SPIN_RPM_MIN, SPIN_RPM_MAX)
        _rx_print(f"spin → rpm={SPIN_RPM_CMD:.0f}")

    elif cmd == "track":
        en = msg.get("enable", None)
        if en is not None:
            TRACK_ENABLED = bool(en) if isinstance(en, bool) else (str(en).lower() in ("1","true","on","yes"))
            _rx_print(f"track → enable={TRACK_ENABLED}")

    elif cmd == "set_target":
        p = str(msg.get("path", "")).strip()
        if p:
            _target_path = p
            _rx_print(f"set_target → path='{_target_path}'")

    else:
        _rx_print(f"unknown cmd: {cmd}")

def _udp_loop():
    print(f"[UDP] Listening on {UDP_HOST}:{UDP_PORT} ...", flush=True)
    while not _udp_stop:
        try:
            data, _ = _udp_sock.recvfrom(8192)
        except socket.timeout:
            continue
        except Exception as e:
            if not _udp_stop: print("[UDP] recv error:", e, flush=True)
            continue

        try:
            _consume_json(json.loads(data.decode("utf-8", "ignore")))
        except Exception:
            for line in data.splitlines():
                try:
                    _consume_json(json.loads(line.decode("utf-8", "ignore") if isinstance(line, bytes) else line))
                except:
                    pass

def start_udp():
    global _udp_sock, _udp_thread, _udp_stop
    if _udp_thread is not None:
        print("[UDP] already running"); return
    _udp_stop = False
    _udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    _udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    _udp_sock.bind((UDP_HOST, UDP_PORT))
    _udp_sock.settimeout(0.1)
    _udp_thread = threading.Thread(target=_udp_loop, daemon=True)
    _udp_thread.start()

def stop_udp():
    global _udp_sock, _udp_thread, _udp_stop
    _udp_stop = True
    if _udp_sock:
        try: _udp_sock.close()
        except: pass
        _udp_sock = None
    if _udp_thread:
        _udp_thread.join(timeout=1.0); _udp_thread = None
    print("[UDP] stopped", flush=True)

# ---------- lifecycle ----------
def start():
    global _body_path, _body_handle, _update_sub, _target_path
    iris_root = _get_prim(IRIS_ROOT_PATH)
    if not iris_root:
        print(f"[Iris] Prim not found at {IRIS_ROOT_PATH}"); return

    body_prim = _find_body_prim(iris_root)
    if not body_prim:
        print("[Iris] No RigidBody under Iris"); return

    _body_path = body_prim.GetPath().pathString
    _body_handle = None

    if APPLY_BODY_CONVEX_FIX: _optional_fix_body_convex(body_prim)
    _apply_physx_on_body(body_prim)

    # Camera under body (create only if missing)
    cam_path = _ensure_camera_under_body(_body_path)
    print(f"[Iris] Camera ready at '{cam_path}'", flush=True)

    # --- OPEN PREVIEW WINDOW ---
    if PREVIEW_WINDOW_ENABLE:
        _open_camera_preview(cam_path, PREVIEW_SIZE, PREVIEW_TITLE)

    # Target cube (create if missing) and set as default follow target
    _ensure_target_cube()
    _target_path = TARGET_PRIM_PATH

    # Rotors
    _prepare_rotors(IRIS_ROOT_PATH)

    if _update_sub is None:
        _update_sub = _app.get_update_event_stream().create_subscription_to_pop(
            _on_update, name="IrisHover_UDP_Spin_Camera_Track"
        )

    start_udp()
    if not _timeline.is_playing(): _timeline.play()
    print(f"[Iris] Ready. Body='{_body_path}', Target='{_target_path}'. UDP {UDP_HOST}:{UDP_PORT}", flush=True)

def stop():
    global _update_sub, _body_handle
    if _update_sub: _update_sub.unsubscribe(); _update_sub = None
    _body_handle = None
    _close_camera_preview()
    stop_udp()
    print("[Iris] Controller stopped.", flush=True)

start()
