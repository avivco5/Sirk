# Isaac Sim 5.0 — Iris hover + UDP listener + Rotor spin via orient(Z) controlled by {"cmd":"spin","rpm":...}
# UDP JSON on 0.0.0.0:6000:
#   {"cmd":"setpos","x":<m>,"y":<m>,"alt":<m>,"yaw_deg":<deg-optional>}
#   {"cmd":"nudge","dx":<m>,"dy":<m>,"dalt":<m>,"dyaw_deg":<deg-optional>}   # BODY-frame
#   {"cmd":"setyaw","yaw_deg":<deg>}
#   {"cmd":"spin","rpm":<rpm>}   # ← חדש: שליטת RPM בפרופים (ויזואלי)

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
# =========================================================

# ===== Targets & gains =====
TARGET_POS_XY = Gf.Vec2d(0.0, 0.0)  # world meters
TARGET_ALT_Z  = 2.0                 # world Z up
TARGET_YAW    = 0.0                 # radians (about world Z)

# PD (linear)
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
TAU_Z_CLAMP      = 8.0     # N*m
YAW_ERR_DB, YAW_RATE_DB = math.radians(1.0), math.radians(2.0)

ANG_DAMPING, LIN_DAMPING = 2.0, 0.4
MAX_ANG_VEL = 12.0

# ===== Rotor spin settings (visual, via xformOp:orient about local Z) =====
VISUAL_SPIN         = True
ROTOR_NAME_PREFIX   = "rotor"        # Xforms like rotor0/1/2/3
SPIN_RPM_CMD        = 0.0            # ← נקבע ע"י פקודות UDP מה-GUI
SPIN_RPM_MIN        = 0.0
SPIN_RPM_MAX        = 40000.0        # קצה עליון סביר לוויזואל
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
_last_udp_rx = 0.0  # perf-counter timestamp of last valid packet (לוג בלבד)

# Rotor state
_rotors = []  # [{path, orient_op, base_q(Gf.Quatf), angle_deg, sign}]
_last_t = time.perf_counter()
_last_log = time.perf_counter()

# ---------- Helpers ----------
def _get_prim(path):
    p = _stage.GetPrimAtPath(path)
    return p if p and p.IsValid() else None

def _find_body_prim(root_prim):
    for name in BODY_CANDIDATES:
        cand = _stage.GetPrimAtPath(f"{root_prim.GetPath().pathString}/{name}")
        if cand and cand.IsValid():
            return cand
    for p in Usd.PrimRange(root_prim):
        try:
            if p.HasAPI(UsdPhysics.RigidBodyAPI):
                return p
        except:
            pass
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
        if prim.GetTypeName() == "Mesh":
            if not prim.HasAPI(UsdPhysics.CollisionAPI): UsdPhysics.CollisionAPI.Apply(prim)
            m = UsdPhysics.MeshCollisionAPI.Apply(prim)
            m.CreateApproximationAttr("convexDecomposition").Set("convexDecomposition")
            print("[Fix] Body mesh set to convexDecomposition", flush=True)
    except Exception as e:
        print("[Fix] WARN:", e, flush=True)

def _wrap_pi(a): return (a + math.pi) % (2.0 * math.pi) - math.pi
def _clamp(x, a, b): return b if x > b else a if x < a else x

def _get_pose_dc(h):
    pose = dc.get_rigid_body_pose(h)
    lin  = dc.get_rigid_body_linear_velocity(h)
    ang  = dc.get_rigid_body_angular_velocity(h)
    p = (pose.p.x, pose.p.y, pose.p.z)
    v = (lin.x, lin.y, lin.z)
    w = (ang.x, ang.y, ang.z)
    qx, qy, qz, qw = pose.r.x, pose.r.y, pose.r.z, pose.r.w
    siny_cosp = 2.0 * (qw*qz + qx*qy)
    cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return p, v, w, yaw

# ---------- Rotor via orient(Z) ----------
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
        if isinstance(q, Gf.Quatf):
            return q
        if isinstance(q, Gf.Quatd):
            return _quatd_to_quatf(q)
    except Exception:
        pass
    return Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0))

def _prepare_rotors(root_path):
    """Find Xforms named rotor* and set up orient spin around local Z."""
    global _rotors
    _rotors = []
    s = _stage
    root = s.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        print(f"⚠️ rotor-scan: invalid root '{root_path}'"); return
    # Edit in session layer (non-destructive)
    s.SetEditTarget(s.GetSessionLayer())
    found = []
    for p in Usd.PrimRange(root):
        if p.GetTypeName() == "Xform" and p.GetName().lower().startswith(ROTOR_NAME_PREFIX):
            found.append(p)
    found.sort(key=lambda prm: prm.GetPath().pathString)
    for i, prm in enumerate(found):
        xf = UsdGeom.Xformable(prm)
        if not xf: continue
        orient_op = _get_or_add_orient_op(xf)
        base_q    = _get_quatf_or_identity(orient_op)
        _rotors.append({
            "path": prm.GetPath().pathString,
            "orient_op": orient_op,
            "base_q": base_q,
            "angle_deg": 0.0,
            "sign": (+1.0 if (i % 2 == 0) else -1.0)  # alternate CW/CCW
        })
    print(f"🔎 Prepared {len(_rotors)} rotors for orient(Z) spin:")
    for r in _rotors: print("   •", r["path"])

def _spin_rotors(dt):
    """סיבוב לפי פקודת RPM שנקלטה ב-UDP (SPIN_RPM_CMD)."""
    if not VISUAL_SPIN or not _rotors: return
    rpm = _clamp(SPIN_RPM_CMD, SPIN_RPM_MIN, SPIN_RPM_MAX)
    rate_dps = rpm * 6.0  # rpm -> deg/s
    if rate_dps <= 0.0: return
    for r in _rotors:
        r["angle_deg"] = (r["angle_deg"] + r["sign"] * rate_dps * dt) % 360.0
        rotZ = Gf.Rotation(Gf.Vec3d(0, 0, 1), r["angle_deg"]).GetQuaternion()  # Quatd
        qZf  = _quatd_to_quatf(rotZ)
        # סיבוב סביב Z בלוקאל אחרי ה-orientation הבסיסי
        q_final = r["base_q"] * qZf
        r["orient_op"].Set(q_final)

# ---------- Per-frame controller ----------
def _on_update(e):
    global _body_handle, _vx_meas_f, _vy_meas_f, _vz_meas_f
    global _vx_cmd_s, _vy_cmd_s, _vz_cmd_s, _measured_yaw, _last_yaw_dbg
    global _last_t, _last_log

    if not _timeline.is_playing():
        return

    now = time.perf_counter()
    dt  = max(1e-4, now - _last_t)
    _last_t = now

    # Rotor spin driven by RPM command
    _spin_rotors(dt)

    if _body_path is None:
        return
    if _body_handle is None:
        try: _body_handle = dc.get_rigid_body(_body_path)
        except: _body_handle = None
        if _body_handle is None: return

    try: dc.wake_up_rigid_body(_body_handle)
    except: pass

    p, v, w, yaw = _get_pose_dc(_body_handle)
    _measured_yaw = yaw

    # Yaw torque control
    wz = float(w[2])
    eyaw = _wrap_pi(TARGET_YAW - yaw)
    if abs(eyaw) < YAW_ERR_DB and abs(wz) < YAW_RATE_DB:
        tau_z = 0.0
    else:
        tau_z = _clamp(KPY_TAU * eyaw + KDY_TAU * (-wz), -TAU_Z_CLAMP, TAU_Z_CLAMP)
    dc.apply_body_torque(_body_handle, carb.Float3(0.0, 0.0, tau_z), True)

    tnow = time.time()
    if tnow - _last_yaw_dbg > 0.5:
        print(f"[Yaw] tgt={math.degrees(TARGET_YAW):.1f}° cur={math.degrees(yaw):.1f}° err={math.degrees(eyaw):.1f}° τz={tau_z:.2f}", flush=True)
        _last_yaw_dbg = tnow

    # Z velocity PD
    _vz_meas_f = (1.0 - EMA_VZ) * _vz_meas_f + EMA_VZ * v[2]
    ez, evz = TARGET_ALT_Z - p[2], -_vz_meas_f
    vz_raw = 0.0 if (abs(ez) < Z_ERR_DB and abs(_vz_meas_f) < Z_VEL_DB) else (KP_Z * ez + KD_Z * evz)
    _vz_cmd_s = (1.0 - SMOOTH_Z) * _vz_cmd_s + SMOOTH_Z * _clamp(vz_raw, -VEL_CLAMP_Z, VEL_CLAMP_Z)

    # XY velocity PD
    _vx_meas_f = (1.0 - EMA_VXY) * _vx_meas_f + EMA_VXY * v[0]
    _vy_meas_f = (1.0 - EMA_VXY) * _vy_meas_f + EMA_VXY * v[1]
    ex, ey = TARGET_POS_XY[0] - p[0], TARGET_POS_XY[1] - p[1]
    evx, evy = -_vx_meas_f, -_vy_meas_f
    vx_raw = 0.0 if (abs(ex) < XY_ERR_DB and abs(_vx_meas_f) < XY_VEL_DB) else (KP_XY * ex + KD_XY * evx)
    vy_raw = 0.0 if (abs(ey) < XY_ERR_DB and abs(_vy_meas_f) < XY_VEL_DB) else (KP_XY * ey + KD_XY * evy)
    _vx_cmd_s = (1.0 - SMOOTH_XY) * _vx_cmd_s + SMOOTH_XY * _clamp(vx_raw, -VEL_CLAMP_XY, VEL_CLAMP_XY)
    _vy_cmd_s = (1.0 - SMOOTH_XY) * _vy_cmd_s + SMOOTH_XY * _clamp(vy_raw, -VEL_CLAMP_XY, VEL_CLAMP_XY)

    dc.set_rigid_body_linear_velocity(_body_handle, carb.Float3(_vx_cmd_s, _vy_cmd_s, _vz_cmd_s))

    # log
    if now - _last_log > 2.0:
        print(f"[Spin] rotors={len(_rotors)} rpm_cmd={SPIN_RPM_CMD:.0f}", flush=True)
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
    global TARGET_POS_XY, TARGET_ALT_Z, TARGET_YAW, _measured_yaw, _last_udp_rx, SPIN_RPM_CMD
    cmd = str(msg.get("cmd", "")).lower()

    def yaw_deg_to_rad(k):
        return _wrap_pi(math.radians(float(msg[k]))) if k in msg else None

    if cmd in ("setpos", "setpose"):   # <-- תומך בשני השמות
        x = float(msg.get("x", TARGET_POS_XY[0]))
        y = float(msg.get("y", TARGET_POS_XY[1]))
        alt = float(msg.get("alt", TARGET_ALT_Z))
        yawr = yaw_deg_to_rad("yaw_deg")  # roll/pitch מתעלמים, yaw אופציונלי
        TARGET_POS_XY = Gf.Vec2d(x, y)
        TARGET_ALT_Z  = alt
        if yawr is not None:
            TARGET_YAW = yawr
        _rx_print(f"setpose → x={x:.2f} y={y:.2f} alt={alt:.2f}" + (f" yaw={math.degrees(TARGET_YAW):.1f}°" if yawr is not None else ""))

    elif cmd == "nudge":
        dx_b = float(msg.get("dx", 0.0)); dy_b = float(msg.get("dy", 0.0)); da = float(msg.get("dalt", 0.0))
        dyaw_deg = float(msg.get("dyaw_deg", 0.0))
        c, s = math.cos(_measured_yaw), math.sin(_measured_yaw)
        dx_w = c*dx_b - s*dy_b
        dy_w = s*dx_b + c*dy_b
        TARGET_POS_XY = Gf.Vec2d(TARGET_POS_XY[0] + dx_w, TARGET_POS_XY[1] + dy_w)
        TARGET_ALT_Z  = TARGET_ALT_Z + da
        if abs(dyaw_deg) > 1e-6:
            TARGET_YAW = _wrap_pi(TARGET_YAW + math.radians(dyaw_deg))
        _rx_print(f"nudge(body) → dB=({dx_b:.2f},{dy_b:.2f}) dW=({dx_w:.2f},{dy_w:.2f}) ⇒ x={TARGET_POS_XY[0]:.2f} y={TARGET_POS_XY[1]:.2f} alt={TARGET_ALT_Z:.2f} yaw={math.degrees(TARGET_YAW):.1f}°")

    elif cmd == "setyaw":
        yawd = float(msg.get("yaw_deg", math.degrees(TARGET_YAW)))
        TARGET_YAW = _wrap_pi(math.radians(yawd))
        _rx_print(f"setyaw → yaw={yawd:.1f}°")

    elif cmd == "spin":
        try:
            rpm = float(msg.get("rpm", 0.0))
        except:
            rpm = 0.0
        SPIN_RPM_CMD = _clamp(rpm, SPIN_RPM_MIN, SPIN_RPM_MAX)
        _rx_print(f"spin → rpm={SPIN_RPM_CMD:.0f}")

    else:
        _rx_print(f"unknown cmd: {cmd}")

    _last_udp_rx = time.perf_counter()

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

        # Try single JSON; if fails, try NDJSON lines
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
    global _body_path, _body_handle, _update_sub
    iris_root = _get_prim(IRIS_ROOT_PATH)
    if not iris_root:
        print(f"[Iris] Prim not found at {IRIS_ROOT_PATH}"); return
    body_prim = _find_body_prim(iris_root)
    if not body_prim:
        print("[Iris] No RigidBody under Iris"); return
    _body_path = body_prim.GetPath().pathString; _body_handle = None
    if APPLY_BODY_CONVEX_FIX: _optional_fix_body_convex(body_prim)
    _apply_physx_on_body(body_prim)

    # Prepare rotor spinners (visual)
    if VISUAL_SPIN:
        _prepare_rotors(IRIS_ROOT_PATH)

    if _update_sub is None:
        _update_sub = _app.get_update_event_stream().create_subscription_to_pop(
            _on_update, name="IrisHover_UDP_BodyNudge_Spin"
        )
    start_udp()
    if not _timeline.is_playing(): _timeline.play()
    print(f"[Iris] Hover attached to {_body_path}. UDP on {UDP_HOST}:{UDP_PORT}.", flush=True)

def stop():
    global _update_sub, _body_handle
    if _update_sub: _update_sub.unsubscribe(); _update_sub = None
    _body_handle = None
    stop_udp()
    print("[Iris] Hover controller stopped.", flush=True)

start()
