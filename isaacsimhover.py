# Isaac Sim 5.0 — Script Editor
# EXISTING Iris + UDP JSON listener for position setpoints + smooth hover controller (verbose).
# Listens on UDP 0.0.0.0:6000 for JSON:
#   {"cmd":"setpos", "x": <meters>, "y": <meters>, "alt": <meters>}
#   {"cmd":"nudge",  "dx": <m>, "dy": <m>, "dalt": <m>}  # שינוי יחסי
#
# Does NOT spawn prims; attaches to your existing Iris by path below.

from pxr import Usd, UsdPhysics, PhysxSchema, Gf
import carb
import omni
import omni.usd
import omni.timeline
import omni.kit.app
from omni.isaac.dynamic_control import _dynamic_control
import threading, time, socket, json

# ===================== USER SETTINGS =====================
IRIS_ROOT_PATH = "/World/iris"          # existing Iris prim (case-sensitive)
BODY_CANDIDATES = ["body", "base_link"]

# UDP server (listening) — same machine or LAN
UDP_HOST = "0.0.0.0"
UDP_PORT = 6000

# NED/world: כאן זה פשוט world XY ו-alt למעלה (בלי NED)
# Verbose
VERBOSE_RX = True
RX_PRINT_PERIOD = 0.25     # seconds
APPLY_BODY_CONVEX_FIX = True  # נסה להשתיק את ה-warning על body mesh פעם אחת
# =========================================================

# ===== Hover target & controller tuning =====
TARGET_POS_XY = Gf.Vec2d(0.0, 0.0)  # meters
TARGET_ALT_Z  = 2.0                 # meters AGL-ish

KP_Z,  KD_Z   = 1.8, 3.4
KP_XY, KD_XY  = 1.2, 0.8
VEL_CLAMP_Z   = 6.0
VEL_CLAMP_XY  = 6.0

Z_ERR_DB, Z_VEL_DB   = 0.03, 0.05
XY_ERR_DB, XY_VEL_DB = 0.02, 0.05

EMA_VZ, EMA_VXY = 0.25, 0.25
SMOOTH_Z, SMOOTH_XY = 0.5, 0.5

ZERO_ANGVEL_EACH_FRAME = True
ANG_DAMPING, LIN_DAMPING = 12.0, 0.4
# ===========================================

# ---------- Internals ----------
dc = _dynamic_control.acquire_dynamic_control_interface()
_stage = omni.usd.get_context().get_stage()
_app = omni.kit.app.get_app()
_timeline = omni.timeline.get_timeline_interface()
_update_sub = None
_body_path = None
_body_handle = None

_vx_meas_f = 0.0
_vy_meas_f = 0.0
_vz_meas_f = 0.0
_vx_cmd_s  = 0.0
_vy_cmd_s  = 0.0
_vz_cmd_s  = 0.0

# UDP state
_udp_sock = None
_udp_thread = None
_udp_stop = False
_last_rx_print = 0.0

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
        except Exception:
            pass
    return root_prim

def _apply_damping(prim):
    if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI.Apply(prim)
    if not prim.HasAPI(UsdPhysics.CollisionAPI):
        UsdPhysics.CollisionAPI.Apply(prim)
    rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    rb.CreateAngularDampingAttr(ANG_DAMPING)
    rb.CreateLinearDampingAttr(LIN_DAMPING)

def _optional_fix_body_convex(body_prim):
    # אם ה-body עצמו Mesh ויש לו collision triangle — שנה ל-convexDecomposition
    try:
        if body_prim.GetTypeName() == "Mesh":
            if not body_prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI.Apply(body_prim)
            mapi = UsdPhysics.MeshCollisionAPI.Apply(body_prim)
            mapi.CreateApproximationAttr("convexDecomposition").Set("convexDecomposition")
            print("[Fix] Body mesh set to convexDecomposition", flush=True)
    except Exception as e:
        print("[Fix] WARN:", e, flush=True)

def _get_pose_dc(handle):
    pose = dc.get_rigid_body_pose(handle)
    lin  = dc.get_rigid_body_linear_velocity(handle)
    ang  = dc.get_rigid_body_angular_velocity(handle)
    p = Gf.Vec3d(pose.p.x, pose.p.y, pose.p.z)
    v = Gf.Vec3d(lin.x, lin.y, lin.z)
    w = Gf.Vec3d(ang.x, ang.y, ang.z)
    return p, v, w

def _clamp(val, lo, hi): return hi if val > hi else lo if val < lo else val

# ---------- Per-frame controller ----------
def _on_update(e: carb.events.IEvent):
    global _body_handle, _vx_meas_f, _vy_meas_f, _vz_meas_f, _vx_cmd_s, _vy_cmd_s, _vz_cmd_s
    if not _timeline.is_playing() or _body_path is None:
        return

    if _body_handle is None:
        try:
            _body_handle = dc.get_rigid_body(_body_path)
        except Exception:
            _body_handle = None
            return
    if _body_handle is None:
        return

    p, v, w = _get_pose_dc(_body_handle)

    if ZERO_ANGVEL_EACH_FRAME:
        dc.set_rigid_body_angular_velocity(_body_handle, carb.Float3(0.0, 0.0, 0.0))

    # Z
    _vz_meas_f = (1.0 - EMA_VZ) * _vz_meas_f + EMA_VZ * v[2]
    ez, evz = TARGET_ALT_Z - p[2], -_vz_meas_f
    vz_raw = 0.0 if (abs(ez) < Z_ERR_DB and abs(_vz_meas_f) < Z_VEL_DB) else (KP_Z*ez + KD_Z*evz)
    vz_raw = _clamp(vz_raw, -VEL_CLAMP_Z, VEL_CLAMP_Z)
    _vz_cmd_s = (1.0 - SMOOTH_Z)*_vz_cmd_s + SMOOTH_Z*vz_raw

    # XY
    _vx_meas_f = (1.0 - EMA_VXY) * _vx_meas_f + EMA_VXY * v[0]
    _vy_meas_f = (1.0 - EMA_VXY) * _vy_meas_f + EMA_VXY * v[1]
    ex, ey = TARGET_POS_XY[0] - p[0], TARGET_POS_XY[1] - p[1]
    evx, evy = -_vx_meas_f, -_vy_meas_f
    vx_raw = 0.0 if (abs(ex) < XY_ERR_DB and abs(_vx_meas_f) < XY_VEL_DB) else (KP_XY*ex + KD_XY*evx)
    vy_raw = 0.0 if (abs(ey) < XY_ERR_DB and abs(_vy_meas_f) < XY_VEL_DB) else (KP_XY*ey + KD_XY*evy)
    vx_raw = _clamp(vx_raw, -VEL_CLAMP_XY, VEL_CLAMP_XY)
    vy_raw = _clamp(vy_raw, -VEL_CLAMP_XY, VEL_CLAMP_XY)
    _vx_cmd_s = (1.0 - SMOOTH_XY)*_vx_cmd_s + SMOOTH_XY*vx_raw
    _vy_cmd_s = (1.0 - SMOOTH_XY)*_vy_cmd_s + SMOOTH_XY*vy_raw

    dc.set_rigid_body_linear_velocity(_body_handle, carb.Float3(_vx_cmd_s, _vy_cmd_s, _vz_cmd_s))

# ---------- UDP JSON listener ----------
def _udp_loop():
    global TARGET_POS_XY, TARGET_ALT_Z, _udp_sock, _udp_stop, _last_rx_print
    print(f"[UDP] Listening on {UDP_HOST}:{UDP_PORT} ...", flush=True)
    while not _udp_stop:
        try:
            data, addr = _udp_sock.recvfrom(8192)
        except socket.timeout:
            continue
        except Exception as e:
            if not _udp_stop:
                print("[UDP] recv error:", e, flush=True)
            continue
        try:
            msg = json.loads(data.decode("utf-8", "ignore"))
        except Exception:
            # allow newline-delimited JSON bursts
            for line in data.splitlines():
                try:
                    msg = json.loads(line.decode("utf-8", "ignore") if isinstance(line, bytes) else line)
                except Exception:
                    continue
                _handle_json(msg)
            continue
        _handle_json(msg)

def _handle_json(msg: dict):
    global TARGET_POS_XY, TARGET_ALT_Z, _last_rx_print
    cmd = str(msg.get("cmd", "")).lower()
    if cmd == "setpos":
        x  = float(msg.get("x", TARGET_POS_XY[0]))
        y  = float(msg.get("y", TARGET_POS_XY[1]))
        alt= float(msg.get("alt", TARGET_ALT_Z))
        TARGET_POS_XY = Gf.Vec2d(x, y)
        TARGET_ALT_Z  = alt
        _throttled_print(f"[UDP RX] setpos → x={x:.2f} y={y:.2f} alt={alt:.2f}")
    elif cmd == "nudge":
        dx = float(msg.get("dx", 0.0))
        dy = float(msg.get("dy", 0.0))
        da = float(msg.get("dalt", 0.0))
        TARGET_POS_XY = Gf.Vec2d(TARGET_POS_XY[0] + dx, TARGET_POS_XY[1] + dy)
        TARGET_ALT_Z  = TARGET_ALT_Z + da
        _throttled_print(f"[UDP RX] nudge → x={TARGET_POS_XY[0]:.2f} y={TARGET_POS_XY[1]:.2f} alt={TARGET_ALT_Z:.2f}")
    else:
        _throttled_print(f"[UDP RX] unknown cmd: {cmd}")

def _throttled_print(s: str):
    global _last_rx_print
    if not VERBOSE_RX: return
    now = time.time()
    if now - _last_rx_print >= RX_PRINT_PERIOD:
        print(s, flush=True)
        _last_rx_print = now

def start_udp():
    global _udp_sock, _udp_thread, _udp_stop
    if _udp_thread is not None:
        print("[UDP] already running", flush=True); return
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
        except Exception: pass
        _udp_sock = None
    if _udp_thread:
        _udp_thread.join(timeout=1.0)
        _udp_thread = None
    print("[UDP] stopped", flush=True)

# ---------- Lifecycle ----------
def start():
    """Attach controller to existing Iris and start UDP listener."""
    global _body_path, _body_handle, _update_sub

    iris_root = _get_prim(IRIS_ROOT_PATH)
    if iris_root is None:
        print(f"[Iris] Prim not found at {IRIS_ROOT_PATH}. Update IRIS_ROOT_PATH.", flush=True); return

    body_prim = _find_body_prim(iris_root)
    if body_prim is None:
        print("[Iris] No RigidBody found under Iris.", flush=True); return

    _body_path = body_prim.GetPath().pathString
    _body_handle = None

    if APPLY_BODY_CONVEX_FIX:
        _optional_fix_body_convex(body_prim)

    _apply_damping(body_prim)

    if _update_sub is None:
        _update_sub = _app.get_update_event_stream().create_subscription_to_pop(
            _on_update, name="IrisHoverUpdate_UDP"
        )

    start_udp()

    if not _timeline.is_playing():
        _timeline.play()

    print(f"[Iris] Hover attached to {_body_path}. UDP on {UDP_HOST}:{UDP_PORT}.", flush=True)

def stop():
    """Detach controller and stop UDP listener."""
    global _update_sub, _body_handle
    if _update_sub is not None:
        _update_sub.unsubscribe()
        _update_sub = None
    _body_handle = None
    stop_udp()
    print("[Iris] Hover controller stopped.", flush=True)

# Auto-start
start()
