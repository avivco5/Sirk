# Isaac Sim 5.0 — Script Editor
# Use EXISTING Iris at IRIS_ROOT_PATH:
#   (A) Fix dynamic collision setup: apply MeshCollisionAPI and set convex approximations
#       on /World/iris/body and all descendant Meshes (or whole Iris subtree if configured).
#       Optionally author density/mass and PhysX damping.
#   (B) Attach a smoothed hover controller (PD + deadband + low-pass + cmd smoothing) to reduce "dance".
#
# Nothing is spawned or moved. Case-sensitive paths!

from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Sdf, Gf
import carb
import omni
import omni.usd
import omni.timeline
import omni.kit.app
from omni.isaac.dynamic_control import _dynamic_control

# ====== USER SETTINGS (edit to your scene) ====================================
IRIS_ROOT_PATH   = "/World/iris"              # existing Iris root prim
BODY_CANDIDATES  = ["body", "base_link"]      # first one found is used as the dynamic body

# What to scan/fix for collisions:
FIX_SCOPE        = "body_subtree"             # "body_subtree" | "root_subtree"
APPROXIMATION    = "convexDecomposition"      # "convexDecomposition" (recommended) or "convexHull"

# Mass authoring (choose one):
MASS_MODE        = "density"                  # "none" | "mass" | "density"
MASS_KG          = 1.2                        # used if MASS_MODE == "mass"
DENSITY_KG_M3    = 300.0                      # used if MASS_MODE == "density"

# PhysX damping (helps stability; won't lock angles):
ANG_DAMPING      = 12.0
LIN_DAMPING      = 0.4
# ==============================================================================

# ====== Hover controller tuning ===============================================
TARGET_POS_XY    = Gf.Vec2d(0.0, 0.0)
TARGET_ALT_Z     = 2.0

# PD gains & clamps
KP_Z, KD_Z       = 5.0, 3.4
KP_XY, KD_XY     = 5.0, 0.8
VEL_CLAMP_Z      = 10.0
VEL_CLAMP_XY     = 10.0

# Deadbands (m / m/s)
Z_ERR_DB         = 0.03
Z_VEL_DB         = 0.05
XY_ERR_DB        = 0.02
XY_VEL_DB        = 0.05

# Low-pass filtering for measured velocities (EMA alphas)
EMA_VZ           = 0.25
EMA_VXY          = 0.25

# Command smoothing alphas
SMOOTH_Z         = 0.5
SMOOTH_XY        = 0.5

ZERO_ANGVEL_EACH_FRAME = True  # soft “attitude hold”
# ==============================================================================

# ---------- Internals (no edit) -----------
dc = _dynamic_control.acquire_dynamic_control_interface()
_stage = omni.usd.get_context().get_stage()
_app = omni.kit.app.get_app()
_timeline = omni.timeline.get_timeline_interface()

_update_sub = None
_body_path = None
_body_handle = None

# Filters' state
_vx_meas_f = 0.0
_vy_meas_f = 0.0
_vz_meas_f = 0.0
_vx_cmd_s  = 0.0
_vy_cmd_s  = 0.0
_vz_cmd_s  = 0.0

def _get_prim(path):
    p = _stage.GetPrimAtPath(path)
    return p if p and p.IsValid() else None

def _find_body_prim(root_prim):
    # 1) try common child names
    for name in BODY_CANDIDATES:
        cand = _get_prim(f"{root_prim.GetPath().pathString}/{name}")
        if cand:
            return cand
    # 2) first prim that already has a rigid body API
    for p in Usd.PrimRange(root_prim):
        try:
            if p.HasAPI(UsdPhysics.RigidBodyAPI):
                return p
        except Exception:
            pass
    # 3) fallback to root
    return root_prim

def _ensure_body_and_collider(prim):
    if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI.Apply(prim)
    if not prim.HasAPI(UsdPhysics.CollisionAPI):
        UsdPhysics.CollisionAPI.Apply(prim)

def _set_mesh_approx(mesh_prim, approx_token):
    # Ensure Collision + MeshCollisionAPI; set approximation token
    if not mesh_prim.HasAPI(UsdPhysics.CollisionAPI):
        UsdPhysics.CollisionAPI.Apply(mesh_prim)
    mapi = UsdPhysics.MeshCollisionAPI.Apply(mesh_prim)
    # Create... returns the attr (existing or new); Set to enforce value
    attr = mapi.CreateApproximationAttr(approx_token)
    attr.Set(approx_token)

def _author_mass(body_prim):
    if MASS_MODE == "none":
        print("[Fix] MASS_MODE=none -> leave mass to default/auto")
        return
    if not body_prim.HasAPI(UsdPhysics.MassAPI):
        UsdPhysics.MassAPI.Apply(body_prim)
    m = UsdPhysics.MassAPI(body_prim)
    if MASS_MODE == "mass":
        m.CreateMassAttr(MASS_KG)
        # clear density if authored
        dens = m.GetDensityAttr()
        if dens and dens.HasAuthoredValueOpinion():
            dens.Clear()
        print(f"[Fix] Mass authored: {MASS_KG} kg on {body_prim.GetPath()}")
    elif MASS_MODE == "density":
        m.CreateDensityAttr(DENSITY_KG_M3)
        mass = m.GetMassAttr()
        if mass and mass.HasAuthoredValueOpinion():
            mass.Clear()
        print(f"[Fix] Density authored: {DENSITY_KG_M3} kg/m^3 on {body_prim.GetPath()}")

def _author_damping(body_prim):
    try:
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(body_prim)
        # set only if not authored already
        ang = rb.GetAngularDampingAttr()
        lin = rb.GetLinearDampingAttr()
        if not ang.HasAuthoredValueOpinion():
            rb.CreateAngularDampingAttr(ANG_DAMPING)
        if not lin.HasAuthoredValueOpinion():
            rb.CreateLinearDampingAttr(LIN_DAMPING)
        print(f"[Fix] Damping authored (angular≈{ANG_DAMPING}, linear≈{LIN_DAMPING})")
    except Exception as e:
        print(f"[Fix] WARN: could not author PhysX damping: {e}")

def fix_collisions_and_mass():
    """Apply convex approximation to triangle-mesh colliders and author mass/damping."""
    iris_root = _get_prim(IRIS_ROOT_PATH)
    if not iris_root:
        print(f"[Fix] ERROR: Prim not found at '{IRIS_ROOT_PATH}'")
        return None

    body_prim = _find_body_prim(iris_root)
    if not body_prim:
        print(f"[Fix] ERROR: Could not locate body under '{IRIS_ROOT_PATH}'")
        return None

    print(f"[Fix] Using body prim: {body_prim.GetPath()}")
    _ensure_body_and_collider(body_prim)

    # Choose traversal root
    traversal_root = body_prim if FIX_SCOPE == "body_subtree" else iris_root

    updated = []
    skipped = []

    # Include the traversal_root itself if it's a Mesh (this fixes the exact warning on /World/iris/body)
    if traversal_root.GetTypeName() == "Mesh":
        try:
            _set_mesh_approx(traversal_root, APPROXIMATION)
            updated.append(str(traversal_root.GetPath()))
        except Exception as e:
            skipped.append(f"{traversal_root.GetPath()} (body mesh) -> {e}")

    for p in Usd.PrimRange(traversal_root):
        if p == traversal_root:
            continue  # already handled
        if p.GetTypeName() == "Mesh":
            try:
                _set_mesh_approx(p, APPROXIMATION)
                updated.append(str(p.GetPath()))
            except Exception as e:
                skipped.append(f"{p.GetPath()} -> {e}")

    print(f"[Fix] Applied approximation='{APPROXIMATION}' on {len(updated)} Mesh collider(s).")
    if skipped:
        print("[Fix] Skipped/Errors:")
        for s in skipped:
            print("   -", s)

    # Mass/density & damping
    _author_mass(body_prim)
    _author_damping(body_prim)

    return body_prim

def _get_pose_dc(handle):
    pose = dc.get_rigid_body_pose(handle)
    lin  = dc.get_rigid_body_linear_velocity(handle)
    ang  = dc.get_rigid_body_angular_velocity(handle)
    p = Gf.Vec3d(pose.p.x, pose.p.y, pose.p.z)
    v = Gf.Vec3d(lin.x, lin.y, lin.z)
    w = Gf.Vec3d(ang.x, ang.y, ang.z)
    return p, v, w

def _clamp(val, lo, hi):
    return hi if val > hi else lo if val < lo else val

def _on_update(e: carb.events.IEvent):
    """Per-frame hover controller with XY+Z filtering and smoothing."""
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

    # Soft attitude damp: kill angular vel
    if ZERO_ANGVEL_EACH_FRAME:
        dc.set_rigid_body_angular_velocity(_body_handle, carb.Float3(0.0, 0.0, 0.0))

    # ---- Z axis ----
    _vz_meas_f = (1.0 - EMA_VZ) * _vz_meas_f + EMA_VZ * v[2]
    ez  = TARGET_ALT_Z - p[2]
    evz = -_vz_meas_f

    if abs(ez) < Z_ERR_DB and abs(_vz_meas_f) < Z_VEL_DB:
        vz_cmd_raw = 0.0
    else:
        vz_cmd_raw = KP_Z * ez + KD_Z * evz

    vz_cmd_raw = _clamp(vz_cmd_raw, -VEL_CLAMP_Z, VEL_CLAMP_Z)
    _vz_cmd_s  = (1.0 - SMOOTH_Z) * _vz_cmd_s + SMOOTH_Z * vz_cmd_raw

    # ---- XY axes ----
    _vx_meas_f = (1.0 - EMA_VXY) * _vx_meas_f + EMA_VXY * v[0]
    _vy_meas_f = (1.0 - EMA_VXY) * _vy_meas_f + EMA_VXY * v[1]

    ex, ey = TARGET_POS_XY[0] - p[0], TARGET_POS_XY[1] - p[1]
    evx, evy = -_vx_meas_f, -_vy_meas_f

    if abs(ex) < XY_ERR_DB and abs(_vx_meas_f) < XY_VEL_DB:
        vx_raw = 0.0
    else:
        vx_raw = KP_XY * ex + KD_XY * evx

    if abs(ey) < XY_ERR_DB and abs(_vy_meas_f) < XY_VEL_DB:
        vy_raw = 0.0
    else:
        vy_raw = KP_XY * ey + KD_XY * evy

    vx_raw = _clamp(vx_raw, -VEL_CLAMP_XY, VEL_CLAMP_XY)
    vy_raw = _clamp(vy_raw, -VEL_CLAMP_XY, VEL_CLAMP_XY)

    _vx_cmd_s = (1.0 - SMOOTH_XY) * _vx_cmd_s + SMOOTH_XY * vx_raw
    _vy_cmd_s = (1.0 - SMOOTH_XY) * _vy_cmd_s + SMOOTH_XY * vy_raw

    # Command linear velocity (carb.Float3; no extra bool)
    dc.set_rigid_body_linear_velocity(_body_handle, carb.Float3(_vx_cmd_s, _vy_cmd_s, _vz_cmd_s))

def start():
    """Run fix phase, then attach hover to existing body."""
    global _body_path, _body_handle, _update_sub
    body_prim = fix_collisions_and_mass()
    if body_prim is None:
        return

    _body_path = body_prim.GetPath().pathString
    _body_handle = None  # lazy acquire

    if _update_sub is None:
        _update_sub = _app.get_update_event_stream().create_subscription_to_pop(
            _on_update, name="IrisHoverUpdateExisting"
        )

    if not _timeline.is_playing():
        _timeline.play()

    print(f"[Iris] Hover controller attached to {_body_path}. Use stop() to detach.")

def stop():
    """Detach controller (does not delete or move your prim)."""
    global _update_sub, _body_handle
    if _update_sub is not None:
        _update_sub.unsubscribe()
        _update_sub = None
    _body_handle = None
    print("[Iris] Hover controller stopped.")

# --- Run ---
start()

