import math

# ── planning group / frames ────────────────────────────────────────
PLANNING_GROUP = 'xarm7'
BASE_FRAME     = 'link_base'
EEF_LINK       = 'link_eef'
JOINT_NAMES    = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']

# ── planning parameters ────────────────────────────────────────────
MAX_VELOCITY_SCALING     = 0.2
MAX_ACCELERATION_SCALING = 0.2
PLANNING_TIME            = 10.0
NUM_PLANNING_ATTEMPTS    = 5
PLAN_RETRIES             = 3

LIFT_VELOCITY_SCALING     = 0.1
LIFT_ACCELERATION_SCALING = 0.1

# ── joint waypoints (degrees) ──────────────────────────────────────
PRE_GRASP_JOINTS_DEG = [-61.1, -3.7,  -30.5,  2.0, -181.5,  84.7,  -90.0]
DROP_JOINTS_DEG      = [-35.0, -25.0,   0.0,  75.0, -160.0, -10.0, -180.0]

# ── cartesian waypoints ────────────────────────────────────────────
# (x, y, z in metres, roll/pitch/yaw in degrees)
# grasp orientation
GRASP_RPY = dict(roll=89.7, pitch=-90.0, yaw=0.0)
# drop orientation  
DROP_RPY  = dict(roll=96.7, pitch=-87.2, yaw=85.1)

# (x, y, z in metres, roll/pitch/yaw in degrees)
GRASP_POSE = dict(x=0.0,   y=-0.350, z=0.160, **GRASP_RPY)
LIFT_POSE  = dict(x=0.0,   y=-0.350, z=0.250, **GRASP_RPY)
DROP_POSE  = dict(x=0.500, y=-0.200, z=0.550, **DROP_RPY)

# ── upright transport ──────────────────────────────────────────────
# TRANSPORT_Z: height (m) at which the arm swings horizontally between
# the grasp zone and the drop zone. Must clear all obstacles.
TRANSPORT_Z = 0.40

# DROP_APPROACH_POSE: Cartesian position above the drop zone, reached via
# the Cartesian transport path before the final Pilz PTP drop move.
# Uses DROP_RPY — the drop zone is reachable with this orientation and
# pitch ≈ -87° keeps the seedling roughly upright (within ~15° of grasp).
# compute_cartesian_path SLERPs from GRASP_RPY → DROP_RPY during the swing,
# giving a smooth orientation transition with no sudden EEF rotation.
# *** Calibrate x/y/z to match your drop zone geometry. ***
DROP_APPROACH_POSE = dict(x=0.350, y=-0.150, z=0.45, **DROP_RPY)

# Max Cartesian interpolation step for compute_cartesian_path (metres).
# Smaller = smoother orientation tracking; larger = faster computation.
CARTESIAN_MAX_STEP = 0.01

# Jump threshold for compute_cartesian_path (radians, L-inf norm across joints).
# Rejects any IK step where a joint changes by more than this between consecutive
# interpolated points. Wrist flips are ~π rad and are caught; normal motion per
# 0.01m step is well under 0.3 rad. Set to 0.0 to disable (not recommended).
CARTESIAN_JUMP_THRESHOLD = 0.3

# ── collision scene ────────────────────────────────────────────────
# (name, x, y, z, size_x, size_y, size_z) all in metres
COLLISION_BOXES = [
    #('floor',               0.0,   0.0,  -0.05, 3.0,  3.0,  0.05),
    #('rail_left',          -0.30,  0.35,  0.085, 1.40, 0.29, 0.17),
    #('rail_right',         -0.30, -0.70,  0.085, 1.40, 0.29, 0.17),
    ('planting_assembly',   0.45,  0.0,   0.20,  0.2,  1.0,  0.5),
]


# ── upright constraint tolerances (radians) ────────────────────────
# Tight (15°) causes OMPL fallback to fail when Pilz LIN can't plan.
# 0.52 rad (~30°) gives OMPL enough headroom while still keeping the
# seedling roughly upright during the lift.
UPRIGHT_TOL_XY = 0.52   # ~30 degrees
UPRIGHT_TOL_Z  = 6.28   # free rotation around z

# ── RViz trajectory visualization ─────────────────────────────────
# When True: plan → publish to /display_planned_path → wait → execute
VIZ_BEFORE_EXEC = True
VIZ_PAUSE_SEC   = 5.0   # seconds to inspect the trajectory in RViz before executing