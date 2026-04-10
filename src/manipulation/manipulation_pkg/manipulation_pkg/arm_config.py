import math

# ── planning group / frames ────────────────────────────────────────
PLANNING_GROUP = 'xarm7'
BASE_FRAME     = 'link_base'
EEF_LINK       = 'link_eef'
JOINT_NAMES    = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']

# ── planning parameters ────────────────────────────────────────────
MAX_VELOCITY_SCALING     = 0.3
MAX_ACCELERATION_SCALING = 0.3
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

# ── collision scene ────────────────────────────────────────────────
# (name, x, y, z, size_x, size_y, size_z) all in metres
COLLISION_BOXES = [
    ('floor',               0.0,   0.0,  -0.05, 3.0,  3.0,  0.05),
    ('rail_left',          -0.30,  0.35,  0.085, 1.40, 0.29, 0.17),
    # ('rail_right',         -0.30, -0.70,  0.085, 1.40, 0.29, 0.17),
    ('planting_assembly',   0.45,  0.0,   0.20,  0.2,  1.0,  0.5),
]


# ── upright constraint tolerances (radians) ────────────────────────
UPRIGHT_TOL_XY = 0.26   # ~15 degrees
UPRIGHT_TOL_Z  = 6.28   # free rotation around z

# ── RViz trajectory visualization ─────────────────────────────────
# When True: plan → publish to /display_planned_path → wait → execute
VIZ_BEFORE_EXEC = True
VIZ_PAUSE_SEC   = 5.0   # seconds to inspect the trajectory in RViz before executing