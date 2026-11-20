import numpy as np
import rclpy
from rclpy.node import Node, ParameterDescriptor, ParameterType
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.distance import pdist
from matplotlib import pyplot as plt
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
import utm
import math
from scipy.spatial.transform.rotation import Rotation as R
from scipy import stats
from skimage.draw import disk
from scipy.ndimage import distance_transform_edt, label
from skimage.graph import route_through_array

# ROS2 message definitions
# Subscribes to /odometry/gps from robot_localization, nav_msgs/Odometry
# https://docs.ros.org/en/melodic/api/robot_localization/html/state_estimation_nodes.html
from diagnostic_msgs.msg import DiagnosticStatus
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Twist, PoseStamped, PointStamped, Point
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from canopy_msgs.msg import (
    TrajectoryCandidates,
    TrajectoryCandidate,
    Mode,
    PlantingPlan,
    ObstacleDebug,
)
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, Bool, Empty, Float32


class Candidate:
    def __init__(
        self, speed: float, omega: float, trajectory: np.ndarray, cost: float = -1
    ):
        self.speed = speed
        self.omega = omega
        self.trajectory = trajectory
        self.cost = cost


class PlannerNode(Node):
    def __init__(self):
        super().__init__("trajectory_planner")

        self.setUpParameters()

        self.TARGET_DOWNHILL_YAW = np.pi  # radians from east

        self.create_subscription(Twist, "/cmd_vel/teleop", self.teleopTwistCb, 1)
        self.create_subscription(OccupancyGrid, "/cost/total", self.totalCostCb, 1)
        self.create_subscription(OccupancyGrid, "/cost/occupancy", self.occCb, 1)
        self.create_subscription(
            GeoPoint, "/planning/goal_pose_geo", self.goalPointGeoCb, 1
        )
        #self.create_subscription(Odometry, "/odometry/gps", self.gpsodomCb, 1)
        #self.create_subscription(Odometry, "/odometry/filtered", self.localodomCb, 1)
        self.create_subscription(Float32, "/gnss/yaw", self.egoYawCb, 1)
        self.create_subscription(Bool, "/gnss/heading_valid", self.headingValidCb, 1)
        # The W200 publishes its e-stop state best effort (10-26 bag), and a
        # reliable subscription (the default) never matches a best-effort
        # publisher, so the planner never saw the e-stop. Best effort matches
        # both, so this keeps working if the publisher changes.
        estop_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            Bool, "/w200_0120/platform/emergency_stop", self.estopCb, estop_qos
        )
        self.create_subscription(Mode, "/planning/current_mode", self.currentModeCb, 1)
        self.create_subscription(Bool, "/behavior/is_planting", self.isPlantingCb, 1)
        self.create_subscription(
            Bool, "/behavior/is_turning_downhill", self.isTurningDownhillCb, 1
        )
        self.create_subscription(
            PointStamped, "/planning/closest_seedling_bl", self.closestPointBlCb, 1
        )

        self.create_subscription(
            PlantingPlan, "/planning/remaining_plan", self.planCb, 1
        )

        self.twist_pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.facing_downhill_pub = self.create_publisher(
            Empty, "/behavior/facing_downhill", 1
        )
        self.twist_path_pub = self.create_publisher(Path, "/cmd_vel/path", 1)
        self.candidates_pub = self.create_publisher(
            TrajectoryCandidates, "/planning/candidates", 1
        )
        self.status_pub = self.create_publisher(DiagnosticStatus, "/diagnostics", 1)
        self.detour_path_pub = self.create_publisher(Path, "/planning/detour_path", 1)
        # Obstacle-avoidance debugging for Foxglove (see publishObstacleDebug)
        self.obstacle_debug_pub = self.create_publisher(ObstacleDebug, "/planning/obstacle_debug", 1)
        self.obstacle_markers_pub = self.create_publisher(MarkerArray, "/vis/obstacle_debug", 1)

        self.generateCandidates()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info("Hello, world!")

        self.goal_point = None
        self.ego_pos = None
        self.ego_yaw = None
        self.ego_linear_vel = 0.0
        self.seedling_points = []
        self.total_cost_map = None
        self.grid_info = None
        self.cached_teleop = Twist()
        self.current_mode = Mode.STOPPED
        self.is_planting = False
        self.is_turning_downhill = False
        self.closest_point_bl = None
        self.closest_point_bl_time = None  # when the last bearing arrived
        self.BEARING_OLD_AGE = 0.5  # s; older bearings are flagged (not acted on) in obstacle_debug
        self.remaining_seedling_count = 0
        self.heading_valid = False
        self.estopped = False
        self.point_turn_direction = 0  # +1 left, -1 right, 0 not point turning

        self.occ_map = None
        self.occ_info = None
        self.occ_stamp = None  # header stamp of the latest grid

        # The lidar pipeline is stopped while planting, so after planting the last grid
        # is stale. Right after the pipeline restarts, grids can also arrive in bursts.
        # Don't drive until a steady run of grids stamped after `fresh_since` arrives
        # (parameters fresh_grids_required / fresh_grid_max_gap). A stale-data hold
        # re-arms the same gate, so a bursty stream gives one hold, not stop/go.
        self.awaiting_fresh_occ = False
        self.fresh_since = None       # only grids stamped after this count
        self.fresh_reason = ""        # why we're waiting, for status/log
        self.steady_grids = 0         # consecutive fresh grids, each < max gap apart
        self.last_fresh_occ_stamp = None

        # After planting, drive straight ahead before resuming normal navigation, so
        # the seedling passes fully under the robot and the next turn can't run it
        # over. One full robot length clears it wherever the planter is mounted.
        self.ROBOT_LENGTH = 1.52        # m, Warthog (same as nav_viz_node)
        self.POST_PLANT_LEEWAY = 0.3    # m, margin for wheel slip (odom over-reads) and stopping
        self.POST_PLANT_DISTANCE = self.ROBOT_LENGTH + self.POST_PLANT_LEEWAY  # m
        self.POST_PLANT_SPEED = 0.3     # m/s
        self.POST_PLANT_TIMEOUT = 2.0 * self.POST_PLANT_DISTANCE / self.POST_PLANT_SPEED  # s
        self.post_plant_drive_active = False
        self.post_plant_start_xy = None    # odom-frame position where the drive started
        self.post_plant_drive_time = 0.0   # s spent commanding forward (for the timeout)

        # Obstacle stop box, in base_link meters
        self.STOP_BOX_MIN_X = 1.0  # starts past the occupancy node's self-filter
        self.STOP_BOX_MAX_X = 3.0
        self.STOP_BOX_HALF_WIDTH =  1.0 # robot half-width plus margin, (warthog width=1.34m)
        self.MIN_OBSTACLE_CELLS = 3  # reject single-point LiDAR noise

        # Detour planning (meters)
        self.NO_GO_RADIUS = 1.1        # > stop box half-width (1.0), so a planned path never trips the box
        self.SOFT_WEIGHT = 3.0         # how strongly the path avoids the band outside no-go
        self.SOFT_FALLOFF = 0.3        # how fast that soft cost fades (tuned: ~1.7 m clearance)
        self.START_CLEAR_RADIUS = 1.0  # lets the search leave the robot's own spot
        self.MAX_GOAL_SHIFT = 2.0      # if the seedling cell is blocked, accept a free cell this close

        # Detour following
        self.DETOUR_SETTLE_TIME = 1.0  # s after stopping before a grid counts as fresh
        # Dynamic obstacles: after stopping, wait this long before planning a detour.
        # If the stop box clears in that time (e.g. a person walked past), resume the
        # normal path; otherwise treat the obstacle as static and detour around it.
        self.DYNAMIC_OBSTACLE_WAIT = 3.0  # s
        # Consecutive clear scans needed before treating a stopped-for obstacle as gone.
        # Counted per new grid, not per tick, since the grid can arrive at 1-10 Hz.
        # Stops a one-scan LiDAR dropout from resuming the robot and restarting the wait.
        self.OBSTACLE_CLEAR_SCANS = 3
        self.DETOUR_WP_SPACING = 2.0   # m between waypoints kept from the planned path
        self.DETOUR_WP_REACHED = 0.7   # m; closer than this counts as reached
        self.DETOUR_SPEED = 0.3        # m/s cap while detouring
        self.DETOUR_ALIGNED = np.radians(20)  # facing the waypoint this well + box blocked = path blocked
        # Steer at a point this far ahead along the path. The Warthog starts turning
        # ~1.5 s after a command (11-3 bag), ~0.45 m at detour speed; this is ~3x that.
        self.DETOUR_LOOKAHEAD = 1.5    # m

        # Detour exit: leave the detour once the straight line to the seedling is clear,
        # instead of following the path out to the grid edge.
        self.DETOUR_EXIT_CLEARANCE = self.STOP_BOX_HALF_WIDTH + 0.3  # m; keeps the stop box clear after exiting
        self.DETOUR_EXIT_LOOKAHEAD = 6.0  # m of the line to the seedling that must be clear
        self.DETOUR_EXIT_TICKS = 5        # consecutive clear ticks (0.5 s) before exiting

        # Detour state. detour_map is None when not detouring.
        self.detour_map = None         # Nx2 path points in the map frame
        self.detour_idx = 0            # index of the closest path point (progress along the path)
        self.detour_clear_ticks = 0    # consecutive ticks the exit check passed
        # The grid only shows obstacles in front of the robot, so the obstacles we
        # planned around are remembered (map frame) to tell when they're behind us.
        self.detour_obstacles_map = np.zeros((0, 2))
        self.obstacle_stop_time = None # when we stopped for the current obstacle
        self.obstacle_clear_scans = 0  # consecutive clear grids seen while stopped
        self.last_clear_stamp = None   # grid stamp last counted as clear
        self.last_planned_stamp = None # grid stamp we last planned on (plan once per grid)
        self.stop_reason = ""          # shown in the status while stopped for an obstacle
        # The obstacle the current detour goes around (cells touching the stop box when
        # the detour was planned, map frame). Only used for the debug markers.
        self.detour_target_map = np.zeros((0, 2))
        self.last_status = ""          # latest /diagnostics message, echoed in obstacle_debug

        self.previous_twist = Twist()

        self.create_timer(0.1, self.plannerTick)
        # self.create_timer(0.1, self.updateTrajectory)

    def closestPointBlCb(self, msg: PointStamped):
        self.closest_point_bl = [msg.point.x, msg.point.y]
        # cost_map_node stamps this with the time of the /gnss/odom pose it was
        # computed from, so the age check below sees how old the bearing really is.
        self.closest_point_bl_time = Time.from_msg(msg.header.stamp)

    def bearingAge(self):
        """s since the pose the seedling bearing was computed from, or None with no bearing."""
        if self.closest_point_bl_time is None:
            return None
        return (self.get_clock().now() - self.closest_point_bl_time).nanoseconds * 1e-9

    def isPlantingCb(self, msg: Bool):
        if msg.data and not self.is_planting:
            self.clearDetour()  # seedling reached; any detour belonged to it
            self.post_plant_drive_active = False
        if self.is_planting and not msg.data:
            self.awaitSteadyOcc("after planting")
            self.post_plant_drive_active = True
            self.post_plant_start_xy = None
            self.post_plant_drive_time = 0.0
        self.is_planting = msg.data

    def awaitSteadyOcc(self, reason: str):
        """Hold until a steady run of grids stamped from now on arrives (see occCb)."""
        self.awaiting_fresh_occ = True
        self.fresh_since = self.get_clock().now()
        self.fresh_reason = reason
        self.steady_grids = 0
        self.last_fresh_occ_stamp = None

    def isTurningDownhillCb(self, msg: Bool):
        self.is_turning_downhill = msg.data

    def teleopTwistCb(self, msg: Twist):
        self.cached_teleop = msg

    def currentModeCb(self, msg: Mode):
        if msg.level != self.current_mode:
            # e.g. operator took over in teleop: the old detour no longer applies
            self.clearDetour()
            self.obstacle_stop_time = None
        self.current_mode = msg.level

    # separated gps and local odomertry because ekf is not ready and there's no fully fused global odometry yet
    # def gpsodomCb(self, msg: Odometry):
    #     pos = msg.pose.pose.position
    #     self.ego_pos = (pos.x, pos.y)

    # def localodomCb(self, msg: Odometry):
    #     q = msg.pose.pose.orientation
    #     r = R.from_quat([q.x, q.y, q.z, q.w])
    #     self.ego_yaw = r.as_euler("xyz")[2]

    #     self.ego_linear_vel = msg.twist.twist.linear.x

    def egoYawCb(self, msg: Float32):
        # self.get_logger().info("Updated ego yaw")
        self.ego_yaw = msg.data

    def headingValidCb(self, msg: Bool):
        self.heading_valid = msg.data

    def estopCb(self, msg: Bool):
        self.estopped = msg.data

    def goalPointGeoCb(self, msg: GeoPoint):
        self.goal_point = list(self.latLonToMap(msg.latitude, msg.longitude))

    def totalCostCb(self, msg: OccupancyGrid):
        arr = np.asarray(msg.data).reshape(msg.info.height, msg.info.width)

        self.total_cost_map = arr
        self.grid_info = msg.info

    def occCb(self, msg: OccupancyGrid):
        self.occ_map = np.asarray(msg.data).reshape(msg.info.height, msg.info.width)
        self.occ_info = msg.info
        self.occ_stamp = Time.from_msg(msg.header.stamp)

        if not self.awaiting_fresh_occ or self.occ_stamp <= self.fresh_since:
            return

        # Count consecutive fresh grids by their stamps (occupancy_grid_node stamps each
        # grid when it publishes it), so planner callback delays don't affect the gaps.
        # Read the parameters here so `ros2 param set` takes effect without a restart.
        required = max(1, int(self.get_parameter("fresh_grids_required").value))
        max_gap = float(self.get_parameter("fresh_grid_max_gap").value)
        gap = None if self.last_fresh_occ_stamp is None else \
            (self.occ_stamp - self.last_fresh_occ_stamp).nanoseconds * 1e-9
        self.steady_grids = self.steady_grids + 1 if (gap is not None and gap < max_gap) else 1
        self.last_fresh_occ_stamp = self.occ_stamp

        if self.steady_grids >= required:
            self.awaiting_fresh_occ = False
            waited = (self.get_clock().now() - self.fresh_since).nanoseconds / 1e9
            self.get_logger().info(
                f"Steady obstacle data ({required} grids) {waited:.1f} s {self.fresh_reason}; resuming")

    def isObstacleAhead(self, max_x=None) -> bool:
        """True if enough occupied cells sit in a box directly ahead of the robot.
        max_x shortens the box (used mid-detour to check only up to the next waypoint)."""
        return self.stopBoxCellCount(max_x) >= self.MIN_OBSTACLE_CELLS

    def stopBoxCellCount(self, max_x=None) -> int:
        """Occupied cells in the stop box (see stopBoxSlices)."""
        box = self.stopBoxSlices(max_x)
        if box is None:
            return 0
        return int(np.count_nonzero(self.occ_map[box] >= 50))

    def stopBoxSlices(self, max_x=None):
        """(rows, cols) slices of the stop box in occ_map, or None with no grid yet."""
        if self.occ_map is None or self.occ_info is None:
            return None

        res = self.occ_info.resolution
        ox = self.occ_info.origin.position.x
        oy = self.occ_info.origin.position.y
        height, width = self.occ_map.shape
        box_max_x = self.STOP_BOX_MAX_X if max_x is None else min(max_x, self.STOP_BOX_MAX_X)

        # round, not truncate: (0.6 - -10.0) / 0.2 is 52.99999... in floating point,
        # and int() would silently drop the far row of the box.
        col_lo = int(np.clip(round((self.STOP_BOX_MIN_X - ox) / res), 0, width))
        col_hi = int(np.clip(round((box_max_x - ox) / res), 0, width))
        row_lo = int(np.clip(round((-self.STOP_BOX_HALF_WIDTH - oy) / res), 0, height))
        row_hi = int(np.clip(round((self.STOP_BOX_HALF_WIDTH - oy) / res), 0, height))

        return slice(row_lo, row_hi), slice(col_lo, col_hi)

    def stopBoxObstacleBl(self, max_x=None):
        """Cell centers (Nx2, base_link) of every obstacle touching the stop box: each
        connected blob of occupied cells with at least one cell in the box, so the whole
        obstacle shows, not just the part inside the box. Used for the debug markers."""
        box = self.stopBoxSlices(max_x)
        if box is None:
            return np.zeros((0, 2))
        blobs, _ = label(self.occ_map >= 50, structure=np.ones((3, 3)))
        ids = np.unique(blobs[box])
        ids = ids[ids > 0]
        if len(ids) == 0:
            return np.zeros((0, 2))
        rows, cols = np.nonzero(np.isin(blobs, ids))
        res = self.occ_info.resolution
        ox, oy = self.occ_info.origin.position.x, self.occ_info.origin.position.y
        return np.column_stack((ox + (cols + 0.5) * res, oy + (rows + 0.5) * res))

    def metersToCell(self, x, y):
        res = self.occ_info.resolution
        ox, oy = self.occ_info.origin.position.x, self.occ_info.origin.position.y
        return int(math.floor((y - oy) / res)), int(math.floor((x - ox) / res))

    def cellToMeters(self, r, c):
        res = self.occ_info.resolution
        ox, oy = self.occ_info.origin.position.x, self.occ_info.origin.position.y
        return ox + (c + 0.5) * res, oy + (r + 0.5) * res   # cell center

    def buildCostArray(self):
        occ = self.occ_map >= 50
        if not occ.any():
            return np.ones(occ.shape)   # needed: the distance calc gives garbage with no obstacles
        dist_m = distance_transform_edt(~occ) * self.occ_info.resolution   # distance from every cell to nearest obstacle (in meters)
        cost = 1.0 + self.SOFT_WEIGHT * np.exp(-(dist_m - self.NO_GO_RADIUS) / self.SOFT_FALLOFF)
        cost[dist_m < self.NO_GO_RADIUS] = np.inf    # inf = impassable for route_through_array
        return cost

    def planDetour(self, goal_bl):
        cost = self.buildCostArray()
        res = self.occ_info.resolution
        h, w = cost.shape

        # Start: the robot's cell. If it's inside a no-go zone (e.g. a bush beside us),
        # make the cells around it expensive instead of impassable so the search can leave.
        start = self.metersToCell(0.0, 0.0)
        rr, cc = disk(start, self.START_CLEAR_RADIUS / res, shape=cost.shape)
        patch = cost[rr, cc]
        patch[np.isinf(patch)] = 20.0
        cost[rr, cc] = patch

        # Goal: if the seedling is outside the grid, pull it back along the line
        # from the robot until it's just inside the edge.
        gx, gy = goal_bl
        ox, oy = self.occ_info.origin.position.x, self.occ_info.origin.position.y
        xmin, xmax = ox + res, ox + w * res - res
        ymin, ymax = oy + res, oy + h * res - res
        s = 1.0
        if gx > xmax: s = min(s, xmax / gx)
        if gx < xmin: s = min(s, xmin / gx)
        if gy > ymax: s = min(s, ymax / gy)
        if gy < ymin: s = min(s, ymin / gy)
        goal = self.metersToCell(gx * s, gy * s)

        # If the goal cell is blocked (seedling next to a rock), use the nearest free cell.
        if np.isinf(cost[goal]):
            dist, (ir, ic) = distance_transform_edt(~np.isfinite(cost), return_indices=True)
            if dist[goal] * res > self.MAX_GOAL_SHIFT:
                return None
            goal = (ir[goal], ic[goal])

        try:
            cells, _ = route_through_array(cost, start, goal, fully_connected=True, geometric=True)
        except ValueError:          # raised when no path exists
            return None
        return np.array([self.cellToMeters(r, c) for r, c in cells])

    def thinPath(self, path_bl):
        """Keep about one point every DETOUR_WP_SPACING m, plus the final point.
        Dropping the first cell (the robot itself) on purpose."""
        step = max(1, int(round(self.DETOUR_WP_SPACING / self.occ_info.resolution)))
        pts = path_bl[step::step]
        if len(pts) == 0 or not np.allclose(pts[-1], path_bl[-1]):
            pts = np.vstack((pts, path_bl[-1:]))
        return pts

    def getPoseInMap(self):
        """(x, y, yaw) of base_link in the map frame, or None if TF is missing or old."""
        try:
            tf = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warning(f"No map->base_link: {ex}", throttle_duration_sec=2.0)
            return None
        age = (self.get_clock().now() - Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9
        if age > 1.0:
            self.get_logger().warning(f"map->base_link is {age:.1f} s old", throttle_duration_sec=2.0)
            return None
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return tf.transform.translation.x, tf.transform.translation.y, yaw

    def getOdomXY(self):
        """(x, y) of base_link in the odom frame (Warthog wheel odometry), or None if
        missing or old. Used for short relative moves: it's smooth and doesn't depend
        on gnss_interface's map->odom, which can lag several seconds (11-13 bags)."""
        try:
            tf = self.tf_buffer.lookup_transform("odom", "base_link", rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warning(f"No odom->base_link: {ex}", throttle_duration_sec=2.0)
            return None
        age = (self.get_clock().now() - Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9
        if age > 1.0:
            self.get_logger().warning(f"odom->base_link is {age:.1f} s old", throttle_duration_sec=2.0)
            return None
        return np.array([tf.transform.translation.x, tf.transform.translation.y])

    def blToMap(self, pts_bl, pose):
        """base_link points (Nx2) -> map frame, given the robot pose (x, y, yaw) in map."""
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        px, py = pts_bl[:, 0], pts_bl[:, 1]
        return np.column_stack((x + c * px - s * py, y + s * px + c * py))

    def mapToBl(self, pts_map, pose):
        """map points (Nx2) -> base_link. Inverse of blToMap."""
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = pts_map[:, 0] - x, pts_map[:, 1] - y
        return np.column_stack((c * dx + s * dy, -s * dx + c * dy))

    def nextDetourWaypoint(self, pose):
        """The point to steer at, in base_link: DETOUR_LOOKAHEAD m along the path past
        the closest path point. None once the end of the path is reached.
        A target that slides along the path doesn't jump at handoffs, and its bearing
        doesn't swing wildly the way a waypoint's does as the robot gets close to it."""
        robot = np.array(pose[:2])
        path = self.detour_map
        if np.linalg.norm(path[-1] - robot) < self.DETOUR_WP_REACHED:
            return None
        # Closest point, searched only ~4 m ahead so progress never goes backward.
        window = path[self.detour_idx:self.detour_idx + 20]
        self.detour_idx += int(np.argmin(np.linalg.norm(window - robot, axis=1)))
        along = np.concatenate(
            ([0.0], np.cumsum(np.linalg.norm(np.diff(path[self.detour_idx:], axis=0), axis=1))))
        j = min(self.detour_idx + int(np.searchsorted(along, self.DETOUR_LOOKAHEAD)), len(path) - 1)
        return self.mapToBl(path[j][None, :], pose)[0]

    def occupiedCellsBl(self):
        """Centers of occupied grid cells (Nx2), in base_link."""
        rows, cols = np.nonzero(self.occ_map >= 50)
        res = self.occ_info.resolution
        ox, oy = self.occ_info.origin.position.x, self.occ_info.origin.position.y
        return np.column_stack((ox + (cols + 0.5) * res, oy + (rows + 0.5) * res))

    def seedlingLineClear(self, pose):
        """True if the first DETOUR_EXIT_LOOKAHEAD m of the straight line to the seedling
        stays DETOUR_EXIT_CLEARANCE from every obstacle, remembered or currently seen."""
        gx, gy = self.closest_point_bl
        d = math.hypot(gx, gy)
        if d < 1e-3:
            return True
        ab = np.array([gx, gy]) / d * min(d, self.DETOUR_EXIT_LOOKAHEAD)
        obs = np.vstack((self.mapToBl(self.detour_obstacles_map, pose), self.occupiedCellsBl()))
        if len(obs) == 0:
            return True
        # Distance from each obstacle point to the segment from the robot to ab.
        t = np.clip(obs @ ab / (ab @ ab), 0.0, 1.0)
        return float(np.linalg.norm(obs - t[:, None] * ab, axis=1).min()) >= self.DETOUR_EXIT_CLEARANCE

    def stopAndPlanDetour(self):
        """Obstacle ahead and no detour yet: stop, wait to see if it moves, then plan
        once per grid. If the obstacle clears during the wait, updateTrajectorySimply
        stops calling this and the normal path resumes."""
        self.twist_pub.publish(self.getSmoothed(Twist()))
        self.point_turn_direction = 0

        now = self.get_clock().now()
        if self.obstacle_stop_time is None:
            self.obstacle_stop_time = now
            self.stop_reason = "checking if obstacle is static or dynamic"
            self.get_logger().info(
                f"Obstacle ahead; waiting {self.DYNAMIC_OBSTACLE_WAIT:.1f} s before detouring")
        waited = (now - self.obstacle_stop_time).nanoseconds * 1e-9
        status = f"Stopped ({waited:.0f} s): {self.stop_reason}"

        # Only plan on a grid built after the dynamic-obstacle wait (which is also
        # long enough for the scan to settle after stopping), and only once per grid.
        wait = max(self.DETOUR_SETTLE_TIME, self.DYNAMIC_OBSTACLE_WAIT)
        settled = self.obstacle_stop_time + Duration(seconds=wait)
        already_tried = self.last_planned_stamp is not None and self.occ_stamp == self.last_planned_stamp
        if self.occ_stamp is None or self.occ_stamp < settled or already_tried:
            self.publishStatus(status, DiagnosticStatus.WARN)
            return
        self.last_planned_stamp = self.occ_stamp

        path_bl = None
        pose = self.getPoseInMap() if self.heading_valid else None
        if not self.heading_valid:
            self.stop_reason = "obstacle ahead, heading unknown"
        elif pose is None:
            self.stop_reason = "obstacle ahead, robot pose unavailable"
        else:
            path_bl = self.planDetour(self.closest_point_bl)
            if path_bl is None or len(path_bl) < 3:
                # No way around (or can't get any closer). Stay stopped; retry on the next grid.
                self.stop_reason = "no way around obstacle, retrying on each new scan"
                path_bl = None
        if path_bl is None:
            self.publishStatus(f"Stopped ({waited:.0f} s): {self.stop_reason}", DiagnosticStatus.WARN)
            return

        # Keep the full path (first cell is the robot itself); nextDetourWaypoint
        # steers at a point sliding along it.
        self.detour_map = self.blToMap(path_bl[1:], pose)
        self.detour_idx = 0
        self.detour_clear_ticks = 0
        occ_bl = self.occupiedCellsBl()
        self.detour_obstacles_map = self.blToMap(
            occ_bl[np.hypot(occ_bl[:, 0], occ_bl[:, 1]) < 8.0], pose)
        self.detour_target_map = self.blToMap(self.stopBoxObstacleBl(), pose)
        self.obstacle_stop_time = None
        self.publishDetourPath()
        self.get_logger().info(
            f"Detour planned: {len(self.detour_map)} waypoints, {waited:.1f} s after stopping")

    def clearDetour(self):
        if self.detour_map is not None:
            self.detour_map = None
            self.publishDetourPath()  # empty path clears it in Foxglove
        self.detour_target_map = np.zeros((0, 2))
        self.point_turn_direction = 0
        self.detour_clear_ticks = 0

    def publishDetourPath(self):
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.detour_map is not None:
            for x, y in self.detour_map:
                pose_msg = PoseStamped()
                pose_msg.header = msg.header
                pose_msg.pose.position.x = float(x)
                pose_msg.pose.position.y = float(y)
                msg.poses.append(pose_msg)
        self.detour_path_pub.publish(msg)

    # ---- Obstacle-avoidance debugging (Foxglove) ----

    def plannerTick(self):
        self.updateTrajectorySimply()
        # Debug output must never stop the robot from being driven.
        try:
            self.publishObstacleDebug()
        except Exception as e:
            self.get_logger().error(f"Obstacle debug publish failed: {e!r}",
                                    throttle_duration_sec=5.0)

    def obstacleDebugState(self, waited):
        """(state, detail) for /planning/obstacle_debug, derived from the planner's
        obstacle state so the driving code doesn't need to report it separately."""
        if self.detour_map is not None:
            detail = f"detour point {self.detour_idx + 1} / {len(self.detour_map)}"
            if self.detour_clear_ticks > 0:
                detail += f", obstacle passed check {self.detour_clear_ticks} / {self.DETOUR_EXIT_TICKS}"
            return ObstacleDebug.DETOURING, detail
        if self.obstacle_stop_time is None:
            # Not stopped yet, but the box can already be blocked: e.g. the tick a blocked
            # detour is dropped, before the next tick stops the robot.
            if self.isObstacleAhead():
                return ObstacleDebug.OBSTACLE_DETECTED, "stopping"
            return ObstacleDebug.CLEAR, ""
        if self.stop_reason.startswith("no way around"):
            return ObstacleDebug.NO_PATH, "no route around obstacle; retrying on each new scan"
        if abs(self.previous_twist.linear.x) > 0.05 or abs(self.previous_twist.angular.z) > 0.05:
            return ObstacleDebug.OBSTACLE_DETECTED, "stopping"
        # Stopped: stopAndPlanDetour waits DETOUR_SETTLE_TIME, then plans on the next fresh grid.
        detail = f"{min(waited, self.DETOUR_SETTLE_TIME):.1f} / {self.DETOUR_SETTLE_TIME:.1f} s"
        if waited >= self.DETOUR_SETTLE_TIME:
            # Planning on the next fresh grid, or blocked by a missing heading/pose.
            reason = self.stop_reason if "heading" in self.stop_reason or "pose" in self.stop_reason \
                else "planning detour on next scan"
            detail += f" ({reason})"
        else:
            detail += " (waiting for fresh scan)"
        return ObstacleDebug.CHECKING, detail

    def publishObstacleDebug(self):
        now = self.get_clock().now()
        waited = 0.0 if self.obstacle_stop_time is None else \
            (now - self.obstacle_stop_time).nanoseconds * 1e-9
        state, detail = self.obstacleDebugState(waited)
        detouring = self.detour_map is not None
        # Mid-detour the planner only checks the stretch just ahead (see updateTrajectorySimply).
        box_max_x = self.DETOUR_LOOKAHEAD + 0.5 if detouring else None

        bearing_age = self.bearingAge()
        bearing_old = bearing_age is not None and bearing_age > self.BEARING_OLD_AGE
        if bearing_old:
            detail = (f"{detail}\n" if detail else "") + \
                f"BEARING OLD: {bearing_age:.1f} s"

        msg = ObstacleDebug()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "base_link"
        msg.state = state
        msg.detail = detail
        msg.planner_status = self.last_status
        msg.wait_elapsed = float(waited)
        msg.wait_required = float(self.DETOUR_SETTLE_TIME)
        # clear_scans / clear_scans_required stay 0: no clear-scan check on this branch.
        msg.bearing_age = -1.0 if bearing_age is None else float(bearing_age)
        msg.bearing_old_age = float(self.BEARING_OLD_AGE)
        msg.bearing_old = bearing_old
        msg.stop_box_cells = int(min(self.stopBoxCellCount(box_max_x), 65535))
        msg.stop_box_cells_required = int(self.MIN_OBSTACLE_CELLS)
        msg.grid_age = -1.0 if self.occ_stamp is None else float((now - self.occ_stamp).nanoseconds * 1e-9)
        if detouring:
            msg.detour_waypoint = int(min(self.detour_idx + 1, 65535))
            msg.detour_length = int(min(len(self.detour_map), 65535))
        msg.detour_exit_ticks = int(min(self.detour_clear_ticks, 255))
        msg.detour_exit_ticks_required = int(self.DETOUR_EXIT_TICKS)
        self.obstacle_debug_pub.publish(msg)

        self.obstacle_markers_pub.publish(self.obstacleMarkers(msg, box_max_x))

    STATE_COLORS = {  # RGBA, matches the Indicator panel in config/foxglove/nav_debug_layout.json
        ObstacleDebug.CLEAR: (0.13, 0.77, 0.37, 1.0),               # green
        ObstacleDebug.OBSTACLE_DETECTED: (0.98, 0.80, 0.08, 1.0),   # yellow
        ObstacleDebug.CHECKING: (0.96, 0.62, 0.04, 1.0),            # amber
        ObstacleDebug.DETOURING: (0.23, 0.51, 0.96, 1.0),           # blue
        ObstacleDebug.NO_PATH: (0.94, 0.27, 0.27, 1.0),             # red
    }

    def obstacleMarkers(self, dbg: ObstacleDebug, box_max_x):
        """Stop box, the obstacle in it (from detection, through the wait, and the one a
        detour goes around), the remembered obstacles, the detour exit line, and a label."""
        stamp = dbg.header.stamp

        def rgba(c):
            return ColorRGBA(r=c[0], g=c[1], b=c[2], a=c[3])

        color = rgba(self.STATE_COLORS[dbg.state])
        markers = [Marker(action=Marker.DELETEALL)]  # drop last tick's markers

        def marker(ns, mtype, frame="base_link"):
            m = Marker(ns=ns, id=0, type=mtype, action=Marker.ADD)
            m.header.frame_id = frame
            m.header.stamp = stamp
            m.pose.orientation.w = 1.0
            markers.append(m)
            return m

        def cells(ns, pts, color_tuple, frame="base_link", z=0.15):
            if len(pts) == 0:
                return
            m = marker(ns, Marker.CUBE_LIST, frame)
            res = self.occ_info.resolution if self.occ_info is not None else 0.2
            m.scale.x = m.scale.y = res * 0.9
            m.scale.z = 0.3
            m.color = rgba(color_tuple)
            m.points = [Point(x=float(x), y=float(y), z=z) for x, y in pts]

        # Stop box outline (the short box while detouring), coloured by state.
        box = marker("stop_box", Marker.LINE_STRIP)
        x0 = self.STOP_BOX_MIN_X
        x1 = self.STOP_BOX_MAX_X if box_max_x is None else min(box_max_x, self.STOP_BOX_MAX_X)
        w = self.STOP_BOX_HALF_WIDTH
        box.points = [Point(x=x, y=y, z=0.05) for x, y in
                      ((x0, -w), (x1, -w), (x1, w), (x0, w), (x0, -w))]
        box.scale.x = 0.08
        box.color = color

        # The obstacle in the stop box, from the latest grid: shows from the first tick
        # it's seen and follows it through the static-or-dynamic wait.
        if dbg.stop_box_cells > 0:
            cells("obstacle", self.stopBoxObstacleBl(box_max_x), self.STATE_COLORS[dbg.state])

        pose = self.getPoseInMap() if self.detour_map is not None else None
        if self.detour_map is not None and pose is not None:
            # Fixed in the map: what the detour was planned around (red) and every
            # remembered obstacle the exit check still uses (grey).
            cells("remembered_obstacles", self.detour_obstacles_map, (0.6, 0.6, 0.6, 0.35),
                  frame="map", z=0.05)
            cells("detour_obstacle", self.detour_target_map, (0.94, 0.27, 0.27, 0.8),
                  frame="map", z=0.2)
            # Straight line to the seedling the exit check tests; green while it passes.
            if self.closest_point_bl is not None:
                gx, gy = self.closest_point_bl
                d = math.hypot(gx, gy)
                if d > 1e-3:
                    k = min(d, self.DETOUR_EXIT_LOOKAHEAD) / d
                    line = marker("exit_line", Marker.LINE_STRIP)
                    line.points = [Point(z=0.1), Point(x=gx * k, y=gy * k, z=0.1)]
                    line.scale.x = 0.06
                    line.color = rgba(((0.13, 0.77, 0.37, 1.0) if self.detour_clear_ticks > 0
                                            else (0.7, 0.7, 0.7, 0.8)))

        # State label above the robot.
        text = marker("label", Marker.TEXT_VIEW_FACING)
        text.pose.position.x = 0.0
        text.pose.position.z = 2.0
        text.scale.z = 0.5
        text.color = color
        text.text = dbg.state + (f"\n{dbg.detail}" if dbg.detail else "")

        return MarkerArray(markers=markers)


    def generateCandidates(self, top_speed=1.5, time_horizon=5.0, dt=0.5):

        # Start with top speed
        candidates = []

        v = top_speed
        Omega = np.linspace(-0.1, 0.1, 3)
        candidates_at_speed = []
        mega_mask = np.zeros((100, 100))
        for omega in Omega:
            pose = np.zeros(4)  # Start at ego position, zero speed, zero (relative) yaw
            trajectory = []

            for t in np.arange(0, time_horizon, dt):
                trajectory.append(pose.copy())
                pose[0] += v * math.cos(pose[2]) * dt
                pose[1] += v * math.sin(pose[2]) * dt
                pose[2] += omega * dt
                pose[3] = t + dt

            for second_omega in Omega:
                second_trajectory = trajectory.copy()
                second_pose = pose.copy()

                for t in np.arange(0, time_horizon, dt):
                    second_trajectory.append(second_pose.copy())
                    second_pose[0] += v * math.cos(second_pose[2]) * dt
                    second_pose[1] += v * math.sin(second_pose[2]) * dt
                    second_pose[2] += second_omega * dt
                    second_pose[3] = t + dt

                second_trajectory = np.asarray(second_trajectory)
                candidate = Candidate(v, omega, second_trajectory)
                mask = self.getCandidateMask(candidate)
                candidates_at_speed.append([omega, mask, second_trajectory])
                mega_mask = np.logical_or(mega_mask, mask)
        # plt.imshow(mega_mask, extent=[-8, 12, -10, 10])
        # plt.show()

        candidates.append([v, candidates_at_speed])

        # Now med speed
        v = top_speed * 0.667
        Omega = np.linspace(-0.2, 0.2, 5)
        candidates_at_speed = []
        for omega in Omega:
            pose = np.zeros(4)  # Start at ego position, zero speed, zero (relative) yaw
            trajectory = []

            for t in np.arange(0, time_horizon, dt):
                trajectory.append(pose.copy())
                pose[0] += v * math.cos(pose[2]) * dt
                pose[1] += v * math.sin(pose[2]) * dt
                pose[2] += omega * dt
                pose[3] = t + dt

            for second_omega in Omega:
                second_trajectory = trajectory.copy()
                second_pose = pose.copy()

                for t in np.arange(0, time_horizon, dt):
                    second_trajectory.append(second_pose.copy())
                    second_pose[0] += v * math.cos(second_pose[2]) * dt
                    second_pose[1] += v * math.sin(second_pose[2]) * dt
                    second_pose[2] += second_omega * dt
                    second_pose[3] = t + dt

                second_trajectory = np.asarray(second_trajectory)
                candidate = Candidate(v, omega, second_trajectory)
                mask = self.getCandidateMask(candidate)
                candidates_at_speed.append([omega, mask, second_trajectory])
                mega_mask = np.logical_or(mega_mask, mask)

            # plt.imshow(mask, extent=[-8, 12, -10, 10])
            # plt.show()

        # plt.imshow(mega_mask, extent=[-8, 12, -10, 10])
        # plt.show()

        candidates.append([v, candidates_at_speed])

        # Now low speed
        mega_mask = np.zeros((100, 100))

        v = top_speed * 0.333
        Omega = np.linspace(-0.3, 0.3, 7)
        candidates_at_speed = []
        for omega in Omega:
            pose = np.zeros(4)  # Start at ego position, zero speed, zero (relative) yaw
            trajectory = []

            for t in np.arange(0, time_horizon, dt):
                trajectory.append(pose.copy())
                pose[0] += v * math.cos(pose[2]) * dt
                pose[1] += v * math.sin(pose[2]) * dt
                pose[2] += omega * dt
                pose[3] = t + dt

            for second_omega in Omega:
                second_trajectory = trajectory.copy()
                second_pose = pose.copy()

                for t in np.arange(0, time_horizon, dt):
                    second_trajectory.append(second_pose.copy())
                    second_pose[0] += v * math.cos(second_pose[2]) * dt
                    second_pose[1] += v * math.sin(second_pose[2]) * dt
                    second_pose[2] += second_omega * dt
                    second_pose[3] = t + dt

                second_trajectory = np.asarray(second_trajectory)
                candidate = Candidate(v, omega, second_trajectory)
                mask = self.getCandidateMask(candidate)
                candidates_at_speed.append([omega, mask, second_trajectory])
                mega_mask = np.logical_or(mega_mask, mask)

        # plt.imshow(mega_mask, extent=[-8, 12, -10, 10])
        # plt.show()

        candidates.append([v, candidates_at_speed])

        # plt.figure()
        # plt.gca().set_aspect("equal")

        candidates_msg = TrajectoryCandidates()

        for v, candidates_at_speed in candidates:

            for omega, mask, trajectory in candidates_at_speed:
                candidate_msg = TrajectoryCandidate()
                candidate_msg.omega = omega
                candidate_msg.speed = v

                path_msg = Path()

                stamp = self.get_clock().now().to_msg()
                for x, y, omega, t in trajectory:
                    pose_msg = PoseStamped()
                    pose_msg.pose.position.x = x
                    pose_msg.pose.position.y = y
                    pose_msg.header.frame_id = "base_link"
                    pose_msg.header.stamp = stamp

                    path_msg.poses.append(pose_msg)

                candidate_msg.trajectory = path_msg
                candidates_msg.candidates.append(candidate_msg)

        self.candidates_msg = candidates_msg
        self.candidates_pub.publish(candidates_msg)

        # for trajectory in trajectories:
        #     trajectory = np.asarray(trajectory)
        #     plt.plot(trajectory[:, 0], trajectory[:, 1])

        # plt.savefig("candidates.png")

        self.candidates = candidates

    def getCandidateMask(self, candidate: Candidate, collision_radius: float = 1.0):

        collision_radius_px = collision_radius / 0.2

        grid_coords = candidate.trajectory.copy()[:, :2]
        grid_coords[:, 1] += 10
        grid_coords[:, 0] += 8
        grid_coords /= 0.2

        # grid_coords[:, 0] = self.grid_info.height - grid_coords[:, 1]

        # display_img = img.copy()
        mask = np.zeros((100, 100), dtype=bool)

        for pixel_coord in grid_coords:
            rr, cc = disk(pixel_coord, collision_radius_px, shape=(100, 100))
            # display_img[cc, rr] = 50
            mask[cc, rr] = True

        return mask

    def trajectoryToPath(self, trajectory: np.ndarray) -> Path:
        path_msg = Path()
        path_msg.header.frame_id = "base_link"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for x, y, _, _ in trajectory:
            pose_msg = PoseStamped()
            pose_msg.header = path_msg.header
            pose_msg.pose.position.x = float(x)
            pose_msg.pose.position.y = float(y)
            path_msg.poses.append(pose_msg)

        return path_msg

    def getTotalCost(self, candidate: Candidate, collision_radius: float = 1.0):

        collision_radius_px = collision_radius / self.grid_info.resolution

        if self.total_cost_map is None:
            return

        grid_coords = candidate.trajectory.copy()[:, :2]
        grid_coords[:, 1] -= self.grid_info.origin.position.y
        grid_coords[:, 0] -= self.grid_info.origin.position.x
        grid_coords /= self.grid_info.resolution

        # grid_coords[:, 0] = self.grid_info.height - grid_coords[:, 1]

        img = self.total_cost_map.copy()
        # display_img = img.copy()
        mask = np.zeros_like(self.total_cost_map)

        total_cost = 0

        # fig = plt.figure()

        for pixel_coord in grid_coords:
            rr, cc = disk(pixel_coord, collision_radius_px, shape=img.shape)
            # display_img[cc, rr] = 50
            mask[cc, rr] = 1

            total_cost += np.sum(self.total_cost_map[cc, rr])

        total_cost = np.sum(self.total_cost_map[mask > 0])

        # plt.plot(candidate.trajectory[:, 0], candidate.trajectory[:, 1])
        # plt.imshow(display_img, extent=[-8, 12, -10, 10], cmap="summer")
        # plt.title(f"v={candidate.speed}, o={candidate.omega}, {total_cost}")
        # # plt.show()
        # plt.savefig("candidate_cost.png")
        # plt.close(fig)

        # Now penalize slow candidates

        if (candidate.speed) > 0.9:
            total_cost *= 0.5

        return total_cost

    def latLonToMap(self, lat: float, lon: float):
        lat0, lon0, _ = self.get_parameter("map_origin_lat_lon_alt_degrees").value
        origin_x, origin_y, _, __ = utm.from_latlon(lat0, lon0)

        x, y, _, __ = utm.from_latlon(lat, lon)

        x = x - origin_x
        y = y - origin_y

        return (x, y)
    
    def transformToBaselink(self, points):

        if len(points) < 1:
            return None

        # Form a 2D homogeneous transform matrix
        t = -self.ego_yaw
        u = -self.ego_pos[0]
        v = -self.ego_pos[1]

        H = np.identity(3)
        H[0, 2] = u
        H[1, 2] = v

        R = np.asarray(
            [
                [math.cos(t), -math.sin(t)],
                [math.sin(t), math.cos(t)],
            ]
        )

        points = np.asarray(points)

        # fig, (ax1, ax2, ax3) = plt.subplots(1, 3)
        # ax1.scatter(points[:, 0], points[:, 1])
        # ax1.scatter(*self.ego_pos, c="red")
        # ax1.set_title(f"Yaw: {self.ego_yaw}")

        # Transform to base_link
        try:
            pts_homog = np.vstack((points.T, np.ones(len(points))))
            # self.get_logger().info(f"{pts_homog}")
            pts_tfed = H @ pts_homog
            pts_tfed = pts_tfed.T
            pts_tfed = pts_tfed[:, :-1]
            # self.get_logger().info(f"{self.ego_yaw}")
            # self.get_logger().info(f"{pts_tfed}")

        except ValueError as e:
            # There were no points nearby
            return None

        # ax2.scatter(pts_tfed[:, 0], pts_tfed[:, 1])
        # ax2.scatter(0, 0, c="red")
        # ax2.set_xlim((-20, 20))
        # ax2.set_ylim((-20, 20))

        pts_tfed = (R @ pts_tfed.T).T
        # ax3.scatter(pts_tfed[:, 0], pts_tfed[:, 1])
        # ax3.scatter(0, 0, c="red")
        # ax3.set_xlim((-20, 20))
        # ax3.set_ylim((-20, 20))

        # plt.show()

        return pts_tfed

    def getClosestSeedlingInBaselink(self):
        if self.total_cost_map is None or self.grid_info is None:
            self.get_logger().warning(
                "Could not find closest seedling point. Cost map metadata unavailable."
            )
            return None

        pixel_coords = np.unravel_index(self.total_cost_map.argmin(), self.total_cost_map.shape)
        goal_x = (
            pixel_coords[1] * self.grid_info.resolution
            + self.grid_info.origin.position.x
        )
        goal_y = (
            pixel_coords[0] * self.grid_info.resolution
            + self.grid_info.origin.position.y
        )

        return [goal_x, goal_y]

    def planCb(self, msg: PlantingPlan):
        self.remaining_seedling_count = len(msg.seedlings)
        self.get_logger().info(f"Got plan with {self.remaining_seedling_count} seedlings")

    def getYawError(self, goal_point_bl):
        yaw_error = math.atan2(goal_point_bl[0], goal_point_bl[1]) - np.pi / 2
        yaw_error *= -1

        yaw_error = (yaw_error + np.pi) % (2 * np.pi) - np.pi
        # print(f"{goal_point_bl} -> {yaw_error / np.pi * 180}")

        return yaw_error

    def pointTurnFromYawError(self, yaw_error, omega=0.8, linear=0.2):
        cmd_msg = Twist()

        if yaw_error < 0:
            omega *= -1

        cmd_msg.angular.z = omega
        cmd_msg.linear.x = linear

        self.twist_pub.publish(self.getSmoothed(cmd_msg))

    def publishStatus(self, desc: str, level=DiagnosticStatus.OK):
        self.last_status = desc
        self.status_pub.publish(
            DiagnosticStatus(message=desc, level=level, name=self.get_name())
        )

    def getSmoothed(
        self, in_twist: Twist, linear_max_delta=0.2, angular_max_delta=0.3
    ) -> Twist:
        previous_linear_speed = self.previous_twist.linear.x
        previous_angular_speed = self.previous_twist.angular.z

        requested_linear_speed = in_twist.linear.x
        requested_angular_speed = in_twist.angular.z

        if requested_linear_speed > previous_linear_speed + linear_max_delta:
            smoothed_linear_speed = previous_linear_speed + linear_max_delta
        elif requested_linear_speed < previous_linear_speed - linear_max_delta:
            smoothed_linear_speed = previous_linear_speed - linear_max_delta
        else:
            smoothed_linear_speed = requested_linear_speed

        if requested_angular_speed > previous_angular_speed + angular_max_delta:
            smoothed_angular_speed = previous_angular_speed + angular_max_delta
        elif requested_angular_speed < previous_angular_speed - angular_max_delta:
            smoothed_angular_speed = previous_angular_speed - angular_max_delta
        else:
            smoothed_angular_speed = requested_angular_speed

        # print(f"{requested_linear_speed} -> {smoothed_linear_speed}")

        smoothed_twist = Twist()
        smoothed_twist.linear.x = smoothed_linear_speed
        smoothed_twist.angular.z = smoothed_angular_speed

        self.previous_twist = smoothed_twist
        return smoothed_twist

    def updateTrajectorySimply(self):
        if self.estopped:
            # Don't wind up a turn command while the platform can't move; it
            # would be executed at full rate the moment the e-stop is released.
            self.publishStatus("E-stopped")
            self.previous_twist = Twist()
            self.point_turn_direction = 0
            self.twist_pub.publish(Twist())
            return

        if self.current_mode == Mode.STOPPED:
            self.publishStatus("Paused")
            self.twist_pub.publish(self.getSmoothed(Twist()))
            return

        if self.is_turning_downhill:
            self.publishStatus("Turning downhill before planting")
            yaw_error = self.TARGET_DOWNHILL_YAW - self.ego_yaw
            self.get_logger().info(f"{yaw_error}")

            if abs(yaw_error) < 0.2:

                self.facing_downhill_pub.publish(Empty())
                return

            self.pointTurnFromYawError(yaw_error, omega=0.8, linear=0.2)
            return

        if self.is_planting:
            self.publishStatus("Planting a seedling")
            self.twist_pub.publish(self.getSmoothed(Twist()))
            return

        if self.current_mode == Mode.TELEOP:
            self.twist_pub.publish(self.getSmoothed(self.cached_teleop))
            self.publishStatus("Following teleop commands")
            return

        if self.current_mode == Mode.ASSISTED:
            self.get_logger().error("Assisted teleop is not yet supported!")
            return

        if self.closest_point_bl is None:
            self.get_logger().error("Seedling waypoint unknown. Stopping.")
            self.twist_pub.publish(self.getSmoothed(Twist()))
            return

        if self.remaining_seedling_count < 1:
            self.publishStatus("Plan complete.")
            self.twist_pub.publish(self.getSmoothed(Twist()))
            return

        if self.awaiting_fresh_occ:
            self.twist_pub.publish(self.getSmoothed(Twist()))
            self.publishStatus(f"Waiting for steady obstacle data {self.fresh_reason} "
                               f"({self.steady_grids} grids so far)", DiagnosticStatus.WARN)
            return

        # The bearing comes from cost_map_node, computed from /gnss/odom and stamped
        # with that pose's time. It's often late (11-20 bag: fresh poses arrived 0.6-1.2 s
        # old, every 1-2 s), and holding until it was fresh gave stop/go jitter. So keep
        # driving on the last bearing; its age is shown in obstacle_debug (Foxglove).
        bearing_age = self.bearingAge()
        if bearing_age > self.BEARING_OLD_AGE:
            self.get_logger().warn(f"Seedling bearing is {bearing_age:.1f} s old; driving on it",
                                   throttle_duration_sec=5.0)

        # Hold existing grid if lidar data is old
        occ_age = None if self.occ_stamp is None else (self.get_clock().now() - self.occ_stamp).nanoseconds * 1e-9
        if occ_age is None or occ_age > 1.5:
            self.publishStatus("Obstacle data missing" if occ_age is None else
                            f"Obstacle data {occ_age:.1f} s old. Holding.", DiagnosticStatus.WARN)
            self.twist_pub.publish(self.getSmoothed(Twist()))
            # Don't resume on the next single grid: wait for a steady run again.
            reason = "at startup" if occ_age is None else f"after a {occ_age:.1f} s gap"
            self.get_logger().warn(f"Obstacle data stale; waiting for steady obstacle data {reason}")
            self.awaitSteadyOcc(reason)
            return

        # Post-plant drive: straight ahead for POST_PLANT_DISTANCE, then fall through
        # to normal navigation. Runs after the steady-grid wait above, so the stop box
        # check uses lidar data from after planting.
        if self.post_plant_drive_active:
            xy = self.getOdomXY()
            if xy is None:
                # Can't tell how far we've gone; hold rather than drive blind.
                self.point_turn_direction = 0
                self.twist_pub.publish(self.getSmoothed(Twist()))
                self.publishStatus("Post-plant drive: wheel odometry unavailable. Holding.",
                                   DiagnosticStatus.WARN)
                return
            if self.post_plant_start_xy is None:
                self.post_plant_start_xy = xy
            driven = float(np.linalg.norm(xy - self.post_plant_start_xy))

            if driven >= self.POST_PLANT_DISTANCE:
                self.get_logger().info(f"Post-plant drive done ({driven:.2f} m)")
                self.post_plant_drive_active = False
            elif self.post_plant_drive_time > self.POST_PLANT_TIMEOUT:
                # Commanded forward this long without covering the distance (stuck,
                # slipping, or pose not updating). Give up rather than push forever.
                self.get_logger().warn(f"Post-plant drive timed out after {driven:.2f} m")
                self.post_plant_drive_active = False
            elif self.isObstacleAhead():
                # Don't drive into it; let the normal obstacle handling below take over.
                self.get_logger().info(f"Obstacle ahead; ending post-plant drive after {driven:.2f} m")
                self.post_plant_drive_active = False
            else:
                self.point_turn_direction = 0
                cmd_msg = Twist()
                cmd_msg.linear.x = self.POST_PLANT_SPEED
                self.twist_pub.publish(self.getSmoothed(cmd_msg))
                # Only time spent driving counts toward the timeout, so pausing,
                # e-stop or holding for pose/grid data doesn't use it up.
                self.post_plant_drive_time += 0.1  # timer period
                self.publishStatus(
                    f"Driving forward after planting ({driven:.1f}/{self.POST_PLANT_DISTANCE:.1f} m)")
                return

        # Obstacle handling: stop, plan a detour once, then follow its waypoints
        # with the same steering code below. The goal is either the seedling or
        # the current detour waypoint.
        if self.detour_map is not None and not self.heading_valid:
            self.clearDetour()  # a map-frame path can't be followed without heading

        # Leave the detour as soon as the obstacle is passed: stop box clear and the
        # straight line to the seedling clear of everything we know of, for 0.5 s.
        if self.detour_map is not None:
            pose = self.getPoseInMap()
            clear = pose is not None and not self.isObstacleAhead() and self.seedlingLineClear(pose)
            self.detour_clear_ticks = self.detour_clear_ticks + 1 if clear else 0
            if self.detour_clear_ticks >= self.DETOUR_EXIT_TICKS:
                self.get_logger().info("Obstacle passed, line to seedling clear; ending detour")
                self.clearDetour()  # falls through to normal seedling steering below

        if self.detour_map is None:
            blocked = self.isObstacleAhead()
            if blocked:
                self.obstacle_clear_scans = 0
                self.stopAndPlanDetour()
                return
            if self.obstacle_stop_time is not None:
                # Stopped for an obstacle that now reads clear. Stay stopped until it
                # stays clear for a few scans, keeping obstacle_stop_time so a flicker
                # doesn't restart the dynamic-obstacle wait.
                # rclpy Time raises TypeError when compared with None, so check None first.
                if self.last_clear_stamp is None or self.occ_stamp != self.last_clear_stamp:
                    self.last_clear_stamp = self.occ_stamp
                    self.obstacle_clear_scans += 1
                if self.obstacle_clear_scans < self.OBSTACLE_CLEAR_SCANS:
                    self.twist_pub.publish(self.getSmoothed(Twist()))
                    self.point_turn_direction = 0
                    self.publishStatus(
                        f"Stopped: checking if obstacle is static or dynamic. "
                        f"{self.obstacle_clear_scans}/{self.OBSTACLE_CLEAR_SCANS} clear scans",
                        DiagnosticStatus.WARN)
                    return
                self.obstacle_clear_scans = 0
                waited = (self.get_clock().now() - self.obstacle_stop_time).nanoseconds * 1e-9
                self.get_logger().info(f"Obstacle cleared after {waited:.1f} s; resuming path")
            self.obstacle_stop_time = None
            goal_point = self.closest_point_bl
            target = "seedling"
        else:
            pose = self.getPoseInMap()
            if pose is None:
                self.publishStatus("Detour: robot pose unavailable. Holding.", DiagnosticStatus.WARN)
                self.twist_pub.publish(self.getSmoothed(Twist()))
                return
            goal_point = self.nextDetourWaypoint(pose)
            if goal_point is None:
                self.get_logger().info("Detour complete; heading to seedling")
                self.clearDetour()
                self.twist_pub.publish(self.getSmoothed(Twist()))
                return
            # Only check the stretch we're about to drive. The full 3 m box would see
            # the obstacle we're going around and stop the detour.
            blocked = self.isObstacleAhead(max_x=self.DETOUR_LOOKAHEAD + 0.5)
            if blocked and abs(self.getYawError(goal_point)) < self.DETOUR_ALIGNED:
                # Facing along the path and the stop box is still blocked: something
                # new is on the path. Drop it; next tick stops and plans from here.
                self.get_logger().info("Detour path blocked; replanning")
                self.clearDetour()
                self.twist_pub.publish(self.getSmoothed(Twist()))
                return
            target = f"detour point {self.detour_idx + 1}/{len(self.detour_map)}"

        distance_remaining = np.linalg.norm(goal_point)
        yaw_error = self.getYawError(goal_point)

        if not self.heading_valid:
            # GPS track only gives heading once we're moving, and the value the
            # receiver reports before that is stale (0 deg = north at startup).
            # Drive straight until gnss_interface has a heading instead of
            # turning towards a goal computed from a heading we don't have.
            self.point_turn_direction = 0
            if distance_remaining < 2.0:
                self.publishStatus("Heading unknown and seedling too close to acquire it. Stopping.",
                                   level=DiagnosticStatus.WARN)
                self.twist_pub.publish(self.getSmoothed(Twist()))
                return
            cmd_msg = Twist()
            cmd_msg.linear.x = 0.5
            self.twist_pub.publish(self.getSmoothed(cmd_msg))
            self.publishStatus("Driving straight to acquire heading")
            return

        # Point turn with hysteresis: start above 45 deg, stop below 30 deg. The
        # direction is only held while the goal is roughly behind us (where the
        # sign of the error flips from cycle to cycle); otherwise turn the short
        # way, so a wrong first choice (e.g. from a stale bearing) corrects itself.
        # Near the goal, SBAS position noise swings the bearing a lot, so only
        # turn in place if the goal is clearly behind us.
        POINT_TURN_START = np.pi / 4
        # The Warthog keeps turning ~0.4 s / ~15 deg (up to ~35) after the stop
        # command (11-13 bags), so stopping at 30 deg lands near 10-15 deg and the
        # drive steering below takes out the rest while moving.
        POINT_TURN_STOP = np.radians(30)
        KEEP_DIRECTION_ABOVE = np.radians(135)
        NEAR_GOAL = 2.0  # m
        # The near-goal exception is for SBAS noise around the seedling; detour
        # waypoints are always ~2 m away, so they use the normal threshold.
        far_or_detour = distance_remaining > NEAR_GOAL or self.detour_map is not None
        start_threshold = POINT_TURN_START if far_or_detour else np.pi / 2

        if self.point_turn_direction == 0 and abs(yaw_error) > start_threshold:
            self.point_turn_direction = 1 if yaw_error > 0 else -1
        elif self.point_turn_direction != 0:
            if abs(yaw_error) < POINT_TURN_STOP:
                self.point_turn_direction = 0
            elif abs(yaw_error) < KEEP_DIRECTION_ABOVE:
                self.point_turn_direction = 1 if yaw_error > 0 else -1

        if self.point_turn_direction != 0:
            direction_string = "left" if self.point_turn_direction > 0 else "right"
            self.publishStatus(f"Turning {direction_string} toward {target}")
            # Slow down as the error shrinks so lag in the heading/bearing can't
            # carry us through the stop window.
            # Capped at 0.5 rad/s (was 0.8): the coast after stopping and any
            # bearing delay both scale with turn speed.
            omega = float(np.clip(abs(yaw_error), 0.3, 0.5))
            # No forward creep while something is in the stop box (only possible mid-detour).
            self.pointTurnFromYawError(self.point_turn_direction, omega=omega,
                                       linear=0.0 if blocked else 0.2)
            return

        Kp_angular = 1.0
        MAX_DRIVE_ANGULAR = 0.5  # rad/s
        target_angular = float(np.clip(yaw_error * Kp_angular, -MAX_DRIVE_ANGULAR, MAX_DRIVE_ANGULAR))

        SPEED_LIMIT = self.DETOUR_SPEED if self.detour_map is not None else 0.6  # m/s
        # 0.375 = 0.25 * 1.5, what the old velocity-feedback term amounted to
        # (it read ego_linear_vel, which nothing ever updated from 0).
        Kp_linear = 0.375
        target_speed = min(distance_remaining * Kp_linear, SPEED_LIMIT)
        if blocked:
            # Only reachable mid-detour: keep steering toward the waypoint but
            # don't drive into whatever is in the stop box.
            target_speed = 0.0

        cmd_msg = Twist()
        cmd_msg.linear.x = target_speed
        cmd_msg.angular.z = target_angular
        self.twist_pub.publish(self.getSmoothed(cmd_msg))
        self.publishStatus(
            f"Driving {target_speed:.2} m/s toward {target}, {distance_remaining:.2}m away"
        )

    def publishAssistedTwist(self):
        u_0 = self.cached_teleop
        v_0 = u_0.linear.x
        omega_0 = u_0.angular.z

        N = 100  # number of sampled trajectories

        weights = np.ones(N)
        SIGMA_V = 0.3
        SIGMA_OMEGA = 0.3

        v_norm = stats.norm(v_0, SIGMA_V)
        omega_norm = stats.norm(omega_0, SIGMA_OMEGA)
        twist_samples = np.zeros((N, 2))
        twist_samples[:, 0] = v_0 + (np.random.randn(N) * SIGMA_V)
        twist_samples[:, 1] = omega_0 + (np.random.randn(N) * SIGMA_OMEGA)
        # x = np.linspace(norm.ppf(0.01), norm.ppf(0.99), 100)
        # plt.plot(x, norm.pdf(x), "r-", lw=5, alpha=0.6, label="norm pdf")
        plt.show()
        weights *= v_norm.pdf(twist_samples[:, 0])
        weights *= omega_norm.pdf(twist_samples[:, 1])
        plt.scatter(twist_samples[:, 0], twist_samples[:, 1], c=weights)
        plt.colorbar()

        self.twist_pub.publish(u_0)
        self.publishStatus("Driving with automated assistance")

    def updateTrajectory(self):
        if self.current_mode == Mode.STOPPED:
            self.publishStatus("Paused")
            self.twist_pub.publish(self.getSmoothed(Twist()))
            return

        if self.is_turning_downhill:
            self.publishStatus("Turning downhill before planting")
            yaw_error = self.TARGET_DOWNHILL_YAW - self.ego_yaw

            if abs(yaw_error) < 0.2:
                self.facing_downhill_pub.publish(Empty())
                return

            self.pointTurnFromYawError(yaw_error, omega=0.8, linear=0.2)
            return

        if self.is_planting:
            self.publishStatus("Planting a seedling")
            self.twist_pub.publish(self.getSmoothed(Twist()))
            return

        if self.current_mode == Mode.TELEOP:
            self.twist_pub.publish(self.getSmoothed(self.cached_teleop))
            self.publishStatus("Following teleop commands")
            return

        if self.current_mode == Mode.ASSISTED:
            self.publishAssistedTwist()
            return

        if self.total_cost_map is None:
            self.get_logger().warning(
                "Could not plan trajectory. Total cost unavailable."
            )
            self.twist_pub.publish(self.getSmoothed(Twist()))
            return

        if self.ego_yaw is None: #or self.ego_pos is None:
            self.get_logger().warning("Could not plan trajectory. Ego pose unavailable.")
            self.twist_pub.publish(self.getSmoothed(Twist()))
            return
        
        if self.remaining_seedling_count < 1:
            self.publishStatus("No remaining seedlings in plan.")
            self.twist_pub.publish(self.getSmoothed(Twist()))
            return
        
        try:
            bl_to_map_tf = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
            ego_x = bl_to_map_tf.transform.translation.x
            ego_y = bl_to_map_tf.transform.translation.y
            self.ego_pos = (ego_x, ego_y)

            q = bl_to_map_tf.transform.rotation
            r = R.from_quat([q.x, q.y, q.z, q.w])
            self.ego_yaw = r.as_euler("xyz")[2]

        except TransformException as ex:
            self.get_logger().warning(f"Could not get ego position: {ex}")
            return

        if self.closest_point_bl is None:
            self.get_logger().warning(
                "Could not plan trajectory. Closest seedling in base_link unavailable."
            )
            self.twist_pub.publish(self.getSmoothed(Twist()))
            return

        goal_point = self.closest_point_bl
        self.get_logger().info(f"goal point: {goal_point}")

        yaw_error = self.getYawError(goal_point)

        POINT_TURN_YAW_ERROR_THRESHOLD = np.pi / 8
        if abs(yaw_error) > POINT_TURN_YAW_ERROR_THRESHOLD:
            direction_string = "left" if yaw_error > 0 else "right"
            self.publishStatus(f"Turning {direction_string} toward seedling")
            self.pointTurnFromYawError(yaw_error)
            return

        cmd_msg = Twist()
        for speed_level in self.candidates:
            speed, candidates = speed_level

            obstacle_free_candidates = []
            for candidate in candidates:
                omega, mask, trajectory = candidate
                max_cost = np.max(self.total_cost_map[mask])
                if max_cost >= 100:
                    continue

                total_cost = np.sum(self.total_cost_map[mask])
                obstacle_free_candidates.append([speed, omega, total_cost, trajectory])

            if len(obstacle_free_candidates) > 0:
                best_candidate = min(obstacle_free_candidates, key=lambda c: c[2])
                cmd_msg.linear.x = best_candidate[0]
                cmd_msg.angular.z = best_candidate[1]
                self.twist_path_pub.publish(self.trajectoryToPath(best_candidate[3]))
                self.twist_pub.publish(self.getSmoothed(cmd_msg))
                self.publishStatus("Driving toward seedling")
                return

            self.get_logger().debug(f"No obstacle-free candidates at speed {speed}")

        self.twist_pub.publish(self.getSmoothed(Twist()))
        self.publishStatus("Stopped for obstacle.")

        return

    def onGoalPointReached(self):


        if len(self.seedling_points) < 1:
            self.get_logger().warn("All seedlings reached. End.")
            self.goal_point = None
            return
        # Remove the seedling point that we just reached
        self.seedling_points.remove(self.goal_point)

        # Set the goal point to be the next closest seedling point
        min_dist = 9999.9
        for seedling_pt in self.seedling_points:
            seedling_x, seedling_y = seedling_pt

            dist = pdist([self.ego_pos, [seedling_x, seedling_y]])[0]

            if dist < min_dist:
                min_dist = dist
                self.goal_point = [seedling_x, seedling_y]

    def setUpParameters(self):
        param_desc = ParameterDescriptor()
        param_desc.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        self.declare_parameter(
            "map_origin_lat_lon_alt_degrees",
            [40.44132949798969, -79.94451105594635, 293.0],
        )
        # Steady-stream gate before driving after planting or a stale-data hold.
        # Normal grid rate is 10 Hz; ~4 Hz on a loaded Jetson. 1 = old behaviour
        # (resume on the first fresh grid).
        self.declare_parameter("fresh_grids_required", 5)
        self.declare_parameter("fresh_grid_max_gap", 0.7)  # s between consecutive grids
# [40.4431653, -79.9402844, 288.0961589] # steward
        #lat0, lon0, _ = self.get_parameter("map_origin_lat_lon_alt_degrees").value
        #self._map_origin_utm_x, self._map_origin_utm_y, _, __ = utm.from_latlon(lat0, lon0)


def main(args=None):
    rclpy.init(args=args)

    node = PlannerNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()