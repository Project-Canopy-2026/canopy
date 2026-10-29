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
from scipy.ndimage import distance_transform_edt
from skimage.graph import route_through_array

# ROS2 message definitions
# Subscribes to /odometry/gps from robot_localization, nav_msgs/Odometry
# https://docs.ros.org/en/melodic/api/robot_localization/html/state_estimation_nodes.html
from diagnostic_msgs.msg import DiagnosticStatus
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from canopy_msgs.msg import (
    TrajectoryCandidates,
    TrajectoryCandidate,
    Mode,
    PlantingPlan,
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
        self.remaining_seedling_count = 0
        self.heading_valid = False
        self.estopped = False
        self.point_turn_direction = 0  # +1 left, -1 right, 0 not point turning

        self.occ_map = None
        self.occ_info = None
        self.occ_stamp = None  # header stamp of the latest grid

        # The lidar pipeline is stopped while planting, so after planting the last grid
        # is stale. Don't drive until a grid stamped after planting ended arrives.
        self.awaiting_fresh_occ = False
        self.planting_end_time = None

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
        self.DETOUR_WP_SPACING = 2.0   # m between waypoints kept from the planned path
        self.DETOUR_WP_REACHED = 0.7   # m; closer than this counts as reached
        self.DETOUR_SPEED = 0.3        # m/s cap while detouring
        self.DETOUR_ALIGNED = np.radians(20)  # facing the waypoint this well + box blocked = path blocked

        # Detour state. detour_map is None when not detouring.
        self.detour_map = None         # Nx2 waypoints in the map frame
        self.detour_idx = 0            # index of the waypoint we're driving to
        self.obstacle_stop_time = None # when we stopped for the current obstacle
        self.last_planned_stamp = None # grid stamp we last planned on (plan once per grid)
        self.stop_reason = ""          # shown in the status while stopped for an obstacle

        self.previous_twist = Twist()

        self.create_timer(0.1, self.updateTrajectorySimply)
        # self.create_timer(0.1, self.updateTrajectory)

    def closestPointBlCb(self, msg: PointStamped):
        self.closest_point_bl = [msg.point.x, msg.point.y]
        self.closest_point_bl_time = self.get_clock().now()

    def isPlantingCb(self, msg: Bool):
        if msg.data and not self.is_planting:
            self.clearDetour()  # seedling reached; any detour belonged to it
        if self.is_planting and not msg.data:
            self.awaiting_fresh_occ = True
            self.planting_end_time = self.get_clock().now()
        self.is_planting = msg.data

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

        if self.awaiting_fresh_occ and Time.from_msg(msg.header.stamp) > self.planting_end_time:
            self.awaiting_fresh_occ = False
            waited = (self.get_clock().now() - self.planting_end_time).nanoseconds / 1e9
            self.get_logger().info(f"Fresh obstacle data {waited:.1f} s after planting; resuming")

    def isObstacleAhead(self, max_x=None) -> bool:
        """True if enough occupied cells sit in a box directly ahead of the robot.
        max_x shortens the box (used mid-detour to check only up to the next waypoint)."""
        if self.occ_map is None or self.occ_info is None:
            return False

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

        patch = self.occ_map[row_lo:row_hi, col_lo:col_hi]
        return int(np.count_nonzero(patch >= 50)) >= self.MIN_OBSTACLE_CELLS

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
        """The waypoint to drive to, in base_link. Skips waypoints that are reached
        or already passed. None once the last one is reached."""
        robot = np.array(pose[:2])
        while self.detour_idx < len(self.detour_map):
            wp = self.detour_map[self.detour_idx]
            nxt = self.detour_idx + 1
            reached = np.linalg.norm(wp - robot) < self.DETOUR_WP_REACHED
            # Passed: we're already closer to the next waypoint than this one is.
            # Stops GPS jitter or a wide turn from sending us back for a missed point.
            passed = nxt < len(self.detour_map) and (
                np.linalg.norm(self.detour_map[nxt] - robot) < np.linalg.norm(self.detour_map[nxt] - wp)
            )
            if not (reached or passed):
                return self.mapToBl(wp[None, :], pose)[0]
            self.detour_idx += 1
        return None

    def stopAndPlanDetour(self):
        """Obstacle ahead and no detour yet: stop, wait for a fresh grid, plan once per grid."""
        self.twist_pub.publish(self.getSmoothed(Twist()))
        self.point_turn_direction = 0

        now = self.get_clock().now()
        if self.obstacle_stop_time is None:
            self.obstacle_stop_time = now
            self.stop_reason = "obstacle ahead, waiting for fresh scan"
        waited = (now - self.obstacle_stop_time).nanoseconds * 1e-9
        status = f"Stopped ({waited:.0f} s): {self.stop_reason}"

        # Only plan on a grid built after we'd stopped, and only once per grid.
        settled = self.obstacle_stop_time + Duration(seconds=self.DETOUR_SETTLE_TIME)
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

        self.detour_map = self.blToMap(self.thinPath(path_bl), pose)
        self.detour_idx = 0
        self.obstacle_stop_time = None
        self.publishDetourPath()
        self.get_logger().info(
            f"Detour planned: {len(self.detour_map)} waypoints, {waited:.1f} s after stopping")

    def clearDetour(self):
        if self.detour_map is not None:
            self.detour_map = None
            self.publishDetourPath()  # empty path clears it in Foxglove
        self.point_turn_direction = 0

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
            self.publishStatus("Waiting for fresh obstacle data after planting", DiagnosticStatus.WARN)
            return

        # The bearing comes from cost_map_node via TF. If it stops arriving, stop
        # rather than keep turning on an old value.
        bearing_age = (self.get_clock().now() - self.closest_point_bl_time).nanoseconds * 1e-9
        if bearing_age > 0.5:
            self.publishStatus(f"Seedling bearing is {bearing_age:.1f}s old. Holding.",
                               level=DiagnosticStatus.WARN)
            self.twist_pub.publish(self.getSmoothed(Twist()))
            return

        # Obstacle handling: stop, plan a detour once, then follow its waypoints
        # with the same steering code below. The goal is either the seedling or
        # the current detour waypoint.
        if self.detour_map is not None and not self.heading_valid:
            self.clearDetour()  # a map-frame path can't be followed without heading

        if self.detour_map is None:
            blocked = self.isObstacleAhead()
            if blocked:
                self.stopAndPlanDetour()
                return
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
            # Only check the stretch we're about to drive: up to the waypoint. The full
            # 3 m box would see the obstacle we're going around and stop the detour.
            blocked = self.isObstacleAhead(max_x=np.linalg.norm(goal_point))
            if blocked and abs(self.getYawError(goal_point)) < self.DETOUR_ALIGNED:
                # Facing along the path and the stop box is still blocked: something
                # new is on the path. Drop it; next tick stops and plans from here.
                self.get_logger().info("Detour path blocked; replanning")
                self.clearDetour()
                self.twist_pub.publish(self.getSmoothed(Twist()))
                return
            target = f"detour waypoint {self.detour_idx + 1}/{len(self.detour_map)}"

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

        # Point turn with hysteresis: start above 45 deg, stop below 20 deg. The
        # direction is only held while the goal is roughly behind us (where the
        # sign of the error flips from cycle to cycle); otherwise turn the short
        # way, so a wrong first choice (e.g. from a stale bearing) corrects itself.
        # Near the goal, SBAS position noise swings the bearing a lot, so only
        # turn in place if the goal is clearly behind us.
        POINT_TURN_START = np.pi / 4
        POINT_TURN_STOP = np.radians(20)
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
            omega = float(np.clip(abs(yaw_error), 0.4, 0.8))
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