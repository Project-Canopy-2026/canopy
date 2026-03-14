import numpy as np
import rclpy
from array import array as Array
from rclpy.node import Node, ParameterDescriptor, ParameterType
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from time import time
from tqdm import tqdm, trange
from scipy.spatial.distance import pdist, squareform
from scipy.linalg import norm
from matplotlib import pyplot as plt
from matplotlib import patches
import cv2
from enum import IntEnum
import json
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
import utm
import math
from scipy.spatial.transform.rotation import Rotation as R
from scipy import stats

from skimage.draw import disk


# ROS2 message definitions
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Pose, Point, Twist, PoseStamped, PointStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData, Path
from steward_msgs.msg import (
    FailedChecks,
    HealthCheck,
    SystemwideStatus,
    TrajectoryCandidates,
    TrajectoryCandidate,
    Mode,
    PlantingPlan,
    Seedling,
)
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String, Float32, Bool, Empty


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

        self.create_subscription(String, "planning/plan_json", self.planCb, 1)
        self.create_subscription(Twist, "/cmd_vel/teleop", self.teleopTwistCb, 1)
        self.create_subscription(OccupancyGrid, "/cost/total", self.totalCostCb, 1)
        self.create_subscription(
            GeoPoint, "/planning/goal_pose_geo", self.goalPointGeoCb, 1
        )
        self.create_subscription(Float32, "/gnss/yaw", self.egoYawCb, 1)
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

        # self.generateCandidates()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info("Hello, world!")

        self.goal_point = None
        self.ego_pos = None
        self.ego_yaw = None
        self.seedling_points = []
        self.total_cost_map = None
        self.grid_info = None
        self.cached_teleop = Twist()
        self.current_mode = Mode.STOPPED
        self.is_planting = False
        self.is_turning_downhill = False
        self.closest_point_bl = None
        self.remaining_seedling_count = 0

        self.previous_twist = Twist()

        # self.create_timer(0.1, self.updateTrajectorySimply)
        self.create_timer(0.1, self.updateTrajectory)

    def closestPointBlCb(self, msg: PointStamped):
        self.closest_point_bl = [msg.point.x, msg.point.y]

    def isPlantingCb(self, msg: Bool):
        self.is_planting = msg.data

    def isTurningDownhillCb(self, msg: Bool):
        self.is_turning_downhill = msg.data

    def teleopTwistCb(self, msg: Twist):
        self.cached_teleop = msg

    def currentModeCb(self, msg: Mode):
        self.current_mode = msg.level

    def egoYawCb(self, msg: Float32):
        # self.get_logger().info("Updated ego yaw")
        self.ego_yaw = msg.data

    def goalPointGeoCb(self, msg: GeoPoint):
        lat0, lon0, _ = self.get_parameter("map_origin_lat_lon_alt_degrees").value
        origin_x, origin_y, _, __ = utm.from_latlon(lat0, lon0)
        x, y, _, __ = utm.from_latlon(msg.latitude, msg.longitude)

        x = x - origin_x
        y = y - origin_y
        self.goal_point = [x, y]
        print(msg)

    def totalCostCb(self, msg: OccupancyGrid):
        arr = np.asarray(msg.data).reshape(msg.info.height, msg.info.width)

        self.total_cost_map = arr
        self.grid_info = msg.info

    def generateCandidates(self, top_speed=1.5, time_horizon=5.0, dt=0.5):

        # Start with top speed
        trajectories = []
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
                trajectories.append(second_trajectory)
                candidate = Candidate(v, omega, second_trajectory)
                mask = self.getCandidateMask(candidate)
                candidates_at_speed.append([omega, mask])
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
                trajectories.append(second_trajectory)
                candidate = Candidate(v, omega, second_trajectory)
                mask = self.getCandidateMask(candidate)
                candidates_at_speed.append([omega, mask])
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
                trajectories.append(second_trajectory)
                candidate = Candidate(v, omega, second_trajectory)
                mask = self.getCandidateMask(candidate)
                candidates_at_speed.append([omega, mask])
                mega_mask = np.logical_or(mega_mask, mask)

        # plt.imshow(mega_mask, extent=[-8, 12, -10, 10])
        # plt.show()

        candidates.append([v, candidates_at_speed])

        # plt.figure()
        # plt.gca().set_aspect("equal")

        candidates_msg = TrajectoryCandidates()

        index = 0

        for v, candidates_at_speed in candidates:

            for omega, mask in candidates_at_speed:
                candidate_msg = TrajectoryCandidate()
                candidate_msg.omega = omega
                candidate_msg.speed = v
                trajectory = trajectories[index]

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

                print(v, omega, trajectory)

                index += 1

        self.candidates_msg = candidates_msg

        # self.candidates_pub.publish(candidates_msg)

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

        print(candidate.speed, candidate.omega)
        for pixel_coord in grid_coords:
            print(pixel_coord)
            rr, cc = disk(pixel_coord, collision_radius_px, shape=(100, 100))
            # display_img[cc, rr] = 50
            mask[cc, rr] = True

        return mask

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
        # if self.seedling_points is None or len(self.seedling_points) < 1:
        #     self.get_logger().warning(
        #         f"Could not find closest seedling point. Seedling points unknown."
        #     )
        #     return None

        # closest_distance = 999999.9
        # for seedling_pt in self.seedling_points:
        #     seedling_x, seedling_y = seedling_pt

        #     dist = pdist([self.ego_pos, [seedling_x, seedling_y]])[0]

        #     if dist < closest_distance:
        #         closest_distance = dist
        #         closest_seedling = seedling_pt

        # closest_seedling_bl = self.transformToBaselink([closest_seedling])
        # print(self.ego_yaw, closest_seedling_bl)

        if self.total_cost_map is None:
            self.get_logger().warning(
                f"Could not find closest seedling point. Seedling points unknown."
            )
            return None

        x = self.total_cost_map
        pixel_coords = np.unravel_index(x.argmin(), x.shape)
        return [pixel_coords[1] - 40, pixel_coords[0] - 50]

    def planCb(self, msg: PlantingPlan):
        print(f"Got plan with {len(msg.seedlings)} seedlings")
        self.remaining_seedling_count = len(msg.seedlings)
        # for seedling in msg.seedlings:
        #     seedling: Seedling
        #     print(seedling.species_id)

    def getYawError(self, goal_point_bl):
        yaw_error = math.atan2(goal_point_bl[0], goal_point_bl[1]) - np.pi / 2
        yaw_error *= -1

        if yaw_error < -np.pi:
            yaw_error += 2 * np.pi
        if yaw_error > np.pi:
            yaw_error -= 2 * np.pi
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

        # if self.candidates_msg is not None:
        #     self.candidates_pub.publish(self.candidates_msg)
        # else:
        #     self.get_logger().warning("No candidates message available.")

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

            self.pointTurnFromYawError(yaw_error, omega=1.2, linear=0.8)
            return

        if self.is_planting:
            self.publishStatus("Planting a seedling")
            self.twist_pub.publish(self.getSmoothed(Twist()))
            return

        if self.current_mode == Mode.TELEOP:
            self.twist_pub.publish(self.getSmoothed(self.cached_teleop))
            self.publishStatus("Following teleop commands")
            return

        elif self.current_mode == Mode.ASSISTED:
            self.get_logger().error("Assisted teleop is not yet supported!")
            return

        if self.closest_point_bl is None:
            self.get_logger().error("Seedling waypoint unknown. Stopping.")
            self.twist_pub.publish(self.getSmoothed(Twist()))
            return

        if self.remaining_seedling_count < 1:
            # self.get_logger().error("Seedling waypoint unknown. Stopping.")
            self.publishStatus("Plan complete.")
            self.twist_pub.publish(self.getSmoothed(Twist()))
            return

        # Check yaw error. If |yaw err| > pi/4 (45 deg), point turn.
        goal_point = self.closest_point_bl

        distance_remaining = np.linalg.norm(goal_point)
        # print(goal_point)
        if goal_point is None:
            self.get_logger().warning(
                "Could not get goal point in base_link. Skipping trajectory generation."
            )
            return

        yaw_error = self.getYawError(goal_point)

        # print(f"Yaw err: {yaw_error:.1f}, dist {distance_remaining:.1f}")

        POINT_TURN_YAW_ERROR_THRESHOLD = np.pi / 8  # 22.5 degrees
        if abs(yaw_error) > POINT_TURN_YAW_ERROR_THRESHOLD:

            direction_string = "left" if yaw_error > 0 else "right"
            self.publishStatus(f"Turning {direction_string} toward seedling")

            self.pointTurnFromYawError(yaw_error, omega=1.2, linear=0.8)
            return

        Kp_linear = 0.25
        Kp_angular = 1.0
        target_speed = distance_remaining * Kp_linear
        target_angular = yaw_error * Kp_angular
        SPEED_LIMIT = 0.6  # m/s
        target_speed = min(target_speed, SPEED_LIMIT)

        # cmd_msg = Twist()
        # self.twist_pub.publish(cmd_msg)
        # return

        # print(self.candidates)
        # exit()
        cmd_msg = Twist()
        cmd_msg.linear.x = target_speed
        cmd_msg.angular.z = target_angular
        self.twist_pub.publish(self.getSmoothed(cmd_msg))
        self.publishStatus(
            f"Driving {target_speed:.2} m/s, {distance_remaining:.2}m away"
        )

        return

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

        # if self.candidates_msg is not None:
        #     self.candidates_pub.publish(self.candidates_msg)
        # else:
        #     self.get_logger().warning("No candidates message available.")

        if self.current_mode == Mode.STOPPED:
            self.publishStatus("Paused")
            self.twist_pub.publish(Twist())
            return

        # if self.is_planting:
        #     self.publishStatus("Planting a seedling")
        #     self.twist_pub.publish(Twist())
        #     return

        if self.current_mode == Mode.TELEOP:
            self.twist_pub.publish(self.cached_teleop)
            self.publishStatus("Following teleop commands")
            return

        elif self.current_mode == Mode.ASSISTED:
            # self.get_logger().error("Assisted teleop is not yet supported!")
            self.publishAssistedTwist()
            return

        if self.total_cost_map is None:
            self.get_logger().warning(
                "Could not plan trajectory. Total cost unavailable."
            )
            return
        elif self.ego_yaw is None:
            self.get_logger().warning("Could not plan trajectory. Ego yaw unavailable.")
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

        # Check yaw error. If |yaw err| > pi/4 (45 deg), point turn.
        goal_point = self.getClosestSeedlingInBaselink()
        print(goal_point)
        if goal_point is None:
            self.get_logger().warning(
                "Could not get goal point in base_link. Skipping trajectory generation."
            )
            return

        yaw_error = self.getYawError(goal_point)

        POINT_TURN_YAW_ERROR_THRESHOLD = np.pi / 8
        if abs(yaw_error) > POINT_TURN_YAW_ERROR_THRESHOLD:

            direction_string = "left" if yaw_error > 0 else "right"
            self.publishStatus(f"Turning {direction_string} toward seedling")

            self.pointTurnFromYawError(yaw_error)
            return

        # cmd_msg = Twist()
        # self.twist_pub.publish(cmd_msg)
        # return

        # print(self.candidates)
        # exit()
        print(time())
        cmd_msg = Twist()
        for speed_level in self.candidates:
            speed, candidates = speed_level

            obstacle_free_candidates = []
            for candidate in candidates:
                # print(candidate)
                omega, mask = candidate
                # print(omega, mask)

                max_cost = np.max(self.total_cost_map[mask])
                # print(max_cost)
                # if speed > 0.49 and omega > 0.29:
                #     plt.imshow(mask)
                #     plt.title(f"{max_cost}")
                #     plt.show()

                if max_cost >= 100:
                    print(f"Obst [{speed}, {omega}] had obstacles")
                    continue  # Skip to next if there's an obstacle
                else:
                    total_cost = np.sum(self.total_cost_map[mask])
                    obstacle_free_candidates.append([speed, omega, total_cost])

            if len(obstacle_free_candidates) > 0:
                print(f"RESULTS: {obstacle_free_candidates}")

                # Select the result with the lowest cost
                best_candidate = None
                best_candidate_cost = 999999

                for candidate in obstacle_free_candidates:
                    if candidate[2] < best_candidate_cost:
                        best_candidate_cost = candidate[2]
                        best_candidate = candidate

                cmd_msg.linear.x = best_candidate[0]
                cmd_msg.angular.z = best_candidate[1]
                self.twist_pub.publish(cmd_msg)
                self.publishStatus(f"Driving toward seedling")
                return
            else:
                print(f"No obstacle-free candidates found at speed {speed}")

        self.twist_pub.publish(cmd_msg)
        self.publishStatus(f"Stopped for obstacle.")

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
            [40.4431653, -79.9402844, 288.0961589],
        )


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
