import numpy as np
import rclpy
from rclpy.node import Node, ParameterDescriptor, ParameterType

import cv2
import math
from scipy.spatial.distance import pdist
from scipy.spatial.transform.rotation import Rotation as R
import utm

# ROS2 message definitions
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from std_msgs.msg import Empty, Float32
from canopy_msgs.msg import PlantingPlan, Seedling


class CostMapNode(Node):
    def __init__(self):
        super().__init__("cost_map_node")

        self.setUpParameters()

        self.create_subscription(
            PlantingPlan, "/planning/remaining_plan", self.planCb, 1
        )
        self.create_subscription(OccupancyGrid, "/cost/occupancy", self.occCb, 1)
        self.create_subscription(Odometry, "/odometry/gps", self.odomCb, 1)
        self.create_subscription(
            Empty, "/behavior/on_seedling_reached", self.onSeedlingReached, 1
        )

        self.seedling_dist_map_pub = self.create_publisher(
            OccupancyGrid, "/cost/dist_to_seedlings", 1
        )
        self.closest_seedling_point_pub = self.create_publisher(
            PointStamped, "/planning/closest_seedling_bl", 1
        )
        self.total_cost_pub = self.create_publisher(OccupancyGrid, "/cost/total", 1)
        self.distance_to_seedling_pub = self.create_publisher(
            Float32, "/planning/distance_to_seedling", 1
        )
        self.status_pub = self.create_publisher(DiagnosticStatus, "/diagnostics", 1)

        self.create_timer(0.1, self.updateCosts)

        self.seedling_points = []
        self.seedling_pts_bl = []
        self.cached_occ = np.zeros((100, 100))
        self.ego_pos = None
        self.ego_yaw = None

    def onSeedlingReached(self, msg: Empty):
        self.get_logger().info("Seedling reached")

    def occCb(self, msg: OccupancyGrid):
        arr = np.asarray(msg.data).reshape(msg.info.height, msg.info.width)
        self.cached_occ = arr

    def odomCb(self, msg: Odometry):
        pos = msg.pose.pose.position
        self.ego_pos = (pos.x, pos.y)

        q = msg.pose.pose.orientation
        r = R.from_quat([q.x, q.y, q.z, q.w])
        self.ego_yaw = r.as_euler("xyz")[2]

    def latLonToMap(self, lat: float, lon: float):
        lat0, lon0, _ = self.get_parameter("map_origin_lat_lon_alt_degrees").value
        origin_x, origin_y, _, __ = utm.from_latlon(lat0, lon0)

        x, y, _, __ = utm.from_latlon(lat, lon)

        x = x - origin_x
        y = y - origin_y

        return (x, y)

    def getDistanceToSeedlingMap(self) -> np.ndarray:
        if self.seedling_points is None or len(self.seedling_points) < 1:
            return np.ones((100, 100)) * 100

        if self.ego_pos is None or self.ego_yaw is None:
            self.get_logger().warning(
                "Could not get ego position from /odometry/gps yet."
            )
            return np.ones((100, 100)) * 100

        closest_distance = 99999.9
        closest_seedling = None

        for seedling_pt in self.seedling_points:
            seedling_x, seedling_y = seedling_pt

            dist = pdist([self.ego_pos, [seedling_x, seedling_y]])[0]

            if dist < closest_distance:
                closest_distance = dist
                closest_seedling = seedling_pt.copy()

        if closest_seedling is None:
            return np.ones((100, 100)) * 100

        self.distance_to_seedling_pub.publish(Float32(data=float(closest_distance)))

        RES = 0.2  # meters per pixel
        ORIGIN_X_PX = 40
        ORIGIN_X_M = ORIGIN_X_PX * RES
        ORIGIN_Y_PX = 50
        ORIGIN_Y_M = ORIGIN_Y_PX * RES
        GRID_WIDTH = 100
        GRID_HEIGHT = GRID_WIDTH

        DOWNSAMPLE_RATE = 3

        distance_to_seedling_map = np.zeros(
            (GRID_HEIGHT // DOWNSAMPLE_RATE, GRID_WIDTH // DOWNSAMPLE_RATE)
        )

        closest_seedling_bl = self.transformToBaselink([closest_seedling])

        if closest_seedling_bl is None:
            return np.ones((100, 100)) * 100

        closest_seedling_bl = closest_seedling_bl[0]

        closest_point_msg = PointStamped()
        closest_point_msg.header.stamp = self.get_clock().now().to_msg()
        closest_point_msg.header.frame_id = "base_link"
        closest_point_msg.point.x = float(closest_seedling_bl[0])
        closest_point_msg.point.y = float(closest_seedling_bl[1])
        self.closest_seedling_point_pub.publish(closest_point_msg)

        closest_seedling_px = closest_seedling_bl / (RES * DOWNSAMPLE_RATE)
        closest_seedling_px[0] += int(ORIGIN_X_PX / DOWNSAMPLE_RATE)
        closest_seedling_px[1] += int(ORIGIN_Y_PX / DOWNSAMPLE_RATE)
        closest_seedling_px = closest_seedling_px.astype(int)

        for i in range(distance_to_seedling_map.shape[0]):
            for j in range(distance_to_seedling_map.shape[1]):
                dist = pdist([[i, j], closest_seedling_px])
                distance_to_seedling_map[i, j] = dist

        distance_to_seedling_map = cv2.resize(
            distance_to_seedling_map, (GRID_HEIGHT, GRID_WIDTH)
        )

        MAX_DISTANCE_COST = 100

        distance_to_seedling_map = np.power(distance_to_seedling_map, 1 / 2)
        distance_to_seedling_map /= np.max(distance_to_seedling_map)
        distance_to_seedling_map *= MAX_DISTANCE_COST
        distance_to_seedling_map = (
            np.ones_like(distance_to_seedling_map) * MAX_DISTANCE_COST
            - distance_to_seedling_map
        )
        distance_to_seedling_map = distance_to_seedling_map.astype(np.uint8)

        # FLIP AXES
        distance_to_seedling_map = distance_to_seedling_map.T

        return distance_to_seedling_map.astype(np.uint8)

    def updateCosts(self, do_plot=False):
        RES = 0.2  # meters per pixel
        ORIGIN_X_PX = 40
        ORIGIN_X_M = ORIGIN_X_PX * RES
        ORIGIN_Y_PX = 50
        ORIGIN_Y_M = ORIGIN_Y_PX * RES
        GRID_WIDTH = 100
        GRID_HEIGHT = GRID_WIDTH

        distance_to_seedling_map = self.getDistanceToSeedlingMap()

        # Great distances mean higher cost, not lower cost.
        closeness_to_seedling_map = (
            np.ones_like(distance_to_seedling_map) * 100 - distance_to_seedling_map
        )

        total_cost_map = closeness_to_seedling_map + self.cached_occ
        total_cost_map[total_cost_map > 100] = 100

        if np.min(total_cost_map) < 0:
            self.get_logger().error("Total cost had elements less than 0. Correcting.")
            total_cost_map[total_cost_map < 0] = 0

        origin = Point(x=-ORIGIN_X_M, y=-ORIGIN_Y_M)

        info = MapMetaData(resolution=RES, width=GRID_WIDTH, height=GRID_HEIGHT)
        info.origin.position = origin
        msg = OccupancyGrid()

        try:
            msg.data = distance_to_seedling_map.astype(np.uint8).flatten().tolist()
        except AssertionError as e:
            self.get_logger().warning(
                f"{e}. Min was {np.min(distance_to_seedling_map)}, max was {np.max(distance_to_seedling_map)}, dtype was {distance_to_seedling_map.dtype}"
            )
        msg.info = info

        msg.header.frame_id = "base_link"
        msg.header.stamp = self.get_clock().now().to_msg()

        self.seedling_dist_map_pub.publish(msg)

        try:
            msg.data = total_cost_map.astype(np.uint8).flatten().tolist()
        except AssertionError as e:
            self.get_logger().warning(
                f"{e}. Min was {np.min(total_cost_map)}, max was {np.max(total_cost_map)}"
            )

        self.total_cost_pub.publish(msg)

    def transformToBaselink(self, points):
        if len(points) < 1 or self.ego_pos is None or self.ego_yaw is None:
            return None

        # Form a 2D homogeneous transform matrix
        t = -self.ego_yaw
        u = -self.ego_pos[0]
        v = -self.ego_pos[1]

        H = np.identity(3)
        H[0, 2] = u
        H[1, 2] = v

        R_mat = np.asarray(
            [
                [math.cos(t), -math.sin(t)],
                [math.sin(t), math.cos(t)],
            ]
        )

        points = np.asarray(points)

        try:
            pts_homog = np.vstack((points.T, np.ones(len(points))))
            pts_tfed = H @ pts_homog
            pts_tfed = pts_tfed.T
            pts_tfed = pts_tfed[:, :-1]
        except ValueError:
            return None

        pts_tfed = (R_mat @ pts_tfed.T).T
        return pts_tfed

    def planCb(self, msg: PlantingPlan):
        self.seedling_points.clear()

        lat, lon, alt = self.get_parameter("map_origin_lat_lon_alt_degrees").value
        origin_x, origin_y, _, __ = utm.from_latlon(lat, lon)

        for seedling in msg.seedlings:
            seedling: Seedling
            seedling_x, seedling_y, _, __ = utm.from_latlon(
                seedling.latitude, seedling.longitude
            )

            seedling_x = seedling_x - origin_x
            seedling_y = seedling_y - origin_y

            self.seedling_points.append([seedling_x, seedling_y])

    def setUpParameters(self):
        param_desc = ParameterDescriptor()
        param_desc.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        self.declare_parameter(
            "map_origin_lat_lon_alt_degrees",
            [40.44132949798969, -79.94451105594635, 293.0],
        )


def main(args=None):
    rclpy.init(args=args)

    node = CostMapNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()