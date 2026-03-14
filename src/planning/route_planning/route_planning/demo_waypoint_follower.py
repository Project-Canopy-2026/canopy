import numpy as np
import rclpy
from builtin_interfaces.msg import Time as RosTime
from route_planning.ant_colony import AntColony
from rclpy.node import Node, ParameterDescriptor, ParameterType
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from steward_msgs.msg import Route, ForestPlan
from geometry_msgs.msg import Point, TransformStamped
from time import time
from tqdm import tqdm, trange
from scipy.spatial.distance import pdist, squareform
from matplotlib import pyplot as plt
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math

# ROS message definitions
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32


def deg2rad(deg):
    return deg * (math.pi / 180)


def getDistanceFromLatLon(lat1, lon1, lat2, lon2):
    R = 6371  # Radius of the earth in km
    dLat = deg2rad(lat2 - lat1)  # deg2rad below
    dLon = deg2rad(lon2 - lon1)
    a = math.sin(dLat / 2) * math.sin(dLat / 2) + math.cos(deg2rad(lat1)) * math.cos(
        deg2rad(lat2)
    ) * math.sin(dLon / 2) * math.sin(dLon / 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    d = R * c * 1e3
    # Distance in m
    return d


class DemoWaypointFollower(Node):
    def __init__(self):
        super().__init__("demo_waypoint_follower")

        self.setUpParameters()

        self.waypoint_sub = self.create_subscription(
            PoseStamped, "/planning/goal_pose_geo", self.waypointCb, 1
        )

        self.fix_sub = self.create_subscription(NavSatFix, "/gnss/fix", self.fixCb, 1)
        self.yaw_sub = self.create_subscription(Float32, "/gnss/yaw", self.yawCb, 1)

        self.twist_pub = self.create_publisher(Twist, "/cmd_vel", 1)

        self.cached_waypoint = None
        self.ego_pos = None
        self.ego_yaw = None
        self.origin = [40.4431653, -79.9402844, 288.0961589]

        self.create_timer(0.1, self.tickController)

    def yawCb(self, msg):
        self.ego_yaw = msg.data

    def convertToMap(self, lat, lon):
        origin_lat = self.origin[0]
        origin_lon = self.origin[1]
        dX = getDistanceFromLatLon(origin_lat, origin_lon, origin_lat, lon)
        if lon < origin_lon:
            dX *= -1

        dY = getDistanceFromLatLon(origin_lat, origin_lon, lat, origin_lon)

        if lat < origin_lat:
            dY *= -1

        # self.get_logger().info(f"{dX}, {dY}")

        return [dX, dY]

    def fixCb(self, msg: NavSatFix):
        # Convert lat lon into meters from origin

        self.ego_pos = self.convertToMap(msg.latitude, msg.longitude)

    def waypointCb(self, msg: PoseStamped):
        self.cached_waypoint = self.convertToMap(
            msg.pose.position.y, msg.pose.position.x
        )
        # self.get_logger().info(
        #     f"{msg.pose.position.x}, {msg.pose.position.y} -> {self.cached_waypoint}"
        # )

    def tickController(self):
        if self.ego_pos is None or self.cached_waypoint is None or self.ego_yaw is None:
            return

        target_yaw = math.atan2(
            self.cached_waypoint[1] - self.ego_pos[1],
            self.cached_waypoint[0] - self.ego_pos[0],
        )

        dist_err = pdist([self.cached_waypoint, self.ego_pos])
        yaw_err = target_yaw - self.ego_yaw

        while yaw_err < -math.pi:
            yaw_err += 2 * math.pi

        while yaw_err > math.pi:
            yaw_err -= 2 * math.pi

        # Construct a Twist message to send
        msg = Twist()

        yaw_kP = 0.7
        msg.angular.z = yaw_err * yaw_kP

        lin_kP = 0.5

        if abs(yaw_err) < math.pi / 4:
            self.get_logger().info("Driving forward!")
            msg.linear.x = min(float(dist_err), 0.5)

        else:
            msg.linear.x = 0.0

        if dist_err < 1.0:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.twist_pub.publish(msg)
        self.get_logger().info(
            f"Lin err: {dist_err}, yaw err: {yaw_err}. CMD: {msg.linear.x}, {msg.angular.z}"
        )

    def setUpParameters(self):
        pass


def main(args=None):
    rclpy.init(args=args)

    node = DemoWaypointFollower()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
