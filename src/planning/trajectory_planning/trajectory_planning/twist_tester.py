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
from std_msgs.msg import Header, String, Float32, Bool


class Candidate:
    def __init__(
        self, speed: float, omega: float, trajectory: np.ndarray, cost: float = -1
    ):
        self.speed = speed
        self.omega = omega
        self.trajectory = trajectory
        self.cost = cost


class TwistTesterNode(Node):
    def __init__(self):
        super().__init__("twist_tester")

        self.setUpParameters()

        self.get_logger().info("Hello, world!")

        self.twist_pub = self.create_publisher(Twist, "/cmd_vel", 1)

        self.start_time = time()
        self.twist_duration = 5  # sec
        self.angular_vel = -np.pi * 2 / self.twist_duration

        self.create_timer(0.01, self.publishTestTwist)

    def publishTestTwist(self):
        twist_msg = Twist()
        twist_msg.angular.z = self.angular_vel
        self.twist_pub.publish(twist_msg)

    def setUpParameters(self):
        pass


def main(args=None):
    rclpy.init(args=args)

    node = TwistTesterNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
