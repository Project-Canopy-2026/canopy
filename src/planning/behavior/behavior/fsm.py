import numpy as np
import rclpy
from rclpy.node import Node, ParameterDescriptor, ParameterType
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
import json
from time import time
from tqdm import tqdm, trange
from scipy.spatial.distance import pdist, squareform
from matplotlib import pyplot as plt
import cv2
from enum import IntEnum
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
import utm


# ROS2 message definitions
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Header
from steward_msgs.msg import FailedChecks, HealthCheck, SystemwideStatus, Mode
from std_msgs.msg import Empty, String, Bool

# class SystemwideStatus:
#     HEALTHY = 0
#     WARN = 1
#     TELEOP_ONLY = 2
#     OOS = 3


class FsmNode(Node):
    def __init__(self):
        super().__init__("behavior_fsm")

        self.setUpParameters()

        self.get_logger().info("Hello, world.")

        self.create_subscription(
            Mode, "/planning/requested_mode", self.requestedModeCb, 1
        )
        # self.create_subscription(String, "/planning/plan_json", self.planCb, 1)

        self.current_mode_pub = self.create_publisher(Mode, "/planning/current_mode", 1)
        self.planting_locked_pub = self.create_publisher(
            Bool, "/behavior/is_planting", 1
        )
        self.turning_downhill_pub = self.create_publisher(
            Bool, "/behavior/is_turning_downhill", 1
        )
        self.status_pub = self.create_publisher(DiagnosticStatus, "/diagnostics", 1)
        self.do_plant_pub = self.create_publisher(Empty, "/behavior/do_plant", 1)

        self.create_subscription(
            Empty, "/behavior/on_seedling_reached", self.onSeedlingReachedCb, 1
        )

        self.create_subscription(
            Empty, "/behavior/facing_downhill", self.onFacingDownhillCb, 1
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.current_mode = Mode.STOPPED
        self.seedling_points = []

        self.create_timer(0.1, self.publishCurrentMode)
        self.seedling_reached_distance = 1.0  # meters
        self.is_planting = False
        self.is_turning_downhill = False
        self.PLANTING_DURATION = 10  # seconds
        self.planting_start_time = time()

    def onSeedlingReachedCb(self, msg: Empty):
        self.get_logger().info("Seedling reached!")

        self.is_turning_downhill = True

    def onFacingDownhillCb(self, msg: Empty):
        self.is_turning_downhill = False
        self.is_planting = True
        self.do_plant_pub.publish(Empty())
        self.planting_start_time = time()

    def publishStatus(self, desc: str, level=DiagnosticStatus.OK):
        self.status_pub.publish(
            DiagnosticStatus(message=desc, level=level, name=self.get_name())
        )

    def publishCurrentMode(self):
        self.current_mode_pub.publish(Mode(level=self.current_mode))

        self.publishStatus(f"Setting current mode to {self.current_mode}")

        if time() - self.planting_start_time > self.PLANTING_DURATION:
            self.is_planting = False

        self.planting_locked_pub.publish(Bool(data=self.is_planting))
        self.turning_downhill_pub.publish(Bool(data=self.is_turning_downhill))

    def requestedModeCb(self, msg: Mode):
        self.get_logger().info(f"Current mode is now {msg.level}")

        self.current_mode = msg.level

    def setUpParameters(self):
        param_desc = ParameterDescriptor()
        param_desc.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        self.declare_parameter(
            "map_origin_lat_lon_alt_degrees",
            [40.4431653, -79.9402844, 288.0961589],
        )


def main(args=None):
    rclpy.init(args=args)

    node = FsmNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
