# import numpy as np
import rclpy
from rclpy.node import Node, ParameterDescriptor, ParameterType
# from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
# import json
# from time import time
# from tqdm import tqdm, trange
# from scipy.spatial.distance import pdist, squareform
# from matplotlib import pyplot as plt
# import cv2
from enum import IntEnum
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
# import utm


# ROS2 message definitions
from diagnostic_msgs.msg import DiagnosticStatus
# from std_msgs.msg import Header
from canopy_msgs.msg import Mode
from std_msgs.msg import Empty, String, Bool, Int32

# class SystemwideStatus:
#     HEALTHY = 0
#     WARN = 1
#     TELEOP_ONLY = 2
#     OOS = 3


class FsmNode(Node):
    # Planting states that release the navigation lock.
    #
    # DIGGING_OBSTACLE is deliberately NOT one of them. The planting FSM
    # publishes that state the instant it decides to abort - the auger is
    # still in the soil and has only just been sent LINAK,1,STOP - so
    # releasing there lets the rover drive off with the drill in the ground.
    # The abort path continues: STOPPED:LINAK1 -> LINAK,1,HOME -> on its
    # DONE/STALL the auger motor is stopped and the FSM enters IDLE. The
    # success path ends COMPLETE -> IDLE. So IDLE is the one state that means
    # "auger home, nothing turning" on both paths, and it is the only safe
    # point to hand navigation back.
    #
    # FAULT stays in the list because it is terminal - it never reaches IDLE,
    # so nothing else would ever release the lock and the rover would sit at
    # the seedling forever.
    PLANTING_RELEASE_STATES = ("IDLE", "FAULT")

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
        self.do_plant_pub = self.create_publisher(Empty, "/behavior/do_planting", 1)
        self.planted_count_pub = self.create_publisher(
            Int32, "/behavior/planting_success_count", 10
        )

        self.create_subscription(
            Empty, "/behavior/on_seedling_reached", self.onSeedlingReachedCb, 1
        )

        self.create_subscription(
            Empty, "/behavior/facing_downhill", self.onFacingDownhillCb, 1
        )

        self.create_subscription(
            String, "/planting_state", self.plantingStateUpdate, 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.current_mode = Mode.STOPPED
        self.seedling_points = []

        self.create_timer(0.1, self.publishCurrentMode)
        # self.seedling_reached_distance = 1.0  # meters
        self.seedling_reached_distance = 0.8  # meters
        self.is_planting = False
        self.is_turning_downhill = False
        self.planting_success_count = 0
        # Last state seen on /planting_state. IDLE is reached from both
        # COMPLETE and the obstacle abort, so the state BEFORE it is what says
        # which outcome this was.
        self.planting_state = None
        #self.PLANTING_DURATION = 10  # seconds
        #self.planting_start_time = time()

    def onSeedlingReachedCb(self, msg: Empty):
        if self.is_planting:
            return

        self.get_logger().info("Seedling reached!")

        self.is_planting = True

        self.planting_locked_pub.publish(Bool(data=True))
        self.do_plant_pub.publish(Empty())
        #self.planting_start_time = time()
        # self.is_turning_downhill = True

    def onFacingDownhillCb(self, msg: Empty):
        return
        # self.is_turning_downhill = False
        # self.is_planting = True
        # self.do_plant_pub.publish(Empty())
        #self.planting_start_time = time()

    def plantingStateUpdate(self, msg: String):
        # Every terminal state of the planting FSM has to release the nav lock,
        # but only once the hardware is actually parked - see
        # PLANTING_RELEASE_STATES for why that is IDLE and not COMPLETE or
        # DIGGING_OBSTACLE.
        state = msg.data.strip()
        prev, self.planting_state = self.planting_state, state

        if not self.is_planting:
            return

        if state == "DIGGING_OBSTACLE":
            # Holding the lock on purpose: the auger is in the hole and the
            # retract has only just been commanded. The release comes with the
            # IDLE that follows it, a drill-depth retract later.
            self.get_logger().warn(
                "Obstacle at seedling site - drill aborting. Holding "
                "navigation until the auger is back home."
            )
            return

        if state not in self.PLANTING_RELEASE_STATES:
            return

        self.is_planting = False
        self.planting_locked_pub.publish(Bool(data=False))

        if state == "FAULT":
            # FAULT e-stops the Arduino motors but does NOT retract LINAK 1,
            # so unlike IDLE this does not promise the auger is out of the
            # ground. Released anyway, because nothing else ever will be.
            self.get_logger().error(
                "Planting FSM faulted - nothing planted, skipping. "
                "Navigation can resume; planting needs a node restart. "
                "WARNING: a fault does not retract the auger - check it is "
                "clear of the ground before driving."
            )
        elif prev == "COMPLETE":
            self.planting_success_count += 1
            self.get_logger().info(
                f"Planting complete ({self.planting_success_count} planted). "
                "Navigation can resume."
            )
        else:
            # IDLE off the obstacle path (prev == DIGGING_OBSTACLE), or off any
            # other bail-out. Either way the auger is home and nothing was
            # planted here.
            self.get_logger().warn(
                f"Planting ended at IDLE via {prev} - nothing planted, "
                "skipping. Auger is home; navigation can resume."
            )

        self.planted_count_pub.publish(Int32(data=self.planting_success_count))

    def publishStatus(self, desc: str, level=DiagnosticStatus.OK):
        self.status_pub.publish(
            DiagnosticStatus(message=desc, level=level, name=self.get_name())
        )

    def publishCurrentMode(self):
        self.current_mode_pub.publish(Mode(level=self.current_mode))

        self.publishStatus(f"Setting current mode to {self.current_mode}")

        # if time() - self.planting_start_time > self.PLANTING_DURATION:
        #     self.is_planting = False

        self.planting_locked_pub.publish(Bool(data=self.is_planting))
        self.turning_downhill_pub.publish(Bool(data=self.is_turning_downhill))
        self.planted_count_pub.publish(Int32(data=self.planting_success_count))

    def requestedModeCb(self, msg: Mode):
        self.get_logger().info(f"Current mode is now {msg.level}")

        self.current_mode = msg.level

    def setUpParameters(self):
        param_desc = ParameterDescriptor()
        param_desc.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        self.declare_parameter(
            "map_origin_lat_lon_alt_degrees",
            [40.44132949798969, -79.94451105594635, 293.0],
        )
# [40.4431653, -79.9402844, 288.0961589] # steward

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