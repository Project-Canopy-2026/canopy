#!/usr/bin/env python3
"""Relay the Warthog's namespaced TF onto the global /tf and /tf_static.

The Warthog publishes TF (odom -> base_link, its URDF) on /<ns>/tf and
/<ns>/tf_static. Our nodes use the default /tf topics, so without this relay
they see an empty tree. The relay is one-way; our own transforms (map -> odom,
sensor mounts) stay on /tf and never go back to the robot.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


class TfRelay(Node):
    def __init__(self):
        super().__init__("tf_relay")

        ns = self.declare_parameter("robot_namespace", "w200_0120").value.strip("/")

        tf_qos = QoSProfile(
            depth=100,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        # Latched so nodes started later still receive the static transforms.
        static_qos = QoSProfile(
            depth=100,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        tf_pub = self.create_publisher(TFMessage, "/tf", tf_qos)
        static_pub = self.create_publisher(TFMessage, "/tf_static", static_qos)

        self.create_subscription(TFMessage, f"/{ns}/tf", tf_pub.publish, tf_qos)
        self.create_subscription(TFMessage, f"/{ns}/tf_static", static_pub.publish, static_qos)

        self.get_logger().info(f"Relaying /{ns}/tf -> /tf and /{ns}/tf_static -> /tf_static")


def main(args=None):
    rclpy.init(args=args)
    node = TfRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
