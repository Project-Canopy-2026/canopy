#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToBaseTfPublisher(Node):
    def __init__(self):
        super().__init__("odom_to_base_tf_publisher")

        self.parent_frame = "odom"
        self.child_frame = "base_link"

        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            Odometry,
            "/odometry/filtered",
            self.odom_cb,
            50
        )

        self.get_logger().info(
            f"Publishing TF {self.parent_frame} -> {self.child_frame} from /odometry/filtered"
        )

    def odom_cb(self, msg: Odometry):
        tf_msg = TransformStamped()

        # Use the odometry message timestamp so TF matches the bag time
        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = self.parent_frame
        tf_msg.child_frame_id = self.child_frame

        tf_msg.transform.translation.x = msg.pose.pose.position.x
        tf_msg.transform.translation.y = msg.pose.pose.position.y
        tf_msg.transform.translation.z = msg.pose.pose.position.z

        tf_msg.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToBaseTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
