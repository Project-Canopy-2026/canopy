#!/usr/bin/env python3
"""Quick visualizer for robot trajectory, cmd_vel, and distance to target seedling.

Run (after sourcing the workspace):
    python3 scripts/viz_debug.py

In RViz/Foxglove, set Fixed Frame = "odom" and add displays for:
    /viz/robot_trajectory   (Path)
    /viz/cmd_vel_marker     (MarkerArray)
    /viz/seedling_target    (Marker)
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray


class VizDebug(Node):
    def __init__(self):
        super().__init__("viz_debug")

        self.path = Path()
        self.path.header.frame_id = "odom"

        self.ego_pose: PoseStamped | None = None
        self.last_twist = Twist()
        self.distance_to_seedling: float | None = None
        self.closest_seedling_bl: PointStamped | None = None

        self.create_subscription(Odometry, "/odometry/gps", self.odomCb, 10)
        self.create_subscription(Twist, "/cmd_vel", self.twistCb, 10)
        self.create_subscription(
            Float32, "/planning/distance_to_seedling", self.distanceCb, 10
        )
        self.create_subscription(
            PointStamped, "/planning/closest_seedling_bl", self.seedlingBlCb, 10
        )

        self.path_pub = self.create_publisher(Path, "/viz/robot_trajectory", 10)
        self.cmd_pub = self.create_publisher(MarkerArray, "/viz/cmd_vel_marker", 10)
        self.target_pub = self.create_publisher(Marker, "/viz/seedling_target", 10)

        self.create_timer(0.1, self.publishAll)

        self.get_logger().info("viz_debug running")

    def odomCb(self, msg: Odometry):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.ego_pose = pose

        self.path.poses.append(pose)
        if len(self.path.poses) > 5000:
            self.path.poses = self.path.poses[-5000:]

    def twistCb(self, msg: Twist):
        self.last_twist = msg

    def distanceCb(self, msg: Float32):
        self.distance_to_seedling = msg.data

    def seedlingBlCb(self, msg: PointStamped):
        self.closest_seedling_bl = msg

    def publishAll(self):
        now = self.get_clock().now().to_msg()

        self.path.header.stamp = now
        self.path_pub.publish(self.path)

        if self.ego_pose is not None:
            self.cmd_pub.publish(self.makeCmdMarkers(now))
            if self.closest_seedling_bl is not None:
                self.target_pub.publish(self.makeTargetMarker(now))

    def makeCmdMarkers(self, stamp) -> MarkerArray:
        arr = MarkerArray()

        arrow = Marker()
        arrow.header.frame_id = "base_link"
        arrow.header.stamp = stamp
        arrow.ns = "cmd_vel"
        arrow.id = 0
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.scale.x = max(0.05, abs(self.last_twist.linear.x))
        arrow.scale.y = 0.15
        arrow.scale.z = 0.15
        arrow.color.r = 0.1
        arrow.color.g = 1.0
        arrow.color.b = 0.1
        arrow.color.a = 1.0
        yaw = math.atan2(self.last_twist.angular.z, 1.0) * 0.5
        arrow.pose.orientation.z = math.sin(yaw / 2)
        arrow.pose.orientation.w = math.cos(yaw / 2)
        arr.markers.append(arrow)

        text = Marker()
        text.header.frame_id = "base_link"
        text.header.stamp = stamp
        text.ns = "cmd_vel_text"
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.z = 1.5
        text.scale.z = 0.4
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        text.text = (
            f"v={self.last_twist.linear.x:.2f} m/s  "
            f"ω={self.last_twist.angular.z:.2f} rad/s"
        )
        arr.markers.append(text)

        return arr

    def makeTargetMarker(self, stamp) -> Marker:
        m = Marker()
        m.header.frame_id = "base_link"
        m.header.stamp = stamp
        m.ns = "seedling_target"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = self.closest_seedling_bl.point.x
        m.pose.position.y = self.closest_seedling_bl.point.y
        m.pose.position.z = 0.3
        m.scale.x = 0.6
        m.scale.y = 0.6
        m.scale.z = 0.6
        m.color.r = 1.0
        m.color.g = 0.3
        m.color.b = 0.3
        m.color.a = 0.9

        if self.distance_to_seedling is not None:
            label = Marker()
            label.header = m.header
            label.ns = "seedling_target"
            label.id = 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = self.closest_seedling_bl.point.x
            label.pose.position.y = self.closest_seedling_bl.point.y
            label.pose.position.z = 1.2
            label.scale.z = 0.5
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = f"{self.distance_to_seedling:.2f} m"
            self.target_pub.publish(label)

        return m


def main():
    rclpy.init()
    node = VizDebug()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
