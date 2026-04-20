#!/usr/bin/env python3
"""Record and visualize the robot's trajectory, starting only when it begins to move.

Waits for the robot's actual velocity (from /odometry/gps) to exceed a small
threshold, then begins appending poses to a nav_msgs/Path. Publishes the path
continuously so RViz/Foxglove show the trace grow in real time.

Run:
    source install/setup.bash
    python3 scripts/trajectory_on_move.py

RViz: Fixed Frame = "odom", add a Path display on /viz/trajectory_on_move.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Empty


MOVE_LINEAR_THRESHOLD = 0.05   # m/s
MOVE_ANGULAR_THRESHOLD = 0.05  # rad/s


class TrajectoryOnMove(Node):
    def __init__(self):
        super().__init__("trajectory_on_move")

        self.path = Path()
        self.path.header.frame_id = "odom"

        self.started = False
        self.last_pose: PoseStamped | None = None

        self.create_subscription(Odometry, "/odometry/gps", self.odomCb, 10)
        self.create_subscription(Empty, "/viz/reset_trajectory", self.resetCb, 1)

        self.path_pub = self.create_publisher(Path, "/viz/trajectory_on_move", 10)

        self.create_timer(0.1, self.publishPath)

        self.get_logger().info(
            f"Waiting for motion (>{MOVE_LINEAR_THRESHOLD} m/s or "
            f"{MOVE_ANGULAR_THRESHOLD} rad/s) before recording."
        )

    def resetCb(self, _msg: Empty):
        self.path.poses = []
        self.started = False
        self.get_logger().info("Trajectory reset.")

    def odomCb(self, msg: Odometry):
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z

        if not self.started:
            if abs(v) < MOVE_LINEAR_THRESHOLD and abs(w) < MOVE_ANGULAR_THRESHOLD:
                return
            self.started = True
            self.get_logger().info("Motion detected — recording trajectory.")

        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        if self.last_pose is None or self._hasMoved(self.last_pose, pose, 0.05):
            self.path.poses.append(pose)
            self.last_pose = pose
            if len(self.path.poses) > 20000:
                self.path.poses = self.path.poses[-20000:]

    @staticmethod
    def _hasMoved(a: PoseStamped, b: PoseStamped, min_dist: float) -> bool:
        dx = b.pose.position.x - a.pose.position.x
        dy = b.pose.position.y - a.pose.position.y
        return math.hypot(dx, dy) >= min_dist

    def publishPath(self):
        if not self.started:
            return
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path)


def main():
    rclpy.init()
    node = TrajectoryOnMove()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
