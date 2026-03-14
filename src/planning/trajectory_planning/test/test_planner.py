import math
import unittest
from unittest.mock import MagicMock, patch

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from scipy.spatial.transform.rotation import Rotation as R

from trajectory_planning.planner import PlannerNode


def make_node():
    """Construct a PlannerNode with ROS timers and pubs/subs disabled."""
    with patch.object(PlannerNode, "create_timer"), \
         patch.object(PlannerNode, "create_subscription"), \
         patch.object(PlannerNode, "create_publisher", return_value=MagicMock()):
        node = PlannerNode()
    return node


class TestPlannerNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = make_node()

    def tearDown(self):
        self.node.destroy_node()

    # ------------------------------------------------------------------
    # getYawError
    # ------------------------------------------------------------------

    def test_yaw_error_straight_ahead(self):
        # Goal directly in front (positive x in base_link) → zero yaw error
        yaw_error = self.node.getYawError([1.0, 0.0])
        self.assertAlmostEqual(yaw_error, 0.0, places=5)

    def test_yaw_error_left(self):
        # Goal to the left (positive y) → positive yaw error
        yaw_error = self.node.getYawError([0.0, 1.0])
        self.assertGreater(yaw_error, 0.0)

    def test_yaw_error_right(self):
        # Goal to the right (negative y) → negative yaw error
        yaw_error = self.node.getYawError([0.0, -1.0])
        self.assertLess(yaw_error, 0.0)

    def test_yaw_error_wrapped_within_pi(self):
        # Result must always be in (-pi, pi]
        for x in [-5.0, -1.0, 0.0, 1.0, 5.0]:
            for y in [-5.0, -1.0, 0.0, 1.0, 5.0]:
                if x == 0 and y == 0:
                    continue
                err = self.node.getYawError([x, y])
                self.assertGreaterEqual(err, -math.pi)
                self.assertLessEqual(err, math.pi)

    # ------------------------------------------------------------------
    # getSmoothed
    # ------------------------------------------------------------------

    def test_smoothed_ramps_up_linearly(self):
        self.node.previous_twist = Twist()  # start at 0
        cmd = Twist()
        cmd.linear.x = 1.0  # big jump
        out = self.node.getSmoothed(cmd)
        self.assertAlmostEqual(out.linear.x, 0.2)  # capped at linear_max_delta

    def test_smoothed_ramps_down_linearly(self):
        prev = Twist()
        prev.linear.x = 0.5
        self.node.previous_twist = prev
        cmd = Twist()
        cmd.linear.x = 0.0  # sudden stop
        out = self.node.getSmoothed(cmd)
        self.assertAlmostEqual(out.linear.x, 0.3)  # 0.5 - 0.2

    def test_smoothed_passes_through_small_change(self):
        prev = Twist()
        prev.linear.x = 0.3
        self.node.previous_twist = prev
        cmd = Twist()
        cmd.linear.x = 0.4  # within delta
        out = self.node.getSmoothed(cmd)
        self.assertAlmostEqual(out.linear.x, 0.4)

    def test_smoothed_updates_previous_twist(self):
        cmd = Twist()
        cmd.linear.x = 0.1
        self.node.getSmoothed(cmd)
        self.assertAlmostEqual(self.node.previous_twist.linear.x, 0.1)

    def test_smoothed_angular_clamped(self):
        self.node.previous_twist = Twist()
        cmd = Twist()
        cmd.angular.z = 2.0  # big angular jump
        out = self.node.getSmoothed(cmd)
        self.assertAlmostEqual(out.angular.z, 0.3)  # capped at angular_max_delta

    # ------------------------------------------------------------------
    # odomCb — state extraction
    # ------------------------------------------------------------------

    def _make_odom(self, x, y, yaw_rad, linear_vel):
        msg = Odometry()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        q = R.from_euler("z", yaw_rad).as_quat()
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        msg.twist.twist.linear.x = linear_vel
        return msg

    def test_odom_cb_sets_position(self):
        self.node.odomCb(self._make_odom(3.0, 4.0, 0.0, 0.0))
        self.assertAlmostEqual(self.node.ego_pos[0], 3.0)
        self.assertAlmostEqual(self.node.ego_pos[1], 4.0)

    def test_odom_cb_sets_yaw(self):
        self.node.odomCb(self._make_odom(0.0, 0.0, math.pi / 4, 0.0))
        self.assertAlmostEqual(self.node.ego_yaw, math.pi / 4, places=5)

    def test_odom_cb_sets_linear_vel(self):
        self.node.odomCb(self._make_odom(0.0, 0.0, 0.0, 0.75))
        self.assertAlmostEqual(self.node.ego_linear_vel, 0.75)

    def test_odom_cb_yaw_zero(self):
        self.node.odomCb(self._make_odom(0.0, 0.0, 0.0, 0.0))
        self.assertAlmostEqual(self.node.ego_yaw, 0.0, places=5)

    def test_odom_cb_yaw_negative(self):
        self.node.odomCb(self._make_odom(0.0, 0.0, -math.pi / 2, 0.0))
        self.assertAlmostEqual(self.node.ego_yaw, -math.pi / 2, places=5)

    # ------------------------------------------------------------------
    # latLonToMap
    # ------------------------------------------------------------------

    def test_lat_lon_origin_maps_to_zero(self):
        # The origin itself should map to (0, 0)
        origin_lat, origin_lon = 40.4431653, -79.9402844
        x, y = self.node.latLonToMap(origin_lat, origin_lon)
        self.assertAlmostEqual(x, 0.0, places=2)
        self.assertAlmostEqual(y, 0.0, places=2)

    def test_lat_lon_north_is_positive_y(self):
        origin_lat, origin_lon = 40.4431653, -79.9402844
        _, y = self.node.latLonToMap(origin_lat + 0.001, origin_lon)
        self.assertGreater(y, 0.0)

    def test_lat_lon_east_is_positive_x(self):
        origin_lat, origin_lon = 40.4431653, -79.9402844
        x, _ = self.node.latLonToMap(origin_lat, origin_lon + 0.001)
        self.assertGreater(x, 0.0)

    # ------------------------------------------------------------------
    # getClosestSeedlingInBaselink
    # ------------------------------------------------------------------

    def test_closest_seedling_returns_none_without_cost_map(self):
        self.node.total_cost_map = None
        result = self.node.getClosestSeedlingInBaselink()
        self.assertIsNone(result)

    def test_closest_seedling_returns_min_cost_location(self):
        cost_map = np.ones((100, 80), dtype=np.int8) * 50
        cost_map[60, 45] = 0  # minimum cost pixel
        self.node.total_cost_map = cost_map
        result = self.node.getClosestSeedlingInBaselink()
        # pixel_coords = (row=60, col=45) → [col - 40, row - 50] = [5, 10]
        self.assertEqual(result[0], 5)
        self.assertEqual(result[1], 10)

    # ------------------------------------------------------------------
    # velocity feedback math
    # ------------------------------------------------------------------

    def test_velocity_feedback_boosts_when_slow(self):
        # Robot is going slower than desired → target_speed should exceed desired_speed
        Kp_linear = 0.25
        Kp_vel = 0.5
        SPEED_LIMIT = 0.6
        distance_remaining = 3.0
        ego_linear_vel = 0.1  # slow

        desired_speed = min(distance_remaining * Kp_linear, SPEED_LIMIT)
        vel_error = desired_speed - ego_linear_vel
        target_speed = max(0.0, min(desired_speed + Kp_vel * vel_error, SPEED_LIMIT))

        self.assertGreater(target_speed, desired_speed)
        self.assertLessEqual(target_speed, SPEED_LIMIT)

    def test_velocity_feedback_reduces_when_fast(self):
        Kp_linear = 0.25
        Kp_vel = 0.5
        SPEED_LIMIT = 0.6
        distance_remaining = 3.0
        ego_linear_vel = 0.5  # faster than desired

        desired_speed = min(distance_remaining * Kp_linear, SPEED_LIMIT)
        vel_error = desired_speed - ego_linear_vel
        target_speed = max(0.0, min(desired_speed + Kp_vel * vel_error, SPEED_LIMIT))

        self.assertLess(target_speed, desired_speed)
        self.assertGreaterEqual(target_speed, 0.0)

    def test_velocity_feedback_never_exceeds_speed_limit(self):
        Kp_linear = 0.25
        Kp_vel = 0.5
        SPEED_LIMIT = 0.6
        distance_remaining = 100.0  # far away
        ego_linear_vel = 0.0

        desired_speed = min(distance_remaining * Kp_linear, SPEED_LIMIT)
        vel_error = desired_speed - ego_linear_vel
        target_speed = max(0.0, min(desired_speed + Kp_vel * vel_error, SPEED_LIMIT))

        self.assertLessEqual(target_speed, SPEED_LIMIT)

    def test_velocity_feedback_never_negative(self):
        Kp_linear = 0.25
        Kp_vel = 0.5
        SPEED_LIMIT = 0.6
        distance_remaining = 0.01  # nearly arrived
        ego_linear_vel = 0.5  # still moving fast

        desired_speed = min(distance_remaining * Kp_linear, SPEED_LIMIT)
        vel_error = desired_speed - ego_linear_vel
        target_speed = max(0.0, min(desired_speed + Kp_vel * vel_error, SPEED_LIMIT))

        self.assertGreaterEqual(target_speed, 0.0)


if __name__ == "__main__":
    unittest.main()
