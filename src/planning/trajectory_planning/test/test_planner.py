import math
import unittest
from unittest.mock import MagicMock, patch

import numpy as np
import rclpy
from rclpy.clock import ClockType
from rclpy.time import Time
from canopy_msgs.msg import Mode
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from scipy.spatial.transform.rotation import Rotation as R
from tf2_ros import TransformException

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
        # origin_lat, origin_lon = 40.4431653, -79.9402844 # steward
        origin_lat, origin_lon = 40.44132949798969, -79.94451105594635
        x, y = self.node.latLonToMap(origin_lat, origin_lon)
        self.assertAlmostEqual(x, 0.0, places=2)
        self.assertAlmostEqual(y, 0.0, places=2)

    def test_lat_lon_north_is_positive_y(self):
        # origin_lat, origin_lon = 40.4431653, -79.9402844 # steward
        origin_lat, origin_lon = 40.44132949798969, -79.94451105594635
        _, y = self.node.latLonToMap(origin_lat + 0.001, origin_lon)
        self.assertGreater(y, 0.0)

    def test_lat_lon_east_is_positive_x(self):
        # origin_lat, origin_lon = 40.4431653, -79.9402844 # steward
        origin_lat, origin_lon = 40.44132949798969, -79.94451105594635
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


def rosTime(sec):
    """Time on the node's clock type (ROS time), as get_clock().now() returns."""
    return Time(nanoseconds=int(sec * 1e9), clock_type=ClockType.ROS_TIME)


class TestStalePoseStop(unittest.TestCase):
    """The planner holds when map->base_link stops updating (11-9 bridge freeze).

    Time is faked: self.t is the Jetson clock, self.stamp is the map->base_link
    header stamp, which carries the W200's clock (seconds off from the Jetson's).
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = make_node()
        # make_node gives every publisher the same mock; split them up.
        self.node.twist_pub = MagicMock()
        self.node.status_pub = MagicMock()
        self.node.facing_downhill_pub = MagicMock()

        self.t = 1000.0
        self.stamp = 1000.0
        clock = MagicMock()
        clock.now.side_effect = lambda: rosTime(self.t)
        self.node.get_clock = lambda: clock

        self.tf_missing = False
        self.node.tf_buffer = MagicMock()
        self.node.tf_buffer.lookup_transform.side_effect = self.lookup

    def tearDown(self):
        self.node.destroy_node()

    def lookup(self, target, source, _time):
        if self.tf_missing:
            raise TransformException("no map->base_link")
        tf = TransformStamped()
        tf.header.frame_id = target
        tf.child_frame_id = source
        tf.header.stamp = rosTime(self.stamp).to_msg()
        tf.transform.translation.x = 1.0
        tf.transform.translation.y = 2.0
        tf.transform.rotation.w = 1.0
        return tf

    def tick(self, dt=0.1, pose_moves=True):
        self.t += dt
        if pose_moves:
            self.stamp += dt

    def lastTwist(self):
        return self.node.twist_pub.publish.call_args[0][0]

    def lastStatus(self):
        return self.node.status_pub.publish.call_args[0][0]

    def setUpAutoDriving(self):
        """State in which updateTrajectorySimply reaches the seedling-steering code."""
        self.node.current_mode = Mode.AUTO
        self.node.remaining_seedling_count = 5
        self.node.closest_point_bl = np.array([2.78, 0.0])
        self.node.closest_point_bl_time = self.node.get_clock().now()
        self.node.previous_twist = Twist()
        self.node.previous_twist.linear.x = 0.6

    # ------------------------------------------------------------------
    # getPoseInMap
    # ------------------------------------------------------------------

    def test_pose_fresh_while_stamp_advances_whatever_the_clock_offset(self):
        # The W200 clock was 2.4 s ahead on 11-3; it must not matter which way.
        for offset in [-100.0, -2.4, 0.0, 2.4, 100.0]:
            with self.subTest(offset=offset):
                self.stamp = self.t + offset
                for _ in range(50):
                    self.tick()
                    self.assertIsNotNone(self.node.getPoseInMap())

    def test_pose_returns_map_position(self):
        x, y, yaw = self.node.getPoseInMap()
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, 2.0)
        self.assertAlmostEqual(yaw, 0.0)

    def test_pose_stale_after_timeout_when_stamp_stops(self):
        self.assertIsNotNone(self.node.getPoseInMap())
        self.tick(0.9, pose_moves=False)
        self.assertIsNotNone(self.node.getPoseInMap())  # normal gaps are < 0.5 s
        self.tick(0.2, pose_moves=False)
        self.assertIsNone(self.node.getPoseInMap())

    def test_pose_recovers_when_stamp_moves_again(self):
        self.node.getPoseInMap()
        self.tick(5.0, pose_moves=False)
        self.assertIsNone(self.node.getPoseInMap())
        self.tick(0.1)
        self.assertIsNotNone(self.node.getPoseInMap())

    def test_repeated_calls_in_one_tick_dont_reset_the_timer(self):
        self.node.getPoseInMap()
        self.tick(1.5, pose_moves=False)
        for _ in range(3):
            self.assertIsNone(self.node.getPoseInMap())

    def test_no_tf_returns_none(self):
        self.tf_missing = True
        self.assertIsNone(self.node.getPoseInMap())

    # ------------------------------------------------------------------
    # holdIfPoseStale
    # ------------------------------------------------------------------

    def test_hold_does_nothing_when_pose_fresh(self):
        self.assertFalse(self.node.holdIfPoseStale())
        self.node.twist_pub.publish.assert_not_called()

    def test_hold_ramps_to_zero_and_warns_when_pose_stale(self):
        self.node.previous_twist.linear.x = 0.6
        self.node.getPoseInMap()
        self.tick(2.0, pose_moves=False)

        speeds = []
        for _ in range(5):
            self.assertTrue(self.node.holdIfPoseStale())
            speeds.append(self.lastTwist().linear.x)
        self.assertEqual(speeds[-1], 0.0)
        self.assertEqual(speeds, sorted(speeds, reverse=True))
        self.assertEqual(self.lastStatus().message, "Robot pose not updating. Holding.")
        self.assertEqual(self.lastStatus().level, DiagnosticStatus.WARN)

    # ------------------------------------------------------------------
    # updateTrajectorySimply
    # ------------------------------------------------------------------

    def test_auto_holds_on_frozen_pose_like_11_9(self):
        # 11-9: bearing kept arriving with fresh stamps, pose frozen at 2.78 m.
        self.setUpAutoDriving()
        self.node.getPoseInMap()
        for _ in range(20):
            self.tick(pose_moves=False)
            self.node.closest_point_bl_time = self.node.get_clock().now()
            self.node.updateTrajectorySimply()
        self.assertEqual(self.lastTwist().linear.x, 0.0)
        self.assertEqual(self.lastStatus().message, "Robot pose not updating. Holding.")

    def test_auto_gets_past_pose_check_when_pose_fresh(self):
        # With a fresh pose, the next check (stale bearing) is what holds.
        self.setUpAutoDriving()
        self.node.closest_point_bl_time = rosTime(self.t - 5.0)
        self.node.updateTrajectorySimply()
        self.assertIn("Seedling bearing", self.lastStatus().message)

    def test_teleop_not_blocked_by_stale_pose(self):
        self.node.current_mode = Mode.TELEOP
        self.node.cached_teleop = Twist()
        self.node.cached_teleop.linear.x = 0.1
        self.node.getPoseInMap()
        self.tick(5.0, pose_moves=False)
        self.node.updateTrajectorySimply()
        self.assertAlmostEqual(self.lastTwist().linear.x, 0.1)
        self.assertEqual(self.lastStatus().message, "Following teleop commands")

    def test_downhill_turn_holds_on_stale_pose(self):
        self.node.current_mode = Mode.AUTO
        self.node.is_turning_downhill = True
        self.node.ego_yaw = 0.0  # pi away from downhill: would point turn
        self.node.getPoseInMap()
        self.tick(5.0, pose_moves=False)
        self.node.updateTrajectorySimply()
        self.assertEqual(self.lastTwist().angular.z, 0.0)
        self.assertEqual(self.lastStatus().message, "Robot pose not updating. Holding.")
        self.node.facing_downhill_pub.publish.assert_not_called()

    def test_estop_still_checked_first(self):
        self.node.estopped = True
        self.node.getPoseInMap()
        self.tick(5.0, pose_moves=False)
        self.node.updateTrajectorySimply()
        self.assertEqual(self.lastStatus().message, "E-stopped")


if __name__ == "__main__":
    unittest.main()
