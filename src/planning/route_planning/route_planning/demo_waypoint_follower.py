import rclpy
from rclpy.node import Node, ParameterDescriptor, ParameterType
from scipy.spatial.distance import pdist
import utm
import math

from geometry_msgs.msg import Twist
from geographic_msgs.msg import GeoPoint
from nav_msgs.msg import Odometry


class DemoWaypointFollower(Node):
    def __init__(self):
        super().__init__("demo_waypoint_follower")

        self.setUpParameters()

        self.create_subscription(GeoPoint, "/planning/goal_pose_geo", self.waypointCb, 1)
        self.create_subscription(Odometry, "/odometry/gps", self.odomCb, 1)

        self.twist_pub = self.create_publisher(Twist, "/cmd_vel", 1)

        self.cached_waypoint = None
        self.ego_pos = None
        self.ego_yaw = None

        self.create_timer(0.1, self.tickController)

    def odomCb(self, msg: Odometry):
        pos = msg.pose.pose.position
        self.ego_pos = (pos.x, pos.y)

        q = msg.pose.pose.orientation
        self.ego_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def waypointCb(self, msg: GeoPoint):
        self.cached_waypoint = self.latLonToMap(msg.latitude, msg.longitude)

    def latLonToMap(self, lat: float, lon: float):
        x, y, _, __ = utm.from_latlon(lat, lon)
        return (x - self._map_origin_utm_x, y - self._map_origin_utm_y)

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

        msg = Twist()

        yaw_kP = 0.7
        msg.angular.z = yaw_err * yaw_kP

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
        param_desc = ParameterDescriptor()
        param_desc.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        self.declare_parameter(
            "map_origin_lat_lon_alt_degrees",
            [40.44132949798969, -79.94451105594635, 293.0],
        )
# [40.4431653, -79.9402844, 288.0961589] # steward
        lat0, lon0, _ = self.get_parameter("map_origin_lat_lon_alt_degrees").value
        self._map_origin_utm_x, self._map_origin_utm_y, _, __ = utm.from_latlon(lat0, lon0)


def main(args=None):
    rclpy.init(args=args)

    node = DemoWaypointFollower()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
