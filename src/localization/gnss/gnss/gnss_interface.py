#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import utm
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from gps_msgs.msg import GPSFix
from std_msgs.msg import Header, Float32

from tf2_ros.buffer import Buffer
from tf2_ros import TransformException, TransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from scipy.spatial.transform import Rotation as R

class gnss_interface(Node):

    def __init__(self):
        super().__init__("gnss_interface")

        self.setUpParameters()

        self.map_frame = "map"
        self.odom_frame = "odom"
        self.base_frame = "base_link"

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.odom_pub = self.create_publisher(Odometry, "/gnss/odom", 10)
        self.yaw_pub = self.create_publisher(Float32, "/gnss/yaw", 10)

        self.create_subscription(GPSFix, "/gps/gpsfix", self.gpsFix_cb, 10)
        #self.create_subscription(Odometry, "/odometry/gps", self.gps_cb, 10)
        #self.create_timer(0.1, self.publish_tf)

        lat0, lon0, alt0 = self.get_parameter("map_origin_lat_lon_alt_degrees").value
        self.origin_utm_x, self.origin_utm_y, self.origin_zone_num, self.origin_zone_letter = utm.from_latlon(lat0, lon0)


    def gpsFix_cb(self, msg: GPSFix):

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.map_frame
        odom_msg.child_frame_id = self.base_frame

        gps_utm_x, gps_utm_y, gps_zone_num, gps_zone_letter = utm.from_latlon(
            msg.latitude, msg.longitude
        )

        # safety check
        if gps_zone_num != self.origin_zone_num or gps_zone_letter != self.origin_zone_letter:
            raise RuntimeError("GPS fix is not in the same UTM zone as MAP_ORIGIN")

        gps_x = gps_utm_x - self.origin_utm_x
        gps_y = gps_utm_y - self.origin_utm_y

        odom_msg.pose.pose.position.x = gps_x
        odom_msg.pose.pose.position.y = gps_y
        odom_msg.pose.pose.position.z = msg.altitude

        # This calculates the orientation for the entire robot.
        # Yes, this assumes that we're on a flat plane.
        yaw = self.trueTrackToEnuRads(msg.track)
        q = R.from_euler("xyz", [0.0, 0.0, yaw]).as_quat()

        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        # publish map -> base-link tf
        t = TransformStamped()
        t.transform.translation.x = gps_x
        t.transform.translation.y = gps_y
        t.transform.translation.z = msg.altitude
        t.transform.rotation = odom_msg.pose.pose.orientation
        t.header = odom_msg.header
        t.child_frame_id = odom_msg.child_frame_id
        self.tf_broadcaster.sendTransform(t)

        self.odom_pub.publish(odom_msg)
        self.yaw_pub.publish(Float32(data=yaw))


    def trueTrackToEnuRads(self, track_deg: float):
        enu_yaw = track_deg

        enu_yaw -= 90

        enu_yaw = 360 - enu_yaw

        if enu_yaw < 0:
            enu_yaw += 360
        elif enu_yaw > 360:
            enu_yaw -= 360

        enu_yaw *= math.pi / 180.0
        return enu_yaw
    

    # def publish_tf(self):
    #     if self.gps_odom is None:
    #         return

    #     try:
    #         odom_to_base = self.tf_buffer.lookup_transform(
    #             self.odom_frame,
    #             self.base_frame,
    #             rclpy.time.Time()
    #         )
    #     except Exception as e:
    #         self.get_logger().warn(f"Could not lookup {self.odom_frame}->{self.base_frame}: {e}")
    #         return

    #     # Treat /odometry/gps position as robot position in map
    #     map_x = self.gps_odom.pose.pose.position.x
    #     map_y = self.gps_odom.pose.pose.position.y

    #     # TF gives robot position in odom
    #     odom_x = odom_to_base.transform.translation.x
    #     odom_y = odom_to_base.transform.translation.y

    #     # Translation-only map -> odom
    #     tx = map_x - odom_x
    #     ty = map_y - odom_y

    #     tf_msg = TransformStamped()
    #     tf_msg.header.stamp = self.get_clock().now().to_msg()
    #     tf_msg.header.frame_id = self.map_frame
    #     tf_msg.child_frame_id = self.odom_frame

    #     tf_msg.transform.translation.x = tx
    #     tf_msg.transform.translation.y = ty
    #     tf_msg.transform.translation.z = 0.0

    #     # Identity rotation for version 1
    #     tf_msg.transform.rotation.x = 0.0
    #     tf_msg.transform.rotation.y = 0.0
    #     tf_msg.transform.rotation.z = 0.0
    #     tf_msg.transform.rotation.w = 1.0

    #     self.tf_broadcaster.sendTransform(tf_msg)

    def setUpParameters(self):
        #param_desc = ParameterDescriptor()
        #param_desc.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        self.declare_parameter(
            "map_origin_lat_lon_alt_degrees",
            [40.44132949798969, -79.94451105594635, 293.0],
        )

def main(args=None):
    rclpy.init(args=args)
    node = gnss_interface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

