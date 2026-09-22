#!/usr/bin/env python3

import numpy as np
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

        lat0, lon0, alt0 = self.get_parameter("map_origin_lat_lon_alt_degrees").value
        self.origin_utm_x, self.origin_utm_y, self.origin_zone_num, self.origin_zone_letter = utm.from_latlon(lat0, lon0)

        # GPS track is the direction the antenna is travelling, not the robot's
        # heading. The antenna sits 0.495 m behind base_link, so while turning it
        # swings sideways and track trails the true heading by up to ~35 deg. Only
        # correct map->odom yaw from track while driving straight; the rest of the
        # time hold it and let odom->base_link (wheel/IMU) carry the heading.
        self.MIN_TRACK_SPEED = 0.4      # m/s
        self.MAX_TRACK_YAW_RATE = 0.1   # rad/s
        self.q_map_odom = None
        self.prev_odom_yaw = None
        self.prev_odom_time = None

        # The GPS fix is at the antenna, not at base_link. Keep in sync with
        # base_to_gps_static_tf in gnss_bringup.launch.py.
        self.t_base_antenna = np.array([-0.495, 0.0, 0.778])


    def gpsFix_cb(self, msg: GPSFix):

        gps_utm_x, gps_utm_y, gps_zone_num, gps_zone_letter = utm.from_latlon(
            msg.latitude, msg.longitude
        )

        if gps_zone_num != self.origin_zone_num or gps_zone_letter != self.origin_zone_letter:
            raise RuntimeError("GPS fix is not in the same UTM zone as MAP_ORIGIN")

        gps_x = gps_utm_x - self.origin_utm_x
        gps_y = gps_utm_y - self.origin_utm_y

        # Heading from GPS track, only trusted while driving straight (see below).
        # /gnss/odom and /gnss/yaw are published further down, once the fused
        # heading is available, so they agree with the map->base_link TF.
        yaw = self.trueTrackToEnuRads(msg.track)

        # Look up odom -> base_link (from wheel odometry bridged from Warthog)
        try:
            odom_to_base = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time()
            )
        except TransformException as e:
            self.get_logger().warn(f"Could not lookup {self.odom_frame}->{self.base_frame}: {e}", throttle_duration_sec=2.0)
            return

        # Compute map -> odom = (map -> base_link from GPS) * inv(odom -> base_link)
        t_map_antenna = np.array([gps_x, gps_y, msg.altitude])
        q_map_base = R.from_euler("xyz", [0.0, 0.0, yaw])

        t_odom_base = np.array([
            odom_to_base.transform.translation.x,
            odom_to_base.transform.translation.y,
            odom_to_base.transform.translation.z,
        ])
        q_odom_base = R.from_quat([
            odom_to_base.transform.rotation.x,
            odom_to_base.transform.rotation.y,
            odom_to_base.transform.rotation.z,
            odom_to_base.transform.rotation.w,
        ])

        # Previously corrected map->odom yaw from GPS track on every fix:
        # q_map_odom = q_map_base * q_odom_base.inv()

        # Yaw rate from successive odom->base_link lookups, to tell whether the
        # robot is turning (and so whether track can be trusted as heading).
        odom_yaw = q_odom_base.as_euler("xyz")[2]
        odom_time = rclpy.time.Time.from_msg(odom_to_base.header.stamp)
        yaw_rate = math.inf  # unknown until we have two samples; assume turning
        if self.prev_odom_yaw is not None:
            dt = (odom_time - self.prev_odom_time).nanoseconds * 1e-9
            if dt > 0.0:
                yaw_rate = math.remainder(odom_yaw - self.prev_odom_yaw, 2 * math.pi) / dt
        self.prev_odom_yaw = odom_yaw
        self.prev_odom_time = odom_time

        driving_straight = (
            msg.speed > self.MIN_TRACK_SPEED and abs(yaw_rate) < self.MAX_TRACK_YAW_RATE
        )
        if driving_straight or self.q_map_odom is None:
            self.q_map_odom = q_map_base * q_odom_base.inv()

        q_map_odom = self.q_map_odom

        # Fused heading: held map->odom yaw composed with live odom->base_link.
        # Equals GPS track while driving straight, and follows the Warthog's own
        # rotation through turns, where track is corrupted by the antenna offset.
        fused_yaw = (q_map_odom * q_odom_base).as_euler("xyz")[2]
        q_map_base_fused = R.from_euler("xyz", [0.0, 0.0, fused_yaw])
        q = q_map_base_fused.as_quat()

        # The fix is at the antenna, 0.495 m behind and 0.778 m above base_link.
        # Shift it by that lever arm, rotated into the map frame, so that
        # map->base_link really lands on base_link.
        t_map_base = t_map_antenna - q_map_base_fused.apply(self.t_base_antenna)

        # Publish GPS-derived odometry (map frame, base_link child)
        odom_msg = Odometry()
        odom_msg.header.stamp = msg.header.stamp
        odom_msg.header.frame_id = self.map_frame
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose.position.x = t_map_base[0]
        odom_msg.pose.pose.position.y = t_map_base[1]
        odom_msg.pose.pose.position.z = t_map_base[2]
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        self.odom_pub.publish(odom_msg)
        self.yaw_pub.publish(Float32(data=fused_yaw))

        t_map_odom = t_map_base - q_map_odom.apply(t_odom_base)
        q_map_odom_arr = q_map_odom.as_quat()

        # Publish map -> odom TF, stamped with the Warthog's clock to match odom->base_link
        tf_msg = TransformStamped()
        tf_msg.header.stamp = odom_to_base.header.stamp
        tf_msg.header.frame_id = self.map_frame
        tf_msg.child_frame_id = self.odom_frame
        tf_msg.transform.translation.x = t_map_odom[0]
        tf_msg.transform.translation.y = t_map_odom[1]
        tf_msg.transform.translation.z = t_map_odom[2]
        tf_msg.transform.rotation.x = q_map_odom_arr[0]
        tf_msg.transform.rotation.y = q_map_odom_arr[1]
        tf_msg.transform.rotation.z = q_map_odom_arr[2]
        tf_msg.transform.rotation.w = q_map_odom_arr[3]
        self.tf_broadcaster.sendTransform(tf_msg)


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

    def setUpParameters(self):
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
