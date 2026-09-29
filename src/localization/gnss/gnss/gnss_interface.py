#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
import utm
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from gps_msgs.msg import GPSFix
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32

from tf2_ros.buffer import Buffer
from tf2_ros import TransformException, TransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from scipy.spatial.transform import Rotation as R


def wrap(a):
    return math.remainder(a, 2 * math.pi)


class gnss_interface(Node):
    """GPS position + gyro-propagated heading.

    Heading is integrated from the Warthog IMU gyro and slowly corrected towards
    the GPS course-over-ground (COG) only while driving forward and nearly
    straight. Two things in the bags made the previous approaches fail:

    * The Warthog's odom->base_link yaw only registers 5-30% of the real
      rotation (wheel odometry on this platform is badly under-scaled), so it
      can't carry heading through a turn. The IMU gyro can.
    * COG is the antenna's direction of travel. The antenna sits behind the
      point the robot rotates about, so COG = heading - atan(d * yaw_rate / v).
      Fitting the 9-15 and 9-24 bags gives an effective d of 1.0-1.1 m (not the
      0.495 m mount offset), i.e. ~55 deg of COG error during a v=0.8, w=1.2
      turn. We subtract that term and still only trust COG when turning slowly.
    """

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
        self.heading_valid_pub = self.create_publisher(Bool, "/gnss/heading_valid", 10)

        self.create_subscription(GPSFix, "/gps/gpsfix", self.gpsFix_cb, 10)
        self.create_subscription(
            Imu, self.get_parameter("imu_topic").value, self.imuCb, 50
        )

        lat0, lon0, alt0 = self.get_parameter("map_origin_lat_lon_alt_degrees").value
        self.origin_utm_x, self.origin_utm_y, self.origin_zone_num, self.origin_zone_letter = utm.from_latlon(lat0, lon0)

        # The GPS fix is at the antenna, not at base_link. Keep in sync with
        # base_to_gps_static_tf in gnss_bringup.launch.py.
        self.t_base_antenna = np.array([-0.495, 0.0, 0.778])

        self.COG_LEVER_ARM = self.get_parameter("cog_lever_arm_m").value
        self.MIN_COG_SPEED = self.get_parameter("cog_min_speed").value
        self.MAX_COG_YAW_RATE = self.get_parameter("cog_max_yaw_rate").value
        self.HEADING_GAIN = self.get_parameter("heading_gain").value
        self.HEADING_GATE = self.get_parameter("heading_gate").value
        self.INIT_SAMPLES = self.get_parameter("init_samples").value

        self.heading = None          # map-frame yaw of base_link, None until initialised
        self.init_samples = []       # COG headings collected during initialisation
        self.rejected_in_a_row = 0
        self.yaw_rate = 0.0          # low-passed gyro z, used to gate COG
        self.gyro_bias = 0.0
        self.last_imu_time = None
        self.t_map_base = None       # last GPS-derived base_link position

    def imuCb(self, msg: Imu):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        wz = msg.angular_velocity.z - self.gyro_bias

        if self.last_imu_time is not None:
            dt = t - self.last_imu_time
            if 0.0 < dt < 0.2:
                if self.heading is not None:
                    self.heading = wrap(self.heading + wz * dt)
                # keep the initialisation samples aligned with the current heading
                self.init_samples = [h + wz * dt for h in self.init_samples]
        self.last_imu_time = t
        self.yaw_rate += 0.3 * (wz - self.yaw_rate)

        if self.heading is not None:
            self.publishPose()

    def gpsFix_cb(self, msg: GPSFix):

        gps_utm_x, gps_utm_y, gps_zone_num, gps_zone_letter = utm.from_latlon(
            msg.latitude, msg.longitude
        )

        if gps_zone_num != self.origin_zone_num or gps_zone_letter != self.origin_zone_letter:
            raise RuntimeError("GPS fix is not in the same UTM zone as MAP_ORIGIN")

        gps_x = gps_utm_x - self.origin_utm_x
        gps_y = gps_utm_y - self.origin_utm_y

        # Slowly learn the gyro bias while parked.
        if msg.speed < 0.05 and abs(self.yaw_rate) < 0.05:
            self.gyro_bias += 0.01 * self.yaw_rate

        self.updateHeadingFromCog(msg)

        heading = self.heading
        if heading is None:
            # Position is still useful before heading is known; publish it with
            # the raw track so the TF tree exists, and flag heading as invalid.
            heading = self.trueTrackToEnuRads(msg.track)

        t_map_antenna = np.array([gps_x, gps_y, msg.altitude])
        q = R.from_euler("z", heading)
        self.t_map_base = t_map_antenna - q.apply(self.t_base_antenna)

        self.heading_valid_pub.publish(Bool(data=self.heading is not None))
        self.publishPose(heading, stamp=msg.header.stamp)

    def updateHeadingFromCog(self, msg: GPSFix):
        w = self.yaw_rate
        d = self.COG_LEVER_ARM
        if msg.speed < self.MIN_COG_SPEED or abs(w) > self.MAX_COG_YAW_RATE:
            return

        # COG = heading - atan(d * w / v_forward); undo the lever-arm swing.
        track = self.trueTrackToEnuRads(msg.track)
        v_fwd = math.sqrt(max(msg.speed ** 2 - (d * w) ** 2, 1e-3))
        measured = wrap(track + math.atan2(d * w, v_fwd))

        if self.heading is None:
            self.init_samples.append(measured)
            self.init_samples = self.init_samples[-self.INIT_SAMPLES:]
            if len(self.init_samples) == self.INIT_SAMPLES:
                c = np.exp(1j * np.array(self.init_samples))
                spread = math.sqrt(max(-2 * math.log(max(abs(c.mean()), 1e-9)), 0.0))
                if spread < 0.15:
                    self.heading = float(np.angle(c.mean()))
                    self.init_samples = []
                    self.get_logger().info(f"Heading initialised from GPS track: {self.heading:.2f} rad")
            return

        innovation = wrap(measured - self.heading)
        if abs(innovation) > math.pi / 2:
            return  # driving backwards; COG points the other way
        if abs(innovation) < self.HEADING_GATE:
            self.heading = wrap(self.heading + self.HEADING_GAIN * innovation)
            self.rejected_in_a_row = 0
        else:
            # A few outliers are normal; ~3 s of consistent disagreement while
            # driving straight means the gyro heading has drifted - start over.
            self.rejected_in_a_row += 1
            if self.rejected_in_a_row > 30:
                self.get_logger().warn("GPS track disagrees with gyro heading; re-initialising")
                self.heading = None
                self.rejected_in_a_row = 0

    def publishPose(self, heading=None, stamp=None):
        if self.t_map_base is None:
            return
        heading = self.heading if heading is None else heading

        try:
            odom_to_base = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, rclpy.time.Time()
            )
        except TransformException as e:
            self.get_logger().warn(f"Could not lookup {self.odom_frame}->{self.base_frame}: {e}", throttle_duration_sec=2.0)
            return

        q_map_base = R.from_euler("z", heading)
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

        # map->odom chosen so that map->base_link = (GPS position, fused heading)
        # regardless of how wrong the Warthog's odom->base_link is.
        q_map_odom = q_map_base * q_odom_base.inv()
        t_map_odom = self.t_map_base - q_map_odom.apply(t_odom_base)
        q_map_odom_arr = q_map_odom.as_quat()
        q = q_map_base.as_quat()

        if stamp is not None:
            odom_msg = Odometry()
            odom_msg.header.stamp = stamp
            odom_msg.header.frame_id = self.map_frame
            odom_msg.child_frame_id = self.base_frame
            odom_msg.pose.pose.position.x = self.t_map_base[0]
            odom_msg.pose.pose.position.y = self.t_map_base[1]
            odom_msg.pose.pose.position.z = self.t_map_base[2]
            odom_msg.pose.pose.orientation.x = q[0]
            odom_msg.pose.pose.orientation.y = q[1]
            odom_msg.pose.pose.orientation.z = q[2]
            odom_msg.pose.pose.orientation.w = q[3]
            self.odom_pub.publish(odom_msg)
            self.yaw_pub.publish(Float32(data=float(heading)))

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
        self.declare_parameter("imu_topic", "/w200_0120/sensors/imu_0/data")
        # Effective distance of the antenna behind the robot's rotation centre,
        # fitted from the 9-15 and 9-24 bags (1.0-1.1 m).
        self.declare_parameter("cog_lever_arm_m", 1.0)
        self.declare_parameter("cog_min_speed", 0.4)      # m/s
        self.declare_parameter("cog_max_yaw_rate", 0.3)   # rad/s
        self.declare_parameter("heading_gain", 0.1)       # per fix (10 Hz): ~1 s time constant
        self.declare_parameter("heading_gate", 0.6)       # rad
        self.declare_parameter("init_samples", 5)

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
