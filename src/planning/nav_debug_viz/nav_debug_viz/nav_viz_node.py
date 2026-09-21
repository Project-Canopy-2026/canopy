"""Debug visualization for localization, seedlings, and the global target sequence.

Everything is drawn in the "map" frame (ENU, origin = map_origin_lat_lon_alt_degrees),
using the same map -> base_link TF that plan_manager, cost_map_node, and the planner use.

Published:
  /vis/seedlings     MarkerArray  planted (grey), remaining (green), current target (orange)
  /vis/robot         MarkerArray  robot footprint + heading arrow from TF
  /vis/robot_trail   Path         history of map -> base_link
  /vis/global_path   Path         robot -> nearest seedling -> ... (greedy nearest neighbor)
  /vis/robot_fix     NavSatFix    TF pose converted back to lat/lon (for Foxglove Map)
  /vis/geo           GeoJSON      seedlings, path, trail, heading (only if foxglove_msgs exists)
"""

import copy
import json
import math

import numpy as np
import rclpy
import utm
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from scipy.spatial.transform import Rotation as R
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from canopy_msgs.msg import PlantingPlan
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix, NavSatStatus
from visualization_msgs.msg import Marker, MarkerArray

try:
    from foxglove_msgs.msg import GeoJSON
except ImportError:
    GeoJSON = None

# Warthog (w200) body, meters
ROBOT_LENGTH = 1.52
ROBOT_WIDTH = 1.38

TRAIL_MAX_POSES = 2000
TRAIL_MIN_STEP = 0.1  # meters
HEADING_LINE_LENGTH = 3.0  # meters, for the GeoJSON heading line

GREY = (0.6, 0.6, 0.6, 0.8)
GREEN = (0.1, 0.9, 0.2, 0.9)
ORANGE = (1.0, 0.55, 0.0, 1.0)


class NavVizNode(Node):
    def __init__(self):
        super().__init__("nav_viz_node")

        self.declare_parameter(
            "map_origin_lat_lon_alt_degrees",
            [40.44132949798969, -79.94451105594635, 293.0],
        )
        lat0, lon0, _ = self.get_parameter("map_origin_lat_lon_alt_degrees").value
        self.origin_x, self.origin_y, self.zone_num, self.zone_letter = utm.from_latlon(
            lat0, lon0
        )

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(
            PlantingPlan, "/planning/complete_plan", self.completePlanCb, 1
        )
        self.create_subscription(
            PlantingPlan, "/planning/remaining_plan", self.remainingPlanCb, 1
        )

        self.seedlings_pub = self.create_publisher(MarkerArray, "/vis/seedlings", latched)
        self.robot_pub = self.create_publisher(MarkerArray, "/vis/robot", 1)
        self.trail_pub = self.create_publisher(Path, "/vis/robot_trail", 1)
        self.global_path_pub = self.create_publisher(Path, "/vis/global_path", 1)
        self.robot_fix_pub = self.create_publisher(NavSatFix, "/vis/robot_fix", 1)
        self.geo_pub = (
            self.create_publisher(GeoJSON, "/vis/geo", latched) if GeoJSON else None
        )
        if GeoJSON is None:
            self.get_logger().warning(
                "foxglove_msgs not installed; /vis/geo (Foxglove Map GeoJSON) disabled. "
                "Install with: sudo apt install ros-humble-foxglove-msgs"
            )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Each seedling: dict(id, lat, lon, x, y)
        self.all_seedlings = []
        self.remaining_keys = None  # None until a remaining plan arrives
        self.ego_pose = None  # (x, y, yaw)
        self.trail = Path()
        self.trail.header.frame_id = "map"

        self.create_timer(0.2, self.update)

    # ------------------------------------------------------------------
    # Plan handling

    def latLonToMap(self, lat: float, lon: float):
        x, y, _, __ = utm.from_latlon(lat, lon)
        return x - self.origin_x, y - self.origin_y

    def mapToLatLon(self, x: float, y: float):
        return utm.to_latlon(
            x + self.origin_x, y + self.origin_y, self.zone_num, self.zone_letter
        )

    @staticmethod
    def seedlingKey(seedling):
        return (seedling.species_id, seedling.latitude, seedling.longitude)

    def toSeedlingList(self, msg: PlantingPlan):
        seedlings = []
        for s in msg.seedlings:
            x, y = self.latLonToMap(s.latitude, s.longitude)
            seedlings.append(
                dict(key=self.seedlingKey(s), id=s.species_id, lat=float(s.latitude),
                     lon=float(s.longitude), x=x, y=y)
            )
        return seedlings

    def completePlanCb(self, msg: PlantingPlan):
        self.get_logger().info(f"Complete plan: {len(msg.seedlings)} seedlings")
        self.all_seedlings = self.toSeedlingList(msg)
        self.remaining_keys = None
        for s in self.all_seedlings:
            self.get_logger().info(f"  {s['id']}: map x={s['x']:.2f} y={s['y']:.2f}")
        self.publishSeedlings()

    def remainingPlanCb(self, msg: PlantingPlan):
        self.remaining_keys = {self.seedlingKey(s) for s in msg.seedlings}
        if not self.all_seedlings:
            # Started after complete_plan was published: best effort
            self.all_seedlings = self.toSeedlingList(msg)
        self.publishSeedlings()

    def remainingSeedlings(self):
        if self.remaining_keys is None:
            return list(self.all_seedlings)
        return [s for s in self.all_seedlings if s["key"] in self.remaining_keys]

    def targetOrder(self):
        """Remaining seedlings in greedy nearest-neighbor order from the robot.

        The first one is the seedling cost_map_node is currently steering toward.
        """
        remaining = self.remainingSeedlings()
        if self.ego_pose is None:
            return remaining

        order = []
        cur = np.array(self.ego_pose[:2])
        while remaining:
            dists = [np.hypot(s["x"] - cur[0], s["y"] - cur[1]) for s in remaining]
            nearest = remaining.pop(int(np.argmin(dists)))
            order.append(nearest)
            cur = np.array([nearest["x"], nearest["y"]])
        return order

    # ------------------------------------------------------------------
    # Periodic update

    def update(self):
        self.updateEgoPose()
        order = self.targetOrder()
        self.publishSeedlings(order)
        self.publishGlobalPath(order)
        self.publishRobot()
        self.publishGeo(order)

    def updateEgoPose(self):
        try:
            tf = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warning(
                f"No map->base_link TF yet: {ex}", throttle_duration_sec=5.0
            )
            return

        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = R.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz")[2]
        self.ego_pose = (t.x, t.y, yaw)

        poses = self.trail.poses
        if poses:
            last = poses[-1].pose.position
            if math.hypot(t.x - last.x, t.y - last.y) < TRAIL_MIN_STEP:
                return

        pose = PoseStamped()
        pose.header = tf.header
        pose.pose.position.x = t.x
        pose.pose.position.y = t.y
        pose.pose.orientation = q
        poses.append(pose)
        if len(poses) > TRAIL_MAX_POSES:
            del poses[: len(poses) - TRAIL_MAX_POSES]

    # ------------------------------------------------------------------
    # RViz outputs

    def newMarker(self, ns, marker_id, marker_type, rgba):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = marker_id
        m.type = marker_type
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        return m

    def seedlingColor(self, s, target_key):
        if s["key"] == target_key:
            return ORANGE
        if self.remaining_keys is not None and s["key"] not in self.remaining_keys:
            return GREY
        return GREEN

    def publishSeedlings(self, order=None):
        if order is None:
            order = self.targetOrder()
        target_key = order[0]["key"] if order else None

        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for i, s in enumerate(self.all_seedlings):
            color = self.seedlingColor(s, target_key)
            size = 0.8 if s["key"] == target_key else 0.5

            sphere = self.newMarker("seedlings", i, Marker.SPHERE, color)
            sphere.pose.position.x = s["x"]
            sphere.pose.position.y = s["y"]
            sphere.pose.position.z = size / 2
            sphere.scale.x = sphere.scale.y = sphere.scale.z = size
            markers.markers.append(sphere)

            label = self.newMarker("labels", i, Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0, 1.0))
            label.pose.position.x = s["x"]
            label.pose.position.y = s["y"]
            label.pose.position.z = size + 0.5
            label.scale.z = 0.4
            label.text = s["id"] or f"seedling {i}"
            markers.markers.append(label)

        self.seedlings_pub.publish(markers)

    def publishGlobalPath(self, order):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()

        points = []
        if self.ego_pose is not None:
            points.append(self.ego_pose[:2])
        points += [(s["x"], s["y"]) for s in order]

        for x, y in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.global_path_pub.publish(path)

    def publishRobot(self):
        if self.ego_pose is None:
            return
        x, y, yaw = self.ego_pose
        q = R.from_euler("z", yaw).as_quat()
        stamp = self.get_clock().now().to_msg()

        body = self.newMarker("robot", 0, Marker.CUBE, (0.2, 0.4, 1.0, 0.5))
        body.pose.position.x = x
        body.pose.position.y = y
        body.pose.position.z = 0.3
        body.pose.orientation.x, body.pose.orientation.y = q[0], q[1]
        body.pose.orientation.z, body.pose.orientation.w = q[2], q[3]
        body.scale.x, body.scale.y, body.scale.z = ROBOT_LENGTH, ROBOT_WIDTH, 0.6

        arrow = self.newMarker("robot", 1, Marker.ARROW, (1.0, 1.0, 0.0, 1.0))
        arrow.pose = copy.deepcopy(body.pose)
        arrow.pose.position.z = 0.7
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 2.0, 0.15, 0.25

        text = self.newMarker("robot", 2, Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0, 1.0))
        text.pose.position.x = x
        text.pose.position.y = y
        text.pose.position.z = 1.5
        text.scale.z = 0.35
        text.text = f"({x:.1f}, {y:.1f}) yaw {math.degrees(yaw):.0f} deg ENU"

        self.robot_pub.publish(MarkerArray(markers=[body, arrow, text]))

        self.trail.header.stamp = stamp
        self.trail_pub.publish(self.trail)

        lat, lon = self.mapToLatLon(x, y)
        fix = NavSatFix()
        fix.header.stamp = stamp
        fix.header.frame_id = "base_link"
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.latitude, fix.longitude = float(lat), float(lon)
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.robot_fix_pub.publish(fix)

    # ------------------------------------------------------------------
    # Foxglove Map output

    def publishGeo(self, order):
        if self.geo_pub is None:
            return

        target_key = order[0]["key"] if order else None
        features = []

        def feature(geometry, name, color, **extra):
            return {
                "type": "Feature",
                "geometry": geometry,
                "properties": {"name": name, "style": {"color": color}, **extra},
            }

        def rgbHex(rgba):
            return "#%02x%02x%02x" % tuple(int(c * 255) for c in rgba[:3])

        for s in self.all_seedlings:
            features.append(feature(
                {"type": "Point", "coordinates": [s["lon"], s["lat"]]},
                s["id"], rgbHex(self.seedlingColor(s, target_key)),
            ))

        if self.ego_pose is not None:
            x, y, yaw = self.ego_pose
            path_ll = [self.mapToLatLon(x, y)] + [(s["lat"], s["lon"]) for s in order]
            if len(path_ll) > 1:
                features.append(feature(
                    {"type": "LineString", "coordinates": [[lon, lat] for lat, lon in path_ll]},
                    "global path", "#ff8c00",
                ))

            tip = self.mapToLatLon(
                x + HEADING_LINE_LENGTH * math.cos(yaw), y + HEADING_LINE_LENGTH * math.sin(yaw)
            )
            base = self.mapToLatLon(x, y)
            features.append(feature(
                {"type": "LineString", "coordinates": [[base[1], base[0]], [tip[1], tip[0]]]},
                "heading", "#ffff00",
            ))

        trail_ll = [self.mapToLatLon(p.pose.position.x, p.pose.position.y)
                    for p in self.trail.poses[::5]]
        if len(trail_ll) > 1:
            features.append(feature(
                {"type": "LineString", "coordinates": [[lon, lat] for lat, lon in trail_ll]},
                "robot trail", "#3366ff",
            ))

        self.geo_pub.publish(
            GeoJSON(geojson=json.dumps({"type": "FeatureCollection", "features": features}))
        )


def main(args=None):
    rclpy.init(args=args)
    node = NavVizNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
