#!/usr/bin/env python3
"""Visualize robot and all seedlings from /planning/complete_plan.

Outputs two views:

1) RViz (Fixed Frame = "odom"):
     /viz/robot_pose       (PoseStamped)  robot in odom
     /viz/robot_path       (Path)         trajectory in odom
     /viz/seedlings        (MarkerArray)  all seedlings as spheres in odom
                                          (converted from lat/lon via UTM)

2) Foxglove Studio Map panel (satellite tiles):
     /gps/filtered         (NavSatFix)    (already published, robot)
     /viz/seedlings_geojson (std_msgs/String, GeoJSON FeatureCollection)
         In Foxglove, add a "Map" panel and a "Raw Messages"/custom layer
         pointed at this topic. Or use a GeoJSON layer.

Converts seedling lat/lon to odom-frame meters by subtracting UTM of the
robot's current /gps/filtered from the seedling's UTM, then adding the
robot's current /odometry/gps position. This is frame-agnostic — works
regardless of what datum navsat_transform used.

Run:
    source install/setup.bash
    python3 scripts/world_viz.py
"""
import json
import math

import rclpy
import utm
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from canopy_msgs.msg import PlantingPlan, Seedling


class WorldViz(Node):
    def __init__(self):
        super().__init__("world_viz")

        self.robot_odom: Odometry | None = None
        self.robot_fix: NavSatFix | None = None
        self.seedlings: list[Seedling] = []

        self.path = Path()
        self.path.header.frame_id = "odom"

        self.create_subscription(Odometry, "/odometry/gps", self.odomCb, 10)
        self.create_subscription(NavSatFix, "/gps/filtered", self.fixCb, 10)
        self.create_subscription(
            PlantingPlan, "/planning/complete_plan", self.planCb, 10
        )

        self.pose_pub = self.create_publisher(PoseStamped, "/viz/robot_pose", 10)
        self.path_pub = self.create_publisher(Path, "/viz/robot_path", 10)
        self.markers_pub = self.create_publisher(MarkerArray, "/viz/seedlings", 10)
        self.geojson_pub = self.create_publisher(
            String, "/viz/seedlings_geojson", 10
        )

        self.create_timer(0.5, self.publishAll)
        self.get_logger().info("world_viz running")

    def odomCb(self, msg: Odometry):
        self.robot_odom = msg
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)
        if len(self.path.poses) > 5000:
            self.path.poses = self.path.poses[-5000:]

    def fixCb(self, msg: NavSatFix):
        self.robot_fix = msg

    def planCb(self, msg: PlantingPlan):
        self.seedlings = list(msg.seedlings)
        self.get_logger().info(f"Got plan with {len(self.seedlings)} seedlings")

    def publishAll(self):
        now = self.get_clock().now().to_msg()

        self.path.header.stamp = now
        self.path_pub.publish(self.path)

        if self.robot_odom is not None:
            pose = PoseStamped()
            pose.header = self.robot_odom.header
            pose.header.stamp = now
            pose.pose = self.robot_odom.pose.pose
            self.pose_pub.publish(pose)

        self.publishSeedlingMarkers(now)
        self.publishGeoJSON()

    def publishSeedlingMarkers(self, stamp):
        if not self.seedlings or self.robot_odom is None or self.robot_fix is None:
            return

        robot_utm_x, robot_utm_y, _, __ = utm.from_latlon(
            self.robot_fix.latitude, self.robot_fix.longitude
        )
        robot_odom_x = self.robot_odom.pose.pose.position.x
        robot_odom_y = self.robot_odom.pose.pose.position.y

        arr = MarkerArray()

        clear = Marker()
        clear.action = Marker.DELETEALL
        clear.ns = "seedlings"
        arr.markers.append(clear)

        for i, s in enumerate(self.seedlings):
            sx, sy, _, __ = utm.from_latlon(s.latitude, s.longitude)
            # seedling position in odom = robot_odom + (seedling_utm - robot_utm)
            ox = robot_odom_x + (sx - robot_utm_x)
            oy = robot_odom_y + (sy - robot_utm_y)

            m = Marker()
            m.header.frame_id = "odom"
            m.header.stamp = stamp
            m.ns = "seedlings"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = ox
            m.pose.position.y = oy
            m.pose.position.z = 0.3
            m.scale.x = m.scale.y = m.scale.z = 0.8
            m.color.r = 0.2
            m.color.g = 0.9
            m.color.b = 0.2
            m.color.a = 0.9
            arr.markers.append(m)

            label = Marker()
            label.header.frame_id = "odom"
            label.header.stamp = stamp
            label.ns = "seedlings_labels"
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = ox
            label.pose.position.y = oy
            label.pose.position.z = 1.2
            label.scale.z = 0.5
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 1.0
            label.text = f"#{i}"
            arr.markers.append(label)

        self.markers_pub.publish(arr)

    def publishGeoJSON(self):
        if not self.seedlings:
            return

        features = []
        for i, s in enumerate(self.seedlings):
            features.append(
                {
                    "type": "Feature",
                    "geometry": {
                        "type": "Point",
                        "coordinates": [float(s.longitude), float(s.latitude)],
                    },
                    "properties": {"id": i, "species": s.species_id},
                }
            )

        if self.robot_fix is not None:
            features.append(
                {
                    "type": "Feature",
                    "geometry": {
                        "type": "Point",
                        "coordinates": [
                            float(self.robot_fix.longitude),
                            float(self.robot_fix.latitude),
                        ],
                    },
                    "properties": {"id": "robot"},
                }
            )

        fc = {"type": "FeatureCollection", "features": features}
        self.geojson_pub.publish(String(data=json.dumps(fc)))


def main():
    rclpy.init()
    node = WorldViz()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
