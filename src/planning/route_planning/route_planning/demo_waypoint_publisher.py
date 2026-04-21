import rclpy
from rclpy.node import Node, ParameterDescriptor, ParameterType
from scipy.spatial.distance import pdist
import utm
import math

from geometry_msgs.msg import Twist
from geographic_msgs.msg import GeoPoint
from nav_msgs.msg import Odometry



"""## Location 1
Lat: 40.439934617849424
Lon: -79.94084683705644
Track: 262.13745437784496

## Location 2
latitude: 40.43985086748369
longitude: -79.94080144283429
altitude: 282.96770546136753
track: 287.89370924570426

## Location 3
latitude: 40.439777735336314
longitude: -79.94083598133865
altitude: 282.5491612117143
track: 273.1302448622191

## Location 4
latitude: 40.439705141142376
longitude: -79.94086805367668
altitude: 282.2216009884931
track: 277.6960517220166

## Location 5
latitude: 40.439624404447926
longitude: -79.94083533916628
altitude: 282.6154295358104
track: 276.00900595749454
""""

waypoints = [
    (40.439934617849424, -79.94084683705644),
    (40.43985086748369, -79.94080144283429),
    (40.439777735336314, -79.94083598133865),
    (40.439705141142376, -79.94086805367668),
    (40.439624404447926, -79.94083533916628),
]

class DemoWaypointPublisher(Node):
    def __init__(self):
        super().__init__("demo_waypoint_publisher")

        self.create_timer(1.0, self.publishWaypoint)

        self.next_waypoint_pub = self.create_publisher(GeoPoint, "/planning/next_waypoint_geo", 1)
        self.waypoint_pub = self.create_publisher(GeoPoint, "/planning/goal_pose_geo", 1)

    def publishWaypoint(self):
        msg = GeoPoint()
        msg.latitude = 40.439934617849424
        msg.longitude = -79.94084683705644
        msg.altitude = 282.96770546136753

        self.waypoint_pub.publish(msg)


