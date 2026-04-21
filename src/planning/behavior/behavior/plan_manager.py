import rclpy
from rclpy.node import Node, ParameterDescriptor, ParameterType
from scipy.spatial.distance import pdist
import utm

# ROS2 message definitions
from diagnostic_msgs.msg import DiagnosticStatus
from std_msgs.msg import Float32, Empty
from canopy_msgs.msg import PlantingPlan, Seedling
from nav_msgs.msg import Odometry

from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener

class PlanManager(Node):
    def __init__(self):
        super().__init__("plan_manager")

        self.setUpParameters()

        self.get_logger().info("Plan Manager says 'Hello, world!'")

        self.create_subscription(
            PlantingPlan, "/planning/complete_plan", self.completePlanCb, 1
        )
        self.create_subscription(
            Float32, "/planning/distance_to_seedling", self.seedlingDistanceCb, 1
        )
        #self.create_subscription(Odometry, "/odometry/gps", self.odomCb, 1)

        self.remaining_plan_pub = self.create_publisher(
            PlantingPlan, "/planning/remaining_plan", 1
        )
        self.seedling_reached_pub = self.create_publisher(
            Empty, "/behavior/on_seedling_reached", 1
        )
        self.status_pub = self.create_publisher(DiagnosticStatus, "/diagnostics", 1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.original_seedlings = []
        self.remaining_seedlings = []
        self.remaining_seedling_points = []
        self.bounds_geojson = ""
        self.ego_pos = None

    # def odomCb(self, msg: Odometry):
    #     pos = msg.pose.pose.position
    #     self.ego_pos = [pos.x, pos.y]

    def completePlanCb(self, msg: PlantingPlan):
        self.get_logger().info(f"Got complete plan with {len(msg.seedlings)} seedlings")

        self.original_seedlings = []
        self.remaining_seedlings = []
        self.remaining_seedling_points = []
        self.bounds_geojson = msg.bounds_geojson

        lat, lon, alt = self.get_parameter("map_origin_lat_lon_alt_degrees").value
        origin_x, origin_y, _, __ = utm.from_latlon(lat, lon)

        for seedling in msg.seedlings:
            seedling: Seedling
            seedling_x, seedling_y, _, __ = utm.from_latlon(
                seedling.latitude, seedling.longitude
            )

            seedling_x = seedling_x - origin_x
            seedling_y = seedling_y - origin_y

            self.original_seedlings.append(seedling)
            self.remaining_seedling_points.append([seedling_x, seedling_y])

        self.remaining_seedlings = self.original_seedlings.copy()
        self.publishRemainingPlan()

    def publishRemainingPlan(self):
        plan_msg = PlantingPlan()
        plan_msg.bounds_geojson = self.bounds_geojson
        plan_msg.seedlings = self.remaining_seedlings
        self.remaining_plan_pub.publish(plan_msg)

    def seedlingDistanceCb(self, msg: Float32):
        closest_distance = msg.data

        # try:
        #     bl_to_map_tf = self.tf_buffer.lookup_transform(
        #         "map", "base_link", rclpy.time.Time()
        #     )
        #     ego_x = bl_to_map_tf.transform.translation.x
        #     ego_y = bl_to_map_tf.transform.translation.y
        #     self.ego_pos = [ego_x, ego_y]

        # except TransformException as ex:
        #     print(f"Could not get transform: {ex}")
        #     return

        # if self.ego_pos is None:
        #     self.get_logger().warning("Ego pose unavailable")
        #     return

        seedling_reached_distance = (
            self.get_parameter("seedling_reached_distance")
            .get_parameter_value()
            .double_value
        )

        if closest_distance > seedling_reached_distance:
            self.get_logger().info(f"Still {closest_distance:.2f} m away", throttle_duration_sec=1.0)
            return
        
        if closest_distance < seedling_reached_distance:
            self.get_logger().info("SEEEEEEDDDDLING REAAAAAAAACHEDDD")
            return
        
        if len(self.remaining_seedlings) < 1:
            self.get_logger().warning("No remaining seedlings in plan.")
            return

        # nearest_distance = 999999.9
        # closest_seedling_idx = -1
        # for idx, point in enumerate(self.remaining_seedling_points):
        #     dist = pdist([point, self.ego_pos])[0]

        #     if dist < nearest_distance:
        #         nearest_distance = dist
        #         closest_seedling_idx = idx

        # if closest_seedling_idx < 0:
        #     self.get_logger().warning("Could not identify closest remaining seedling.")
        #     return

        # del self.remaining_seedling_points[closest_seedling_idx]
        # del self.remaining_seedlings[closest_seedling_idx]

        # assert len(self.remaining_seedling_points) == len(self.remaining_seedlings)

        del self.remaining_seedlings[0]
        del self.remaining_seedling_points[0]

        self.get_logger().info("SEEDLING REACHED")
        self.publishRemainingPlan()
        self.seedling_reached_pub.publish(Empty())

    def setUpParameters(self):
        param_desc = ParameterDescriptor()
        param_desc.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        self.declare_parameter(
            "map_origin_lat_lon_alt_degrees",
            [40.44132949798969, -79.94451105594635, 293.0],
        )

        param_desc.type = ParameterType.PARAMETER_DOUBLE
        self.declare_parameter(
            "seedling_reached_distance",
            2, #0.8,
        )


def main(args=None):
    rclpy.init(args=args)

    node = PlanManager()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()