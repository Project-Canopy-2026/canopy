import numpy as np
import rclpy
from builtin_interfaces.msg import Time as RosTime
from route_planning.ant_colony import AntColony
from rclpy.node import Node, ParameterDescriptor, ParameterType
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from steward_msgs.msg import Route, ForestPlan
from geometry_msgs.msg import Point, TransformStamped
from time import time
from tqdm import tqdm, trange
from scipy.spatial.distance import pdist, squareform
import fast_tsp
from matplotlib import pyplot as plt
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class RoutePlanner(Node):
    def __init__(self):
        super().__init__("route_planner")

        self.setUpParameters()

        self.last_time_plan_received = -1.0
        self.cached_route_msg = None

        persistent_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        forest_plan_sub = self.create_subscription(
            OccupancyGrid, "/planning/forest_plan", self.forestPlanCb, persistent_qos
        )

        self.full_route_pub = self.create_publisher(Route, "/planning/full_route", 10)

        self.plan_marker_pub = self.create_publisher(
            Marker, "/vis/forest_plan", persistent_qos
        )

        self.route_marker_pub = self.create_publisher(
            Marker, "/vis/route", persistent_qos
        )

        ROUTE_PUB_FREQ = 0.5  # Hz. TODO: Parameterize this.
        self.route_pub_timer = self.create_timer(
            1 / ROUTE_PUB_FREQ, self.publishCachedRoute
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def publishCachedRoute(self) -> None:
        if self.cached_route_msg is None:
            self.get_logger().warning("Route not yet calculated. Waiting to publish.")
            return

        self.full_route_pub.publish(self.cached_route_msg)

    def getHeader(self) -> Header:
        msg = Header()
        msg.frame_id = "map"  # routes are in the map frame
        msg.stamp = self.get_clock().now().to_msg()
        return msg

    def getPointsFromOccupancyGrid(
        self, occupancy_grid: OccupancyGrid, do_plotting=False
    ):
        h = occupancy_grid.info.height
        w = occupancy_grid.info.width
        grid_data = np.asarray(occupancy_grid.data).reshape(h, w)

        if do_plotting:
            fig, axs = plt.subplots(1, 2)
            axs[0].imshow(grid_data)

        pixel_indices = np.asarray(np.where(grid_data > 0)).T

        # Swap columns so that x is the first column
        pixel_indices[:, [0, 1]] = pixel_indices[:, [1, 0]]

        # Invert the y axis to be right-handed
        pixel_indices[:, 1] = (-pixel_indices[:, 1]) + h

        map_points = (
            pixel_indices.astype(np.float64) * occupancy_grid.info.resolution
        )  # convert from pixels to meters

        # Now translate by the Occ. Grid's origin
        grid_origin = occupancy_grid.info.origin.position
        map_points[:, 0] += grid_origin.x
        map_points[:, 1] += grid_origin.y

        if do_plotting:
            plt.gca().set_aspect("equal")  # square aspect ratio for plotting
            axs[1].scatter(map_points[:, 0], map_points[:, 1])
            print(map_points)
            plt.show()

        return map_points

    def publishForestPlanMarker(self, points: list[Point]):
        self.get_logger().info(f"Publishing plan marker with {len(points)} points")
        marker_msg = Marker()
        marker_msg.header = self.getHeader()
        marker_msg.frame_locked = True
        marker_msg.type = Marker.CUBE_LIST
        marker_msg.points = points
        # https://wiki.ros.org/rviz/DisplayTypes/Marker#Points_.28POINTS.3D8.29
        marker_msg.scale.x = 1.0
        marker_msg.scale.y = 1.0
        marker_msg.scale.z = 3.0
        marker_msg.color.g = 1.0
        marker_msg.color.a = 1.0
        marker_msg.ns = "forest_plan"
        marker_msg.action = Marker.ADD
        marker_msg.id = 1
        self.plan_marker_pub.publish(marker_msg)

    def publishRouteMarker(self, points: list[Point]):

        # Repeat points not on ends to satisfy LINE_LIST spec
        modified_points = []
        modified_points.append(points[0])
        for pt in points[1:-1]:
            modified_points.append(pt)
            modified_points.append(pt)  # Yes-- twice!
        modified_points.append(points[-1])

        self.get_logger().info(f"Publishing plan marker with {len(points)} points")
        marker_msg = Marker()
        marker_msg.header = self.getHeader()
        marker_msg.frame_locked = True
        marker_msg.type = Marker.LINE_LIST
        marker_msg.points = modified_points
        # https://wiki.ros.org/rviz/DisplayTypes/Marker#Points_.28POINTS.3D8.29
        marker_msg.scale.x = 0.5
        marker_msg.color.b = 1.0
        marker_msg.color.a = 1.0
        marker_msg.ns = "route"
        marker_msg.action = Marker.ADD
        marker_msg.id = 1
        self.route_marker_pub.publish(marker_msg)

    def rosTimeToSeconds(self, rostime: RosTime) -> float:
        return rostime.sec + 1e9 * rostime.nanosec

    def calculateRoute(self, route_points) -> np.ndarray:
        distances = pdist(route_points)

        use_fasttsp = self.get_parameter("use_fasttsp").value
        if use_fasttsp:
            distances = squareform(
                distances.astype(np.uint16), force="tomatrix", checks=False
            )
            self.get_logger().debug(f"Using Fast-TCP.")
            self.get_logger().debug("Calculating route.")

            route = np.asarray(fast_tsp.find_tour(distances.astype(int)))

        else:
            distances = squareform(distances, force="tomatrix", checks=False)
            np.fill_diagonal(distances, np.inf)  # Required by AntColony class
            ant_colony = AntColony(
                distances,
                n_ants=1,
                n_best=1,
                n_iterations=1000,
                decay=0.95,
                alpha=1,
                beta=1,
            )
            route = ant_colony.run()
            route = np.asarray(route[0])[:, 0]
        return route

    def forestPlanCb(self, forest_plan_msg: OccupancyGrid, do_plotting=False) -> None:

        if self.last_time_plan_received >= self.rosTimeToSeconds(
            forest_plan_msg.info.map_load_time
        ):
            self.get_logger().debug("Skipping stale Forest Plan")
            return
        else:
            self.get_logger().info("Received a new Forest Plan")
            self.last_time_plan_received = self.rosTimeToSeconds(
                forest_plan_msg.info.map_load_time
            )

        try:
            bl_to_map_tf = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().info(
                f"Could not find base_link->map transform. Skipping."
            )
            return

        # Convert list of ROS Point messages to np array
        planting_points = self.getPointsFromOccupancyGrid(forest_plan_msg)

        # append the robot's position
        bl_to_map_tf: TransformStamped
        robot_pos_x = bl_to_map_tf.transform.translation.x
        robot_pos_y = bl_to_map_tf.transform.translation.y
        planting_points = np.vstack(([robot_pos_x, robot_pos_y], planting_points))

        self.get_logger().info(f"PLANTING POINTS: {planting_points.shape}")

        self.get_logger().debug("Calculating distances.")
        self.get_logger().debug("Converting distances to square form.")

        start = time()  # For measuring runtime

        route = self.calculateRoute(planting_points)
        for route_position, original_idx in enumerate(route):
            if original_idx == 0:
                robot_idx = route_position
                # self.get_logger().info(f"ROBOT IS AT {route_position}")

        route = np.roll(route, -robot_idx)

        for route_position, original_idx in enumerate(route):
            if original_idx == 0:
                robot_idx = route_position
                # self.get_logger().info(f"ROBOT IS AT {route_position}")

        self.get_logger().debug(f"Done. Routing took {time() - start} seconds")

        # Form a Route message
        route_msg = Route()
        route_msg.header = self.getHeader()

        for i, route_idx in enumerate(route):
            point_msg = Point()
            point_msg.x, point_msg.y = planting_points[route_idx].tolist()
            route_msg.points.append(point_msg)
            route_msg.ids.append(i)  # TODO: Do we need IDs for each seedling?

        self.publishForestPlanMarker(route_msg.points)
        self.publishRouteMarker(route_msg.points)

        # Publish the Route message
        assert len(route) == len(route_msg.ids), "Route ID length mismatch"

        self.cached_route_msg = route_msg
        # self.full_route_pub.publish(route_msg)

        if do_plotting:

            plt.style.use("seaborn")

            # Re-order planting points w.r.t. route
            planting_points = planting_points[route]

            plt.plot(planting_points[:, 0], planting_points[:, 1])
            plt.scatter(planting_points[:, 0], planting_points[:, 1])
            plt.scatter(planting_points[0, 0], planting_points[0, 1], c="g", s=250)

            plt.xlabel("x (meters)")
            plt.ylabel("y (meters)")
            plt.title(
                f"Iterative stochastic local search result for {len(route)} seedlings"
            )
            plt.show()

    def setUpParameters(self):
        use_fasttsp_param_desc = ParameterDescriptor()
        use_fasttsp_param_desc.description = (
            "Use Fast-TCP instead of Ant Colony Optimization. Defaults to True."
        )
        use_fasttsp_param_desc.type = ParameterType.PARAMETER_BOOL
        self.declare_parameter("use_fasttsp", True, use_fasttsp_param_desc)


def main(args=None):
    rclpy.init(args=args)

    node = RoutePlanner()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
