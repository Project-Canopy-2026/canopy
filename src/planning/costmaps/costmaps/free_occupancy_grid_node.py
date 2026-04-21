import numpy as np
import rclpy
from array import array as Array
from rclpy.node import Node

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, MapMetaData


def numpy_to_occupancy_grid(arr, info=None):
    if not len(arr.shape) == 2:
        raise TypeError("Array must be 2D")
    if not arr.dtype == np.int8:
        raise TypeError("Array must be of int8s")

    grid = OccupancyGrid()
    if isinstance(arr, np.ma.MaskedArray):
        arr = arr.data

    grid.data = Array("b", arr.ravel().astype(np.int8))
    grid.info = info or MapMetaData()
    grid.info.height = arr.shape[0]
    grid.info.width = arr.shape[1]

    return grid


class FreeOccupancyGridNode(Node):
    """Publishes an all-free (zero) occupancy grid on /cost/occupancy.

    Drop-in replacement for occupancy_grid_node used to debug the planning
    stack without LiDAR. Grid geometry matches occupancy_grid_node so
    downstream consumers (cost_map_node, trajectory_planner) see identical
    metadata.
    """

    RES = 0.2
    ORIGIN_X_PX = 40
    ORIGIN_Y_PX = 50
    GRID_WIDTH = 100
    GRID_HEIGHT = 100
    PUBLISH_PERIOD_S = 0.1

    def __init__(self):
        super().__init__("free_occupancy_grid_node")

        self.occ_grid_pub = self.create_publisher(OccupancyGrid, "/cost/occupancy", 1)
        self.create_timer(self.PUBLISH_PERIOD_S, self.publishFreeGrid)

    def publishFreeGrid(self):
        grid = np.zeros((self.GRID_HEIGHT, self.GRID_WIDTH), dtype=np.int8)

        origin = Point(
            x=-self.ORIGIN_X_PX * self.RES,
            y=-self.ORIGIN_Y_PX * self.RES,
        )
        info = MapMetaData(resolution=self.RES)
        info.origin.position = origin

        msg = numpy_to_occupancy_grid(grid, info)
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.get_clock().now().to_msg()

        self.occ_grid_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FreeOccupancyGridNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
