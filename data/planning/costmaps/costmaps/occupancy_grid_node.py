import numpy as np
import rclpy
from array import array as Array
from rclpy.node import Node, ParameterDescriptor, ParameterType
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from time import time
from tqdm import tqdm, trange
from scipy.spatial.distance import pdist, squareform
from scipy.spatial.transform import Rotation as R
from matplotlib import pyplot as plt
import cv2
from enum import IntEnum

# ROS2 message definitions
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Pose, Point
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
from steward_msgs.msg import FailedChecks, HealthCheck, SystemwideStatus
from sensor_msgs.msg import PointCloud2, PointField

# prefix to the names of dummy fields we add to get byte alignment
# correct. this needs to not clash with any actual field names
DUMMY_FIELD_PREFIX = "__"
# mappings between PointField types and numpy types
type_mappings = [
    (PointField.INT8, np.dtype("int8")),
    (PointField.UINT8, np.dtype("uint8")),
    (PointField.INT16, np.dtype("int16")),
    (PointField.UINT16, np.dtype("uint16")),
    (PointField.INT32, np.dtype("int32")),
    (PointField.UINT32, np.dtype("uint32")),
    (PointField.FLOAT32, np.dtype("float32")),
    (PointField.FLOAT64, np.dtype("float64")),
]
pftype_to_nptype = dict(type_mappings)
nptype_to_pftype = dict((nptype, pftype) for pftype, nptype in type_mappings)


def numpy_to_occupancy_grid(arr, info=None):
    if not len(arr.shape) == 2:
        raise TypeError("Array must be 2D")
    if not arr.dtype == np.int8:
        raise TypeError("Array must be of int8s")

    grid = OccupancyGrid()
    if isinstance(arr, np.ma.MaskedArray):
        # We assume that the masked value are already -1, for speed
        arr = arr.data

    grid.data = Array("b", arr.ravel().astype(np.int8))
    grid.info = info or MapMetaData()
    grid.info.height = arr.shape[0]
    grid.info.width = arr.shape[1]

    return grid


def fields_to_dtype(fields, point_step):
    """Convert a list of PointFields to a numpy record datatype."""
    offset = 0
    np_dtype_list = []
    for f in fields:
        while offset < f.offset:
            # might be extra padding between fields
            np_dtype_list.append(("%s%d" % (DUMMY_FIELD_PREFIX, offset), np.uint8))
            offset += 1

        dtype = pftype_to_nptype[f.datatype]
        if f.count != 1:
            dtype = np.dtype((dtype, f.count))

        np_dtype_list.append((f.name, dtype))
        offset += pftype_to_nptype[f.datatype].itemsize * f.count

    # might be extra padding between points
    while offset < point_step:
        np_dtype_list.append(("%s%d" % (DUMMY_FIELD_PREFIX, offset), np.uint8))
        offset += 1

    return np_dtype_list


def pointcloud2_to_array(cloud_msg, squeeze=True):
    """Converts a rospy PointCloud2 message to a numpy recordarray

    Reshapes the returned array to have shape (height, width), even if the
    height is 1.

    The reason for using np.frombuffer rather than struct.unpack is
    speed... especially for large point clouds, this will be <much> faster.
    """
    # construct a numpy record type equivalent to the point type of this cloud
    dtype_list = fields_to_dtype(cloud_msg.fields, cloud_msg.point_step)

    # parse the cloud into an array
    cloud_arr = np.frombuffer(cloud_msg.data, dtype_list)

    # remove the dummy fields that were added
    cloud_arr = cloud_arr[
        [
            fname
            for fname, _type in dtype_list
            if not (fname[: len(DUMMY_FIELD_PREFIX)] == DUMMY_FIELD_PREFIX)
        ]
    ]

    if squeeze and cloud_msg.height == 1:
        return np.reshape(cloud_arr, (cloud_msg.width,))
    else:
        return np.reshape(cloud_arr, (cloud_msg.height, cloud_msg.width))


class OccupancyGridNode(Node):
    def __init__(self):
        super().__init__("occupancy_grid_node")

        self.setUpParameters()

        self.create_subscription(
            PointCloud2, "/zed/point_cloud/cloud_registered", self.pcdCb, 1
        )

        self.create_subscription(PointCloud2, "/depth_pcd", self.pcdCb, 1)

        self.occ_grid_pub = self.create_publisher(OccupancyGrid, "/cost/occupancy", 1)

    def transformPoints(self, pts: np.ndarray) -> np.ndarray:
        assert isinstance(pts, np.ndarray)
        assert pts.shape[1] == 3

        # The ZED camera is tilted about 10 degrees down, so we should
        # adjust the points accordingly

        rot_matrix = R.from_euler("xyz", [0.0, -10.0, 0.0], degrees=True).as_matrix()

        return pts @ rot_matrix

    def pcdCb(self, msg: PointCloud2):
        pts = pointcloud2_to_array(msg)

        # If the pcd comes from a ZED camera, perform additional cleanup
        from_zed = pts.ndim == 2

        pts = pts.flatten()
        pts = np.vstack((pts["x"], pts["y"], pts["z"])).T

        if from_zed:

            # Remove rows with NaN
            pts = pts[~np.isnan(pts).any(axis=1)]

            pts = self.transformPoints(pts)
        else:
            print(pts.dtype)

        # self.get_logger().info(f"Got point cloud with shape {arr.shape}: {arr[0]}!")

        RES = 0.2  # meters per pixel
        ORIGIN_X_PX = 40
        ORIGIN_X_M = ORIGIN_X_PX * RES
        ORIGIN_Y_PX = 50
        ORIGIN_Y_M = ORIGIN_Y_PX * RES
        GRID_WIDTH = 100
        GRID_HEIGHT = GRID_WIDTH

        print(pts)

        # Now we need to project everything to an occupancy grid
        arr = pts / RES
        arr = arr.astype(np.int8)

        # TODO: Perform PROPER plane segmentation
        # For now, we'll naively check height
        HEIGHT_CUTOFF = 0.3
        arr = arr[arr[:, 2] > HEIGHT_CUTOFF]
        arr = arr[:, :2]

        # Discard indices outside of bounds

        # Offset by origin
        arr[:, 0] += ORIGIN_X_PX
        arr[:, 1] += ORIGIN_Y_PX

        arr = arr[np.logical_and(arr[:, 0] > 0, arr[:, 0] < GRID_HEIGHT)]
        arr = arr[np.logical_and(arr[:, 1] > 1, arr[:, 1] < GRID_WIDTH)]

        grid = np.zeros((GRID_HEIGHT, GRID_WIDTH), dtype=np.int8)

        grid[tuple(arr.T)] = 100

        # FLIP AXES
        grid = grid.T

        # fig, (ax1, ax2) = plt.subplots(1, 2)
        # ax1.scatter(arr[:, 0], arr[:, 1])

        # self.get_logger().info(str(arr))

        origin = Point(x=-ORIGIN_X_M, y=-ORIGIN_Y_M)

        info = MapMetaData(resolution=RES)
        info.origin.position = origin
        msg = numpy_to_occupancy_grid(grid, info)

        msg.header.frame_id = "base_link"
        msg.header.stamp = self.get_clock().now().to_msg()

        self.occ_grid_pub.publish(msg)
        # ax2.imshow(grid)
        # plt.show()

    def setUpParameters(self):
        pass


def main(args=None):
    rclpy.init(args=args)

    node = OccupancyGridNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
