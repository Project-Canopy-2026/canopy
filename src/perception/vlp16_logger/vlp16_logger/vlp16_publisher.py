import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
# from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5
)


class VLP16Publisher(Node):

    def __init__(self):
        super().__init__('vlp16_publisher')

        self.sub = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.callback,
            sensor_qos
        )

        self.pub = self.create_publisher(
            PointCloud2,
            '/vlp16/depth_pcd',
            sensor_qos
        )

    def callback(self, msg):
        self.pub.publish(msg)
        self.get_logger().info("Republished VLP-16 point cloud to topic '/vlp16/depth_pcd'")

def main(args=None):
    rclpy.init(args=args)
    node = VLP16Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
