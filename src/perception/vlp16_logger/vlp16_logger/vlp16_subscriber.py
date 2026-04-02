import rclpy
from rclpy.node import Node
# from rclpy.qos import SensorDataQoS
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import PointCloud2

sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5
)

class VLP16Subscriber(Node):

    def __init__(self):
        super().__init__('vlp16_subscriber')
        self.sub = self.create_subscription(
            PointCloud2,
            '/vlp16/depth_pcd',
            self.callback,
            sensor_qos
        )

    def callback(self, msg):
        self.get_logger().info(
            f'Received cloud | frame: {msg.header.frame_id} | width: {msg.width}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = VLP16Subscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
