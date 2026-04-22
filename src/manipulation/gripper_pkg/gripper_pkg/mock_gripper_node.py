import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class MockGripperNode(Node):
    """Simulated gripper node — responds to open/close services without serial hardware."""

    def __init__(self):
        super().__init__('gripper_node')
        self.open_srv  = self.create_service(Trigger, 'gripper/open',  self.open_cb)
        self.close_srv = self.create_service(Trigger, 'gripper/close', self.close_cb)
        self.get_logger().info('Mock gripper node ready (simulation mode, no hardware).')

    def open_cb(self, request, response):
        self.get_logger().info('[SIM] Gripper opened')
        response.success = True
        response.message = 'Opened (simulated)'
        return response

    def close_cb(self, request, response):
        self.get_logger().info('[SIM] Gripper closed')
        response.success = True
        response.message = 'Closed (simulated)'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MockGripperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
