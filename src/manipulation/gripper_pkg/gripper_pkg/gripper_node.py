import rclpy
import serial
import time
from rclpy.node import Node
from std_srvs.srv import Trigger


class GripperNode(Node):
    def __init__(self):
        super().__init__('gripper_node')

        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('baud', 57600)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value

        self.get_logger().info(f'Connecting to Arduino on {port} at {baud}...')
        self.ser = serial.Serial(port, baud, timeout=5)
        time.sleep(2)
        self.ser.reset_input_buffer()
        self.get_logger().info('Serial connection established.')

        self.open_srv  = self.create_service(Trigger, 'gripper/open',  self.open_cb)
        self.close_srv = self.create_service(Trigger, 'gripper/close', self.close_cb)
        self.get_logger().info('Gripper node ready.')

    def _send_and_wait(self, command: str):
        self.ser.reset_input_buffer()
        self.ser.write((command + '\n').encode())
        while True:
            line = self.ser.readline().decode().strip()
            self.get_logger().info(f'Arduino: {line}')
            if line == 'DONE':
                return True
            if line in ('INVALID', ''):
                return False

    def open_cb(self, request, response):
        self.get_logger().info('Service called: open gripper')
        success = self._send_and_wait('O')
        response.success = success
        response.message = 'Opened' if success else 'Failed to open'
        return response

    def close_cb(self, request, response):
        self.get_logger().info('Service called: close gripper')
        success = self._send_and_wait('C')
        response.success = success
        response.message = 'Closed' if success else 'Failed to close'
        return response

    def destroy_node(self):
        if self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
