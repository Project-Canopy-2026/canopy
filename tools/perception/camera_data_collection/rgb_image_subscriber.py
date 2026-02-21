import rclpy
from rclpy.node import Node

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.listener_callback,
            10
        )

        self.get_logger().info("Image subscriber started")

    def listener_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow("ROS2 RGB Image", img)
        cv2.waitKey(1)


if __name__ == '__main__':
    rclpy.init()
    node = ImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

