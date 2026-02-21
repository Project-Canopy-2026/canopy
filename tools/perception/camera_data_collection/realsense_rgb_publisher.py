import rclpy
from rclpy.node import Node

import pyrealsense2 as rs
import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class RealSenseRGBPublisher(Node):
    def __init__(self):
        super().__init__('realsense_rgb_publisher')

        self.publisher_ = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.bridge = CvBridge()

        # RealSense setup
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.pipeline.start(config)

        self.timer = self.create_timer(1.0 / 30.0, self.publish_frame)
        self.get_logger().info("RealSense RGB publisher started")

    def publish_frame(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            self.get_logger().warn("Empty frame")
            return

        img = np.asanyarray(color_frame.get_data())

        msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_color_frame'

        self.publisher_.publish(msg)

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()


def main():
    rclpy.init()
    node = RealSenseRGBPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
