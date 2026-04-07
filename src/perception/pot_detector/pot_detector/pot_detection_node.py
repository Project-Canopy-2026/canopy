#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from cv_bridge import CvBridge
from ultralytics import YOLO
import message_filters
import cv2
import numpy as np

class SeedlingDetector(Node):
    def __init__(self):
        super().__init__('seedling_detector')

        # ROS2 publishers
        self.bbox_pub = self.create_publisher(Int32MultiArray, '/seedling/pot_bbox', 10)
        self.grasp_pub = self.create_publisher(PoseStamped, '/seedling/grasp_point', 10)
        self.vis_pub = self.create_publisher(Image, '/seedling/annotated_image', 10)

        # ROS2 subscribers — synchronized RGB + depth
        rgb_sub = message_filters.Subscriber(self, Image, '/camera/camera/color/image_raw')
        depth_sub = message_filters.Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw')
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=0.05
        )
        self.ts.registerCallback(self.image_callback)
        
        self.bridge = CvBridge()
        
        # self.model = YOLO('/home/teamj/dev/ros2_ws/src/canopy/src/perception/pot_detector/model/best.pt')
        self.model = YOLO('src/perception/pot_detector/model/yolo11n/best.pt')
    def image_callback(self, msg: Image, depth_msg: Image):
        # Convert ROS images to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')  # float32, metres

        # YOLO detection
        results = self.model(cv_image)
        boxes = results[0].boxes  # ultralytics Results API

        if boxes is None or len(boxes) == 0:
            self.get_logger().info('No objects detected')
            return

        # Find the bbox with highest confidence
        best_idx = int(boxes.conf.argmax())
        x1, y1, x2, y2 = boxes.xyxy[best_idx].cpu().numpy()
        conf = float(boxes.conf[best_idx].cpu())
        cls = int(boxes.cls[best_idx].cpu())
        # Publish bounding box (as Float32MultiArray: [x1, y1, x2, y2, confidence])
        bbox_msg = Int32MultiArray()
        bbox_msg.data = [int(x1), int(y1), int(x2), int(y2)]
        self.bbox_pub.publish(bbox_msg)

        # Compute pot center (for simplicity, take bbox center)
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        self.get_logger().info(f'Detected {len(boxes)} pot. Pot center at pixel ({cx}, {cy}) with confidence {conf:.2f} with class id {cls}')
        
        # Look up depth at the pot center pixel
        depth = float(depth_image[int(cy), int(cx)])

        # Convert image coordinates to world coordinates (placeholder)
        # grasp_pose = cal_grasp_pose(cx, cy, depth)

        # Publish annotated image for visualization
        vis_image = cv_image.copy()
        self.visualize(vis_image, boxes)
    
    # Helper function to visulize bbox
    def visualize(self, vis_image, boxes):
        for idx in range(len(boxes)): # Draw all the bbox detection result, with confidence score
            x1, y1, x2, y2 = boxes.xyxy[idx].cpu().numpy()
            conf = float(boxes.conf[idx].cpu())
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0

            cv2.rectangle(vis_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(vis_image, f'{conf:.2f}', (int(x1), int(y1) - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.circle(vis_image, (int(cx), int(cy)), 4, (0, 0, 255), -1)
            self.vis_pub.publish(self.bridge.cv2_to_imgmsg(vis_image, encoding='rgb8'))

    def cal_grasp_pose(self, cx, cy):
        grasp_pose = PoseStamped()
        # Placeholder for actual grasp pose calculation lca szogic
        # Find the 3D point (cx, cy) pixel corresponding to in the camera frame -> robot frame -> then convert to world frame?
        grasp_pose = PoseStamped()
        grasp_pose.header.stamp = self.get_clock().now().to_msg()
        grasp_pose.header.frame_id = 'camera_frame'

        grasp_pose.pose.position.x = ...
        grasp_pose.pose.position.y = ...
        grasp_pose.pose.position.z = ...

        grasp_pose.pose.orientation.x = ...
        grasp_pose.pose.orientation.y = ...
        grasp_pose.pose.orientation.z = ...
        grasp_pose.pose.orientation.w = ...

        return grasp_pose
    
def main(args=None):
    rclpy.init(args=args)
    node = SeedlingDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()