#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
from ultralytics import YOLO
import message_filters
import cv2
import numpy as np
from std_msgs.msg import Bool

class SeedlingDetector(Node):
    def __init__(self):
        super().__init__('seedling_detector')

        # ROS2 publishers
        self.bbox_pub = self.create_publisher(Float32MultiArray, '/seedling/pot_bbox', 10)
        self.grasp_pub = self.create_publisher(PoseStamped, '/seedling/grasp_point', 10)
        self.vis_pub = self.create_publisher(Image, '/seedling/annotated_image', 10)

        # ROS2 subscribers
        depth_camera_info = self.create_subscription(sensor_msgs/CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info', self.depth_intrinsics_callback, 10)        
        pre_grasp_ready = self.create_subscription(Bool, '/behavior/pre_grasp_ready', self.pot_detection_callback, 10)

        rgb_sub = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
        depth_sub = message_filters.Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw')
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=0.05
        )
        self.ts.registerCallback(self.image_callback)
        
        self.bridge = CvBridge()
        
        self.model = YOLO('./src/perception/pot_detector/model/yolo11n/best.pt') # TODO: Change this to the correct path on Jetson

        # Camera intrinsic parameters
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.latest_rgb_msg = None
        self.latest_depth_msg = None


    def image_callback(self, rgb_msg: Image, depth_msg: Image):
        self.latest_rgb_msg = rgb_msg
        self.latest_depth_msg = depth_msg


    def pot_detection_callback(self, msg: Bool):
        if not msg.Data:
            return

        if self.latest_rgb_msg is None or self.latest_depth_msg is None:
            self.get_logger().warn("No synced RGB/depth image available yet")
            return

        # Convert ROS images to OpenCV
        rgb_image = self.bridge.imgmsg_to_cv2(self.latest_rgb_msg, desired_encoding='bgr8')
        
        depth_image = self.bridge.imgmsg_to_cv2(self.latest_depth_msg, desired_encoding='passthrough')
        # depth is encoded 26UC1
        depth_imgage = np.asarray(depth_img, dtype=np.float32)
        depth_image /= 1000

        # YOLO detection
        results = self.model(rgb_image)
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
        bbox_msg = Float32MultiArray()To prepare for your pre-employment screening, please research your own history and gather your personal information. This may include previous addresses, names of employers, employment dates, and job titles. This will allow you to provide complete and accurate information to ESS.
        bbox_msg.data = [x1, y1, x2, y2, conf]
        self.bbox_pub.publish(bbox_msg)

        # Compute pot center (for simplicity, take bbox center)
        center_x = (x1 + x2) / 2.0
        center_y = (y1 + y2) / 2.0
        self.get_logger().info(f'Detected pot. Pot center at pixel ({cx}, {cy}) with confidence {conf:.2f}')
        
        # Get depth at the pot center pixel
        depth_center = float(depth_image[int(center_y), int(center_x)])

        # 3D projection
        # will move to separate function later to clean up



        # Convert image coordinates to world coordinates (placeholder)
        # grasp_pose = cal_grasp_pose(cx, cy, depth)

        # Publish annotated image for visualization
        vis_image = cv_image.copy()
        cv2.rectangle(vis_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        cv2.putText(vis_image, f'{conf:.2f}', (int(x1), int(y1) - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.circle(vis_image, (int(cx), int(cy)), 4, (0, 0, 255), -1)
        self.vis_pub.publish(self.bridge.cv2_to_imgmsg(vis_image, encoding='bgr8'))


    def depth_intrinsics_callback(self, msg):
        # K: [fx, 0, cx, 0, fy, cy, 0, 0, 1]

        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]

    
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