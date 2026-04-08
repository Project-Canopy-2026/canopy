#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from cv_bridge import CvBridge
from ultralytics import YOLO
import message_filters
import cv2
import numpy as np
from std_msgs.msg import Bool
from collections import deque

from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

class PotDetector(Node):
    def __init__(self):
        super().__init__('pot_detector')

        # ROS2 publishers
        self.bbox_pub = self.create_publisher(Int32MultiArray, '/pot/pot_bbox', 10)
        self.pot_pose_pub = self.create_publisher(PointStamped, '/pot/center_point', 10)
        self.vis_pub = self.create_publisher(Image, '/pot/annotated_image', 10)

        # ROS2 subscribers
        depth_camera_info = self.create_subscription(CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info', self.depth_intrinsics_callback, 10)        
        enable_pot_detection = self.create_subscription(Bool, '/behavior/enable_pot_detection', self.pot_detection_callback, 10)

        rgb_sub = message_filters.Subscriber(self, Image, '/camera/camera/color/image_raw')
        depth_sub = message_filters.Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw')
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=0.05
        )
        self.ts.registerCallback(self.image_callback)
        
        self.bridge = CvBridge()
        
        self.model = YOLO('src/perception/pot_detector/pot_detector/model/0402_yolo11s/best.pt') # TODO: Change this to the correct path on Jetson

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Camera intrinsic parameters
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.latest_rgb_msg = None
        self.latest_depth_msg = None

        self.center_x_window = deque(maxlen=5)
        self.center_y_window = deque(maxlen=5)

        self.max_point_dist = 100 # pixels
        self.conf_threshold = 0.7


    def image_callback(self, rgb_msg: Image, depth_msg: Image):
        self.latest_rgb_msg = rgb_msg
        self.latest_depth_msg = depth_msg


    def pot_detection_callback(self, msg: Bool):
        if not msg.data:
            return

        if self.latest_rgb_msg is None or self.latest_depth_msg is None:
            self.get_logger().warn("No synced RGB/depth image available yet")
            return

        # Convert ROS images to OpenCV
        rgb_image = self.bridge.imgmsg_to_cv2(self.latest_rgb_msg, desired_encoding='bgr8')
        
        depth_image = self.bridge.imgmsg_to_cv2(self.latest_depth_msg, desired_encoding='passthrough')
        # depth is encoded 26UC1
        depth_image = np.asarray(depth_image, dtype=np.float32)
        depth_image /= 1000

        # YOLO detection
        results = self.model(rgb_image)
        boxes = results[0].boxes  # ultralytics Results API

        if boxes is None or len(boxes) == 0:
            self.get_logger().info('No objects detected')
            return
        
        # Filter boxes by confidence threshold, then pick closest one
        best_idx = None
        best_depth = float('inf')
        best_center_x = None
        best_center_y = None

        for idx in range(len(boxes)):
            conf = float(boxes.conf[idx].cpu())
            if conf < self.conf_threshold:
                continue

            x1_i, y1_i, x2_i, y2_i = boxes.xyxy[idx].cpu().numpy()
            center_x = (x1_i + x2_i) / 2.0
            center_y = (y1_i + y2_i) / 2.0

            # get depth at the center pixel
            u = int(np.clip(center_x, 0, depth_image.shape[1] - 1))
            v = int(np.clip(center_y, 0, depth_image.shape[0] - 1))
            depth_center = float(depth_image[v, u])

            # Ignore invalid depth values
            if depth_center <= 0.0 or np.isnan(depth_center) or np.isinf(depth_center):
                continue

            if depth_center < best_depth:
                best_depth = depth_center
                best_idx = idx
                best_center_x = center_x
                best_center_y = center_y

        if best_idx is None:
            self.get_logger().info(f'No valid detections above confidence {self.conf_threshold:.2f} with usable depth. Skipping.')
            return

        x1, y1, x2, y2 = boxes.xyxy[best_idx].cpu().numpy()
        conf = float(boxes.conf[best_idx].cpu())
        cls = int(boxes.cls[best_idx].cpu())

        # Publish bounding box (as Int32MultiArray: [x1, y1, x2, y2, confidence])
        bbox_msg = Int32MultiArray()
        bbox_msg.data = [int(x1), int(y1), int(x2), int(y2)]
        self.bbox_pub.publish(bbox_msg)

        # Compute pot center (for simplicity, take bbox center)
        # center_x = (x1 + x2) / 2.0
        # center_y = (y1 + y2) / 2.0
        #self.get_logger().info(f'Detected pot. Pot center at pixel ({center_x}, {center_y}) with confidence {conf:.2f}')

        # add to rolling window
        self.center_x_window.append(best_center_x)
        self.center_y_window.append(best_center_y)

        # get stable average center point across frames and reject if far from average
        stable_result = self.get_stable_center(best_center_x, best_center_y)
        if stable_result is None:
            self.get_logger().warn(f"center is too far from average across {self.max_point_dist} pixels. Rejecting detection.")
            return
        else:
            stable_center_x, stable_center_y = stable_result

        self.get_logger().info(f'Stable pot center at pixel ({stable_center_x:.2f}, {stable_center_y:.2f})')

        # Get depth at the center pixel
        u = int(np.clip(stable_center_x, 0, depth_image.shape[1] - 1))
        v = int(np.clip(stable_center_y, 0, depth_image.shape[0] - 1))

        stable_depth_center = float(depth_image[v, u])

        self.get_logger().info(f'Depth at pot center: {stable_depth_center:.2f} meters')

        # visualize detection and stable center
        vis_image = rgb_image.copy()
        self.visualize(
            vis_image,
            boxes,
            stable_center_x=stable_center_x,
            stable_center_y=stable_center_y,
            stable_depth_center=stable_depth_center
        )

        # project 3D
        if self.fx is None or self.fy is None or self.cx is None or self.cy is None:
            self.get_logger().warn("Camera intrinsics not received yet")
            return

        center_3d_cam = self.project_3d(stable_center_x, stable_center_y, stable_depth_center)

        # transform the 3d projection from camera -> arm_base
        # try:
        #     center_3d_base = self.tf_buffer.transform(
        #         center_3d_cam,
        #         'arm_base',
        #         timeout=rclpy.duration.Duration(seconds=0.2)
        #     )
        # except Exception as e:
        #     self.get_logger().warn(f"Failed to transform point to link_base: {e}")
        #     return

        # get grasp pose
        #grasp_pose = calc_grasp_pose(center_3d_base)

        # publish pot center pose
        self.pot_pose_pub.publish(center_3d_cam)

        self.get_logger().info('Published pot center point in camera frame')


    def project_3d(self, x, y, depth):

        # 3D projection
        X_3d = (x - self.cx) * depth  / self.fx
        Y_3d = (y - self.cy) * depth  / self.fy
        Z_3d = depth

        # add frame and time to 3D point        
        point_3d_cam = PointStamped()
        point_3d_cam.header.stamp = self.latest_depth_msg.header.stamp
        point_3d_cam.header.frame_id = self.latest_depth_msg.header.frame_id

        point_3d_cam.point.x = X_3d
        point_3d_cam.point.y = Y_3d
        point_3d_cam.point.z = Z_3d

        return point_3d_cam


    def get_stable_center(self, x, y):
        # reject if too far from previous average
        # return average across window
        
        if len(self.center_x_window) <= 1 or len(self.center_y_window) <= 1:
            return x, y

        prev_x_avg = np.average(list(self.center_x_window)[:-1])
        prev_y_avg = np.average(list(self.center_y_window)[:-1])

        x_dist = abs(x - prev_x_avg)
        y_dist = abs(y - prev_y_avg)

        if x_dist > self.max_point_dist or y_dist > self.max_point_dist:
            self.center_x_window.pop()
            self.center_y_window.pop()
            return None

        stable_x_avg = np.average(self.center_x_window)
        stable_y_avg = np.average(self.center_y_window)

        return stable_x_avg, stable_y_avg

    def depth_intrinsics_callback(self, msg):
        # K: [fx, 0, cx, 0, fy, cy, 0, 0, 1]

        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    
    # Helper function to visulize bbox
    def visualize(self, vis_image, boxes, stable_center_x=None, stable_center_y=None, stable_depth_center=None):
        for idx in range(len(boxes)):  # Draw all bbox detections with confidence score
            x1, y1, x2, y2 = boxes.xyxy[idx].cpu().numpy()
            conf = float(boxes.conf[idx].cpu())
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0

            cv2.rectangle(vis_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(
                vis_image,
                f'{conf:.2f}',
                (int(x1), int(y1) - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )

            # raw bbox center = red cross
            cv2.drawMarker(
                vis_image,
                (int(cx), int(cy)),
                (0, 0, 255),
                markerType=cv2.MARKER_CROSS,
                markerSize=16,
                thickness=2
            )

        if stable_center_x is not None and stable_center_y is not None:
            scx = int(stable_center_x)
            scy = int(stable_center_y)

            # stable center = very visible blue ring + filled center
            cv2.circle(vis_image, (scx, scy), 14, (255, 0, 0), 3)
            cv2.circle(vis_image, (scx, scy), 5, (255, 0, 0), -1)

            # label text
            if stable_depth_center is not None:
                label = f"depth={stable_depth_center:.2f} m"
            else:
                label = "stable center"

            # text background box so Foxglove can show it clearly
            text_x = scx + 18
            text_y = scy - 18
            (text_w, text_h), baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2
            )

            cv2.rectangle(
                vis_image,
                (text_x - 4, text_y - text_h - 4),
                (text_x + text_w + 4, text_y + baseline + 4),
                (255, 255, 255),
                -1
            )

            cv2.putText(
                vis_image,
                label,
                (text_x, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 0, 0),
                2
            )

        self.vis_pub.publish(self.bridge.cv2_to_imgmsg(vis_image, encoding='bgr8'))


    # def calc_grasp_pose(self, center_point):
    #     # center point is pose stamped
    #     grasp_pose = center_point

        
    #     grasp_pose.header.stamp = self.get_clock().now().to_msg()
    #     grasp_pose.header.frame_id = 'camera_frame'

    #     grasp_pose.pose.position.x = ...
    #     grasp_pose.pose.position.y = ...
    #     grasp_pose.pose.position.z = ...

    #     grasp_pose.pose.orientation.x = ...
    #     grasp_pose.pose.orientation.y = ...
    #     grasp_pose.pose.orientation.z = ...
    #     grasp_pose.pose.orientation.w = ...

    #     return grasp_pose
    

def main(args=None):
    rclpy.init(args=args)
    node = PotDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()