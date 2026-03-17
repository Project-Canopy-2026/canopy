#!/usr/bin/env python3

from ast import Return
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from realsense2_camera_msgs.msg import RGBD
from image_geometry import PinholeCameraModel

from cv_bridge import CvBridge
from ultralytics import YOLO
import torch
import cv2
import numpy as np

class stemDetector(Node):
    def __init__(self):
        super().__init__("stem_detection_node")
        self.bridge = CvBridge()        
        self.cam_model = PinholeCameraModel()
        self.model = YOLO(
            "/home/appleseed_labs/johnny-os/src/perception/stem_processing/models/yolov8_stems/weights/best.pt"
        )
        self.get_logger().info("YOLO model loaded")
        self.subscription = self.create_subscription(
            RGBD, "/camera/camera/rgbd", self.image_callback, 10
        )
        self.create_subscription(
            Empty, "/behavior/get_grasping", self.get_grasping_cb, 1
        )        
        # self.subscription = self.create_subscription(
        #     Image, "/camera/camera/color/image_rect_raw", self.image_callback, 10
        # )        
        self.publisher = self.create_publisher(Point, "/perception/stem_pos", 10)

        self.cam_model = PinholeCameraModel()
        self.filter_thres = 20
        self.filter_count = 0
        self.cx_history = np.zeros(self.filter_thres)
        self.cy_history = np.zeros(self.filter_thres)

        self.stem_pos_cam_frame = []
        self.start_detecting = False

    def image_callback(self, msg):
        self.cam_model.fromCameraInfo(msg.rgb_camera_info)
        rgb_frame = self.bridge.imgmsg_to_cv2(msg.rgb, desired_encoding="bgr8")
                
        ####TAKING ONLY THE CENTER PART OF THE FRAME!!!
        rgb_frame = rgb_frame[27:420,333:600,:]
        
        depth_frame = self.bridge.imgmsg_to_cv2(msg.depth, desired_encoding="passthrough")
        depth_frame = depth_frame[27:420,333:600]

        if not self.start_detecting:
            return

        detections = self.model(rgb_frame)[0]

        class_ids = detections.boxes.cls.tolist() if detections.boxes is not None else []
        class_names = detections.names

        enc = msg.depth.encoding
        if enc.upper().endswith('16UC1') or '16UC1' in enc.upper():
            depth_multiplier = 0.001
        else:
            depth_multiplier = 1

        # Draw bounding boxes and labels
        if detections.boxes is not None:
            for box, cls_id in zip(detections.boxes.xyxy, detections.boxes.cls):
                x1, y1, x2, y2 = map(int, box)
                w = x2-x1
                h = y2-y1
                # self.get_logger().info(f"bounding box height={h:.2f}, width={w:.2f}")
                # area = h*w
                label = class_names[int(cls_id)]
                self.stem_pos_cam_frame = []
                offset_bbox = 0
                if label == "stem" and h > 150:  # and self.start_detecting: # h>150 to filter out small false positives
                    y1 = y1 + offset_bbox
                    y2 = y2 - offset_bbox
                    roi = depth_frame[y1:y2, x1:x2].astype(np.float32) * depth_multiplier
                    max_depth_meters = 0.25
                    valid_mask = (roi > 0.1) & (roi < max_depth_meters)

                    if np.any(valid_mask):
                        self.filter_count+=1
                        # --- Compute center of bbox ---
                        cx_center = int(x1 + w / 2)
                        cy_center = int(y1 + h / 2)

                        # Clamp to image bounds
                        # cy_center = np.clip(cy_center, 0, depth_frame.shape[0]-1)
                        # cx_center = np.clip(cx_center, 0, depth_frame.shape[1]-1)

                        depth_center = depth_frame[cy_center, cx_center] * depth_multiplier

                        # --- Check if center depth is valid ---
                        if 0.1 < depth_center < max_depth_meters:
                            cx_final, cy_final = cx_center, cy_center
                            chosen_depth = depth_center
                            reason = "center valid"
                        else:
                            # --- Search nearby valid pixel if center invalid ---
                            search_radius = 10  # pixels
                            roi_masked = np.where(valid_mask, roi, np.inf)
                            # Find pixel in ROI closest in depth to the valid range (frontmost valid)
                            min_idx = np.unravel_index(np.argmin(roi_masked), roi_masked.shape)
                            cy_final = y1 + min_idx[0]
                            cx_final = x1 + min_idx[1]
                            chosen_depth = roi_masked[min_idx]
                            reason = "closest valid replacement"

                        self.get_logger().info(f"Using {reason} pixel at ({cx_final},{cy_final}) depth={chosen_depth:.3f} m")

                        # --- Filter for temporal stability ---
                        self.cx_history = np.roll(self.cx_history, -1)
                        self.cx_history[-1] = cx_final
                        self.cy_history = np.roll(self.cy_history, -1)
                        self.cy_history[-1] = cy_final

                        if self.filter_count >= self.filter_thres:
                            filtered_cx = int(np.mean(self.cx_history))
                            filtered_cy = int(np.mean(self.cy_history))

                            # --- Project to 3D ---
                            ray = self.cam_model.projectPixelTo3dRay((filtered_cx, filtered_cy))
                            X = ray[0] * chosen_depth
                            Y = ray[1] * chosen_depth
                            Z = ray[2] * chosen_depth
                            self.stem_pos_cam_frame = [X, Y, Z]
                            self.get_logger().info(f"3D grasp pos: X={X:.2f}, Y={Y:.2f}, Z={Z:.2f} m")

                            cv2.circle(rgb_frame, (filtered_cx, filtered_cy), 5, (255, 0, 0), -1)
                            stem_pos = Point(x=X, y=Y, z=Z)
                            self.publisher.publish(stem_pos)    
                            self.start_detecting = False
                            self.filter_count = 0
                    else:
                        self.get_logger().info("No valid depths in ROI")                
                # if label == "stem" and h>200:# and self.start_detecting:                
                #     y1 = y1 + offset_bbox
                #     y2 = y2 - offset_bbox
                #     roi = depth_frame[y1:y2, x1:x2]*depth_multiplier
                #     roi = roi.astype(np.float32)
                #     max_depth_meters = 0.25

                #     valid_mask = (roi > 0.1) & (roi < max_depth_meters)
                #     valid_depths = roi[valid_mask]

                #     if valid_depths.size > 0:
                #         self.filter_count+=1
                #         mean_depth = np.mean(valid_depths)
                #         diff = np.abs(roi - mean_depth)
                #         diff[~valid_mask] = np.inf  # mask out invalid depths
                #         # min_idx = np.unravel_index(np.argmin(diff), diff.shape)  # (row, col)
                #         roi_masked = np.where(valid_mask, roi, np.inf)
                #         min_idx = np.unravel_index(np.argmin(roi_masked), roi_masked.shape)
                #         # min_idx = np.unravel_index(np.argmin(roi, where=valid_mask, initial=np.inf), roi.shape)

                #         # Convert local ROI index to full image coordinates
                #         cy = y1 + min_idx[0]
                #         cx = x1 + min_idx[1]

                #         self.cx_history = np.roll(self.cx_history, -1)
                #         self.cx_history[-1] = cx

                #         self.cy_history = np.roll(self.cy_history, -1)
                #         self.cy_history[-1] = cy
                #         # coords_x.append(cx)
                #         # coords_y.append(cy)
                #         if self.filter_count>=self.filter_thres:
                            
                #             filtered_cx = int(np.mean(self.cx_history))
                #             filtered_cy = int(np.mean(self.cy_history))
                #             # coords_x = [filtered_cx]
                #             # coords_y = [filtered_cy]
                #             # DEBUG
                #             # filtered_cx = int(x1 + (x2-x1)/2)
                #             # filtered_cy = int(y1 + (y2-y1)/2)

                #             ray = self.cam_model.projectPixelTo3dRay((filtered_cx, filtered_cy))
                #             X = ray[0] * mean_depth
                #             Y = ray[1] * mean_depth
                #             Z = ray[2] * mean_depth
                #             self.stem_pos_cam_frame = [X, Y, Z]
                        
                #             self.get_logger().info(f"3D pos for grasping: X={X:.2f}, Y={Y:.2f}, Z={Z:.2f} m")
    
                #             cv2.circle(rgb_frame, (filtered_cx, filtered_cy), 4, (255, 0, 0), -1)                            
                #     else:                        
                #         self.get_logger().info("No stems in front")

                cv2.rectangle(rgb_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)                
                cv2.putText(
                    rgb_frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2
                )
        # Show the frame in a window
        cv2.imshow("YOLO Detections", rgb_frame)
        cv2.waitKey(1)  # Needed to update the OpenCV window

    def get_grasping_cb(self, msg):
        self.start_detecting = True        
        self.filter_count = 0
        self.cx_history[:] = 0
        self.cy_history[:] = 0
        self.get_logger().info("Starting grasping perception")    
       
        # if len(self.stem_pos_cam_frame)>0:
        #     X = self.stem_pos_cam_frame[0]
        #     Y = self.stem_pos_cam_frame[1]
        #     Z = self.stem_pos_cam_frame[2]
        #     stem_pos = Point()   
        #     stem_pos.x = X
        #     stem_pos.y = Y
        #     stem_pos.z = Z                 
        #     self.publisher.publish(stem_pos)    

        #     self.start_detecting = False
        #     self.filter_count = 0

def main(args=None):
    # print("HERE")
    rclpy.init(args=args)
    node = stemDetector()
    try:
        rclpy.spin(node)
    except Exception as e:
        import traceback
        traceback.print_exc()
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == "__main__":
    main()        