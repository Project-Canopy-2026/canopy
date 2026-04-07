#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from xarm_msgs.msg import RobotMsg
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import json
import os
import yaml
from pathlib import Path

SAVE_DIR = "/tmp/hand_eye_charuco"
POSES_FILE = os.path.join(SAVE_DIR, "poses.json")
IMAGES_DIR = os.path.join(SAVE_DIR, "images")
os.makedirs(IMAGES_DIR, exist_ok=True)

# Load ChArUco board configuration
config_path = Path(__file__).parent.parent / "easy_handeye2" / "easy_handeye2" / "config" / "charuco_board.yaml"
with open(config_path, 'r') as f:
    charuco_config = yaml.safe_load(f)['charuco']

# ChArUco board parameters
CHARUCO_ROWS = charuco_config['squares_y']  # vertical squares
CHARUCO_COLS = charuco_config['squares_x']  # horizontal squares
SQUARE_LENGTH = charuco_config['square_length']   # meters
MARKER_LENGTH = charuco_config['marker_length']   # meters

# Convert string to cv2 aruco dictionary constant
ARUCO_DICT_MAP = {
    "DICT_4X4_50": aruco.DICT_4X4_50,
    "DICT_4X4_100": aruco.DICT_4X4_100,
    "DICT_4X4_250": aruco.DICT_4X4_250,
    "DICT_4X4_1000": aruco.DICT_4X4_1000,
    "DICT_5X5_50": aruco.DICT_5X5_50,
    "DICT_5X5_100": aruco.DICT_5X5_100,
    "DICT_5X5_250": aruco.DICT_5X5_250,
    "DICT_5X5_1000": aruco.DICT_5X5_1000,
    "DICT_6X6_50": aruco.DICT_6X6_50,
    "DICT_6X6_100": aruco.DICT_6X6_100,
    "DICT_6X6_250": aruco.DICT_6X6_250,
    "DICT_6X6_1000": aruco.DICT_6X6_1000,
    "DICT_7X7_50": aruco.DICT_7X7_50,
    "DICT_7X7_100": aruco.DICT_7X7_100,
    "DICT_7X7_250": aruco.DICT_7X7_250,
    "DICT_7X7_1000": aruco.DICT_7X7_1000,
}

ARUCO_DICT = aruco.Dictionary_get(ARUCO_DICT_MAP.get(charuco_config['aruco_dict']))
if ARUCO_DICT is None:
    raise ValueError(f"Unknown ArUco dictionary: {charuco_config['aruco_dict']}")

BOARD = aruco.CharucoBoard_create(
    CHARUCO_COLS, CHARUCO_ROWS, SQUARE_LENGTH, MARKER_LENGTH, ARUCO_DICT
)

class CharucoCollector(Node):
    def __init__(self):
        super().__init__('charuco_collector')
        self.bridge = CvBridge()
        self.robot_pose = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.poses_data = []
        self.pose_idx = 0

        # Subscribers
        self.create_subscription(RobotMsg, '/xarm/robot_states', self.robot_cb, 10)
        self.create_subscription(Image, '/camera/camera/color/image_rect_raw', self.image_cb, 10)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.cam_info_cb, 10)

        self.get_logger().info("ChArUco data collector initialized.")

    def cam_info_cb(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3,3)
        self.dist_coeffs = np.array(msg.d)

    def robot_cb(self, msg):
        self.robot_pose = msg

    def image_cb(self, msg):
        if self.robot_pose is None or self.camera_matrix is None:
            return

        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco.detectMarkers(gray, ARUCO_DICT)
        if ids is None or len(ids) == 0:
            return

        ret, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
            corners, ids, gray, BOARD
        )

        if charuco_corners is None or len(charuco_corners) < 4:
            return  # too few corners, skip

        # SolvePnP
        ret, rvec, tvec = cv2.solvePnP(
            BOARD.chessboardCorners,
            charuco_corners,
            self.camera_matrix,
            self.dist_coeffs
        )

        if not ret:
            return

        # Convert robot pose to 4x4
        x = self.robot_pose.position.x
        y = self.robot_pose.position.y
        z = self.robot_pose.position.z
        qx = self.robot_pose.orientation.x
        qy = self.robot_pose.orientation.y
        qz = self.robot_pose.orientation.z
        qw = self.robot_pose.orientation.w
        R = self.quaternion_to_rot_matrix(qx,qy,qz,qw)
        T_robot = np.eye(4)
        T_robot[:3,:3] = R
        T_robot[:3,3] = [x,y,z]

        # Save image and data
        img_file = os.path.join(IMAGES_DIR, f"img_{self.pose_idx}.png")
        cv2.imwrite(img_file, img)

        self.poses_data.append({
            "T_robot": T_robot.tolist(),
            "rvec_cam": rvec.tolist(),
            "tvec_cam": tvec.tolist(),
            "image": img_file
        })
        self.pose_idx += 1
        self.get_logger().info(f"Collected pose {self.pose_idx}")

        # Save periodically
        if self.pose_idx % 5 == 0:
            with open(POSES_FILE,'w') as f:
                json.dump(self.poses_data, f, indent=2)

    def quaternion_to_rot_matrix(self, qx,qy,qz,qw):
        R = np.array([
            [1-2*qy*qy-2*qz*qz, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw],
            [2*qx*qy+2*qz*qw, 1-2*qx*qx-2*qz*qz, 2*qy*qz-2*qx*qw],
            [2*qx*qz-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*qx*qx-2*qy*qy]
        ])
        return R

def main(args=None):
    rclpy.init(args=args)
    node = CharucoCollector()
    rclpy.spin(node)
    rclpy.shutdown()

    # Save final data
    with open(POSES_FILE,'w') as f:
        json.dump(node.poses_data, f, indent=2)
    node.get_logger().info(f"Saved {len(node.poses_data)} poses to {POSES_FILE}")

if __name__ == '__main__':
    main()