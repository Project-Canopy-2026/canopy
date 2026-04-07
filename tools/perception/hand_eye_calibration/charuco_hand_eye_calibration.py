#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import json
import numpy as np
import cv2

POSES_FILE = "/tmp/hand_eye_charuco/poses.json"

def main(args=None):
    rclpy.init(args=args)
    node = Node('charuco_hand_eye_calibration')

    with open(POSES_FILE,'r') as f:
        poses_data = json.load(f)

    R_gripper2base = []
    t_gripper2base = []
    R_target2cam = []
    t_target2cam = []

    for i in range(len(poses_data)-1):
        # Relative gripper motion
        T1 = np.array(poses_data[i]["T_robot"])
        T2 = np.array(poses_data[i+1]["T_robot"])
        A = np.linalg.inv(T1) @ T2
        R_gripper2base.append(A[:3,:3])
        t_gripper2base.append(A[:3,3])

        # Relative camera motion
        rvec1 = np.array(poses_data[i]["rvec_cam"])
        tvec1 = np.array(poses_data[i]["tvec_cam"])
        R1, _ = cv2.Rodrigues(rvec1)
        T1_cam = np.eye(4)
        T1_cam[:3,:3] = R1
        T1_cam[:3,3] = tvec1.flatten()

        rvec2 = np.array(poses_data[i+1]["rvec_cam"])
        tvec2 = np.array(poses_data[i+1]["tvec_cam"])
        R2, _ = cv2.Rodrigues(rvec2)
        T2_cam = np.eye(4)
        T2_cam[:3,:3] = R2
        T2_cam[:3,3] = tvec2.flatten()

        B = T1_cam @ np.linalg.inv(T2_cam)
        R_target2cam.append(B[:3,:3])
        t_target2cam.append(B[:3,3])

    # Hand-eye calibration
    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_gripper2base, t_gripper2base,
        R_target2cam, t_target2cam,
        method=cv2.CALIB_HAND_EYE_TSAI
    )

    node.get_logger().info(f"Rotation (gripper -> camera):\n{R_cam2gripper}")
    node.get_logger().info(f"Translation (gripper -> camera):\n{t_cam2gripper}")
    rclpy.shutdown()

if __name__ == '__main__':
    main()