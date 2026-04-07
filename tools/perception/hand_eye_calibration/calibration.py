#!/usr/bin/env python3
"""
Eye-in-hand calibration: solves for T_hand_to_cam (camera pose relative to end-effector).

Theory (AX = XB):
  A_i = T_base_to_hand_i^{-1} @ T_base_to_hand_j   (relative EEF motion, from robot FK / TF)
  B_i = T_target_to_cam_i @ T_target_to_cam_j^{-1}  (relative target motion in camera frame)
  X   = T_hand_to_cam                                 (what we're solving for)

Usage:
  1. Mount a ChArUco board in a fixed location visible to the camera.
  2. Move the robot arm to N >= 15 distinct poses, pressing SPACE each time to capture.
  3. Press 'q' to run calibration and save the result.

Requirements:
  pip install opencv-contrib-python numpy pyyaml
  ROS2 + tf2_ros running with the robot's TF tree.
"""

import sys
import time
import yaml
from pathlib import Path

import cv2
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener


# ---------------------------------------------------------------------------
# ChArUco board configuration — loaded from config file
# ---------------------------------------------------------------------------
# Load configuration from charuco_board.yaml
config_path = Path(__file__).parent.parent / "easy_handeye2" / "easy_handeye2" / "config" / "charuco_board.yaml"
with open(config_path, 'r') as f:
    charuco_config = yaml.safe_load(f)['charuco']

SQUARES_X      = charuco_config['squares_x']      # number of chessboard squares in X
SQUARES_Y      = charuco_config['squares_y']      # number of chessboard squares in Y
SQUARE_LENGTH  = charuco_config['square_length']  # metres
MARKER_LENGTH  = charuco_config['marker_length']  # metres
ARUCO_DICT_STR = charuco_config['aruco_dict']     # aruco dictionary string

# Convert string to cv2 aruco dictionary constant
ARUCO_DICT_MAP = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
}

ARUCO_DICT = ARUCO_DICT_MAP.get(ARUCO_DICT_STR)
if ARUCO_DICT is None:
    raise ValueError(f"Unknown ArUco dictionary: {ARUCO_DICT_STR}")

# ---------------------------------------------------------------------------
# TF frame names — adjust to match your robot's TF tree
# ---------------------------------------------------------------------------
BASE_FRAME = "link_base"         # robot base
EEF_FRAME  = "link_eef"         # end-effector / camera mount

# ---------------------------------------------------------------------------
# ROS2 topics — matches the RealSense topics used in pot_detection_node.py
# ---------------------------------------------------------------------------
IMAGE_TOPIC       = "/camera/camera/color/image_rect_raw"
CAMERA_INFO_TOPIC = "/camera/camera/color/camera_info"

# ---------------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------------
OUTPUT_FILE  = Path(__file__).parent / "hand_eye_result.yaml"
SAMPLES_FILE = Path(__file__).parent / "hand_eye_samples.json"


def mat_from_tf(tf_msg):
    """Convert a TransformStamped into a 4x4 numpy homogeneous matrix."""
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    # Quaternion -> rotation matrix (Hamilton convention)
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = [t.x, t.y, t.z]
    return T


def decompose(T):
    """Return (rvec, tvec) from a 4x4 homogeneous matrix (for cv2.calibrateHandEye)."""
    R = T[:3, :3]
    t = T[:3,  3]
    rvec, _ = cv2.Rodrigues(R)
    return rvec.flatten(), t


def save_result(T_hand_to_cam: np.ndarray):
    R = T_hand_to_cam[:3, :3]
    t = T_hand_to_cam[:3, 3]
    rvec, _ = cv2.Rodrigues(R)
    data = {
        "T_hand_to_cam": {
            "rows": 4, "cols": 4, "dt": "d",
            "data": T_hand_to_cam.flatten().tolist(),
        },
        "rotation_matrix": {
            "rows": 3, "cols": 3, "dt": "d",
            "data": R.flatten().tolist(),
        },
        "rotation_vector": rvec.flatten().tolist(),
        "translation_vector": t.tolist(),
    }
    with open(OUTPUT_FILE, "w") as f:
        yaml.dump(data, f, default_flow_style=False)
    print(f"\n[✓] Result saved to {OUTPUT_FILE}")


class HandEyeCalibrator(Node):
    def __init__(self):
        super().__init__("hand_eye_calibrator")

        # ChArUco detector setup (legacy OpenCV < 4.7 API)
        self.aruco_dict  = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
        self.board_params = cv2.aruco.DetectorParameters_create()
        self.board = cv2.aruco.CharucoBoard_create(
            SQUARES_X, SQUARES_Y, SQUARE_LENGTH, MARKER_LENGTH, self.aruco_dict
        )

        # TF
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ROS subscriptions
        self.bridge      = CvBridge()
        self.camera_matrix    = None
        self.dist_coeffs      = None
        self.latest_frame     = None
        self._frame_lock      = threading.Lock()

        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self._camera_info_cb, 1)
        self.create_subscription(Image,      IMAGE_TOPIC,       self._image_cb,       1)

        # Collected calibration samples
        self.R_gripper2base = []   # robot: base -> EEF rotations  (3x3 each)
        self.t_gripper2base = []
        self.R_target2cam   = []   # camera: target -> cam rotations (3x3 each)
        self.t_target2cam   = []

        self._load_samples()

        self.get_logger().info(
            f"Waiting for camera info on {CAMERA_INFO_TOPIC} ..."
        )

    # ------------------------------------------------------------------
    def _camera_info_cb(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs   = np.array(msg.d)
            self.get_logger().info("Camera intrinsics received.")

    def _image_cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        with self._frame_lock:
            self.latest_frame = frame

    # ------------------------------------------------------------------
    def _detect_charuco(self, frame):
        """
        Detect ChArUco corners in *frame* and estimate the board pose.
        Returns (rvec, tvec) or (None, None) if detection fails.
        Also returns the annotated frame for display.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.board_params
        )

        vis = frame.copy()
        if ids is None or len(ids) == 0:
            return None, None, vis

        ret, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
            corners, ids, gray, self.board
        )
        if charuco_corners is None or charuco_ids is None or len(charuco_ids) < 4:
            return None, None, vis

        cv2.aruco.drawDetectedCornersCharuco(vis, charuco_corners, charuco_ids)
        ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
            charuco_corners, charuco_ids, self.board,
            self.camera_matrix, self.dist_coeffs, None, None
        )
        if ok:
            cv2.drawFrameAxes(vis, self.camera_matrix, self.dist_coeffs,
                              rvec, tvec, SQUARE_LENGTH * 2)
            return rvec.flatten(), tvec.flatten(), vis

        return None, None, vis

    def _get_eef_pose(self):
        """Lookup the current base→EEF transform from TF2."""
        try:
            tf = self.tf_buffer.lookup_transform(
                BASE_FRAME, EEF_FRAME,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
            return mat_from_tf(tf)
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None

    # ------------------------------------------------------------------
    def capture_sample(self) -> bool:
        """Try to capture one calibration sample. Returns True on success."""
        if self.latest_frame is None or self.camera_matrix is None:
            print("[!] No image or camera info yet.")
            return False

        rvec_t2c, tvec_t2c, _ = self._detect_charuco(self.latest_frame)
        if rvec_t2c is None:
            print("[!] ChArUco board not detected in current frame.")
            return False

        T_base2eef = self._get_eef_pose()
        if T_base2eef is None:
            print("[!] Could not get end-effector pose from TF.")
            return False

        # cv2.calibrateHandEye wants 3x3 rotation matrices
        R_g2b = T_base2eef[:3, :3]          # already a rotation matrix
        t_g2b = T_base2eef[:3, 3]
        R_t2c, _ = cv2.Rodrigues(rvec_t2c)  # rvec → 3x3

        # Validate rotation matrices before storing
        if abs(np.linalg.det(R_g2b) - 1.0) > 0.01:
            print("[!] Bad EEF rotation matrix (det != 1), skipping.")
            return False
        if abs(np.linalg.det(R_t2c) - 1.0) > 0.01:
            print("[!] Bad board rotation matrix (det != 1), skipping.")
            return False

        self.R_gripper2base.append(R_g2b)
        self.t_gripper2base.append(t_g2b.reshape(3, 1))
        self.R_target2cam.append(R_t2c)
        self.t_target2cam.append(tvec_t2c.reshape(3, 1))

        n = len(self.R_gripper2base)
        print(f"[✓] Sample {n} captured.")
        self._save_samples()
        return True

    def run_calibration(self):
        """Solve AX=XB and print / save the result."""
        n = len(self.R_gripper2base)
        if n < 3:
            print(f"[!] Need at least 3 samples (have {n}). Collect more poses.")
            return

        print(f"\nRunning hand-eye calibration with {n} samples ...")
        methods = {
            "TSAI":        cv2.CALIB_HAND_EYE_TSAI,
            "PARK":        cv2.CALIB_HAND_EYE_PARK,
            "HORAUD":      cv2.CALIB_HAND_EYE_HORAUD,
            "ANDREFF":     cv2.CALIB_HAND_EYE_ANDREFF,
            "DANIILIDIS":  cv2.CALIB_HAND_EYE_DANIILIDIS,
        }
        results = {}
        for name, method in methods.items():
            try:
                R, t = cv2.calibrateHandEye(
                    self.R_gripper2base, self.t_gripper2base,
                    self.R_target2cam,   self.t_target2cam,
                    method=method,
                )
                T = np.eye(4)
                T[:3, :3] = R
                T[:3,  3] = t.flatten()
                results[name] = T
            except cv2.error as e:
                print(f"[!] {name} failed: {e}")

        if not results:
            print("[!] All methods failed. Try collecting more diverse poses (vary rotation, not just position).")
            return None

        # Use TSAI as default if available, else first success
        best = "TSAI" if "TSAI" in results else next(iter(results))
        print("\n--- T_hand_to_cam (camera relative to end-effector) ---")
        np.set_printoptions(precision=6, suppress=True)
        for name, T in results.items():
            print(f"\n[{name}]")
            print(T)

        save_result(results[best])
        return results[best]

    def _save_samples(self):
        data = {
            "R_gripper2base": [R.tolist() for R in self.R_gripper2base],
            "t_gripper2base": [t.tolist() for t in self.t_gripper2base],
            "R_target2cam":   [R.tolist() for R in self.R_target2cam],
            "t_target2cam":   [t.tolist() for t in self.t_target2cam],
        }
        with open(SAMPLES_FILE, "w") as f:
            import json
            json.dump(data, f, indent=2)

    def _load_samples(self):
        if not SAMPLES_FILE.exists():
            return
        import json
        with open(SAMPLES_FILE) as f:
            data = json.load(f)
        self.R_gripper2base = [np.array(R) for R in data["R_gripper2base"]]
        self.t_gripper2base = [np.array(t) for t in data["t_gripper2base"]]
        self.R_target2cam   = [np.array(R) for R in data["R_target2cam"]]
        self.t_target2cam   = [np.array(t) for t in data["t_target2cam"]]
        print(f"[✓] Loaded {len(self.R_gripper2base)} existing samples from {SAMPLES_FILE}")

    # ------------------------------------------------------------------
    # def run_interactive(self):
    #     """
    #     Live preview loop.
    #       SPACE  — capture current pose
    #       c      — run calibration (needs ≥ 3 samples)
    #       q/ESC  — quit
    #     """
    #     print("\n=== Hand-Eye Calibration ===")
    #     print(f"  Base frame : {BASE_FRAME}")
    #     print(f"  EEF frame  : {EEF_FRAME}")
    #     print(f"  Camera     : {IMAGE_TOPIC}")
    #     print("\nControls:")
    #     print("  SPACE  — capture sample at current robot pose")
    #     print("  c      — run calibration")
    #     print("  x      — clear all samples")
    #     print("  q/ESC  — quit\n")

    #     cv2.namedWindow("Hand-Eye Calibration", cv2.WINDOW_NORMAL)

    #     while rclpy.ok():
    #         with self._frame_lock:
    #             frame = self.latest_frame.copy() if self.latest_frame is not None else None

    #         if frame is None:
    #             cv2.waitKey(30)
    #             continue

    #         _, _, vis = self._detect_charuco(frame)
    #         n = len(self.R_gripper2base)
    #         cv2.putText(vis, f"Samples: {n}  |  SPACE=capture  c=calibrate  x=clear  q=quit",
    #                     (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    #         cv2.imshow("Hand-Eye Calibration", vis)

    #         key = cv2.waitKey(30)
    #         if key == ord(' '):
    #             self.capture_sample()
    #         elif key == ord('c'):
    #             self.run_calibration()
    #         elif key == ord('x'):
    #             self.R_gripper2base.clear()
    #             self.t_gripper2base.clear()
    #             self.R_target2cam.clear()
    #             self.t_target2cam.clear()
    #             if SAMPLES_FILE.exists():
    #                 SAMPLES_FILE.unlink()
    #             print("[✓] All samples cleared.")
    #         elif key in (ord('q'), 27):   # q or ESC
    #             break

    #     cv2.destroyAllWindows()

    def run_interactive(self):
        """
        Fully robust interactive loop for hand-eye calibration.

        Controls:
        SPACE  — capture sample at current robot pose
        c      — run calibration (needs ≥ 3 samples)
        x      — clear all samples
        q/ESC  — quit
        """

        print("\n=== Hand-Eye Calibration ===")
        print(f"  Base frame : {BASE_FRAME}")
        print(f"  EEF frame  : {EEF_FRAME}")
        print(f"  Camera     : {IMAGE_TOPIC}")
        print("\nControls:")
        print("  SPACE  — capture sample at current robot pose")
        print("  c      — run calibration")
        print("  x      — clear all samples")
        print("  q/ESC  — quit\n")

        # Wait for camera info before starting (executor spins in background)
        while self.camera_matrix is None and rclpy.ok():
            print("Waiting for camera info...")
            time.sleep(0.1)

        cv2.namedWindow("Hand-Eye Calibration", cv2.WINDOW_NORMAL)

        while rclpy.ok():
            frame_to_show = None
            with self._frame_lock:
                if self.latest_frame is not None:
                    # Always copy the latest frame
                    frame_to_show = self.latest_frame.copy()

            if frame_to_show is None:
                # Show placeholder if no frame yet
                placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(placeholder, "Waiting for camera frames...",
                            (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.imshow("Hand-Eye Calibration", placeholder)
                if cv2.waitKey(50) & 0xFF in (ord('q'), 27):
                    break
                continue

            # Detect ChArUco board in the current frame
            rvec, tvec, vis = self._detect_charuco(frame_to_show)

            # Overlay status info
            n_samples = len(self.R_gripper2base)
            status_text = "ChArUco detected" if rvec is not None else "ChArUco NOT detected"
            cv2.putText(vis, status_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 255, 0) if rvec is not None else (0, 0, 255), 2)

            cv2.putText(vis,
                        f"Samples: {n_samples} | SPACE=capture  c=calibrate  x=clear  q=quit",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            cv2.imshow("Hand-Eye Calibration", vis)

            key = cv2.waitKey(30) & 0xFF

            if key == ord(' '):
                success = self.capture_sample()
                if not success:
                    print("[!] Failed to capture sample. Make sure board is visible and TF is available.")
            elif key == ord('c'):
                self.run_calibration()
            elif key == ord('x'):
                self.R_gripper2base.clear()
                self.t_gripper2base.clear()
                self.R_target2cam.clear()
                self.t_target2cam.clear()
                if SAMPLES_FILE.exists():
                    SAMPLES_FILE.unlink()
                print("[✓] All samples cleared.")
            elif key in (ord('q'), 27):  # q or ESC
                break

        cv2.destroyAllWindows()


def main():
    rclpy.init()
    node = HandEyeCalibrator()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        node.run_interactive()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
