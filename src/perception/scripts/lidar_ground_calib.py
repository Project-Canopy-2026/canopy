#!/usr/bin/env python3
"""
Estimate LiDAR roll, pitch and height above ground from a ground-plane fit.

Park the robot on flat, open ground, then run:
    python3 src/perception/scripts/lidar_ground_calib.py [--topic /velodyne_points] [--scans 10]

Angles use the same convention as `static_transform_publisher` (--roll/--pitch) and
Patchwork++'s lidar_roll_deg / lidar_pitch_deg: rotation = Rz(yaw) * Ry(pitch) * Rx(roll),
positive pitch = sensor tilted downwards. Yaw is not observable from the ground plane.
"""
import argparse

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def cloud_to_xyz(msg):
    pts = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    if not isinstance(pts, np.ndarray):  # older sensor_msgs_py returns a generator
        pts = np.array(list(pts), dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32)])
    return np.column_stack([pts["x"], pts["y"], pts["z"]]).astype(np.float64)


def up_vector_in_sensor(roll, pitch):
    """World 'up' expressed in the sensor frame for the given roll/pitch (radians)."""
    return np.array([-np.sin(pitch), np.sin(roll) * np.cos(pitch), np.cos(roll) * np.cos(pitch)])


def ransac_plane(pts, nominal_up, max_tilt_rad, inlier_thresh, iters, rng):
    """Fit n.p + d = 0 with n pointing up and within max_tilt of nominal_up. Returns (n, d, inlier_mask)."""
    idx = rng.integers(0, len(pts), size=(iters, 3))
    p0, p1, p2 = pts[idx[:, 0]], pts[idx[:, 1]], pts[idx[:, 2]]
    normals = np.cross(p1 - p0, p2 - p0)
    norms = np.linalg.norm(normals, axis=1)
    valid = norms > 1e-6
    normals[valid] /= norms[valid, None]
    normals *= np.sign(normals @ nominal_up)[:, None]  # orient upwards
    valid &= (normals @ nominal_up) > np.cos(max_tilt_rad)  # reject walls / steep planes
    if not valid.any():
        return None

    normals, p0 = normals[valid], p0[valid]
    d = -np.sum(normals * p0, axis=1)
    counts = (np.abs(pts @ normals.T + d) < inlier_thresh).sum(axis=0)
    best = np.argmax(counts)
    inliers = np.abs(pts @ normals[best] + d[best]) < inlier_thresh

    # Refine with a least-squares (SVD) fit on the inliers.
    ground = pts[inliers]
    centroid = ground.mean(axis=0)
    n = np.linalg.svd(ground - centroid, full_matrices=False)[2][-1]
    n *= np.sign(n @ nominal_up)
    d = -n @ centroid
    inliers = np.abs(pts @ n + d) < inlier_thresh
    return n, d, inliers


class GroundCalib(Node):
    def __init__(self, args):
        super().__init__("lidar_ground_calib")
        self.args = args
        self.rng = np.random.default_rng(0)
        self.nominal_up = up_vector_in_sensor(0.0, np.radians(args.nominal_pitch_deg))
        self.buffer = []
        self.results = []
        self.create_subscription(PointCloud2, args.topic, self.cloud_cb, qos_profile_sensor_data)
        self.get_logger().info(f"Listening on {args.topic}; fitting every {args.scans} scans. Ctrl+C to stop.")

    def cloud_cb(self, msg):
        pts = cloud_to_xyz(msg)
        r = np.linalg.norm(pts, axis=1)
        self.buffer.append(pts[(r > self.args.min_range) & (r < self.args.max_range)])
        if len(self.buffer) < self.args.scans:
            return

        pts = np.vstack(self.buffer)
        self.buffer.clear()
        if len(pts) > self.args.max_points:
            pts = pts[self.rng.choice(len(pts), self.args.max_points, replace=False)]
        if len(pts) < 100:
            self.get_logger().warn("Too few points in range; check the topic and --min/--max-range.")
            return

        fit = ransac_plane(pts, self.nominal_up, np.radians(self.args.max_tilt_deg),
                           self.args.inlier_thresh, self.args.iters, self.rng)
        if fit is None:
            self.get_logger().warn("No plane found near the nominal orientation; try a larger --max-tilt-deg.")
            return

        n, d, inliers = fit
        roll = np.degrees(np.arctan2(n[1], n[2]))
        pitch = np.degrees(np.arcsin(np.clip(-n[0], -1.0, 1.0)))
        height = d  # distance from sensor origin to the plane (sensor is above it)
        rms = np.sqrt(np.mean((pts[inliers] @ n + d) ** 2))
        self.results.append((roll, pitch, height))

        res = np.array(self.results)
        mean, std = res.mean(axis=0), res.std(axis=0)
        print(
            f"[fit {len(res):3d}] roll {roll:+6.2f} deg  pitch {pitch:+6.2f} deg  height {height:5.3f} m  "
            f"| inliers {inliers.mean() * 100:4.1f}%  rms {rms * 100:4.1f} cm  "
            f"|| mean roll {mean[0]:+6.2f}±{std[0]:.2f}  pitch {mean[1]:+6.2f}±{std[1]:.2f}  "
            f"height {mean[2]:5.3f}±{std[2]:.3f}",
            flush=True,
        )


def main():
    rclpy.init()
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--topic", default="/velodyne_points")
    parser.add_argument("--scans", type=int, default=10, help="scans accumulated per fit")
    parser.add_argument("--nominal-pitch-deg", type=float, default=27.0, help="initial guess, +ve = tilted down")
    parser.add_argument("--max-tilt-deg", type=float, default=15.0, help="max plane deviation from the nominal guess")
    parser.add_argument("--min-range", type=float, default=1.0, help="m; skips returns off the robot body")
    parser.add_argument("--max-range", type=float, default=10.0, help="m")
    parser.add_argument("--inlier-thresh", type=float, default=0.05, help="m")
    parser.add_argument("--iters", type=int, default=300, help="RANSAC iterations")
    parser.add_argument("--max-points", type=int, default=20000, help="subsample size per fit")
    args = parser.parse_args(remove_ros_args()[1:])

    node = GroundCalib(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
