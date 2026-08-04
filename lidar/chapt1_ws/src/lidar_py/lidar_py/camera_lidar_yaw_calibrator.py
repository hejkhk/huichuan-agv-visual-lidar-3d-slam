#!/usr/bin/env python3
"""Estimate camera yaw correction by matching one wall in depth and 2D LiDAR."""

import math
import random

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener

from lidar_py.calibration_env import (
    mark_calibration_restart_required,
    update_env_file,
)


def quaternion_matrix(q):
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1.0e-12:
        return np.eye(3)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def transform_xy(points, transform):
    q = transform.transform.rotation
    rotation = quaternion_matrix((q.x, q.y, q.z, q.w))
    translation = np.array([
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z,
    ])
    xyz = np.column_stack((points, np.zeros(len(points))))
    return (xyz @ rotation.T + translation)[:, :2]


def wrap_half_pi(angle):
    while angle > math.pi / 2.0:
        angle -= math.pi
    while angle < -math.pi / 2.0:
        angle += math.pi
    return angle


def fit_dominant_line(points, threshold, iterations=500):
    if len(points) < 80:
        return None
    best = None
    best_count = 0
    for _ in range(iterations):
        first, second = points[random.sample(range(len(points)), 2)]
        direction = second - first
        length = np.linalg.norm(direction)
        if length < 0.20:
            continue
        direction /= length
        normal = np.array([-direction[1], direction[0]])
        distances = np.abs((points - first) @ normal)
        mask = distances < threshold
        count = int(np.count_nonzero(mask))
        if count > best_count:
            best_count = count
            best = mask
    if best is None or best_count < max(60, int(0.12 * len(points))):
        return None
    inliers = points[best]
    centroid = inliers.mean(axis=0)
    _, singular, vh = np.linalg.svd(inliers - centroid, full_matrices=False)
    direction = vh[0]
    extent = np.ptp((inliers - centroid) @ direction)
    if extent < 0.60 or singular[0] < 4.0 * max(singular[1], 1.0e-9):
        return None
    angle = wrap_half_pi(math.atan2(direction[1], direction[0]))
    normal = np.array([-math.sin(angle), math.cos(angle)])
    wall_distance = abs(float(normal @ centroid))
    return angle, best_count / len(points), extent, wall_distance


class CameraLidarYawCalibrator(Node):
    def __init__(self):
        super().__init__("camera_lidar_yaw_calibrator")
        self.declare_parameter("cloud_topic", "/local_highres_cloud_v21")
        self.declare_parameter("scan_topic", "/scan_timed_v2")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("current_camera_yaw_deg", 0.0)
        self.declare_parameter("frames", 40)
        self.declare_parameter("x_min", 0.60)
        self.declare_parameter("x_max", 3.50)
        self.declare_parameter("y_abs_max", 1.60)
        self.declare_parameter("z_min", 0.15)
        self.declare_parameter("z_max", 1.60)
        self.declare_parameter("line_threshold", 0.035)
        self.declare_parameter("min_inlier_ratio", 0.18)
        self.declare_parameter("min_wall_span", 0.80)
        self.declare_parameter("max_abs_correction_deg", 15.0)
        self.declare_parameter("max_wall_distance_difference", 0.30)
        self.declare_parameter("max_split_correction_difference_deg", 1.5)
        self.declare_parameter("auto_write", False)
        self.declare_parameter("env_file", "")

        self._base_frame = str(self.get_parameter("base_frame").value)
        self._current_yaw_deg = float(
            self.get_parameter("current_camera_yaw_deg").value)
        self._target_frames = int(self.get_parameter("frames").value)
        self._x_min = float(self.get_parameter("x_min").value)
        self._x_max = float(self.get_parameter("x_max").value)
        self._y_abs_max = float(self.get_parameter("y_abs_max").value)
        self._z_min = float(self.get_parameter("z_min").value)
        self._z_max = float(self.get_parameter("z_max").value)
        self._threshold = float(self.get_parameter("line_threshold").value)
        self._min_inlier_ratio = float(
            self.get_parameter("min_inlier_ratio").value)
        self._min_wall_span = float(
            self.get_parameter("min_wall_span").value)
        self._max_abs_correction_deg = float(
            self.get_parameter("max_abs_correction_deg").value)
        self._max_wall_distance_difference = float(
            self.get_parameter("max_wall_distance_difference").value)
        self._max_split_correction_difference_deg = float(
            self.get_parameter("max_split_correction_difference_deg").value)
        self._auto_write = bool(self.get_parameter("auto_write").value)
        self._env_file = str(self.get_parameter("env_file").value)
        self._cloud_samples = []
        self._scan_samples = []
        self._finished = False
        self._shutdown_timer = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("cloud_topic").value),
            self._cloud_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self._timer = self.create_timer(0.5, self._try_solve)
        self.get_logger().info(
            "Yaw calibration started. Keep the AGV stationary 1-3 m from one "
            "large flat wall visible to both Gemini2 and LiDAR. The wall need "
            "not be square to the vehicle.")

    def _cloud_callback(self, msg):
        if self._finished or len(self._cloud_samples) >= self._target_frames:
            return
        try:
            values = point_cloud2.read_points_numpy(
                msg, field_names=["x", "y", "z"], skip_nans=True)
        except Exception as exc:
            self.get_logger().warning(f"Cannot read local cloud: {exc}")
            return
        values = np.asarray(values, dtype=np.float64).reshape(-1, 3)
        mask = (
            (values[:, 0] >= self._x_min) &
            (values[:, 0] <= self._x_max) &
            (np.abs(values[:, 1]) <= self._y_abs_max) &
            (values[:, 2] >= self._z_min) &
            (values[:, 2] <= self._z_max)
        )
        points = values[mask, :2]
        if len(points) < 100:
            return
        # One XY point per 2 cm cell prevents a vertical wall from being
        # overweight merely because it contains many height rows.
        cells = np.round(points / 0.02).astype(np.int32)
        _, unique = np.unique(cells, axis=0, return_index=True)
        self._cloud_samples.append(points[unique])

    def _scan_callback(self, msg):
        if self._finished or len(self._scan_samples) >= self._target_frames:
            return
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        valid = (
            np.isfinite(ranges) &
            (ranges >= max(msg.range_min, self._x_min)) &
            (ranges <= min(msg.range_max, self._x_max + 0.8)) &
            (np.abs(angles) <= math.radians(70.0))
        )
        if np.count_nonzero(valid) < 50:
            return
        points = np.column_stack((
            ranges[valid] * np.cos(angles[valid]),
            ranges[valid] * np.sin(angles[valid]),
        ))
        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame, msg.header.frame_id, rclpy.time.Time())
        except Exception:
            return
        self._scan_samples.append(transform_xy(points, transform))

    def _try_solve(self):
        if self._finished:
            return
        if (len(self._cloud_samples) < self._target_frames or
                len(self._scan_samples) < self._target_frames):
            self.get_logger().info(
                f"Collecting wall frames: camera {len(self._cloud_samples)}/"
                f"{self._target_frames}, lidar {len(self._scan_samples)}/"
                f"{self._target_frames}",
                throttle_duration_sec=2.0,
            )
            return
        camera_points = np.concatenate(self._cloud_samples, axis=0)
        lidar_points = np.concatenate(self._scan_samples, axis=0)
        camera_line = fit_dominant_line(camera_points, self._threshold)
        lidar_line = fit_dominant_line(lidar_points, self._threshold)
        if camera_line is None or lidar_line is None:
            self.get_logger().error(
                "No common dominant wall found. Clear nearby clutter and put "
                "one wall 1-3 m in front of both sensors.")
            self._finished = True
            self._shutdown_timer = self.create_timer(1.0, rclpy.shutdown)
            return

        half = self._target_frames // 2
        camera_line_a = fit_dominant_line(
            np.concatenate(self._cloud_samples[:half], axis=0), self._threshold)
        camera_line_b = fit_dominant_line(
            np.concatenate(self._cloud_samples[half:], axis=0), self._threshold)
        lidar_line_a = fit_dominant_line(
            np.concatenate(self._scan_samples[:half], axis=0), self._threshold)
        lidar_line_b = fit_dominant_line(
            np.concatenate(self._scan_samples[half:], axis=0), self._threshold)
        split_lines = (
            camera_line_a, camera_line_b, lidar_line_a, lidar_line_b)
        split_ok = all(line is not None for line in split_lines)
        split_delta_deg = float("inf")
        if split_ok:
            correction_a = wrap_half_pi(lidar_line_a[0] - camera_line_a[0])
            correction_b = wrap_half_pi(lidar_line_b[0] - camera_line_b[0])
            split_delta_deg = abs(math.degrees(
                wrap_half_pi(correction_a - correction_b)))

        correction = wrap_half_pi(lidar_line[0] - camera_line[0])
        calibrated_yaw = self._current_yaw_deg + math.degrees(correction)
        correction_deg = math.degrees(correction)
        wall_distance_difference = abs(camera_line[3] - lidar_line[3])
        self.get_logger().info("=" * 66)
        self.get_logger().info("CAMERA-LIDAR YAW CALIBRATION RESULT")
        self.get_logger().info(
            f"camera wall       : {math.degrees(camera_line[0]):.3f} deg, "
            f"inliers={camera_line[1] * 100.0:.1f}%, span={camera_line[2]:.2f} m, "
            f"distance={camera_line[3]:.2f} m")
        self.get_logger().info(
            f"lidar wall        : {math.degrees(lidar_line[0]):.3f} deg, "
            f"inliers={lidar_line[1] * 100.0:.1f}%, span={lidar_line[2]:.2f} m, "
            f"distance={lidar_line[3]:.2f} m")
        self.get_logger().info(
            f"wall distance diff: {wall_distance_difference:.3f} m")
        self.get_logger().info(
            f"split stability   : {split_delta_deg:.3f} deg")
        self.get_logger().info(
            f"yaw correction    : {correction_deg:+.3f} deg")
        self.get_logger().info(f"CAMERA_YAW_DEG    : {calibrated_yaw:.3f}")
        quality_ok = (
            camera_line[1] >= self._min_inlier_ratio and
            lidar_line[1] >= self._min_inlier_ratio and
            camera_line[2] >= self._min_wall_span and
            lidar_line[2] >= self._min_wall_span and
            abs(correction_deg) <= self._max_abs_correction_deg and
            wall_distance_difference <= self._max_wall_distance_difference and
            split_ok and
            split_delta_deg <= self._max_split_correction_difference_deg
        )
        if self._auto_write and quality_ok:
            try:
                backup = update_env_file(self._env_file, {
                    "CAMERA_YAW_DEG": f"{calibrated_yaw:.3f}",
                    "CAMERA_EXTRINSIC_CALIBRATED": "true",
                })
                self.get_logger().info(
                    f"AUTO-WRITE OK      : {self._env_file}")
                marker = mark_calibration_restart_required(
                    self._env_file, "yaw")
                self.get_logger().warn(
                    f"RESTART REQUIRED   : {marker}")
                if backup:
                    self.get_logger().info(f"BACKUP             : {backup}")
                self.get_logger().info(
                    "Restart mapping to apply the complete camera extrinsic.")
            except Exception as exc:
                self.get_logger().error(
                    f"Calibration was valid but auto-write failed: {exc}")
        elif self._auto_write:
            self.get_logger().error(
                "AUTO-WRITE REJECTED: wall quality/correction did not pass "
                "the safety limits. No configuration was changed. Required: "
                f"|correction|<={self._max_abs_correction_deg:.1f} deg, "
                f"wall distance difference<="
                f"{self._max_wall_distance_difference:.2f} m and split "
                f"stability<={self._max_split_correction_difference_deg:.1f} deg.")
        else:
            self.get_logger().info(
                "Auto-write is disabled. Copy CAMERA_YAW_DEG manually and "
                "set CAMERA_EXTRINSIC_CALIBRATED=true.")
        self.get_logger().info("=" * 66)
        self._finished = True
        self._shutdown_timer = self.create_timer(1.0, rclpy.shutdown)


def main(args=None):
    rclpy.init(args=args)
    node = CameraLidarYawCalibrator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
