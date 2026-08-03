#!/usr/bin/env python3
"""Estimate camera roll, pitch and height from a stationary flat floor."""

import math
import random

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
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


def rotation_align(source, target):
    source = source / np.linalg.norm(source)
    target = target / np.linalg.norm(target)
    cross = np.cross(source, target)
    sine = np.linalg.norm(cross)
    cosine = float(np.clip(np.dot(source, target), -1.0, 1.0))
    if sine < 1.0e-9:
        return np.eye(3) if cosine > 0.0 else np.diag([1.0, -1.0, -1.0])
    axis = cross / sine
    skew = np.array([
        [0.0, -axis[2], axis[1]],
        [axis[2], 0.0, -axis[0]],
        [-axis[1], axis[0], 0.0],
    ])
    return np.eye(3) + skew * sine + (skew @ skew) * (1.0 - cosine)


def matrix_to_rpy(matrix):
    pitch = math.asin(float(np.clip(-matrix[2, 0], -1.0, 1.0)))
    if abs(math.cos(pitch)) > 1.0e-8:
        roll = math.atan2(matrix[2, 1], matrix[2, 2])
        yaw = math.atan2(matrix[1, 0], matrix[0, 0])
    else:
        roll = math.atan2(-matrix[1, 2], matrix[1, 1])
        yaw = 0.0
    return roll, pitch, yaw


class CameraGroundCalibrator(Node):
    def __init__(self):
        super().__init__("camera_ground_calibrator")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/depth/camera_info")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_link_frame", "camera_link")
        self.declare_parameter("frames", 30)
        self.declare_parameter("sample_stride", 6)
        self.declare_parameter("min_range", 0.25)
        self.declare_parameter("max_range", 3.0)
        self.declare_parameter("ransac_threshold", 0.025)
        self.declare_parameter("min_inlier_ratio", 0.45)
        self.declare_parameter("max_plane_rmse", 0.018)
        self.declare_parameter("min_floor_normal_z", 0.75)
        self.declare_parameter("min_camera_height", 0.25)
        self.declare_parameter("max_camera_height", 0.55)
        self.declare_parameter("input_timeout_sec", 15.0)
        self.declare_parameter("auto_write", False)
        self.declare_parameter("env_file", "")

        self._info = None
        self._points = []
        self._frames = int(self.get_parameter("frames").value)
        self._stride = max(2, int(self.get_parameter("sample_stride").value))
        self._min_range = float(self.get_parameter("min_range").value)
        self._max_range = float(self.get_parameter("max_range").value)
        self._threshold = float(self.get_parameter("ransac_threshold").value)
        self._min_inlier_ratio = float(
            self.get_parameter("min_inlier_ratio").value)
        self._max_plane_rmse = float(
            self.get_parameter("max_plane_rmse").value)
        self._min_floor_normal_z = float(
            self.get_parameter("min_floor_normal_z").value)
        self._min_camera_height = float(
            self.get_parameter("min_camera_height").value)
        self._max_camera_height = float(
            self.get_parameter("max_camera_height").value)
        self._auto_write = bool(self.get_parameter("auto_write").value)
        self._env_file = str(self.get_parameter("env_file").value)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._finished = False
        self._shutdown_timer = None
        self._depth_messages = 0
        self._info_messages = 0
        self._input_timeout_sec = max(
            3.0, float(self.get_parameter("input_timeout_sec").value))
        self._input_timeout_timer = self.create_timer(
            self._input_timeout_sec, self._input_timeout)
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self._info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("depth_topic").value),
            self._depth_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            "Ground calibration started. Keep the vehicle motionless on a flat "
            "floor with the lower half of the depth view unobstructed.")

    def _info_callback(self, msg):
        self._info_messages += 1
        self._info = msg

    def _depth_callback(self, msg):
        self._depth_messages += 1
        if self._finished or self._info is None or len(self._points) >= self._frames:
            return
        if msg.encoding not in ("16UC1", "mono16"):
            self.get_logger().error(f"Unsupported depth encoding: {msg.encoding}")
            self._finished = True
            return
        raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(
            msg.height, msg.step // 2)[:, :msg.width]
        v0 = int(msg.height * 0.45)
        v, u = np.mgrid[v0:msg.height:self._stride, 0:msg.width:self._stride]
        depth = raw[v, u].astype(np.float64) * 0.001
        valid = (depth >= self._min_range) & (depth <= self._max_range)
        if np.count_nonzero(valid) < 100:
            return
        sx = msg.width / max(1, self._info.width)
        sy = msg.height / max(1, self._info.height)
        fx, fy = self._info.k[0] * sx, self._info.k[4] * sy
        cx, cy = self._info.k[2] * sx, self._info.k[5] * sy
        z = depth[valid]
        x = (u[valid] - cx) * z / fx
        y = (v[valid] - cy) * z / fy
        points = np.column_stack((x, y, z))
        if len(points) > 2500:
            points = points[np.random.choice(len(points), 2500, replace=False)]
        self._points.append(points)
        self.get_logger().info(
            f"Calibration frame {len(self._points)}/{self._frames}",
            throttle_duration_sec=1.0,
        )
        if len(self._points) >= self._frames:
            self._solve(msg.header.frame_id)

    def _input_timeout(self):
        if self._finished:
            return
        self.get_logger().error(
            "Calibration input did not complete after "
            f"{self._input_timeout_sec:.1f}s: depth_messages="
            f"{self._depth_messages}, camera_info_messages="
            f"{self._info_messages}, accepted_frames={len(self._points)}/"
            f"{self._frames}. Keep START_DUAL_2D_3D_MAPPING.sh "
            "running in another terminal and verify both terminals use "
            "ROS_DOMAIN_ID=88 and rmw_cyclonedds_cpp. If depth_messages is "
            "nonzero but camera_info_messages is zero, inspect "
            "/camera/depth/camera_info. If both are nonzero, clear the lower "
            "half of the depth view and use a matte, level floor.")
        self._finished = True
        self._shutdown_timer = self.create_timer(0.5, rclpy.shutdown)

    def _solve(self, optical_frame):
        points = np.concatenate(self._points, axis=0)
        if len(points) > 30000:
            points = points[np.random.choice(len(points), 30000, replace=False)]
        best_mask = None
        for _ in range(500):
            sample = points[random.sample(range(len(points)), 3)]
            normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
            norm = np.linalg.norm(normal)
            if norm < 1.0e-8:
                continue
            normal /= norm
            distance = np.abs(points @ normal + (-normal @ sample[0]))
            mask = distance < self._threshold
            if best_mask is None or np.count_nonzero(mask) > np.count_nonzero(best_mask):
                best_mask = mask
        if best_mask is None or np.count_nonzero(best_mask) < 1000:
            self.get_logger().error("No reliable floor plane found; clear the floor view and retry.")
            self._finished = True
            return
        floor = points[best_mask]
        centroid = floor.mean(axis=0)
        _, _, vh = np.linalg.svd(floor - centroid, full_matrices=False)
        normal_optical = vh[-1]
        distance = abs(float(normal_optical @ centroid))
        plane_residuals = np.abs((floor - centroid) @ normal_optical)
        plane_rmse = float(np.sqrt(np.mean(np.square(plane_residuals))))

        base = str(self.get_parameter("base_frame").value)
        camera_link = str(self.get_parameter("camera_link_frame").value)
        try:
            base_tf = self._tf_buffer.lookup_transform(
                base, optical_frame, rclpy.time.Time())
            link_tf = self._tf_buffer.lookup_transform(
                camera_link, optical_frame, rclpy.time.Time())
        except Exception as exc:
            self.get_logger().error(f"TF lookup failed: {exc}")
            self._finished = True
            return
        bq = base_tf.transform.rotation
        lq = link_tf.transform.rotation
        base_from_optical = quaternion_matrix((bq.x, bq.y, bq.z, bq.w))
        link_from_optical = quaternion_matrix((lq.x, lq.y, lq.z, lq.w))
        normal_base = base_from_optical @ normal_optical
        if normal_base[2] < 0.0:
            normal_optical = -normal_optical
            normal_base = -normal_base
        normal_z = float(normal_base[2])
        correction = rotation_align(normal_base, np.array([0.0, 0.0, 1.0]))
        corrected_base_from_optical = correction @ base_from_optical
        corrected_base_from_link = corrected_base_from_optical @ link_from_optical.T
        roll, pitch, yaw = matrix_to_rpy(corrected_base_from_link)
        inlier_ratio = np.count_nonzero(best_mask) / len(points)

        self.get_logger().info("=" * 62)
        self.get_logger().info("GROUND CALIBRATION RESULT")
        self.get_logger().info(f"plane inliers       : {inlier_ratio * 100.0:.1f}%")
        self.get_logger().info(f"plane RMSE          : {plane_rmse * 1000.0:.1f} mm")
        self.get_logger().info(f"floor normal Z      : {normal_z:.4f}")
        self.get_logger().info(f"CAMERA_ROLL_DEG    : {math.degrees(roll):.3f}")
        self.get_logger().info(f"CAMERA_PITCH_DEG   : {math.degrees(pitch):.3f}")
        self.get_logger().info(
            f"CURRENT_YAW_DEG    : {math.degrees(yaw):.3f}  "
            "(reference only; floor cannot calibrate yaw)")
        self.get_logger().info(f"CAMERA_Z           : {distance:.4f}")
        quality_ok = (
            inlier_ratio >= self._min_inlier_ratio and
            plane_rmse <= self._max_plane_rmse and
            normal_z >= self._min_floor_normal_z and
            self._min_camera_height <= distance <= self._max_camera_height
        )
        if self._auto_write and quality_ok:
            try:
                backup = update_env_file(self._env_file, {
                    "CAMERA_ROLL_DEG": f"{math.degrees(roll):.3f}",
                    "CAMERA_PITCH_DEG": f"{math.degrees(pitch):.3f}",
                    "CAMERA_Z": f"{distance:.4f}",
                    "CAMERA_GROUND_CALIBRATED": "true",
                    # Yaw still needs the wall calibration before the complete
                    # extrinsic can be trusted.
                    "CAMERA_EXTRINSIC_CALIBRATED": "false",
                })
                self.get_logger().info(
                    f"AUTO-WRITE OK       : {self._env_file}")
                marker = mark_calibration_restart_required(
                    self._env_file, "ground")
                self.get_logger().warn(
                    f"RESTART REQUIRED    : {marker}")
                if backup:
                    self.get_logger().info(f"BACKUP              : {backup}")
                self.get_logger().info(
                    "Restart mapping, then run CALIBRATE_CAMERA_YAW.sh.")
            except Exception as exc:
                self.get_logger().error(
                    f"Calibration was valid but auto-write failed: {exc}")
        elif self._auto_write:
            self.get_logger().error(
                "AUTO-WRITE REJECTED: floor quality did not pass "
                f"inliers>={self._min_inlier_ratio * 100.0:.0f}% and "
                f"RMSE<={self._max_plane_rmse * 1000.0:.0f} mm, "
                f"normal_z>={self._min_floor_normal_z:.2f}, height="
                f"{self._min_camera_height:.2f}..{self._max_camera_height:.2f} m.")
        else:
            self.get_logger().info(
                "Auto-write is disabled. Copy CAMERA_ROLL_DEG, "
                "CAMERA_PITCH_DEG and CAMERA_Z manually.")
        self.get_logger().info("=" * 62)
        self._finished = True
        self._shutdown_timer = self.create_timer(1.0, rclpy.shutdown)


def main(args=None):
    rclpy.init(args=args)
    node = CameraGroundCalibrator()
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
