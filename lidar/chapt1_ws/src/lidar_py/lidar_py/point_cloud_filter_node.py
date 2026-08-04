"""Lightweight PointCloud2 filter for Raspberry Pi 5 depth-camera mapping.

The node keeps the original camera-frame points in the outgoing cloud so
OctoMap ray casting still starts at the real camera origin.  A temporary copy
is transformed into ``base_link`` only for robot-centric crop filtering and
voxel selection.

Input (default):  /camera/depth/points
Output (default): /camera/depth/points_filtered
"""

from __future__ import annotations

import json
import math
import time
from typing import Optional, Tuple

import numpy as np
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


_XYZ_FIELD_NAMES = ("x", "y", "z")


def _quaternion_to_rotation_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Return a 3x3 rotation matrix for a normalized quaternion."""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1.0e-12:
        return np.eye(3, dtype=np.float32)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float32,
    )


def _extract_xyz(msg: PointCloud2) -> np.ndarray:
    """Read XYZ fields from PointCloud2 without per-point Python iteration."""
    field_map = {field.name: field for field in msg.fields}
    missing = [name for name in _XYZ_FIELD_NAMES if name not in field_map]
    if missing:
        raise ValueError(f"PointCloud2 缺少字段: {', '.join(missing)}")

    endian = ">" if msg.is_bigendian else "<"
    formats = []
    offsets = []
    for name in _XYZ_FIELD_NAMES:
        field = field_map[name]
        if field.datatype != PointField.FLOAT32:
            raise ValueError(f"字段 {name} 不是 FLOAT32，datatype={field.datatype}")
        formats.append(endian + "f4")
        offsets.append(field.offset)

    dtype = np.dtype(
        {
            "names": list(_XYZ_FIELD_NAMES),
            "formats": formats,
            "offsets": offsets,
            "itemsize": msg.point_step,
        }
    )
    total_points = int(msg.width) * int(msg.height)
    structured = np.frombuffer(msg.data, dtype=dtype, count=total_points)
    return np.column_stack((structured["x"], structured["y"], structured["z"]))


def _make_xyz_cloud(header, points: np.ndarray) -> PointCloud2:
    """Create an unorganized XYZ-only PointCloud2 efficiently."""
    points = np.ascontiguousarray(points, dtype=np.float32)
    output = PointCloud2()
    output.header = header
    output.height = 1
    output.width = int(points.shape[0])
    output.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    output.is_bigendian = False
    output.point_step = 12
    output.row_step = output.point_step * output.width
    output.is_dense = True
    output.data = points.astype("<f4", copy=False).tobytes()
    return output


def _stamp_is_zero(stamp: TimeMsg) -> bool:
    return stamp.sec == 0 and stamp.nanosec == 0


class PointCloudFilterNode(Node):
    """Rate-limit, crop and voxel-downsample a depth-camera cloud."""

    def __init__(self) -> None:
        super().__init__("depth_point_cloud_filter")

        self.input_topic = self.declare_parameter("input_topic", "/camera/depth/points").value
        self.output_topic = self.declare_parameter(
            "output_topic", "/camera/depth/points_filtered"
        ).value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value

        self.max_rate_hz = float(self.declare_parameter("max_rate_hz", 5.0).value)
        self.sample_stride = max(1, int(self.declare_parameter("sample_stride", 4).value))
        self.voxel_size = max(0.0, float(self.declare_parameter("voxel_size", 0.06).value))
        self.min_range = max(0.0, float(self.declare_parameter("min_range", 0.30).value))
        self.max_range = float(self.declare_parameter("max_range", 4.0).value)
        self.transform_timeout = max(
            0.01, float(self.declare_parameter("transform_timeout", 0.20).value)
        )

        self.base_x_min = float(self.declare_parameter("base_x_min", -0.50).value)
        self.base_x_max = float(self.declare_parameter("base_x_max", 4.50).value)
        self.base_y_min = float(self.declare_parameter("base_y_min", -3.00).value)
        self.base_y_max = float(self.declare_parameter("base_y_max", 3.00).value)
        self.base_z_min = float(self.declare_parameter("base_z_min", -1.00).value)
        self.base_z_max = float(self.declare_parameter("base_z_max", 2.50).value)

        self.remove_self = bool(self.declare_parameter("remove_self", False).value)
        self.self_x_min = float(self.declare_parameter("self_x_min", -0.40).value)
        self.self_x_max = float(self.declare_parameter("self_x_max", 0.40).value)
        self.self_y_min = float(self.declare_parameter("self_y_min", -0.40).value)
        self.self_y_max = float(self.declare_parameter("self_y_max", 0.40).value)
        self.self_z_min = float(self.declare_parameter("self_z_min", -0.20).value)
        self.self_z_max = float(self.declare_parameter("self_z_max", 1.20).value)

        self.log_every_n = max(1, int(self.declare_parameter("log_every_n", 30).value))

        self._validate_bounds()
        self._last_process_monotonic = 0.0
        self._processed_frames = 0
        self._dropped_rate_frames = 0
        self._last_tf_warning = 0.0

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(
            PointCloud2, self.output_topic, qos_profile_sensor_data
        )
        # 订阅用 RELIABLE + depth=1，只保留最新帧，避免队列积压
        _sub_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.stats_publisher = self.create_publisher(String, "~/stats", 10)
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self._cloud_callback,
            _sub_qos,
        )

        self.get_logger().info(
            "点云过滤器已启动: %s -> %s, %.1fHz, stride=%d, voxel=%.3fm"
            % (
                self.input_topic,
                self.output_topic,
                self.max_rate_hz,
                self.sample_stride,
                self.voxel_size,
            )
        )

    def _validate_bounds(self) -> None:
        pairs = [
            ("base_x", self.base_x_min, self.base_x_max),
            ("base_y", self.base_y_min, self.base_y_max),
            ("base_z", self.base_z_min, self.base_z_max),
            ("self_x", self.self_x_min, self.self_x_max),
            ("self_y", self.self_y_min, self.self_y_max),
            ("self_z", self.self_z_min, self.self_z_max),
        ]
        for name, minimum, maximum in pairs:
            if minimum >= maximum:
                raise ValueError(f"{name}_min 必须小于 {name}_max")
        if self.max_range > 0.0 and self.min_range >= self.max_range:
            raise ValueError("min_range 必须小于 max_range，或把 max_range 设置为负数")

    def _lookup_base_transform(self, msg: PointCloud2) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        if msg.header.frame_id == self.base_frame:
            return np.eye(3, dtype=np.float32), np.zeros(3, dtype=np.float32)

        query_time = Time() if _stamp_is_zero(msg.header.stamp) else Time.from_msg(msg.header.stamp)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                msg.header.frame_id,
                query_time,
                timeout=Duration(seconds=self.transform_timeout),
            )
        except TransformException as exc:
            now = time.monotonic()
            if now - self._last_tf_warning >= 2.0:
                self.get_logger().warning(
                    f"等待 TF {self.base_frame} <- {msg.header.frame_id}: {exc}"
                )
                self._last_tf_warning = now
            return None

        rotation = transform.transform.rotation
        translation = transform.transform.translation
        matrix = _quaternion_to_rotation_matrix(
            rotation.x, rotation.y, rotation.z, rotation.w
        )
        vector = np.array(
            [translation.x, translation.y, translation.z], dtype=np.float32
        )
        return matrix, vector

    def _cloud_callback(self, msg: PointCloud2) -> None:
        now_monotonic = time.monotonic()
        if self.max_rate_hz > 0.0:
            min_period = 1.0 / self.max_rate_hz
            if now_monotonic - self._last_process_monotonic < min_period:
                self._dropped_rate_frames += 1
                return
        self._last_process_monotonic = now_monotonic
        start = time.perf_counter()

        transform = self._lookup_base_transform(msg)
        if transform is None:
            return

        try:
            sensor_points = _extract_xyz(msg)
        except (ValueError, TypeError) as exc:
            self.get_logger().error(f"无法解析输入点云: {exc}")
            return

        input_count = int(sensor_points.shape[0])
        if input_count == 0:
            return

        indices = np.arange(0, input_count, self.sample_stride, dtype=np.int64)
        sampled_sensor = sensor_points[indices]

        finite_mask = np.isfinite(sampled_sensor).all(axis=1)
        squared_range = np.einsum("ij,ij->i", sampled_sensor, sampled_sensor)
        range_mask = squared_range >= self.min_range * self.min_range
        if self.max_range > 0.0:
            range_mask &= squared_range <= self.max_range * self.max_range
        valid_mask = finite_mask & range_mask
        if not np.any(valid_mask):
            return

        indices = indices[valid_mask]
        sampled_sensor = sensor_points[indices]

        rotation, translation = transform
        points_base = sampled_sensor @ rotation.T + translation
        crop_mask = (
            (points_base[:, 0] >= self.base_x_min)
            & (points_base[:, 0] <= self.base_x_max)
            & (points_base[:, 1] >= self.base_y_min)
            & (points_base[:, 1] <= self.base_y_max)
            & (points_base[:, 2] >= self.base_z_min)
            & (points_base[:, 2] <= self.base_z_max)
        )

        if self.remove_self:
            inside_self = (
                (points_base[:, 0] >= self.self_x_min)
                & (points_base[:, 0] <= self.self_x_max)
                & (points_base[:, 1] >= self.self_y_min)
                & (points_base[:, 1] <= self.self_y_max)
                & (points_base[:, 2] >= self.self_z_min)
                & (points_base[:, 2] <= self.self_z_max)
            )
            crop_mask &= ~inside_self

        if not np.any(crop_mask):
            return
        indices = indices[crop_mask]
        points_base = points_base[crop_mask]

        if self.voxel_size > 0.0 and points_base.shape[0] > 1:
            voxel_keys = np.floor(points_base / self.voxel_size).astype(np.int32)
            _, first_indices = np.unique(voxel_keys, axis=0, return_index=True)
            first_indices.sort()
            indices = indices[first_indices]

        output_points = sensor_points[indices]
        output = _make_xyz_cloud(msg.header, output_points)
        self.publisher.publish(output)

        self._processed_frames += 1
        elapsed_ms = (time.perf_counter() - start) * 1000.0
        if self._processed_frames % self.log_every_n == 0:
            stats = {
                "input_points": input_count,
                "output_points": int(output_points.shape[0]),
                "process_ms": round(elapsed_ms, 2),
                "processed_frames": self._processed_frames,
                "rate_dropped_frames": self._dropped_rate_frames,
                "source_frame": msg.header.frame_id,
                "filter_frame": self.base_frame,
            }
            stats_msg = String()
            stats_msg.data = json.dumps(stats, ensure_ascii=False)
            self.stats_publisher.publish(stats_msg)
            self.get_logger().info(
                "点云 %d -> %d, %.1fms, 因限频跳过 %d 帧"
                % (
                    input_count,
                    output_points.shape[0],
                    elapsed_ms,
                    self._dropped_rate_frames,
                )
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[PointCloudFilterNode] = None
    try:
        node = PointCloudFilterNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
