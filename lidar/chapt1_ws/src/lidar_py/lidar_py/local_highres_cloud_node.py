"""Low-latency high-resolution local PointCloud2 filter for collision sensing.

This node is deliberately separate from ``point_cloud_filter_node`` used by
STEP7-STEP9.  It keeps the proven global 3D mapping chain untouched and builds
an independent, robot-centric cloud for future local voxel mapping, URDF
self-filtering and collision prediction.

Input (default):  /camera/depth/points
Output (default): /local_highres_cloud, expressed in ``base_link``
Stats (default):  /local_highres_cloud/stats
Markers:          /local_highres_cloud/crop_markers
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
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


_XYZ_FIELD_NAMES = ("x", "y", "z")


def _as_bool(value) -> bool:
    """Convert ROS parameter values to bool without treating non-empty strings as true."""
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _quaternion_to_rotation_matrix(
    x: float, y: float, z: float, w: float
) -> np.ndarray:
    """Return a float32 3x3 rotation matrix for a quaternion."""
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
    """Read XYZ fields as an ``N x 3`` NumPy array without Python point loops."""
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
            "itemsize": int(msg.point_step),
        }
    )
    height = max(1, int(msg.height))
    width = int(msg.width)
    expected_row_step = int(msg.point_step) * width

    if int(msg.row_step) == expected_row_step:
        structured = np.frombuffer(msg.data, dtype=dtype, count=height * width)
    else:
        structured = np.ndarray(
            shape=(height, width),
            dtype=dtype,
            buffer=msg.data,
            strides=(int(msg.row_step), int(msg.point_step)),
        ).reshape(-1)

    points = np.empty((structured.shape[0], 3), dtype=np.float32)
    points[:, 0] = structured["x"]
    points[:, 1] = structured["y"]
    points[:, 2] = structured["z"]
    return points


def _make_xyz_cloud(header, frame_id: str, points: np.ndarray) -> PointCloud2:
    """Create an XYZ-only cloud while preserving the original timestamp."""
    points = np.ascontiguousarray(points, dtype=np.float32)
    output = PointCloud2()
    output.header.stamp = header.stamp
    output.header.frame_id = frame_id
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


def _ema(previous: float, sample: float, alpha: float = 0.20) -> float:
    if sample <= 0.0 or not math.isfinite(sample):
        return previous
    if previous <= 0.0:
        return sample
    return previous + alpha * (sample - previous)


class LocalHighResCloudNode(Node):
    """Build a fresh, dense, robot-centric cloud for local collision sensing."""

    def __init__(self) -> None:
        super().__init__("local_highres_cloud_filter")

        self.input_topic = str(
            self.declare_parameter("input_topic", "/camera/depth/points").value
        )
        self.output_topic = str(
            self.declare_parameter("output_topic", "/local_highres_cloud").value
        )
        self.stats_topic = str(
            self.declare_parameter("stats_topic", "/local_highres_cloud/stats").value
        )
        self.marker_topic = str(
            self.declare_parameter(
                "marker_topic", "/local_highres_cloud/crop_markers"
            ).value
        )
        self.output_frame = str(
            self.declare_parameter("output_frame", "base_link").value
        )

        self.max_rate_hz = float(self.declare_parameter("max_rate_hz", 12.0).value)
        self.sample_stride = max(
            1, int(self.declare_parameter("sample_stride", 1).value)
        )
        self.voxel_size = max(
            0.0, float(self.declare_parameter("voxel_size", 0.025).value)
        )
        self.min_range = max(
            0.0, float(self.declare_parameter("min_range", 0.20).value)
        )
        self.max_range = float(self.declare_parameter("max_range", 4.0).value)
        self.transform_timeout = max(
            0.001, float(self.declare_parameter("transform_timeout", 0.03).value)
        )

        self.x_min = float(self.declare_parameter("x_min", 0.20).value)
        self.x_max = float(self.declare_parameter("x_max", 4.00).value)
        self.y_min = float(self.declare_parameter("y_min", -2.50).value)
        self.y_max = float(self.declare_parameter("y_max", 2.50).value)
        self.z_min = float(self.declare_parameter("z_min", -0.50).value)
        self.z_max = float(self.declare_parameter("z_max", 2.00).value)

        self.remove_self = _as_bool(
            self.declare_parameter("remove_self", True).value
        )
        self.self_x_min = float(self.declare_parameter("self_x_min", -0.36).value)
        self.self_x_max = float(self.declare_parameter("self_x_max", 0.36).value)
        self.self_y_min = float(self.declare_parameter("self_y_min", -0.36).value)
        self.self_y_max = float(self.declare_parameter("self_y_max", 0.36).value)
        self.self_z_min = float(self.declare_parameter("self_z_min", -0.10).value)
        self.self_z_max = float(self.declare_parameter("self_z_max", 0.90).value)

        self.ground_filter_enabled = _as_bool(
            self.declare_parameter("ground_filter_enabled", False).value
        )
        self.ground_z_min = float(
            self.declare_parameter("ground_z_min", -0.06).value
        )
        self.ground_z_max = float(
            self.declare_parameter("ground_z_max", 0.08).value
        )

        self.stats_period_sec = max(
            0.10, float(self.declare_parameter("stats_period_sec", 1.0).value)
        )
        self.publish_markers = _as_bool(
            self.declare_parameter("publish_markers", True).value
        )
        self._validate_parameters()

        self._received_frames = 0
        self._published_frames = 0
        self._rate_dropped_frames = 0
        self._tf_dropped_frames = 0
        self._empty_dropped_frames = 0
        self._parse_dropped_frames = 0
        self._last_process_monotonic = 0.0
        self._last_input_monotonic = 0.0
        self._last_output_monotonic = 0.0
        self._last_stats_monotonic = 0.0
        self._input_hz = 0.0
        self._output_hz = 0.0
        self._last_tf_warning = 0.0
        self._latest_stats = {}

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        input_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        output_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        marker_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        self.cloud_publisher = self.create_publisher(
            PointCloud2, self.output_topic, output_qos
        )
        self.stats_publisher = self.create_publisher(String, self.stats_topic, 10)
        self.marker_publisher = self.create_publisher(
            MarkerArray, self.marker_topic, marker_qos
        )
        self.subscription = self.create_subscription(
            PointCloud2, self.input_topic, self._cloud_callback, input_qos
        )
        self.marker_timer = self.create_timer(1.0, self._publish_crop_markers)

        self.get_logger().info(
            "STEP10局部高精度点云已启动: %s -> %s, frame=%s, "
            "max_rate=%.1fHz, stride=%d, voxel=%.3fm"
            % (
                self.input_topic,
                self.output_topic,
                self.output_frame,
                self.max_rate_hz,
                self.sample_stride,
                self.voxel_size,
            )
        )

    def _validate_parameters(self) -> None:
        bounds = [
            ("x", self.x_min, self.x_max),
            ("y", self.y_min, self.y_max),
            ("z", self.z_min, self.z_max),
            ("self_x", self.self_x_min, self.self_x_max),
            ("self_y", self.self_y_min, self.self_y_max),
            ("self_z", self.self_z_min, self.self_z_max),
            ("ground_z", self.ground_z_min, self.ground_z_max),
        ]
        for name, minimum, maximum in bounds:
            if minimum >= maximum:
                raise ValueError(f"{name}_min 必须小于 {name}_max")
        if self.max_range > 0.0 and self.min_range >= self.max_range:
            raise ValueError("min_range 必须小于 max_range，或把 max_range 设置为负数")

    def _cloud_age_ms(self, stamp: TimeMsg) -> float:
        if _stamp_is_zero(stamp):
            return -1.0
        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = Time.from_msg(stamp).nanoseconds
        return (now_ns - stamp_ns) / 1.0e6

    def _lookup_transform(
        self, msg: PointCloud2
    ) -> Optional[Tuple[np.ndarray, np.ndarray, float]]:
        if msg.header.frame_id == self.output_frame:
            return (
                np.eye(3, dtype=np.float32),
                np.zeros(3, dtype=np.float32),
                0.0,
            )

        query_time = Time() if _stamp_is_zero(msg.header.stamp) else Time.from_msg(
            msg.header.stamp
        )
        start = time.perf_counter()
        try:
            transform = self.tf_buffer.lookup_transform(
                self.output_frame,
                msg.header.frame_id,
                query_time,
                timeout=Duration(seconds=self.transform_timeout),
            )
        except TransformException as exc:
            self._tf_dropped_frames += 1
            now = time.monotonic()
            if now - self._last_tf_warning >= 2.0:
                self.get_logger().warning(
                    f"等待TF {self.output_frame} <- {msg.header.frame_id}: {exc}"
                )
                self._last_tf_warning = now
            return None

        tf_wait_ms = (time.perf_counter() - start) * 1000.0
        rotation = transform.transform.rotation
        translation = transform.transform.translation
        matrix = _quaternion_to_rotation_matrix(
            rotation.x, rotation.y, rotation.z, rotation.w
        )
        vector = np.array(
            [translation.x, translation.y, translation.z], dtype=np.float32
        )
        return matrix, vector, tf_wait_ms

    def _voxel_first_indices(self, points: np.ndarray) -> np.ndarray:
        """Return one point per voxel using a fast one-dimensional integer key."""
        if self.voxel_size <= 0.0 or points.shape[0] <= 1:
            return np.arange(points.shape[0], dtype=np.int64)

        voxel = np.floor(
            (points - np.array([self.x_min, self.y_min, self.z_min], dtype=np.float32))
            / self.voxel_size
        ).astype(np.int32)
        nx = max(1, int(math.ceil((self.x_max - self.x_min) / self.voxel_size)) + 1)
        ny = max(1, int(math.ceil((self.y_max - self.y_min) / self.voxel_size)) + 1)
        keys = (
            voxel[:, 0].astype(np.int64)
            + np.int64(nx)
            * (
                voxel[:, 1].astype(np.int64)
                + np.int64(ny) * voxel[:, 2].astype(np.int64)
            )
        )
        _, first_indices = np.unique(keys, return_index=True)
        return np.sort(first_indices)

    def _cloud_callback(self, msg: PointCloud2) -> None:
        arrival_monotonic = time.monotonic()
        self._received_frames += 1
        if self._last_input_monotonic > 0.0:
            dt = arrival_monotonic - self._last_input_monotonic
            if dt > 1.0e-6:
                self._input_hz = _ema(self._input_hz, 1.0 / dt)
        self._last_input_monotonic = arrival_monotonic

        if self.max_rate_hz > 0.0:
            min_period = 1.0 / self.max_rate_hz
            if arrival_monotonic - self._last_process_monotonic < min_period:
                self._rate_dropped_frames += 1
                return
        self._last_process_monotonic = arrival_monotonic

        process_start = time.perf_counter()
        input_age_ms = self._cloud_age_ms(msg.header.stamp)
        transform = self._lookup_transform(msg)
        if transform is None:
            return
        rotation, translation, tf_wait_ms = transform

        try:
            sensor_points = _extract_xyz(msg)
        except (ValueError, TypeError, BufferError) as exc:
            self._parse_dropped_frames += 1
            self.get_logger().error(f"无法解析输入点云: {exc}")
            return

        input_count = int(sensor_points.shape[0])
        if input_count == 0:
            self._empty_dropped_frames += 1
            return

        sampled = sensor_points[:: self.sample_stride]
        finite_mask = np.isfinite(sampled).all(axis=1)
        sampled = sampled[finite_mask]
        if sampled.shape[0] == 0:
            self._empty_dropped_frames += 1
            return

        squared_range = np.einsum("ij,ij->i", sampled, sampled)
        range_mask = squared_range >= self.min_range * self.min_range
        if self.max_range > 0.0:
            range_mask &= squared_range <= self.max_range * self.max_range
        sampled = sampled[range_mask]
        if sampled.shape[0] == 0:
            self._empty_dropped_frames += 1
            return

        points = sampled @ rotation.T + translation
        keep = (
            (points[:, 0] >= self.x_min)
            & (points[:, 0] <= self.x_max)
            & (points[:, 1] >= self.y_min)
            & (points[:, 1] <= self.y_max)
            & (points[:, 2] >= self.z_min)
            & (points[:, 2] <= self.z_max)
        )

        if self.remove_self:
            inside_self = (
                (points[:, 0] >= self.self_x_min)
                & (points[:, 0] <= self.self_x_max)
                & (points[:, 1] >= self.self_y_min)
                & (points[:, 1] <= self.self_y_max)
                & (points[:, 2] >= self.self_z_min)
                & (points[:, 2] <= self.self_z_max)
            )
            keep &= ~inside_self

        if self.ground_filter_enabled:
            inside_ground_band = (
                (points[:, 2] >= self.ground_z_min)
                & (points[:, 2] <= self.ground_z_max)
            )
            keep &= ~inside_ground_band

        points = points[keep]
        if points.shape[0] == 0:
            self._empty_dropped_frames += 1
            return

        voxel_indices = self._voxel_first_indices(points)
        points = points[voxel_indices]
        output = _make_xyz_cloud(msg.header, self.output_frame, points)
        self.cloud_publisher.publish(output)

        output_monotonic = time.monotonic()
        self._published_frames += 1
        if self._last_output_monotonic > 0.0:
            dt = output_monotonic - self._last_output_monotonic
            if dt > 1.0e-6:
                self._output_hz = _ema(self._output_hz, 1.0 / dt)
        self._last_output_monotonic = output_monotonic

        process_ms = (time.perf_counter() - process_start) * 1000.0
        output_age_ms = self._cloud_age_ms(msg.header.stamp)
        known_dropped = (
            self._rate_dropped_frames
            + self._tf_dropped_frames
            + self._empty_dropped_frames
            + self._parse_dropped_frames
        )
        self._latest_stats = {
            "input_points": input_count,
            "output_points": int(points.shape[0]),
            "input_age_ms": round(input_age_ms, 2),
            "output_age_ms": round(output_age_ms, 2),
            "process_ms": round(process_ms, 2),
            "tf_wait_ms": round(tf_wait_ms, 2),
            "input_hz": round(self._input_hz, 2),
            "output_hz": round(self._output_hz, 2),
            "received_frames": self._received_frames,
            "published_frames": self._published_frames,
            "known_dropped_frames": known_dropped,
            "rate_dropped_frames": self._rate_dropped_frames,
            "tf_dropped_frames": self._tf_dropped_frames,
            "empty_dropped_frames": self._empty_dropped_frames,
            "parse_dropped_frames": self._parse_dropped_frames,
            "source_frame": msg.header.frame_id,
            "output_frame": self.output_frame,
            "voxel_size_m": self.voxel_size,
            "sample_stride": self.sample_stride,
            "ground_filter_enabled": self.ground_filter_enabled,
            "remove_self": self.remove_self,
        }

        if output_monotonic - self._last_stats_monotonic >= self.stats_period_sec:
            self._last_stats_monotonic = output_monotonic
            stats_msg = String()
            stats_msg.data = json.dumps(self._latest_stats, ensure_ascii=False)
            self.stats_publisher.publish(stats_msg)
            self.get_logger().info(
                "STEP10点云 %d -> %d, %.1fms, age %.1fms, %.1fHz"
                % (
                    input_count,
                    points.shape[0],
                    process_ms,
                    output_age_ms,
                    self._output_hz,
                )
            )

    @staticmethod
    def _cube_marker(
        marker_id: int,
        namespace: str,
        frame_id: str,
        stamp,
        minimum: Tuple[float, float, float],
        maximum: Tuple[float, float, float],
        color: Tuple[float, float, float, float],
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = (minimum[0] + maximum[0]) * 0.5
        marker.pose.position.y = (minimum[1] + maximum[1]) * 0.5
        marker.pose.position.z = (minimum[2] + maximum[2]) * 0.5
        marker.scale.x = maximum[0] - minimum[0]
        marker.scale.y = maximum[1] - minimum[1]
        marker.scale.z = maximum[2] - minimum[2]
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.lifetime = Duration(seconds=1.5).to_msg()
        return marker

    def _publish_crop_markers(self) -> None:
        if not self.publish_markers:
            return
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        markers.markers.append(
            self._cube_marker(
                0,
                "local_highres_crop",
                self.output_frame,
                stamp,
                (self.x_min, self.y_min, self.z_min),
                (self.x_max, self.y_max, self.z_max),
                (0.10, 0.75, 1.00, 0.055),
            )
        )
        if self.remove_self:
            markers.markers.append(
                self._cube_marker(
                    1,
                    "local_highres_self_filter",
                    self.output_frame,
                    stamp,
                    (self.self_x_min, self.self_y_min, self.self_z_min),
                    (self.self_x_max, self.self_y_max, self.self_z_max),
                    (1.00, 0.20, 0.15, 0.15),
                )
            )
        if self.ground_filter_enabled:
            markers.markers.append(
                self._cube_marker(
                    2,
                    "local_highres_ground_filter",
                    self.output_frame,
                    stamp,
                    (self.x_min, self.y_min, self.ground_z_min),
                    (self.x_max, self.y_max, self.ground_z_max),
                    (1.00, 0.75, 0.10, 0.12),
                )
            )
        self.marker_publisher.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[LocalHighResCloudNode] = None
    try:
        node = LocalHighResCloudNode()
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
