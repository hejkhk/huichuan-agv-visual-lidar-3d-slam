#!/usr/bin/env python3
"""Startup-only global relocalization for a frozen Cartographer map.

The scan-to-grid matcher is adapted from the all.beifen implementation, but
navigation is released only after a verified match. Runtime monitoring never
changes the trajectory automatically; a new search can only be requested from
the localization RViz panel.
"""

import math
import time

import numpy as np
import rclpy
from cartographer_ros_msgs.msg import TrajectoryStates
from cartographer_ros_msgs.srv import (
    FinishTrajectory,
    GetTrajectoryStates,
    StartTrajectory,
)
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener


class CartographerRelocalizer(Node):
    """Find an initial map pose and restart the active Cartographer trajectory."""

    def __init__(self):
        super().__init__("cartographer_reloc")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("scan_topic", "/scan_timed_v2_filtered")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("laser_frame", "laser_frame")
        self.declare_parameter("configuration_directory", "")
        self.declare_parameter(
            "configuration_basename", "cartographer_2d_localization.lua")
        self.declare_parameter("coarse_step_cells", 8)
        self.declare_parameter("coarse_yaw_step_deg", 15.0)
        self.declare_parameter("gradient_radius_cells", 24)
        self.declare_parameter("max_scan_points", 100)
        self.declare_parameter("min_scan_points", 45)
        self.declare_parameter("min_valid_fraction", 0.65)
        self.declare_parameter("min_match_score", 0.28)
        self.declare_parameter("min_score_margin", 0.018)
        self.declare_parameter("max_wait_sec", 120.0)
        self.declare_parameter("stationary_hold_sec", 1.0)
        self.declare_parameter("verify_hold_sec", 2.0)

        self.map_topic = str(self.get_parameter("map_topic").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.laser_frame = str(self.get_parameter("laser_frame").value)
        self.config_dir = str(
            self.get_parameter("configuration_directory").value)
        self.config_name = str(
            self.get_parameter("configuration_basename").value)
        self.coarse_step = max(
            2, int(self.get_parameter("coarse_step_cells").value))
        self.yaw_step = math.radians(float(
            self.get_parameter("coarse_yaw_step_deg").value))
        self.gradient_radius = max(
            4, int(self.get_parameter("gradient_radius_cells").value))
        self.max_scan_points = max(
            40, int(self.get_parameter("max_scan_points").value))
        self.min_scan_points = max(
            25, int(self.get_parameter("min_scan_points").value))
        self.min_valid_fraction = float(
            self.get_parameter("min_valid_fraction").value)
        self.min_score = float(self.get_parameter("min_match_score").value)
        self.min_margin = float(
            self.get_parameter("min_score_margin").value)
        self.max_wait_sec = float(self.get_parameter("max_wait_sec").value)
        self.stationary_hold_sec = float(
            self.get_parameter("stationary_hold_sec").value)
        self.verify_hold_sec = float(
            self.get_parameter("verify_hold_sec").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        gate_qos = QoSProfile(depth=1)
        gate_qos.reliability = ReliabilityPolicy.RELIABLE
        gate_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(
            OccupancyGrid, self.map_topic, self._on_map, map_qos)
        self.create_subscription(
            LaserScan, self.scan_topic, self._on_scan, 10)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, 20)
        self.create_subscription(
            Bool, "/cartographer_reloc/trigger", self._on_manual_trigger, 1)
        self.ready_pub = self.create_publisher(
            Bool, "/localization_ready", gate_qos)
        self.state_pub = self.create_publisher(
            String, "/cartographer_reloc/state", 5)

        self.states_client = self.create_client(
            GetTrajectoryStates,
            "/cartographer_node/get_trajectory_states")
        self.finish_client = self.create_client(
            FinishTrajectory, "/cartographer_node/finish_trajectory")
        self.start_client = self.create_client(
            StartTrajectory, "/cartographer_node/start_trajectory")

        self.map_msg = None
        self.map_data = None
        self.score_grid = None
        self.latest_scan = None
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.stationary_since = None
        self.state = "waiting_inputs"
        self.detail = "startup"
        self.best_score = 0.0
        self.second_score = 0.0
        self.started_at = time.monotonic()
        self.pending_search = True
        self.busy = False
        self.manual_request = False
        self.verify_since = None
        self.expected_pose = None
        self.active_trajectory_id = None
        self.reference_trajectory_id = None

        self._publish_ready(False)
        self.create_timer(0.25, self._tick)
        self.create_timer(0.5, self._publish_state)
        self.get_logger().info(
            "Startup relocalizer armed; Nav2 remains inactive until verified")

    def _on_map(self, msg):
        if msg.info.width == 0 or msg.info.height == 0:
            return
        changed = (
            self.map_msg is None
            or self.map_msg.info.width != msg.info.width
            or self.map_msg.info.height != msg.info.height
            or abs(self.map_msg.info.resolution - msg.info.resolution) > 1e-9
        )
        self.map_msg = msg
        self.map_data = np.asarray(msg.data, dtype=np.int16).reshape(
            msg.info.height, msg.info.width)
        if changed or self.score_grid is None:
            self._build_score_grid()

    def _on_scan(self, msg):
        self.latest_scan = msg

    def _on_odom(self, msg):
        self.linear_speed = abs(float(msg.twist.twist.linear.x))
        self.angular_speed = abs(float(msg.twist.twist.angular.z))
        if self.linear_speed < 0.015 and self.angular_speed < 0.02:
            if self.stationary_since is None:
                self.stationary_since = time.monotonic()
        else:
            self.stationary_since = None

    def _on_manual_trigger(self, msg):
        if not msg.data or self.busy:
            return
        self.get_logger().warn("Manual relocalization requested")
        self.manual_request = True
        self.pending_search = True
        self.started_at = time.monotonic()
        self.verify_since = None
        self.state = "waiting_stop"
        self.detail = "manual request; Nav2 paused"
        self._publish_ready(False)

    def _tick(self):
        if self.state == "verifying":
            self._verify_pose()
            return
        if not self.pending_search or self.busy:
            return
        if time.monotonic() - self.started_at > self.max_wait_sec:
            self.state = "failed"
            self.detail = "startup timeout; Nav2 remains locked"
            self.pending_search = False
            self._publish_ready(False)
            return
        if self.map_msg is None or self.score_grid is None:
            self.state = "waiting_map"
            self.detail = self.map_topic
            return
        if self.latest_scan is None:
            self.state = "waiting_scan"
            self.detail = self.scan_topic
            return
        if self.stationary_since is None or (
                time.monotonic() - self.stationary_since
                < self.stationary_hold_sec):
            self.state = "waiting_stop"
            self.detail = "hold the vehicle still"
            return
        if not (self.states_client.service_is_ready()
                and self.finish_client.service_is_ready()
                and self.start_client.service_is_ready()):
            self.state = "waiting_cartographer"
            self.detail = "trajectory services"
            return
        self._start_search()

    def _build_score_grid(self):
        try:
            from scipy.ndimage import distance_transform_edt
        except ImportError:
            self.state = "failed"
            self.detail = "python3-scipy is missing"
            self.get_logger().error(
                "Relocalization requires python3-scipy")
            return
        obstacle = self.map_data >= 65
        if not np.any(obstacle):
            self.score_grid = None
            return
        distance = distance_transform_edt(~obstacle)
        self.score_grid = np.clip(
            1.0 - distance / float(self.gradient_radius), 0.0, 1.0
        ).astype(np.float32)

    def _scan_points_in_base(self):
        scan = self.latest_scan
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        angles = scan.angle_min + np.arange(ranges.size) * scan.angle_increment
        valid = (
            np.isfinite(ranges)
            & (ranges >= max(scan.range_min, 0.12))
            & (ranges <= min(scan.range_max, 8.0))
        )
        ranges = ranges[valid]
        angles = angles[valid]
        if ranges.size < self.min_scan_points:
            return None
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, scan.header.frame_id or self.laser_frame,
                rclpy.time.Time())
        except Exception as exc:
            self.detail = f"laser TF unavailable: {exc}"
            return None
        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        lx = ranges * np.cos(angles)
        ly = ranges * np.sin(angles)
        c = math.cos(yaw)
        s = math.sin(yaw)
        x = c * lx - s * ly + transform.transform.translation.x
        y = s * lx + c * ly + transform.transform.translation.y
        if x.size > self.max_scan_points:
            indices = np.linspace(
                0, x.size - 1, self.max_scan_points).astype(np.int32)
            x = x[indices]
            y = y[indices]
        return x, y

    def _start_search(self):
        points = self._scan_points_in_base()
        if points is None:
            self.state = "waiting_scan"
            self.detail = "not enough valid scan points or laser TF"
            return
        self.busy = True
        self.state = "searching"
        self.detail = "global scan-to-map search"
        self._publish_state()
        pose, best, second = self._global_match(*points)
        self.best_score = best
        self.second_score = second
        if pose is None:
            self.busy = False
            self.pending_search = False
            self.state = "failed"
            self.detail = (
                f"ambiguous/weak match best={best:.3f} second={second:.3f}; "
                "move to a more distinctive stationary location and retry")
            self._publish_ready(False)
            return
        self.expected_pose = pose
        self.state = "restarting_trajectory"
        future = self.states_client.call_async(GetTrajectoryStates.Request())
        future.add_done_callback(self._on_states)

    def _global_match(self, bx, by):
        msg = self.map_msg
        resolution = float(msg.info.resolution)
        height, width = self.map_data.shape
        origin_x = float(msg.info.origin.position.x)
        origin_y = float(msg.info.origin.position.y)
        q = msg.info.origin.orientation
        origin_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if abs(origin_yaw) > 1e-3:
            self.get_logger().warn(
                "Rotated OccupancyGrid origins are not supported")
            return None, 0.0, 0.0

        rows = np.arange(0, height, self.coarse_step, dtype=np.int32)
        cols = np.arange(0, width, self.coarse_step, dtype=np.int32)
        grid_c, grid_r = np.meshgrid(cols, rows)
        candidate_r = grid_r.ravel()
        candidate_c = grid_c.ravel()
        free = self.map_data[candidate_r, candidate_c] == 0
        candidate_r = candidate_r[free]
        candidate_c = candidate_c[free]
        if candidate_r.size == 0:
            return None, 0.0, 0.0

        results = []
        yaw_values = np.arange(-math.pi, math.pi, self.yaw_step)
        for yaw in yaw_values:
            c = math.cos(yaw)
            s = math.sin(yaw)
            offset_c = np.rint((c * bx - s * by) / resolution).astype(np.int32)
            offset_r = np.rint((s * bx + c * by) / resolution).astype(np.int32)
            for start in range(0, candidate_r.size, 384):
                cr = candidate_r[start:start + 384]
                cc = candidate_c[start:start + 384]
                rr = cr[:, None] + offset_r[None, :]
                cl = cc[:, None] + offset_c[None, :]
                valid = (rr >= 0) & (rr < height) & (cl >= 0) & (cl < width)
                valid_count = np.sum(valid, axis=1)
                enough = valid_count >= int(bx.size * self.min_valid_fraction)
                if not np.any(enough):
                    continue
                rr_safe = np.clip(rr, 0, height - 1)
                cl_safe = np.clip(cl, 0, width - 1)
                values = self.score_grid[rr_safe, cl_safe] * valid
                scores = np.sum(values, axis=1) / np.maximum(valid_count, 1)
                scores[~enough] = -1.0
                take = min(3, scores.size)
                if take:
                    top = np.argpartition(scores, -take)[-take:]
                    for index in top:
                        if scores[index] > 0:
                            results.append((
                                float(scores[index]),
                                int(cr[index]), int(cc[index]), float(yaw)))

        if not results:
            return None, 0.0, 0.0
        results.sort(reverse=True, key=lambda item: item[0])
        seed = results[0]
        best = self._refine_match(bx, by, seed, resolution)
        second = 0.0
        for candidate in results[1:]:
            dx = (candidate[2] - best[2]) * resolution
            dy = (candidate[1] - best[1]) * resolution
            dyaw = abs(self._wrap(candidate[3] - best[3]))
            if math.hypot(dx, dy) > 0.8 or dyaw > math.radians(20.0):
                second = candidate[0]
                break
        score, row, col, yaw = best
        margin = score - second
        self.get_logger().info(
            f"Relocalization match: best={score:.3f}, second={second:.3f}, "
            f"margin={margin:.3f}")
        if score < self.min_score or margin < self.min_margin:
            return None, score, second
        x = origin_x + col * resolution
        y = origin_y + row * resolution
        return self._pose(x, y, yaw), score, second

    def _refine_match(self, bx, by, seed, resolution):
        best_score, best_row, best_col, best_yaw = seed
        height, width = self.map_data.shape
        for position_step, yaw_step in ((2, 2.0), (1, 0.5)):
            improved = True
            iterations = 0
            while improved and iterations < 20:
                improved = False
                iterations += 1
                for dr in (-position_step, 0, position_step):
                    for dc in (-position_step, 0, position_step):
                        for da in (-yaw_step, 0.0, yaw_step):
                            row = best_row + dr
                            col = best_col + dc
                            yaw = best_yaw + math.radians(da)
                            score = self._score_pose(
                                bx, by, row, col, yaw, resolution,
                                height, width)
                            if score > best_score + 1e-6:
                                best_score = score
                                best_row = row
                                best_col = col
                                best_yaw = yaw
                                improved = True
        return best_score, best_row, best_col, self._wrap(best_yaw)

    def _score_pose(self, bx, by, row, col, yaw, resolution, height, width):
        c = math.cos(yaw)
        s = math.sin(yaw)
        cc = col + np.rint((c * bx - s * by) / resolution).astype(np.int32)
        rr = row + np.rint((s * bx + c * by) / resolution).astype(np.int32)
        valid = (rr >= 0) & (rr < height) & (cc >= 0) & (cc < width)
        if np.count_nonzero(valid) < bx.size * self.min_valid_fraction:
            return -1.0
        return float(np.mean(self.score_grid[rr[valid], cc[valid]]))

    def _on_states(self, future):
        try:
            response = future.result()
            if response.status.code != 0:
                raise RuntimeError(response.status.message)
            ids = response.trajectory_states.trajectory_id
            states = response.trajectory_states.trajectory_state
            active = [i for i, s in zip(ids, states)
                      if s == TrajectoryStates.ACTIVE]
            frozen = [i for i, s in zip(ids, states)
                      if s == TrajectoryStates.FROZEN]
            if len(active) > 1 or not frozen:
                raise RuntimeError(
                    f"expected at most one active and at least one frozen trajectory; "
                    f"active={active}, frozen={frozen}")
            self.reference_trajectory_id = frozen[0]
            if not active:
                self.active_trajectory_id = None
                self._start_new_trajectory()
                return
            self.active_trajectory_id = active[0]
            request = FinishTrajectory.Request()
            request.trajectory_id = self.active_trajectory_id
            next_future = self.finish_client.call_async(request)
            next_future.add_done_callback(self._on_finished)
        except Exception as exc:
            self._fail(f"trajectory-state query failed: {exc}")

    def _on_finished(self, future):
        try:
            response = future.result()
            if response.status.code != 0:
                raise RuntimeError(response.status.message)
            self._start_new_trajectory()
        except Exception as exc:
            self._fail(f"finish trajectory failed: {exc}")

    def _start_new_trajectory(self):
        try:
            request = StartTrajectory.Request()
            request.configuration_directory = self.config_dir
            request.configuration_basename = self.config_name
            request.use_initial_pose = True
            request.initial_pose = self.expected_pose
            request.relative_to_trajectory_id = self.reference_trajectory_id
            next_future = self.start_client.call_async(request)
            next_future.add_done_callback(self._on_started)
        except Exception as exc:
            self._fail(f"start trajectory request failed: {exc}")

    def _on_started(self, future):
        try:
            response = future.result()
            if response.status.code != 0:
                raise RuntimeError(response.status.message)
            self.active_trajectory_id = response.trajectory_id
            self.state = "verifying"
            self.detail = f"trajectory {response.trajectory_id}"
            self.verify_since = None
            self.busy = False
            self.pending_search = False
        except Exception as exc:
            self._fail(f"start trajectory failed: {exc}")

    def _verify_pose(self):
        points = self._scan_points_in_base()
        if points is None:
            self.verify_since = None
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", self.base_frame, rclpy.time.Time())
        except Exception:
            self.verify_since = None
            return
        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        msg = self.map_msg
        row = int(round((transform.transform.translation.y
                         - msg.info.origin.position.y) / msg.info.resolution))
        col = int(round((transform.transform.translation.x
                         - msg.info.origin.position.x) / msg.info.resolution))
        score = self._score_pose(
            points[0], points[1], row, col, yaw, msg.info.resolution,
            msg.info.height, msg.info.width)
        self.best_score = score
        if score < self.min_score:
            self.verify_since = None
            self.detail = f"verification score {score:.3f} below {self.min_score:.3f}"
            return
        if self.verify_since is None:
            self.verify_since = time.monotonic()
            return
        if time.monotonic() - self.verify_since < self.verify_hold_sec:
            return
        self.state = "localized"
        self.detail = f"verified score={score:.3f}"
        self.manual_request = False
        self._publish_ready(True)
        self.get_logger().info(
            f"Localization verified; Nav2 may start (score={score:.3f})")

    def _fail(self, detail):
        self.busy = False
        self.pending_search = False
        self.state = "failed"
        self.detail = detail
        self._publish_ready(False)
        self.get_logger().error(detail)

    def _publish_ready(self, ready):
        msg = Bool()
        msg.data = bool(ready)
        self.ready_pub.publish(msg)

    def _publish_state(self):
        msg = String()
        msg.data = (
            f"{self.state}|score={self.best_score:.3f}|"
            f"second={self.second_score:.3f}|{self.detail}")
        self.state_pub.publish(msg)

    @staticmethod
    def _pose(x, y, yaw):
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.orientation.z = math.sin(yaw * 0.5)
        pose.orientation.w = math.cos(yaw * 0.5)
        return pose

    @staticmethod
    def _wrap(angle):
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = CartographerRelocalizer()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
