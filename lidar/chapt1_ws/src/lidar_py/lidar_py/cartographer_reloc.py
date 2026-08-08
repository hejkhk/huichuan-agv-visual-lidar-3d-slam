#!/usr/bin/env python3
"""Startup-only global relocalization for a frozen Cartographer map.

The scan-to-grid matcher is adapted from the all.beifen implementation, but
navigation is released only after a verified match. Runtime monitoring never
changes the trajectory automatically; a new search can only be requested from
the localization RViz panel.
"""

import copy
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
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from .relocalization_logic import (
    BootstrapPoseGate,
    ImmutableCrcLock,
    PoseConsensus,
    occupancy_grid_crc,
    refine_distinct_candidates,
)


def quaternion_yaw(rotation):
    """Return planar yaw from a geometry_msgs quaternion."""
    return math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
    )


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
        self.declare_parameter("strong_match_score", 0.75)
        self.declare_parameter("strong_match_min_margin", 0.012)
        self.declare_parameter("ambiguous_match_min_score", 0.75)
        self.declare_parameter("ambiguous_consensus_required_scans", 6)
        self.declare_parameter("auto_retry_interval_sec", 5.0)
        self.declare_parameter("max_auto_attempts", 5)
        self.declare_parameter("max_wait_sec", 120.0)
        self.declare_parameter("stationary_hold_sec", 1.0)
        self.declare_parameter("verify_hold_sec", 2.0)
        self.declare_parameter("trajectory_restart_delay_sec", 1.0)
        self.declare_parameter("max_verify_tf_age_sec", 0.75)
        self.declare_parameter("min_verify_tf_advance_sec", 0.50)
        self.declare_parameter("verify_timeout_sec", 8.0)
        self.declare_parameter("manual_same_pose_translation_m", 0.15)
        self.declare_parameter("manual_same_pose_yaw_deg", 5.0)
        self.declare_parameter("max_refined_candidates", 8)
        self.declare_parameter("candidate_cluster_translation_m", 0.8)
        self.declare_parameter("candidate_cluster_yaw_deg", 20.0)
        self.declare_parameter("consensus_required_scans", 3)
        self.declare_parameter("consensus_translation_m", 0.35)
        self.declare_parameter("consensus_yaw_deg", 12.0)
        self.declare_parameter("verify_expected_translation_m", 0.50)
        self.declare_parameter("verify_expected_yaw_deg", 20.0)
        self.declare_parameter("bootstrap_enabled", True)
        self.declare_parameter("bootstrap_min_match_score", 0.55)
        self.declare_parameter("bootstrap_hold_sec", 4.0)
        self.declare_parameter("bootstrap_min_observations", 8)
        self.declare_parameter("bootstrap_max_translation_delta_m", 0.20)
        self.declare_parameter("bootstrap_max_yaw_delta_deg", 5.0)

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
        self.strong_match_score = float(
            self.get_parameter("strong_match_score").value)
        self.strong_match_min_margin = float(
            self.get_parameter("strong_match_min_margin").value)
        self.ambiguous_match_min_score = max(
            self.min_score,
            float(self.get_parameter("ambiguous_match_min_score").value),
        )
        self.ambiguous_consensus_required_scans = max(
            4,
            int(self.get_parameter(
                "ambiguous_consensus_required_scans").value),
        )
        self.auto_retry_interval_sec = max(0.25, float(
            self.get_parameter("auto_retry_interval_sec").value))
        self.max_auto_attempts = max(
            1, int(self.get_parameter("max_auto_attempts").value))
        self.max_wait_sec = float(self.get_parameter("max_wait_sec").value)
        self.stationary_hold_sec = float(
            self.get_parameter("stationary_hold_sec").value)
        self.verify_hold_sec = float(
            self.get_parameter("verify_hold_sec").value)
        self.trajectory_restart_delay_sec = max(0.5, float(
            self.get_parameter("trajectory_restart_delay_sec").value))
        self.max_verify_tf_age_sec = max(0.2, float(
            self.get_parameter("max_verify_tf_age_sec").value))
        self.min_verify_tf_advance_sec = max(0.1, float(
            self.get_parameter("min_verify_tf_advance_sec").value))
        self.verify_timeout_sec = max(self.verify_hold_sec + 1.0, float(
            self.get_parameter("verify_timeout_sec").value))
        self.manual_same_pose_translation_m = max(0.0, float(
            self.get_parameter("manual_same_pose_translation_m").value))
        self.manual_same_pose_yaw = math.radians(max(0.0, float(
            self.get_parameter("manual_same_pose_yaw_deg").value)))
        self.max_refined_candidates = max(2, int(
            self.get_parameter("max_refined_candidates").value))
        self.candidate_cluster_translation_m = max(0.10, float(
            self.get_parameter("candidate_cluster_translation_m").value))
        self.candidate_cluster_yaw = math.radians(max(1.0, float(
            self.get_parameter("candidate_cluster_yaw_deg").value)))
        self.verify_expected_translation_m = max(0.10, float(
            self.get_parameter("verify_expected_translation_m").value))
        self.verify_expected_yaw = math.radians(max(1.0, float(
            self.get_parameter("verify_expected_yaw_deg").value)))
        self.consensus = PoseConsensus(
            int(self.get_parameter("consensus_required_scans").value),
            float(self.get_parameter("consensus_translation_m").value),
            math.radians(float(
                self.get_parameter("consensus_yaw_deg").value)),
            self.ambiguous_consensus_required_scans,
        )
        self.bootstrap_enabled = bool(
            self.get_parameter("bootstrap_enabled").value)
        self.bootstrap_gate = BootstrapPoseGate(
            float(self.get_parameter("bootstrap_min_match_score").value),
            float(self.get_parameter("bootstrap_hold_sec").value),
            int(self.get_parameter("bootstrap_min_observations").value),
            float(self.get_parameter(
                "bootstrap_max_translation_delta_m").value),
            math.radians(float(self.get_parameter(
                "bootstrap_max_yaw_delta_deg").value)),
        )

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
        # LiDAR and laser_filters publish sensor streams as BEST_EFFORT.  A
        # default RELIABLE subscription silently receives zero messages from
        # that stream in ROS 2, which leaves localization waiting forever.
        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, 20)
        self.create_subscription(
            Bool, "/cartographer_reloc/trigger", self._on_manual_trigger, 1)
        self.ready_pub = self.create_publisher(
            Bool, "/localization_ready", gate_qos)
        self.state_pub = self.create_publisher(
            String, "/cartographer_reloc/state", 5)

        self.states_client = self.create_client(
            GetTrajectoryStates,
            "/get_trajectory_states")
        self.finish_client = self.create_client(
            FinishTrajectory, "/finish_trajectory")
        self.start_client = self.create_client(
            StartTrajectory, "/start_trajectory")

        self.map_msg = None
        self.map_data = None
        self.score_grid = None
        self.reference_map_lock = ImmutableCrcLock()
        self.rejected_map_crcs = set()
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
        self.verify_started_at = None
        self.verify_tf_first_stamp_ns = None
        self.verify_tf_last_stamp_ns = None
        self.active_trajectory_confirmed = False
        self.trajectory_state_request_pending = False
        self.last_trajectory_state_request_at = 0.0
        self.restart_not_before = 0.0
        self.expected_pose = None
        self.active_trajectory_id = None
        self.reference_trajectory_id = None
        self.search_attempts = 0
        self.next_search_at = 0.0
        self.bootstrap_completed = not self.bootstrap_enabled
        self._last_logged_state = None

        self._publish_ready(False)
        self.create_timer(0.25, self._tick)
        self.create_timer(0.5, self._publish_state)
        self.get_logger().info(
            "Startup relocalizer armed; Nav2 remains inactive until verified")

    def _on_map(self, msg):
        if msg.info.width == 0 or msg.info.height == 0:
            return
        origin = msg.info.origin
        signature = occupancy_grid_crc(
            msg.info.width,
            msg.info.height,
            msg.info.resolution,
            (
                origin.position.x,
                origin.position.y,
                origin.position.z,
                origin.orientation.x,
                origin.orientation.y,
                origin.orientation.z,
                origin.orientation.w,
            ),
            msg.data,
        )
        already_locked = self.reference_map_lock.crc is not None
        accepted = self.reference_map_lock.accept(signature)
        if already_locked:
            if not accepted:
                if signature not in self.rejected_map_crcs:
                    self.rejected_map_crcs.add(signature)
                    self.get_logger().error(
                        "REFERENCE_MAP_MUTATION_REJECTED "
                        f"locked=0x{self.reference_map_lock.crc:08x} "
                        f"incoming=0x{signature:08x}; relocalization keeps "
                        "the originally selected map")
            return

        self.map_msg = copy.deepcopy(msg)
        self.map_data = np.asarray(
            self.map_msg.data, dtype=np.int16).reshape(
                self.map_msg.info.height, self.map_msg.info.width
            ).copy()
        self.map_data.setflags(write=False)
        self._build_score_grid()
        self.get_logger().info(
            "REFERENCE_MAP_LOCKED "
            f"topic={self.map_topic} crc32=0x{signature:08x} "
            f"size={msg.info.width}x{msg.info.height} "
            f"resolution={msg.info.resolution:.6f} "
            f"origin=({origin.position.x:.3f},{origin.position.y:.3f})")

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
        self.search_attempts = 0
        self.next_search_at = 0.0
        self.consensus.reset()
        self.state = "waiting_stop"
        self.detail = "manual request; Nav2 paused"
        self._publish_ready(False)

    def _tick(self):
        if self.state == "restart_wait":
            if time.monotonic() < self.restart_not_before:
                return
            if not self.start_client.service_is_ready():
                if time.monotonic() - self.restart_not_before > 5.0:
                    self._fail("Cartographer start service disappeared during restart")
                return
            self._start_new_trajectory()
            return
        if self.state == "verifying":
            self._verify_pose()
            return
        if (self.bootstrap_enabled and not self.bootstrap_completed
                and not self.manual_request):
            self._bootstrap_tick()
            return
        if not self.pending_search or self.busy:
            return
        if time.monotonic() < self.next_search_at:
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
            self.detail = (
                "trajectory services "
                f"states={self.states_client.service_is_ready()} "
                f"finish={self.finish_client.service_is_ready()} "
                f"start={self.start_client.service_is_ready()}")
            return
        self._start_search()

    def _bootstrap_tick(self):
        """Validate Cartographer's organic PBStream localization."""

        if time.monotonic() - self.started_at > self.max_wait_sec:
            self._fail("Cartographer bootstrap localization timed out")
            return
        if self.map_msg is None or self.score_grid is None:
            self.state = "bootstrap_wait_map"
            self.detail = self.map_topic
            return
        if self.latest_scan is None:
            self.state = "bootstrap_wait_scan"
            self.detail = self.scan_topic
            return
        if self.stationary_since is None or (
                time.monotonic() - self.stationary_since
                < self.stationary_hold_sec):
            self.bootstrap_gate.reset()
            self.state = "bootstrap_wait_stop"
            self.detail = "hold the vehicle still during startup localization"
            return
        if not (self.states_client.service_is_ready()
                and self.finish_client.service_is_ready()
                and self.start_client.service_is_ready()):
            self.state = "bootstrap_wait_cartographer"
            self.detail = "waiting for trajectory services"
            return
        points = self._scan_points_in_base()
        if points is None:
            self.state = "bootstrap_wait_scan"
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", self.base_frame, rclpy.time.Time())
        except Exception as exc:
            self.state = "bootstrap_wait_tf"
            self.detail = f"map TF unavailable: {exc}"
            return

        stamp_ns = (
            transform.header.stamp.sec * 1000000000
            + transform.header.stamp.nanosec)
        now_ns = self.get_clock().now().nanoseconds
        age_sec = (now_ns - stamp_ns) / 1e9 if stamp_ns > 0 else float("inf")
        if stamp_ns <= 0 or age_sec < -0.10 or age_sec > self.max_verify_tf_age_sec:
            self.bootstrap_gate.reset()
            self.state = "bootstrap_wait_tf"
            self.detail = f"waiting for fresh map TF: age={age_sec:.3f}s"
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
        result = self.bootstrap_gate.observe(
            transform.transform.translation.x,
            transform.transform.translation.y,
            yaw,
            stamp_ns,
            time.monotonic(),
            score,
        )
        self.state = "bootstrap_stabilizing"
        reset_text = " reset after pose/score change;" if result.reset else ""
        self.detail = (
            f"{reset_text} score={score:.3f} stable={result.count}/"
            f"{self.bootstrap_gate.min_observations} "
            f"hold={result.duration_sec:.1f}/{self.bootstrap_gate.hold_sec:.1f}s")
        if not result.ready:
            return

        mean_pose = result.pose
        self.expected_pose = self._pose(*mean_pose)
        self.bootstrap_completed = True
        self.busy = True
        self.pending_search = False
        self.state = "restarting_trajectory"
        self.detail = (
            "Cartographer bootstrap verified; switching to tight tracking "
            f"pose=({mean_pose[0]:.2f},{mean_pose[1]:.2f},"
            f"{math.degrees(mean_pose[2]):.1f}deg) score={score:.3f}")
        self.get_logger().info(f"BOOTSTRAP_LOCALIZATION_ACCEPTED {self.detail}")
        future = self.states_client.call_async(GetTrajectoryStates.Request())
        future.add_done_callback(self._on_states)

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
        self.search_attempts += 1
        self.state = "searching"
        self.detail = "global scan-to-map search"
        self._publish_state()
        pose, best, second, match_mode = self._global_match(*points)
        self.best_score = best
        self.second_score = second
        if pose is None:
            self.busy = False
            self.consensus.reset()
            retry_available = (
                not self.manual_request
                and self.search_attempts < self.max_auto_attempts
                and time.monotonic() - self.started_at < self.max_wait_sec)
            if retry_available:
                self.pending_search = True
                self.next_search_at = (
                    time.monotonic() + self.auto_retry_interval_sec)
                self.state = "retry_wait"
                self.detail = (
                    f"ambiguous/weak match best={best:.3f} "
                    f"second={second:.3f}; automatic retry "
                    f"{self.search_attempts + 1}/{self.max_auto_attempts}")
            else:
                self.pending_search = False
                self.state = "failed"
                self.detail = (
                    f"ambiguous/weak match best={best:.3f} "
                    f"second={second:.3f}; use RViz Relocalize at a more "
                    "distinctive stationary location")
            self._publish_ready(False)
            return
        scan_stamp_ns = self._scan_stamp_ns(self.latest_scan)
        yaw = quaternion_yaw(pose.orientation)
        consensus = self.consensus.observe(
            pose.position.x,
            pose.position.y,
            yaw,
            scan_stamp_ns,
            extended=match_mode == "temporal",
        )
        consensus_mode = (
            "temporal" if self.consensus.requires_extended else "strict"
        )
        if consensus.duplicate:
            self.busy = False
            self.pending_search = True
            self.next_search_at = time.monotonic() + self.auto_retry_interval_sec
            self.state = "consensus_wait"
            self.detail = "waiting for a newer LiDAR scan timestamp"
            return
        if not consensus.ready:
            self.busy = False
            self.pending_search = True
            self.next_search_at = time.monotonic() + self.auto_retry_interval_sec
            if (self.consensus.requires_extended
                    and consensus.count >= self.consensus.active_required_count):
                self.pending_search = False
                self.state = "failed"
                self.detail = (
                    "stationary scan remains ambiguous after repeated "
                    "observations; move to a distinctive location and retry")
                self._publish_ready(False)
                return
            self.state = "consensus_wait"
            reset_text = " candidate jump reset consensus;" if consensus.reset else ""
            self.detail = (
                f"{reset_text} stable matches={consensus.count}/"
                f"{self.consensus.active_required_count} "
                f"mode={consensus_mode} pose="
                f"({consensus.pose[0]:.2f},{consensus.pose[1]:.2f},"
                f"{math.degrees(consensus.pose[2]):.1f}deg)")
            self.get_logger().info(f"RELOCALIZATION_CONSENSUS {self.detail}")
            return

        mean_pose = consensus.pose
        self.expected_pose = self._pose(*mean_pose)
        self.get_logger().info(
            "RELOCALIZATION_CONSENSUS accepted "
            f"{consensus.count}/{self.consensus.active_required_count} "
            f"distinct scans mode={consensus_mode} "
            f"pose=({mean_pose[0]:.2f},{mean_pose[1]:.2f},"
            f"{math.degrees(mean_pose[2]):.1f}deg)")
        if self._keep_healthy_manual_trajectory(self.expected_pose):
            return
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
            return None, 0.0, 0.0, "rejected"

        rows = np.arange(0, height, self.coarse_step, dtype=np.int32)
        cols = np.arange(0, width, self.coarse_step, dtype=np.int32)
        grid_c, grid_r = np.meshgrid(cols, rows)
        candidate_r = grid_r.ravel()
        candidate_c = grid_c.ravel()
        free = self.map_data[candidate_r, candidate_c] == 0
        candidate_r = candidate_r[free]
        candidate_c = candidate_c[free]
        if candidate_r.size == 0:
            return None, 0.0, 0.0, "rejected"

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
            return None, 0.0, 0.0, "rejected"
        refined = refine_distinct_candidates(
            results,
            lambda seed: self._refine_match(bx, by, seed, resolution),
            resolution,
            self.candidate_cluster_translation_m,
            self.candidate_cluster_yaw,
            self.max_refined_candidates,
        )
        if not refined:
            return None, 0.0, 0.0, "rejected"
        best = refined[0]
        second_candidate = refined[1] if len(refined) > 1 else None
        second = second_candidate[0] if second_candidate is not None else 0.0
        score, row, col, yaw = best
        margin = score - second
        best_x = origin_x + col * resolution
        best_y = origin_y + row * resolution
        second_text = "none"
        if second_candidate is not None:
            second_text = (
                f"({origin_x + second_candidate[2] * resolution:.2f},"
                f"{origin_y + second_candidate[1] * resolution:.2f},"
                f"{math.degrees(second_candidate[3]):.1f}deg)")
        self.get_logger().info(
            f"Relocalization match: best={score:.3f}, second={second:.3f}, "
            f"margin={margin:.3f}, "
            f"best_pose=({best_x:.2f},{best_y:.2f},"
            f"{math.degrees(yaw):.1f}deg), second_pose={second_text}, "
            f"refined_clusters={len(refined)}")
        normal_match = score >= self.min_score and margin >= self.min_margin
        strong_match = (
            score >= self.strong_match_score
            and margin >= self.strong_match_min_margin)
        if normal_match or strong_match:
            if strong_match and not normal_match:
                self.get_logger().warn(
                    "Accepting high-confidence match through strong-score gate: "
                    f"score={score:.3f} margin={margin:.3f}")
            return self._pose(best_x, best_y, yaw), score, second, "strict"
        if score >= self.ambiguous_match_min_score:
            self.get_logger().warn(
                "Ambiguous high-score match admitted to extended temporal "
                f"consensus only: score={score:.3f} margin={margin:.3f} "
                f"required_scans={self.ambiguous_consensus_required_scans}")
            return self._pose(best_x, best_y, yaw), score, second, "temporal"
        return None, score, second, "rejected"

    def _keep_healthy_manual_trajectory(self, pose):
        """Avoid restarting Cartographer when a manual match confirms its pose."""
        if not self.manual_request or self.active_trajectory_id is None:
            return False
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", self.base_frame, rclpy.time.Time())
        except Exception:
            return False
        stamp_ns = (
            transform.header.stamp.sec * 1000000000
            + transform.header.stamp.nanosec)
        now_ns = self.get_clock().now().nanoseconds
        if stamp_ns <= 0 or (now_ns - stamp_ns) / 1e9 > self.max_verify_tf_age_sec:
            return False
        current_yaw = quaternion_yaw(transform.transform.rotation)
        candidate_yaw = quaternion_yaw(pose.orientation)
        distance = math.hypot(
            pose.position.x - transform.transform.translation.x,
            pose.position.y - transform.transform.translation.y)
        yaw_error = abs(self._wrap(candidate_yaw - current_yaw))
        if (distance > self.manual_same_pose_translation_m
                or yaw_error > self.manual_same_pose_yaw):
            return False
        self.pending_search = False
        self.busy = False
        self.manual_request = False
        self.state = "localized"
        self.detail = (
            f"manual match confirms active trajectory: "
            f"delta={distance:.3f}m/{math.degrees(yaw_error):.2f}deg")
        self._publish_ready(True)
        self.get_logger().info(self.detail)
        return True

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
            # Cartographer's sensor collator keeps the last timestamp from the
            # finished trajectory. Chassis IMU stamps trail wall time by a few
            # hundred milliseconds, so starting immediately can feed an older
            # first IMU sample and abort the whole Cartographer process.
            self.state = "restart_wait"
            self.detail = (
                "finished old trajectory; draining sensor timestamps before "
                "restart")
            self.restart_not_before = (
                time.monotonic() + self.trajectory_restart_delay_sec)
            self._publish_state()
        except Exception as exc:
            self._fail(f"finish trajectory failed: {exc}")

    def _start_new_trajectory(self):
        try:
            self.state = "starting_trajectory"
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
            self.verify_started_at = time.monotonic()
            self.verify_tf_first_stamp_ns = None
            self.verify_tf_last_stamp_ns = None
            self.active_trajectory_confirmed = False
            self.trajectory_state_request_pending = False
            self.last_trajectory_state_request_at = 0.0
            self.busy = False
            self.pending_search = False
        except Exception as exc:
            self._fail(f"start trajectory failed: {exc}")

    def _verify_pose(self):
        now_monotonic = time.monotonic()
        if (self.verify_started_at is not None
                and now_monotonic - self.verify_started_at
                > self.verify_timeout_sec):
            self._fail(
                "new Cartographer trajectory did not produce a fresh, "
                "verified map transform")
            return
        if not (self.states_client.service_is_ready()
                and self.start_client.service_is_ready()):
            self.verify_since = None
            self.detail = "Cartographer trajectory services are unavailable"
            return
        if (not self.trajectory_state_request_pending
                and now_monotonic - self.last_trajectory_state_request_at >= 0.5):
            self.trajectory_state_request_pending = True
            self.last_trajectory_state_request_at = now_monotonic
            future = self.states_client.call_async(GetTrajectoryStates.Request())
            future.add_done_callback(self._on_verify_states)
        if not self.active_trajectory_confirmed:
            self.verify_since = None
            self.detail = (
                f"waiting for active trajectory {self.active_trajectory_id}")
            return
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
        stamp_ns = (
            transform.header.stamp.sec * 1000000000
            + transform.header.stamp.nanosec)
        now_ns = self.get_clock().now().nanoseconds
        age_sec = (now_ns - stamp_ns) / 1e9 if stamp_ns > 0 else float("inf")
        if stamp_ns <= 0 or age_sec < -0.10 or age_sec > self.max_verify_tf_age_sec:
            self.verify_since = None
            self.detail = f"waiting for fresh map TF: age={age_sec:.3f}s"
            return
        if self.verify_tf_first_stamp_ns is None:
            self.verify_tf_first_stamp_ns = stamp_ns
        self.verify_tf_last_stamp_ns = max(
            stamp_ns, self.verify_tf_last_stamp_ns or stamp_ns)
        tf_advance_sec = (
            self.verify_tf_last_stamp_ns - self.verify_tf_first_stamp_ns) / 1e9
        if tf_advance_sec < self.min_verify_tf_advance_sec:
            self.verify_since = None
            self.detail = (
                f"waiting for advancing map TF: advanced={tf_advance_sec:.3f}s")
            return
        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if self.expected_pose is not None:
            expected_yaw = quaternion_yaw(self.expected_pose.orientation)
            expected_distance = math.hypot(
                transform.transform.translation.x - self.expected_pose.position.x,
                transform.transform.translation.y - self.expected_pose.position.y,
            )
            expected_yaw_error = abs(self._wrap(yaw - expected_yaw))
            if (expected_distance > self.verify_expected_translation_m
                    or expected_yaw_error > self.verify_expected_yaw):
                self.verify_since = None
                self.detail = (
                    "fresh Cartographer pose disagrees with multi-scan "
                    f"consensus: delta={expected_distance:.3f}m/"
                    f"{math.degrees(expected_yaw_error):.1f}deg")
                return
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

    def _on_verify_states(self, future):
        self.trajectory_state_request_pending = False
        if self.state != "verifying":
            return
        try:
            response = future.result()
            if response.status.code != 0:
                raise RuntimeError(response.status.message)
            active = [
                trajectory_id
                for trajectory_id, state in zip(
                    response.trajectory_states.trajectory_id,
                    response.trajectory_states.trajectory_state)
                if state == TrajectoryStates.ACTIVE
            ]
            if self.active_trajectory_id not in active:
                self._fail(
                    f"Cartographer trajectory {self.active_trajectory_id} "
                    f"is not active; active={active}")
                return
            self.active_trajectory_confirmed = True
        except Exception as exc:
            self._fail(f"trajectory verification failed: {exc}")

    def _fail(self, detail):
        self.busy = False
        self.pending_search = False
        self.state = "failed"
        self.detail = detail
        self.consensus.reset()
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
        if self.state != self._last_logged_state:
            self.get_logger().info(
                f"Relocalization state={self.state}: {self.detail}")
            self._last_logged_state = self.state

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

    @staticmethod
    def _scan_stamp_ns(scan):
        if scan is None:
            return 0
        return int(scan.header.stamp.sec) * 1000000000 + int(
            scan.header.stamp.nanosec)


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
