#!/usr/bin/env python3
"""Fail-closed Nav2 lifecycle gate for Cartographer relocalization."""

import rclpy
from nav2_msgs.srv import ClearEntireCostmap, ManageLifecycleNodes
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener


class LocalizationBringup(Node):
    """Start or resume Nav2 only while the verified localization gate is open."""

    def __init__(self):
        super().__init__("localization_bringup")
        self.declare_parameter(
            "manager_service", "/lifecycle_manager_navigation/manage_nodes")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("retry_period_sec", 0.5)

        service = str(self.get_parameter("manager_service").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        period = float(self.get_parameter("retry_period_sec").value)

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Bool, "/localization_ready", self._on_ready, qos)
        self.create_subscription(
            Bool, "/robot/navigation_sensor_healthy",
            self._on_sensor_health, 10)
        self.system_ready_pub = self.create_publisher(
            Bool, "/robot/system_ready", 10)
        self.state_pub = self.create_publisher(
            String, "/localization_bringup/state", 5)
        self.client = self.create_client(ManageLifecycleNodes, service)
        self.clear_local_client = self.create_client(
            ClearEntireCostmap,
            "/local_costmap/clear_entirely_local_costmap")
        self.clear_global_client = self.create_client(
            ClearEntireCostmap,
            "/global_costmap/clear_entirely_global_costmap")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.ready = False
        self.sensor_healthy = False
        self.started = False
        self.paused = False
        self.motion_released = False
        self.request_in_flight = False
        self.pending_command = None
        self.last_detail = "waiting for verified localization"
        self.create_timer(period, self._tick)

    def _on_ready(self, msg):
        self.ready = bool(msg.data)
        if not self.ready and self.started and not self.paused:
            self.pending_command = ManageLifecycleNodes.Request.PAUSE
            self.last_detail = "localization invalid; pausing Nav2"
        elif self.ready:
            self.pending_command = (
                ManageLifecycleNodes.Request.RESUME
                if self.started else ManageLifecycleNodes.Request.STARTUP)

    def _on_sensor_health(self, msg):
        self.sensor_healthy = bool(msg.data)

    def _publish_motion_gate(self):
        release = bool(
            self.ready and self.sensor_healthy and
            self.started and not self.paused)
        self.system_ready_pub.publish(Bool(data=release))
        if release == self.motion_released:
            return
        self.motion_released = release
        if release:
            self.get_logger().info(
                "Startup motion gate released in-process: localization, "
                "Nav2 lifecycle and collision sensors are ready")
        else:
            self.get_logger().warn(
                "Startup motion gate locked: localization, Nav2 lifecycle "
                "or collision-sensor readiness was lost")

    def _tick(self):
        self._publish_motion_gate()
        if self.request_in_flight:
            return
        if not self.client.service_is_ready():
            self.last_detail = "waiting for Nav2 lifecycle manager"
            self._publish_state()
            return
        if self.ready and not self.tf_buffer.can_transform(
                self.map_frame, self.base_frame, rclpy.time.Time()):
            self.last_detail = "verified gate open; waiting for map->base_link TF"
            self._publish_state()
            return
        if self.pending_command is None:
            self._publish_state()
            return
        request = ManageLifecycleNodes.Request()
        request.command = self.pending_command
        command = self.pending_command
        self.pending_command = None
        self.request_in_flight = True
        future = self.client.call_async(request)
        future.add_done_callback(
            lambda result, cmd=command: self._on_command_done(result, cmd))

    def _on_command_done(self, future, command):
        self.request_in_flight = False
        try:
            response = future.result()
            if not response.success:
                raise RuntimeError("lifecycle manager rejected command")
            if command == ManageLifecycleNodes.Request.STARTUP:
                self.started = True
                self.paused = False
                self.last_detail = "Nav2 active after verified startup localization"
            elif command == ManageLifecycleNodes.Request.PAUSE:
                self.paused = True
                self.last_detail = "Nav2 paused for manual relocalization"
            elif command == ManageLifecycleNodes.Request.RESUME:
                self.paused = False
                self.last_detail = "Nav2 resumed after verified relocalization"
                self._clear_costmaps()
        except Exception as exc:
            self.last_detail = f"lifecycle command failed: {exc}; retrying"
            self.pending_command = command
            self.get_logger().error(self.last_detail)
        self._publish_state()

    def _clear_costmaps(self):
        for client in (self.clear_local_client, self.clear_global_client):
            if client.service_is_ready():
                client.call_async(ClearEntireCostmap.Request())

    def _publish_state(self):
        msg = String()
        msg.data = self.last_detail
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationBringup()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Humble exposes shutdown wait-set failures only through its private
        # pybind module, not rclpy.exceptions. Suppress them strictly after the
        # shared ROS context has already been shut down.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
