#!/usr/bin/env python3
"""Keep the existing web API while using the native Jazzy frontier service."""

import json
import time

import rclpy
from action_msgs.msg import GoalStatus
from action_msgs.msg import GoalStatusArray
from frontier_exploration_ros2.srv import ControlExploration
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, String
from std_srvs.srv import SetBool


class FrontierWebBridge(Node):
    def __init__(self):
        super().__init__("frontier_web_bridge")
        self.declare_parameter(
            "frontier_control_service", "/control_exploration")
        self.declare_parameter("web_control_topic", "/robot/web_control")
        self.declare_parameter("status_topic", "/auto_mapping/status")
        self.declare_parameter("set_enabled_service", "/auto_mapping/set_enabled")
        self.declare_parameter("initial_enabled", False)

        self.desired_enabled = bool(self.get_parameter("initial_enabled").value)
        self.enabled = self.desired_enabled
        self.goal_active = False
        self.completed = False
        self.state = "running" if self.enabled else "disabled"
        self.message = (
            "Jazzy frontier autostart is active" if self.enabled
            else "Jazzy frontier is idle")
        self.request_in_flight = False
        self.pending_enable = None
        self.stop_started_at = 0.0

        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), status_qos)
        self.control_client = self.create_client(
            ControlExploration,
            str(self.get_parameter("frontier_control_service").value),
        )
        self.set_enabled_service = self.create_service(
            SetBool,
            str(self.get_parameter("set_enabled_service").value),
            self._on_set_enabled,
        )
        self.web_sub = self.create_subscription(
            String,
            str(self.get_parameter("web_control_topic").value),
            self._on_web_control,
            10,
        )
        self.selected_sub = self.create_subscription(
            PoseStamped, "/explore/selected_frontier",
            self._on_selected_frontier, 10)
        self.completion_sub = self.create_subscription(
            Empty, "/exploration_complete", self._on_completion, 10)
        self.action_status_sub = self.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self._on_action_status,
            10,
        )
        self.create_timer(0.5, self._on_timer)
        self._publish_status()
        self.get_logger().info(
            "Web auto-mapping bridge ready for native Jazzy frontier service")

    def _on_web_control(self, msg: String):
        try:
            command = json.loads(msg.data or "{}").get("command")
        except json.JSONDecodeError:
            return
        if command == "auto_mapping_start":
            self._request_enabled(True)
        elif command == "auto_mapping_stop":
            self._request_enabled(False)

    def _on_set_enabled(self, request: SetBool.Request,
                        response: SetBool.Response):
        enable = bool(request.data)
        was_goal_active = self.goal_active
        accepted = self._request_enabled(enable)
        response.success = accepted
        if accepted:
            response.message = "frontier_request_queued=true"
            if not enable and was_goal_active:
                response.message += "; nav_goal_canceling=true"
        else:
            response.message = "frontier control request is already pending"
        return response

    def _request_enabled(self, enable: bool) -> bool:
        self.desired_enabled = enable
        self.completed = False if enable else self.completed
        if self.request_in_flight:
            self.pending_enable = enable
            self.state = "switching"
            self._publish_status()
            return True
        if not self.control_client.service_is_ready():
            self.control_client.wait_for_service(timeout_sec=0.25)
        if not self.control_client.service_is_ready():
            self.state = "service_unavailable"
            self.message = "frontier control service is unavailable"
            self._publish_status()
            return False

        request = ControlExploration.Request()
        request.action = (
            ControlExploration.Request.ACTION_START if enable
            else ControlExploration.Request.ACTION_STOP)
        request.delay_seconds = 0.0
        request.quit_after_stop = False
        self.request_in_flight = True
        self.state = "starting" if enable else "stopping"
        if not enable:
            self.stop_started_at = time.monotonic()
        self._publish_status()
        future = self.control_client.call_async(request)
        future.add_done_callback(
            lambda done, requested=enable: self._on_control_response(done, requested))
        return True

    def _on_control_response(self, future, requested: bool):
        self.request_in_flight = False
        try:
            result = future.result()
        except Exception as exc:
            self.state = "error"
            self.message = str(exc)
            self._publish_status()
            return

        self.message = result.message
        if not result.accepted:
            self.state = "rejected"
        elif requested:
            self.enabled = True
            self.state = "running"
        else:
            self.enabled = False
            self.state = "stopping" if self.goal_active else "disabled"
        self._publish_status()

        pending = self.pending_enable
        self.pending_enable = None
        if pending is not None and pending != requested:
            self._request_enabled(pending)

    def _on_selected_frontier(self, _msg: PoseStamped):
        if self.enabled:
            self.goal_active = True
            self.state = "navigating"
            self._publish_status()

    def _on_action_status(self, msg: GoalStatusArray):
        active_states = {
            GoalStatus.STATUS_ACCEPTED,
            GoalStatus.STATUS_EXECUTING,
            GoalStatus.STATUS_CANCELING,
        }
        any_active = any(item.status in active_states for item in msg.status_list)
        if self.enabled:
            self.goal_active = any_active
            if any_active:
                self.state = "navigating"
            elif self.state == "navigating":
                self.state = "running"
        elif self.state == "stopping":
            self.goal_active = any_active
            if not any_active and time.monotonic() - self.stop_started_at >= 0.20:
                self.state = "disabled"
        self._publish_status()

    def _on_completion(self, _msg: Empty):
        self.desired_enabled = False
        self.enabled = False
        self.goal_active = False
        self.completed = True
        self.state = "completed"
        self.message = "frontier exploration completed"
        self._publish_status()

    def _on_timer(self):
        if self.state == "stopping" and (
                time.monotonic() - self.stop_started_at > 2.5):
            self.goal_active = False
            self.state = "disabled"
            self.message = "frontier stop grace period completed"
        self._publish_status()

    def _publish_status(self):
        msg = String()
        msg.data = json.dumps({
            "enabled": bool(self.enabled),
            "desired_enabled": bool(self.desired_enabled),
            "goal_active": bool(self.goal_active),
            "completed": bool(self.completed),
            "state": self.state,
            "message": self.message,
            "backend": "frontier_exploration_ros2_jazzy",
        }, ensure_ascii=False)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierWebBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
