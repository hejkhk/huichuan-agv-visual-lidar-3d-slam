#!/usr/bin/env python3
"""Bridge web map goals into the Nav2 NavigateToPose action."""

import json
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def is_default_orientation(q: Quaternion) -> bool:
    return (
        abs(q.x) < 1e-6
        and abs(q.y) < 1e-6
        and abs(q.z) < 1e-6
        and abs(q.w - 1.0) < 1e-6
    )


class WebGoalNavNode(Node):
    def __init__(self):
        super().__init__("web_goal_nav_node")
        self.declare_parameter("goal_topic", "/web/nav_goal")
        self.declare_parameter("robot_pose_topic", "/robot_pose")
        self.declare_parameter("nav_action", "navigate_to_pose")
        self.declare_parameter("force_yaw_to_goal", True)
        self.declare_parameter("goal_frame", "map")
        self.declare_parameter("disable_auto_mapping_on_goal", True)
        self.declare_parameter("auto_mapping_service", "/auto_mapping/set_enabled")

        self.force_yaw_to_goal = bool(self.get_parameter("force_yaw_to_goal").value)
        self.goal_frame = str(self.get_parameter("goal_frame").value)
        self.last_pose = None
        self.current_goal_handle = None
        self.goal_sequence = 0
        self.pending_goal = None
        self.goal_request_in_flight = False
        self.cancel_ready_sequence = 0
        self.pause_ready_sequence = 0
        self.pause_wait_sequence = 0
        self.pause_wait_deadline = 0.0
        self.server_retry_sequence = 0
        self.server_retry_deadline = 0.0
        self.auto_mapping_goal_active = False

        self.client = ActionClient(
            self,
            NavigateToPose,
            str(self.get_parameter("nav_action").value),
        )
        self.auto_mapping_client = self.create_client(
            SetBool,
            str(self.get_parameter("auto_mapping_service").value),
        )
        self.goal_sub = self.create_subscription(
            PoseStamped,
            str(self.get_parameter("goal_topic").value),
            self._on_goal,
            10,
        )
        self.pose_sub = self.create_subscription(
            PoseStamped,
            str(self.get_parameter("robot_pose_topic").value),
            self._on_pose,
            10,
        )
        self.web_control_sub = self.create_subscription(
            String,
            "/robot/web_control",
            self._on_web_control,
            10,
        )
        self.emergency_stop_sub = self.create_subscription(
            Bool,
            "/robot/emergency_stop_state",
            self._on_emergency_stop,
            10,
        )
        self.auto_mapping_status_sub = self.create_subscription(
            String,
            "/auto_mapping/status",
            self._on_auto_mapping_status,
            10,
        )
        self.create_timer(0.1, self._on_pause_wait_timer)
        self.get_logger().info("Web goal Nav2 action bridge started")

    def _on_pose(self, msg: PoseStamped):
        self.last_pose = msg.pose

    def _on_web_control(self, msg: String):
        try:
            data = json.loads(msg.data or "{}")
        except json.JSONDecodeError:
            return
        if data.get("command") in (
                "cancel_nav_goal",
                "clear_nav_goal",
                "emergency_stop"):
            self.goal_sequence += 1
            self.pending_goal = None
            self.pause_wait_sequence = 0
            self.server_retry_sequence = 0
            self._cancel_current_goal()

    def _on_emergency_stop(self, msg: Bool):
        if not msg.data:
            return
        if (
                self.current_goal_handle is None
                and self.pending_goal is None
                and not self.goal_request_in_flight
                and self.pause_wait_sequence == 0
                and self.server_retry_sequence == 0):
            return
        self.goal_sequence += 1
        self.pending_goal = None
        self.pause_wait_sequence = 0
        self.server_retry_sequence = 0
        self._cancel_current_goal()

    def _on_auto_mapping_status(self, msg: String):
        try:
            data = json.loads(msg.data or "{}")
        except json.JSONDecodeError:
            return
        self.auto_mapping_goal_active = bool(data.get("goal_active", False))
        if self.pause_wait_sequence == self.goal_sequence \
                and not self.auto_mapping_goal_active:
            sequence = self.pause_wait_sequence
            self.pause_wait_sequence = 0
            self.pause_ready_sequence = sequence
            self._try_send_pending_goal(sequence)

    def _on_pause_wait_timer(self):
        sequence = self.pause_wait_sequence
        if sequence != 0 and sequence == self.goal_sequence and (
                not self.auto_mapping_goal_active
                or time.monotonic() >= self.pause_wait_deadline):
            if self.auto_mapping_goal_active:
                self.get_logger().warn(
                    "Timed out waiting for the frontier Nav2 goal to cancel; "
                    "sending the confirmed web goal")
            self.pause_wait_sequence = 0
            self.pause_ready_sequence = sequence
            self._try_send_pending_goal(sequence)

        if self.pending_goal is not None:
            pending_sequence, _goal = self.pending_goal
            self._try_send_pending_goal(pending_sequence)

    def _on_goal(self, msg: PoseStamped):
        goal = PoseStamped()
        goal.header = msg.header
        goal.pose = msg.pose
        if not goal.header.frame_id:
            goal.header.frame_id = self.goal_frame
        # Browser clocks may differ from the ROS host. Restamp web goals here so
        # Nav2/TF always evaluates the goal against the robot's current ROS time.
        goal.header.stamp = self.get_clock().now().to_msg()

        if self.force_yaw_to_goal and self.last_pose is not None:
            dx = goal.pose.position.x - self.last_pose.position.x
            dy = goal.pose.position.y - self.last_pose.position.y
            if math.hypot(dx, dy) > 0.05 or is_default_orientation(goal.pose.orientation):
                goal.pose.orientation = yaw_to_quaternion(math.atan2(dy, dx))

        self.goal_sequence += 1
        sequence = self.goal_sequence
        self.pending_goal = (sequence, goal)
        self.cancel_ready_sequence = 0
        self.pause_ready_sequence = 0
        self.pause_wait_sequence = 0
        self.server_retry_sequence = 0
        self._begin_cancel_current_goal(sequence)

        if bool(self.get_parameter("disable_auto_mapping_on_goal").value):
            if not self.auto_mapping_client.service_is_ready():
                self.auto_mapping_client.wait_for_service(timeout_sec=0.25)
            if self.auto_mapping_client.service_is_ready():
                request = SetBool.Request()
                request.data = False
                future = self.auto_mapping_client.call_async(request)
                future.add_done_callback(
                    lambda done, seq=sequence: self._on_auto_mapping_disabled(done, seq))
                return
            self.get_logger().warn(
                "Auto-mapping control service is unavailable; sending web goal directly")

        self.pause_ready_sequence = sequence
        self._try_send_pending_goal(sequence)

    def _on_auto_mapping_disabled(self, future, sequence: int):
        if sequence != self.goal_sequence:
            return
        try:
            response = future.result()
            if not response.success:
                self.get_logger().warn(
                    "Auto-mapping rejected pause request: %s" % response.message)
            elif "nav_goal_canceling=true" in response.message:
                self.auto_mapping_goal_active = True
                self.pause_wait_sequence = sequence
                self.pause_wait_deadline = time.monotonic() + 2.0
                return
        except Exception as exc:
            self.get_logger().warn("Failed to pause auto-mapping: %s" % exc)
        self.pause_ready_sequence = sequence
        self._try_send_pending_goal(sequence)

    def _try_send_pending_goal(self, sequence: int):
        if self.pending_goal is None or sequence != self.goal_sequence:
            return
        if self.goal_request_in_flight:
            return
        if self.cancel_ready_sequence != sequence or self.pause_ready_sequence != sequence:
            return
        pending_sequence, goal = self.pending_goal
        if pending_sequence != sequence:
            return

        now = time.monotonic()
        if self.server_retry_sequence == sequence and now < self.server_retry_deadline:
            return
        if not self.client.server_is_ready():
            if self.server_retry_sequence != sequence:
                self.get_logger().info(
                    "Waiting for Nav2 navigate_to_pose action server...")
            self.server_retry_sequence = sequence
            self.server_retry_deadline = now + 0.5
            return

        self.server_retry_sequence = 0
        self.pending_goal = None
        action_goal = NavigateToPose.Goal()
        action_goal.pose = goal
        if hasattr(action_goal, "behavior_tree"):
            action_goal.behavior_tree = ""

        self.get_logger().info(
            "Sending web nav goal: x=%.2f y=%.2f frame=%s"
            % (goal.pose.position.x, goal.pose.position.y, goal.header.frame_id)
        )
        self.goal_request_in_flight = True
        future = self.client.send_goal_async(action_goal)
        future.add_done_callback(
            lambda done, seq=sequence: self._on_goal_response(done, seq))

    def _begin_cancel_current_goal(self, sequence: int):
        goal_handle = self.current_goal_handle
        self.current_goal_handle = None
        if goal_handle is None:
            self.cancel_ready_sequence = sequence
            self._try_send_pending_goal(sequence)
            return
        try:
            future = goal_handle.cancel_goal_async()
            future.add_done_callback(
                lambda done, seq=sequence: self._on_current_goal_cancelled(done, seq))
        except Exception as exc:
            self.get_logger().warn(f"Web nav goal cancel failed: {exc}")
            self.cancel_ready_sequence = sequence
            self._try_send_pending_goal(sequence)

    def _on_current_goal_cancelled(self, _future, sequence: int):
        if sequence != self.goal_sequence:
            return
        self.cancel_ready_sequence = sequence
        self._try_send_pending_goal(sequence)

    def _cancel_current_goal(self):
        if self.current_goal_handle is not None:
            try:
                self.current_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.current_goal_handle = None

    def _on_goal_response(self, future, sequence: int):
        self.goal_request_in_flight = False
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().warn(f"Web nav goal send failed: {exc}")
            self._try_send_pending_goal(self.goal_sequence)
            return
        if not goal_handle.accepted:
            self.get_logger().warn("Web nav goal rejected by Nav2")
            self._try_send_pending_goal(self.goal_sequence)
            return
        if sequence != self.goal_sequence:
            self.goal_request_in_flight = True
            try:
                future = goal_handle.cancel_goal_async()
                future.add_done_callback(self._on_stale_goal_cancelled)
            except Exception:
                self.goal_request_in_flight = False
                self._try_send_pending_goal(self.goal_sequence)
            return
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done, seq=sequence: self._on_result(done, seq))
        self.get_logger().info("Web nav goal accepted")

    def _on_stale_goal_cancelled(self, _future):
        self.goal_request_in_flight = False
        self._try_send_pending_goal(self.goal_sequence)

    def _on_result(self, future, sequence: int):
        try:
            status = future.result().status
        except Exception as exc:
            self.get_logger().warn(f"Web nav goal failed: {exc}")
            return
        if sequence == self.goal_sequence:
            self.current_goal_handle = None
        self.get_logger().info(f"Web nav goal finished with status={status}")


def main(args=None):
    rclpy.init(args=args)
    node = WebGoalNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
