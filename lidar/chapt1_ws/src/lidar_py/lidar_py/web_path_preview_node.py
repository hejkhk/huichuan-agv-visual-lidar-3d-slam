#!/usr/bin/env python3
"""Preview Nav2 paths for the web map without starting navigation."""

import json

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose
from std_msgs.msg import String


class WebPathPreviewNode(Node):
    def __init__(self):
        super().__init__("web_path_preview_node")
        self.declare_parameter("preview_goal_topic", "/web/preview_goal")
        self.declare_parameter("preview_path_topic", "/web/preview_path")
        self.declare_parameter("planner_action", "compute_path_to_pose")
        self.declare_parameter("planner_id", "")
        self.declare_parameter("replan_period", 1.0)

        self.goal = None
        self.goal_sequence = 0
        self.in_flight = False
        self.client = ActionClient(
            self,
            ComputePathToPose,
            self.get_parameter("planner_action").value,
        )
        self.path_pub = self.create_publisher(
            Path, self.get_parameter("preview_path_topic").value, 10
        )
        self.goal_sub = self.create_subscription(
            PoseStamped,
            self.get_parameter("preview_goal_topic").value,
            self._on_goal,
            10,
        )
        self.web_control_sub = self.create_subscription(
            String, "/robot/web_control", self._on_web_control, 10
        )
        period = max(0.25, float(self.get_parameter("replan_period").value))
        self.create_timer(period, self._request_plan)
        self.get_logger().info("Web path preview node started")

    def _on_goal(self, msg: PoseStamped):
        self.goal_sequence += 1
        self.goal = PoseStamped()
        self.goal.header = msg.header
        self.goal.pose = msg.pose
        if not self.goal.header.frame_id:
            self.goal.header.frame_id = "map"
        # The browser and ROS host clocks are not guaranteed to match.
        self.goal.header.stamp = self.get_clock().now().to_msg()
        self._request_plan()

    def _on_web_control(self, msg: String):
        try:
            data = json.loads(msg.data or "{}")
        except json.JSONDecodeError:
            return
        if data.get("command") == "clear_preview_goal":
            self.goal_sequence += 1
            self.goal = None
            self.path_pub.publish(Path())

    def _request_plan(self):
        if self.goal is None or self.in_flight:
            return
        if not self.client.server_is_ready():
            if not self.client.wait_for_server(timeout_sec=0.01):
                return

        goal_msg = ComputePathToPose.Goal()
        self.goal.header.stamp = self.get_clock().now().to_msg()
        # Nav2 deployments have used both field names; keep the bridge
        # source-compatible instead of crashing the node on a map click.
        if hasattr(goal_msg, "pose"):
            goal_msg.pose = self.goal
        elif hasattr(goal_msg, "goal"):
            goal_msg.goal = self.goal
        else:
            self.get_logger().error(
                "ComputePathToPose goal has neither 'pose' nor 'goal'; "
                "path preview is disabled")
            return
        goal_msg.planner_id = str(self.get_parameter("planner_id").value)
        if hasattr(goal_msg, "use_start"):
            goal_msg.use_start = False

        self.in_flight = True
        sequence = self.goal_sequence
        send_future = self.client.send_goal_async(goal_msg)
        send_future.add_done_callback(
            lambda done, seq=sequence: self._on_goal_response(done, seq))

    def _on_goal_response(self, future, sequence: int):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.in_flight = False
            self.get_logger().warn(f"Preview path goal failed: {exc}")
            return
        if not goal_handle.accepted:
            self.in_flight = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done, seq=sequence: self._on_result(done, seq))

    def _on_result(self, future, sequence: int):
        self.in_flight = False
        try:
            result = future.result().result
        except Exception as exc:
            self.get_logger().warn(f"Preview path request failed: {exc}")
            return
        if sequence == self.goal_sequence and self.goal is not None:
            self.path_pub.publish(result.path)


def main(args=None):
    rclpy.init(args=args)
    node = WebPathPreviewNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
