#!/usr/bin/env python3
"""Pause RTAB-Map while no persistent 3D RViz display is subscribed."""

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Empty


class RtabmapDemandManager(Node):
    def __init__(self):
        super().__init__("rtabmap_demand_manager")
        self.declare_parameter("map_data_topic", "/rtabmap_3d/mapData")
        self.declare_parameter(
            "octomap_topic", "/rtabmap_3d/octomap_occupied_space")
        self.declare_parameter("pause_service", "/rtabmap_3d/pause")
        self.declare_parameter("resume_service", "/rtabmap_3d/resume")
        self.declare_parameter("idle_delay_sec", 3.0)
        self.declare_parameter("startup_grace_sec", 8.0)

        self._topics = [
            str(self.get_parameter("map_data_topic").value),
            str(self.get_parameter("octomap_topic").value),
        ]
        pause_service = str(self.get_parameter("pause_service").value)
        resume_service = str(self.get_parameter("resume_service").value)
        self._idle_delay = max(
            0.5, float(self.get_parameter("idle_delay_sec").value))
        self._startup_deadline = time.monotonic() + max(
            0.0, float(self.get_parameter("startup_grace_sec").value))
        self._last_subscriber_time = time.monotonic()
        self._paused = False
        self._request_in_flight = False
        self._pause_client = self.create_client(Empty, pause_service)
        self._resume_client = self.create_client(Empty, resume_service)
        self._timer = self.create_timer(0.5, self._poll)
        self.get_logger().info(
            "RTAB-Map on-demand control active: enable either persistent "
            "MapCloud or optimized OctoMap display to resume it")

    def _call(self, client, pause):
        if self._request_in_flight or not client.service_is_ready():
            return
        self._request_in_flight = True
        future = client.call_async(Empty.Request())

        def done(completed):
            self._request_in_flight = False
            try:
                completed.result()
            except Exception as exc:  # pragma: no cover - ROS runtime path
                self.get_logger().warning(f"RTAB-Map demand service failed: {exc}")
                return
            self._paused = pause
            state = "paused" if pause else "resumed"
            self.get_logger().info(f"RTAB-Map {state} by RViz MapCloud demand")

        future.add_done_callback(done)

    def _poll(self):
        now = time.monotonic()
        subscribers = sum(
            self.count_subscribers(topic) for topic in self._topics)
        if subscribers > 0:
            self._last_subscriber_time = now
            if self._paused:
                self._call(self._resume_client, pause=False)
            return
        if now < self._startup_deadline:
            return
        if not self._paused and now - self._last_subscriber_time >= self._idle_delay:
            self._call(self._pause_client, pause=True)


def main(args=None):
    rclpy.init(args=args)
    node = RtabmapDemandManager()
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
