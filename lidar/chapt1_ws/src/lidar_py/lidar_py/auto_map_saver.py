#!/usr/bin/env python3
"""Save a Cartographer map and pbstream after frontier exploration completes."""

from datetime import datetime
import json
import os
import subprocess
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger


class AutoMapSaver(Node):
    def __init__(self):
        super().__init__("auto_map_saver")
        self.declare_parameter("completion_topic", "/exploration_complete")
        self.declare_parameter("output_dir", "~/maps/auto_mapping")
        self.declare_parameter("file_prefix", "auto_map")
        self.declare_parameter("save_pbstream", True)
        self.declare_parameter("command_timeout_sec", 30.0)

        qos = QoSProfile(depth=1)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.status_pub = self.create_publisher(String, "/auto_mapping/save_status", qos)
        self.create_subscription(
            Empty,
            str(self.get_parameter("completion_topic").value),
            self._on_complete,
            qos,
        )
        self.create_service(Trigger, "/auto_mapping/save_now", self._on_save_now)
        self.save_lock = threading.Lock()
        self.save_started = False
        self.get_logger().info("Automatic map saver ready")

    def _on_complete(self, _msg):
        self._start_save("exploration_complete")

    def _on_save_now(self, _request, response):
        started = self._start_save("manual")
        response.success = started
        response.message = "map save started" if started else "map save is already running"
        return response

    def _start_save(self, reason: str) -> bool:
        with self.save_lock:
            if self.save_started:
                return False
            self.save_started = True
        thread = threading.Thread(target=self._save, args=(reason,), daemon=True)
        thread.start()
        return True

    def _publish(self, state: str, detail: str, **extra):
        payload = {"state": state, "detail": detail}
        payload.update(extra)
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.status_pub.publish(message)

    def _run(self, command, timeout: float):
        return subprocess.run(
            command,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=timeout,
            text=True,
        )

    def _save(self, reason: str):
        output_dir = os.path.abspath(os.path.expanduser(
            str(self.get_parameter("output_dir").value)))
        prefix = str(self.get_parameter("file_prefix").value).strip() or "auto_map"
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        basename = "%s_%s" % (prefix, stamp)
        os.makedirs(output_dir, exist_ok=True)
        map_path = os.path.join(output_dir, basename)
        pbstream_path = map_path + ".pbstream"
        timeout = max(5.0, float(self.get_parameter("command_timeout_sec").value))
        self._publish("saving", reason, map=map_path)

        success = True
        details = []
        try:
            map_result = self._run([
                "ros2", "run", "nav2_map_server", "map_saver_cli",
                "-f", map_path,
                "--ros-args",
                "-p", "map_subscribe_transient_local:=true",
                "-p", "save_map_timeout:=20000",
            ], timeout)
            success = map_result.returncode == 0
            details.append("map_saver=%d" % map_result.returncode)
            if map_result.returncode != 0:
                self.get_logger().error("map_saver_cli failed:\n%s" % map_result.stdout[-2000:])

            if bool(self.get_parameter("save_pbstream").value):
                request = "{filename: '%s'}" % pbstream_path.replace("'", "")
                state_result = self._run([
                    "ros2", "service", "call", "/write_state",
                    "cartographer_ros_msgs/srv/WriteState", request,
                ], timeout)
                success = success and state_result.returncode == 0
                details.append("write_state=%d" % state_result.returncode)
                if state_result.returncode != 0:
                    self.get_logger().error("Cartographer write_state failed:\n%s" % state_result.stdout[-2000:])
        except subprocess.TimeoutExpired as exc:
            success = False
            details.append("timeout=%s" % exc.cmd[0])
            self.get_logger().error("Map save command timed out: %s" % exc)
        except Exception as exc:
            success = False
            details.append("exception=%s" % exc)
            self.get_logger().error("Map save failed: %s" % exc)

        state = "saved" if success else "error"
        detail = ", ".join(details)
        self._publish(state, detail, map=map_path, pbstream=pbstream_path)
        if success:
            self.get_logger().info("Automatic map saved: %s" % map_path)
        with self.save_lock:
            self.save_started = False


def main(args=None):
    rclpy.init(args=args)
    node = AutoMapSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
