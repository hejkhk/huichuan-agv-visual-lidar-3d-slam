#!/usr/bin/env python3
"""Low-overhead /proc resource accounting for the dual 2D/3D stack."""

import csv
import os
import pathlib
import shutil
import time
from collections import defaultdict
from datetime import datetime
from typing import Dict, Iterable, Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


GROUP_ORDER = (
    "camera",
    "cartographer",
    "rtabmap_octomap",
    "nav2",
    "perception_3d",
    "chassis_lidar",
    "rviz",
    "web_bridge",
    "stack_misc",
    "resource_monitor",
)


class SystemResourceMonitor(Node):
    """Write grouped CPU, memory, thread and I/O usage to log and CSV."""

    CSV_FIELDS = (
        "timestamp",
        "group",
        "process_count",
        "pids",
        "cpu_percent",
        "rss_mb",
        "threads",
        "read_mb_s",
        "write_mb_s",
        "system_cpu_percent",
        "load_1m",
        "memory_used_mb",
        "memory_available_mb",
        "memory_total_mb",
        "temperature_c",
        "disk_free_gb",
        "run_directory_mb",
    )

    def __init__(self):
        super().__init__("system_resource_monitor")
        self.declare_parameter("sample_interval_sec", 2.0)
        self.declare_parameter("report_interval_sec", 20.0)
        self.declare_parameter("csv_file", "")
        self.declare_parameter("project_root", "")
        self.declare_parameter("run_directory", "")

        self.sample_interval = max(
            1.0, float(self.get_parameter("sample_interval_sec").value))
        self.report_interval = max(
            self.sample_interval,
            float(self.get_parameter("report_interval_sec").value),
        )
        self.project_root = pathlib.Path(
            str(self.get_parameter("project_root").value) or os.getcwd()
        ).resolve()
        run_value = str(self.get_parameter("run_directory").value)
        self.run_directory = (
            pathlib.Path(run_value).resolve() if run_value else None
        )
        self.csv_path = str(self.get_parameter("csv_file").value).strip()
        self.csv_handle = None
        self.csv_writer = None
        self.clock_ticks = float(os.sysconf("SC_CLK_TCK"))
        self.page_size = float(os.sysconf("SC_PAGE_SIZE"))

        self.previous_processes: Dict[int, Tuple[int, int, int]] = {}
        self.previous_system_cpu = self._read_system_cpu()
        self.last_sample_time = time.monotonic()
        self.last_report_time = self.last_sample_time
        self.accumulator = defaultdict(
            lambda: {
                "samples": 0,
                "cpu": 0.0,
                "rss": 0.0,
                "threads": 0.0,
                "read": 0.0,
                "write": 0.0,
            }
        )
        self.latest_groups = {}
        self.system_cpu_sum = 0.0
        self.system_cpu_samples = 0

        self._open_csv()
        self._prime_process_counters()
        self.timer = self.create_timer(self.sample_interval, self._sample)
        self.get_logger().info(
            "RESOURCE_MONITOR active "
            f"sample={self.sample_interval:.1f}s "
            f"report={self.report_interval:.1f}s "
            f"csv={self.csv_path or 'disabled'}"
        )

    def _open_csv(self) -> None:
        if not self.csv_path:
            return
        path = pathlib.Path(self.csv_path)
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            needs_header = not path.exists() or path.stat().st_size == 0
            self.csv_handle = path.open("a", encoding="utf-8", newline="")
            self.csv_writer = csv.DictWriter(
                self.csv_handle, fieldnames=self.CSV_FIELDS
            )
            if needs_header:
                self.csv_writer.writeheader()
                self.csv_handle.flush()
        except OSError as exc:
            self.get_logger().error(
                f"RESOURCE_MONITOR cannot open CSV {path}: {exc}"
            )
            self.csv_handle = None
            self.csv_writer = None

    def _prime_process_counters(self) -> None:
        for pid, _, ticks, read_bytes, write_bytes, _, _ in self._processes():
            self.previous_processes[pid] = (ticks, read_bytes, write_bytes)

    @staticmethod
    def _read_system_cpu() -> Optional[Tuple[int, int]]:
        try:
            fields = pathlib.Path("/proc/stat").read_text(
                encoding="ascii"
            ).splitlines()[0].split()[1:]
            values = [int(value) for value in fields]
            total = sum(values)
            idle = values[3] + (values[4] if len(values) > 4 else 0)
            return total, idle
        except (OSError, ValueError, IndexError):
            return None

    @staticmethod
    def _read_memory() -> Tuple[float, float, float]:
        values = {}
        try:
            for line in pathlib.Path("/proc/meminfo").read_text(
                encoding="ascii"
            ).splitlines():
                key, raw = line.split(":", 1)
                values[key] = int(raw.strip().split()[0]) / 1024.0
        except (OSError, ValueError, IndexError):
            return 0.0, 0.0, 0.0
        total = values.get("MemTotal", 0.0)
        available = values.get("MemAvailable", values.get("MemFree", 0.0))
        return max(0.0, total - available), available, total

    @staticmethod
    def _read_temperature() -> Optional[float]:
        temperatures = []
        for path in pathlib.Path("/sys/class/thermal").glob(
            "thermal_zone*/temp"
        ):
            try:
                value = float(path.read_text(encoding="ascii").strip())
                if value > 1000.0:
                    value /= 1000.0
                if 0.0 < value < 150.0:
                    temperatures.append(value)
            except (OSError, ValueError, TypeError, UnicodeError):
                continue
        return max(temperatures) if temperatures else None

    @staticmethod
    def _directory_size(path: Optional[pathlib.Path]) -> float:
        if path is None or not path.is_dir():
            return 0.0
        total = 0
        try:
            for root, _, files in os.walk(path):
                for name in files:
                    try:
                        total += os.path.getsize(os.path.join(root, name))
                    except OSError:
                        continue
        except OSError:
            return 0.0
        return total / (1024.0 * 1024.0)

    @staticmethod
    def _classify(command: str) -> Optional[str]:
        value = command.lower()
        if "system_resource_monitor" in value:
            return "resource_monitor"
        if "rviz2" in value:
            return "rviz"
        if any(token in value for token in (
            "orbbec_camera", "camera_container", "depth_image_proc",
        )):
            return "camera"
        if "cartographer" in value:
            return "cartographer"
        if any(token in value for token in (
            "rtabmap", "octomap_server", "global_cloud_relay",
        )):
            return "rtabmap_octomap"
        if any(token in value for token in (
            "mutable_navigation_map", "depth_image_to_local_cloud",
            "local_cloud_collision_gate", "persistent_visual_wall_filter",
            "rgbd_timestamp_monitor", "safety_fusion_node",
        )):
            return "perception_3d"
        if any(token in value for token in (
            "controller_server", "planner_server", "bt_navigator",
            "behavior_server", "waypoint_follower", "velocity_smoother",
            "lifecycle_manager", "map_server", "short_goal",
            "frontier_exploration",
        )):
            return "nav2"
        if any(token in value for token in (
            "chassis_node", "lidar_node", "laser_filter",
            "robot_pose_publisher",
        )):
            return "chassis_lidar"
        if any(token in value for token in (
            "rosbridge", "web_goal_nav", "web_path_preview",
            "frontier_web_bridge", "mjpeg", "vite",
        )):
            return "web_bridge"
        if any(token in value for token in (
            "dual_resolution_3d_slam.launch.py", "robot_state_publisher",
            "localization_bringup", "cartographer_reloc",
            "slam_correction_guard",
        )):
            return "stack_misc"
        return None

    @staticmethod
    def _read_status(pid_path: pathlib.Path) -> Tuple[float, int]:
        rss_mb = 0.0
        threads = 0
        try:
            for line in (pid_path / "status").read_text(
                encoding="ascii", errors="replace"
            ).splitlines():
                if line.startswith("VmRSS:"):
                    rss_mb = int(line.split()[1]) / 1024.0
                elif line.startswith("Threads:"):
                    threads = int(line.split()[1])
        except (OSError, ValueError, IndexError):
            pass
        return rss_mb, threads

    @staticmethod
    def _read_io(pid_path: pathlib.Path) -> Tuple[int, int]:
        read_bytes = 0
        write_bytes = 0
        try:
            for line in (pid_path / "io").read_text(
                encoding="ascii", errors="replace"
            ).splitlines():
                if line.startswith("read_bytes:"):
                    read_bytes = int(line.split()[1])
                elif line.startswith("write_bytes:"):
                    write_bytes = int(line.split()[1])
        except (OSError, ValueError, IndexError, PermissionError):
            pass
        return read_bytes, write_bytes

    def _processes(self) -> Iterable[Tuple[int, str, int, int, int, float, int]]:
        for pid_path in pathlib.Path("/proc").iterdir():
            if not pid_path.name.isdigit():
                continue
            try:
                pid = int(pid_path.name)
                command_raw = (pid_path / "cmdline").read_bytes()
                if not command_raw:
                    continue
                command = command_raw.replace(b"\0", b" ").decode(
                    "utf-8", errors="replace"
                )
                group = self._classify(command)
                if group is None:
                    continue
                stat = (pid_path / "stat").read_text(
                    encoding="ascii", errors="replace"
                )
                fields = stat[stat.rfind(")") + 2:].split()
                ticks = int(fields[11]) + int(fields[12])
                read_bytes, write_bytes = self._read_io(pid_path)
                rss_mb, threads = self._read_status(pid_path)
                yield (
                    pid, group, ticks, read_bytes, write_bytes,
                    rss_mb, threads,
                )
            except (OSError, ValueError, IndexError):
                continue

    def _sample(self) -> None:
        now = time.monotonic()
        elapsed = max(0.001, now - self.last_sample_time)
        self.last_sample_time = now
        groups = {
            name: {
                "pids": [], "cpu": 0.0, "rss": 0.0, "threads": 0,
                "read": 0.0, "write": 0.0,
            }
            for name in GROUP_ORDER
        }
        current_processes = {}
        for (
            pid, group, ticks, read_bytes, write_bytes, rss_mb, threads
        ) in self._processes():
            current_processes[pid] = (ticks, read_bytes, write_bytes)
            previous = self.previous_processes.get(pid)
            cpu = read_rate = write_rate = 0.0
            if previous is not None:
                cpu = (
                    max(0.0, ticks - previous[0])
                    / self.clock_ticks / elapsed * 100.0
                )
                read_rate = (
                    max(0, read_bytes - previous[1])
                    / (1024.0 * 1024.0) / elapsed
                )
                write_rate = (
                    max(0, write_bytes - previous[2])
                    / (1024.0 * 1024.0) / elapsed
                )
            target = groups[group]
            target["pids"].append(pid)
            target["cpu"] += cpu
            target["rss"] += rss_mb
            target["threads"] += threads
            target["read"] += read_rate
            target["write"] += write_rate
        self.previous_processes = current_processes
        self.latest_groups = groups

        current_cpu = self._read_system_cpu()
        system_cpu = 0.0
        if current_cpu is not None and self.previous_system_cpu is not None:
            total_delta = current_cpu[0] - self.previous_system_cpu[0]
            idle_delta = current_cpu[1] - self.previous_system_cpu[1]
            if total_delta > 0:
                system_cpu = max(
                    0.0, min(100.0, 100.0 * (total_delta - idle_delta) /
                    total_delta)
                )
        self.previous_system_cpu = current_cpu
        self.system_cpu_sum += system_cpu
        self.system_cpu_samples += 1

        for name, values in groups.items():
            aggregate = self.accumulator[name]
            aggregate["samples"] += 1
            for key in ("cpu", "rss", "threads", "read", "write"):
                aggregate[key] += float(values[key])

        if now - self.last_report_time >= self.report_interval:
            self.last_report_time = now
            self._report()

    def _report(self) -> None:
        memory_used, memory_available, memory_total = self._read_memory()
        try:
            load_1m = os.getloadavg()[0]
        except (AttributeError, OSError):
            load_1m = 0.0
        temperature = self._read_temperature()
        try:
            disk_free_gb = (
                shutil.disk_usage(self.project_root).free / (1024.0 ** 3)
            )
        except OSError:
            disk_free_gb = 0.0
        run_directory_mb = self._directory_size(self.run_directory)
        system_cpu = (
            self.system_cpu_sum / self.system_cpu_samples
            if self.system_cpu_samples else 0.0
        )
        stamp = datetime.now().astimezone().isoformat(timespec="seconds")
        temp_text = f"{temperature:.1f}C" if temperature is not None else "n/a"
        self.get_logger().info(
            f"RESOURCE_SYSTEM cpu={system_cpu:.1f}% load1={load_1m:.2f} "
            f"memory={memory_used:.0f}/{memory_total:.0f}MB "
            f"available={memory_available:.0f}MB temp={temp_text} "
            f"disk_free={disk_free_gb:.1f}GB run_dir={run_directory_mb:.1f}MB"
        )

        for name in GROUP_ORDER:
            aggregate = self.accumulator[name]
            samples = max(1, int(aggregate["samples"]))
            latest = self.latest_groups.get(name, {})
            row = {
                "timestamp": stamp,
                "group": name,
                "process_count": len(latest.get("pids", [])),
                "pids": ";".join(
                    str(pid) for pid in latest.get("pids", [])
                ),
                "cpu_percent": f"{aggregate['cpu'] / samples:.3f}",
                "rss_mb": f"{aggregate['rss'] / samples:.3f}",
                "threads": f"{aggregate['threads'] / samples:.2f}",
                "read_mb_s": f"{aggregate['read'] / samples:.4f}",
                "write_mb_s": f"{aggregate['write'] / samples:.4f}",
                "system_cpu_percent": f"{system_cpu:.3f}",
                "load_1m": f"{load_1m:.3f}",
                "memory_used_mb": f"{memory_used:.3f}",
                "memory_available_mb": f"{memory_available:.3f}",
                "memory_total_mb": f"{memory_total:.3f}",
                "temperature_c": (
                    f"{temperature:.3f}" if temperature is not None else ""
                ),
                "disk_free_gb": f"{disk_free_gb:.3f}",
                "run_directory_mb": f"{run_directory_mb:.3f}",
            }
            self.get_logger().info(
                f"RESOURCE_GROUP name={name} "
                f"processes={row['process_count']} "
                f"cpu={aggregate['cpu'] / samples:.1f}% "
                f"rss={aggregate['rss'] / samples:.1f}MB "
                f"threads={aggregate['threads'] / samples:.1f} "
                f"io_read={aggregate['read'] / samples:.3f}MB/s "
                f"io_write={aggregate['write'] / samples:.3f}MB/s"
            )
            if self.csv_writer is not None:
                self.csv_writer.writerow(row)

        if system_cpu >= 90.0 or (
            memory_total > 0.0 and memory_available < 512.0
        ) or (
            temperature is not None and temperature >= 80.0
        ):
            self.get_logger().warn(
                f"RESOURCE_PRESSURE cpu={system_cpu:.1f}% "
                f"available={memory_available:.0f}MB temp={temp_text}; "
                "inspect resource_usage.csv before changing SLAM parameters"
            )
        if self.csv_handle is not None:
            self.csv_handle.flush()
        self.accumulator.clear()
        self.system_cpu_sum = 0.0
        self.system_cpu_samples = 0

    def destroy_node(self):
        if self.csv_handle is not None:
            try:
                self.csv_handle.flush()
                self.csv_handle.close()
            except OSError:
                pass
            self.csv_handle = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SystemResourceMonitor()
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
