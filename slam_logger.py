#!/usr/bin/env python3
"""
SLAM Logger — 默认关闭；开启后每 3s 保存完整地图 + TF 位姿 + STM32 陀螺仪数据
结构:
  SLAM_Log/YYYY-MM-DD_HH-MM-SS/
    Maps/         → 时间戳命名 PGM+YAML + robot_pose.jsonl
    陀螺仪/       → 原始.json (hex) + 解析.json (yaw/vx/vz)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
import json
import os
import struct
import time
import math
from datetime import datetime

try:
    from PIL import Image, ImageDraw
    HAS_PIL = True
except ImportError:
    HAS_PIL = False


def occ_to_pgm_pixel(occ: int) -> int:
    """OccupancyGrid 值 → PGM 像素值"""
    if occ < 0:
        return 205  # unknown = gray
    return max(0, min(255, int(255 - occ * 255.0 / 100.0 + 0.5)))


def write_pgm(path: str, width: int, height: int, data):
    """写 PGM P5 文件"""
    with open(path, 'wb') as f:
        f.write(f"P5\n{width} {height}\n255\n".encode())
        f.write(bytes(occ_to_pgm_pixel(int(d)) for d in data))


def write_yaml(path: str, image_rel: str, resolution: float, origin_x: float, origin_y: float):
    """写地图 YAML 元数据"""
    content = (
        f"image: {image_rel}\n"
        f"mode: trinary\n"
        f"resolution: {resolution}\n"
        f"origin: [{origin_x:.3f}, {origin_y:.3f}, 0]\n"
        f"negate: 0\n"
        f"occupied_thresh: 0.65\n"
        f"free_thresh: 0.25\n"
    )
    with open(path, 'w') as f:
        f.write(content)


def fmt_ts(sec: float) -> str:
    """ROS 秒时间戳 → 'MM-DD  HH:MM:SS.s' 格式"""
    dt = datetime.fromtimestamp(sec)
    return dt.strftime("%m-%d  %H:%M:%S") + f".{int(dt.microsecond / 100000)}"


def fmt_ts_slash(sec: float) -> str:
    """ROS 秒时间戳 → 'MM/DD  HH:MM:SS.s' 格式 (for JSON entries)"""
    dt = datetime.fromtimestamp(sec)
    return dt.strftime("%m/%d  %H:%M:%S") + f".{int(dt.microsecond / 100000)}"


class SlamLogger(Node):
    def __init__(self):
        super().__init__('slam_logger')

        self.declare_parameter('log_dir', os.path.join(
            os.path.dirname(os.path.abspath(__file__)), 'SLAM_Log'))
        self.declare_parameter('save_interval_sec', 3.0)
        self.declare_parameter('start_enabled', False)
        log_base = self.get_parameter('log_dir').value
        save_interval_sec = self._normalize_interval(
            self.get_parameter('save_interval_sec').value)

        # 创建会话目录
        session_name = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.session_dir = os.path.join(log_base, session_name)
        self.maps_dir = os.path.join(self.session_dir, "Maps")
        self.gyro_dir = os.path.join(self.session_dir, "陀螺仪")
        os.makedirs(self.maps_dir, exist_ok=True)
        os.makedirs(self.gyro_dir, exist_ok=True)

        self.raw_path = os.path.join(self.gyro_dir, "原始.json")
        self.parsed_path = os.path.join(self.gyro_dir, "解析.json")
        self.pose_path = os.path.join(self.maps_dir, "robot_pose.jsonl")

        # TF 监听
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 订阅
        self.create_subscription(OccupancyGrid, '/map', self._on_map, 10)
        self.create_subscription(String, '/robot/serial_debug', self._on_serial_debug, 50)
        self.create_subscription(String, '/robot/web_control', self._on_web_control, 10)
        self.create_subscription(String, '/robot/control_state', self._on_control_state, 10)

        # 默认仍由网页控制；专项测试可通过参数在启动时直接开启。
        self.log_enabled = bool(self.get_parameter('start_enabled').value)

        # 最新缓存
        self.latest_map_msg = None

        # 陀螺仪日志缓冲
        self.gyro_raw_lines = []
        self.gyro_parsed_lines = []

        self.save_interval_sec = save_interval_sec
        self.save_timer = self.create_timer(self.save_interval_sec, self._save_tick)

        # map topic 质量统计
        self.last_map_time = 0.0
        self.map_seq = 0

        self.get_logger().info(f"SLAM Logger started → {self.session_dir}")
        self.get_logger().info(f"  Maps dir    : {self.maps_dir}")
        self.get_logger().info(f"  Gyro dir    : {self.gyro_dir}")
        self.get_logger().info(f"  Save interval: {self.save_interval_sec:.3f}s")
        if self.log_enabled:
            self.get_logger().info("SLAM Log ENABLED on startup parameter")

    def _on_map(self, msg: OccupancyGrid):
        self.latest_map_msg = msg

    def _on_serial_debug(self, msg: String):
        """接收 /robot/serial_debug, 只记录 NAVI 帧"""
        if not self.log_enabled:
            return
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if data.get("kind") != "navi":
            return

        stamp_sec = data.get("stamp_sec", time.time())
        ts = fmt_ts_slash(stamp_sec)

        # 原始帧
        checksum = data.get("checksum", 0)
        calc = data.get("calc", 0)
        hex_str = data.get("hex", "")
        raw_entry = f'{ts}{{\n    checksum=0x{checksum:02X} calc=0x{calc:02X}\n    {hex_str}\n}}\n\n'
        self.gyro_raw_lines.append(raw_entry)

        # 解析数据
        yaw = data.get("yaw_deg", 0.0)
        vx = data.get("vx_mps", 0.0)
        vz_deg = data.get("vz_deg_s", 0.0)
        vz_rad = data.get("vz_rad_s", 0.0)
        parsed_entry = f'{ts}{{\n    yaw={yaw:.2f} deg\n    vx={vx:.3f} m/s\n    vz={vz_deg:.2f} deg/s ({vz_rad:.3f} rad/s)\n}}\n\n'
        self.gyro_parsed_lines.append(parsed_entry)

    @staticmethod
    def _normalize_interval(value) -> float:
        try:
            interval = float(value)
        except (TypeError, ValueError):
            return 3.0
        if not math.isfinite(interval):
            return 3.0
        return round(min(60.0, max(0.5, interval)) * 2.0) / 2.0

    def _set_save_interval(self, value) -> bool:
        interval = self._normalize_interval(value)
        if abs(interval - self.save_interval_sec) < 1e-9:
            return False
        self.save_interval_sec = interval
        if hasattr(self, 'save_timer'):
            self.destroy_timer(self.save_timer)
            self.save_timer = self.create_timer(self.save_interval_sec, self._save_tick)
        self.get_logger().info(
            f"SLAM Log save interval changed to {self.save_interval_sec:.1f}s")
        return True

    def _on_web_control(self, msg: String):
        """接收网页控制命令，处理 SLAM 日志开关和记录间隔。"""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        cmd = data.get("command", "")
        if cmd not in ("slam_log_enable", "slam_log_disable", "slam_log_config"):
            return
        if "interval_sec" in data:
            self._set_save_interval(data.get("interval_sec"))
        if cmd == "slam_log_enable":
            self.log_enabled = True
            self.get_logger().info(
                f"SLAM Log ENABLED by web console ({self.save_interval_sec:.1f}s)")
        elif cmd == "slam_log_disable":
            self.log_enabled = False
            self.get_logger().info("SLAM Log DISABLED by web console")

    def _on_control_state(self, msg: String):
        """从底盘共享状态恢复日志配置，避免日志节点错过网页瞬时指令。"""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if "slam_log_interval_sec" in data:
            self._set_save_interval(data.get("slam_log_interval_sec"))
        if isinstance(data.get("slam_log_enabled"), bool):
            self.log_enabled = data["slam_log_enabled"]

    def _save_tick(self):
        """按配置间隔触发；默认关闭记录，开启后默认每 3 秒保存。"""
        if not self.log_enabled:
            return
        now = self.get_clock().now().nanoseconds / 1e9

        # 1. 保存当前地图
        if self.latest_map_msg is not None:
            self._save_map_snapshot(now)

        # 2. 刷写陀螺仪日志
        if self.gyro_raw_lines:
            with open(self.raw_path, 'a') as f:
                f.writelines(self.gyro_raw_lines)
            self.gyro_raw_lines.clear()
            with open(self.parsed_path, 'a') as f:
                f.writelines(self.gyro_parsed_lines)
            self.gyro_parsed_lines.clear()

    def _save_map_snapshot(self, stamp_sec: float):
        """保存地图 PGM(干净) + PNG(带彩色箭头) + YAML"""
        msg = self.latest_map_msg
        ts_name = fmt_ts(stamp_sec)

        pgm_path = os.path.join(self.maps_dir, f"{ts_name}.pgm")
        png_path = os.path.join(self.maps_dir, f"{ts_name}.png")
        yaml_path = os.path.join(self.maps_dir, f"{ts_name}.yaml")

        w = msg.info.width
        h = msg.info.height
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y

        # 干净 PGM
        write_pgm(pgm_path, w, h, msg.data)
        write_yaml(yaml_path, f"{ts_name}.pgm", res, ox, oy)

        # 获取 TF 位姿
        robot_px = robot_py = robot_yaw = None
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0))
            rx = tf.transform.translation.x
            ry = tf.transform.translation.y
            q = tf.transform.rotation
            robot_yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
            robot_px = int((rx - ox) / res)
            robot_py = int(h - (ry - oy) / res)
        except Exception:
            pass

        # 彩色 PNG 图层 — 红色三角形箭头 + 光晕
        if HAS_PIL and robot_px is not None and 0 <= robot_px < w and 0 <= robot_py < h:
            # 灰度底图转 RGB
            gray = Image.new("L", (w, h))
            gray.putdata([occ_to_pgm_pixel(int(d)) for d in msg.data])
            img = gray.convert("RGB")
            draw = ImageDraw.Draw(img)

            # 箭头尺寸
            arrow_r = max(8, min(int(0.18 / res), 24))
            body_r = max(4, arrow_r // 2)

            # 三角形顶点（车头方向 = yaw，PGM y 轴反转）
            tip_x = robot_px + int(arrow_r * math.cos(robot_yaw))
            tip_y = robot_py - int(arrow_r * math.sin(robot_yaw))

            # 车尾两点
            rear_angle_left = robot_yaw + math.pi * 0.75
            rear_angle_right = robot_yaw - math.pi * 0.75
            rear_lx = robot_px + int(body_r * math.cos(rear_angle_left))
            rear_ly = robot_py - int(body_r * math.sin(rear_angle_left))
            rear_rx = robot_px + int(body_r * math.cos(rear_angle_right))
            rear_ry = robot_py - int(body_r * math.sin(rear_angle_right))

            # 绿色光晕（外圈粗线）
            for glow_offset in [(-1, -1), (1, -1), (-1, 1), (1, 1), (0, -1), (0, 1), (-1, 0), (1, 0)]:
                ox_g, oy_g = glow_offset
                pts = [
                    (tip_x + ox_g, tip_y + oy_g),
                    (rear_lx + ox_g, rear_ly + oy_g),
                    (rear_rx + ox_g, rear_ry + oy_g),
                ]
                draw.polygon(pts, fill=(0, 220, 80))

            # 红色实心三角形
            draw.polygon([(tip_x, tip_y), (rear_lx, rear_ly), (rear_rx, rear_ry)], fill=(240, 50, 30))

            # 白色小圆点（中心位置）
            r = 3
            draw.ellipse([robot_px-r, robot_py-r, robot_px+r, robot_py+r], fill=(255, 255, 255))
            draw.ellipse([robot_px-r+1, robot_py-r+1, robot_px+r-1, robot_py+r-1], fill=(240, 50, 30))

            img.save(png_path)

        # 写 pose.jsonl（备用）
        pose_info = {"ts": stamp_sec, "ts_str": ts_name}
        if robot_px is not None:
            pose_info["x"] = round(rx, 3)
            pose_info["y"] = round(ry, 3)
            pose_info["yaw_deg"] = round(math.degrees(robot_yaw), 2)
            pose_info["yaw_rad"] = round(robot_yaw, 4)
        else:
            pose_info["x"] = None
            pose_info["y"] = None
            pose_info["yaw_deg"] = None
        with open(self.pose_path, 'a') as f:
            f.write(json.dumps(pose_info, ensure_ascii=False) + '\n')

    def destroy_node(self):
        # 最后刷写一次
        if self.gyro_raw_lines:
            with open(self.raw_path, 'a') as f:
                f.writelines(self.gyro_raw_lines)
            with open(self.parsed_path, 'a') as f:
                f.writelines(self.gyro_parsed_lines)
        self.get_logger().info(f"SLAM Logger stopped. Data saved to {self.session_dir}")
        super().destroy_node()


def main():
    rclpy.init()
    node = SlamLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
