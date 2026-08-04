"""
camera_input.py

统一相机输入层：
    1. CAMERA_BACKEND="ros2"：订阅 Orbbec ROS2 Wrapper 发布的 RGB + Depth topic。
    2. CAMERA_BACKEND="sdk" ：回退到旧 pyorbbecsdk Pipeline() 直连初始化。

为什么要合到一个入口文件：
    main.py 不应该关心“相机到底是 ROS2 topic 还是 SDK Pipeline”。
    main.py 只调用：
        cam = UnifiedCameraManager(...)
        cam.start()
        color, depth = cam.get_frames()
        cam.stop()

注意：
    ROS2 模式需要你先单独启动 orbbec_camera launch。
    SDK 模式会调用旧 camera_orbbec.py 中的 OrbbecCameraManager，方便保留回退能力。
"""

from __future__ import annotations

import time
import threading
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2 as cv
import numpy as np

from config_switches import *

# ROS2 相关库在树莓派系统 Python 里才有；这里做成可选导入，避免 SDK 模式下直接崩。
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
    from sensor_msgs.msg import Image, CameraInfo
    ROS2_AVAILABLE = True
    ROS2_IMPORT_ERROR = None
except Exception as e:
    rclpy = None
    Node = object
    SingleThreadedExecutor = None
    QoSProfile = None
    QoSReliabilityPolicy = None
    QoSHistoryPolicy = None
    QoSDurabilityPolicy = None
    Image = None
    CameraInfo = None
    ROS2_AVAILABLE = False
    ROS2_IMPORT_ERROR = e


@dataclass
class _SimpleIntrinsic:
    """兼容旧 utils.get_xyz() 里 cam_param.rgb_intrinsic.fx 这种访问方式。"""
    fx: float = 0.0
    fy: float = 0.0
    cx: float = 0.0
    cy: float = 0.0


@dataclass
class _SimpleCameraParam:
    """把 ROS2 CameraInfo 里的 K 矩阵包装成类似 Orbbec SDK 的 cam_param。"""
    rgb_intrinsic: _SimpleIntrinsic


def _stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _camera_info_to_cam_param(msg) -> _SimpleCameraParam:
    k = list(msg.k)
    return _SimpleCameraParam(
        rgb_intrinsic=_SimpleIntrinsic(
            fx=float(k[0]),
            fy=float(k[4]),
            cx=float(k[2]),
            cy=float(k[5]),
        )
    )


if ROS2_AVAILABLE:
    class _Ros2CameraBridge(Node):
        """
        ROS2 图像订阅节点：只缓存最新 RGB / Depth，不堆积旧帧。
        """
        def __init__(
            self,
            color_topic: str,
            depth_topic: str,
            color_info_topic: str,
            depth_info_topic: str,
            resize_depth_to_color: bool = True,
            force_resize_to_config_size: bool = True,
        ):
            super().__init__("road_unified_camera_bridge")

            self.resize_depth_to_color = resize_depth_to_color
            self.force_resize_to_config_size = force_resize_to_config_size

            image_qos = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,  # 关键：永远只保留最新帧，避免图像排队造成延迟
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
            )

            self._lock = threading.Lock()
            self._color_img: Optional[np.ndarray] = None
            self._depth_img: Optional[np.ndarray] = None
            self._color_count = 0
            self._depth_count = 0
            self._color_stamp_ns = 0
            self._depth_stamp_ns = 0
            self._color_encoding = ""
            self._depth_encoding = ""
            self._cam_param: Optional[_SimpleCameraParam] = None

            self._last_report_time = time.perf_counter()
            self._last_color_count = 0
            self._last_depth_count = 0

            self.create_subscription(Image, color_topic, self._on_color, image_qos)
            self.create_subscription(Image, depth_topic, self._on_depth, image_qos)
            self.create_subscription(CameraInfo, color_info_topic, self._on_color_info, image_qos)
            self.create_subscription(CameraInfo, depth_info_topic, self._on_depth_info, image_qos)

            self.get_logger().info(f"订阅 RGB topic:   {color_topic}")
            self.get_logger().info(f"订阅 Depth topic: {depth_topic}")

        def _on_color(self, msg):
            try:
                img = self._image_to_bgr(msg)
            except Exception as e:
                self.get_logger().warning(f"RGB 转换失败 encoding={msg.encoding}: {e}")
                return
            with self._lock:
                self._color_img = img
                self._color_count += 1
                self._color_stamp_ns = _stamp_to_ns(msg.header.stamp)
                self._color_encoding = msg.encoding

        def _on_depth(self, msg):
            try:
                img = self._image_to_depth_mm(msg)
            except Exception as e:
                self.get_logger().warning(f"Depth 转换失败 encoding={msg.encoding}: {e}")
                return
            with self._lock:
                self._depth_img = img
                self._depth_count += 1
                self._depth_stamp_ns = _stamp_to_ns(msg.header.stamp)
                self._depth_encoding = msg.encoding

        def _on_color_info(self, msg):
            with self._lock:
                self._cam_param = _camera_info_to_cam_param(msg)

        def _on_depth_info(self, msg):
            # 这里暂时不使用 depth_info。D2C 打开后，我们主要按 RGB 坐标做 ROI。
            pass

        @property
        def cam_param(self):
            with self._lock:
                return self._cam_param

        def get_latest_pair(self, wait_for_new_depth: bool, last_depth_count: int, timeout_sec: float):
            """
            返回最新 color/depth。
            wait_for_new_depth=True 时，主循环跟随 depth FPS，避免 10Hz depth 被重复算 30 次。
            """
            t0 = time.perf_counter()
            while rclpy.ok():
                with self._lock:
                    has_pair = self._color_img is not None and self._depth_img is not None
                    new_depth = self._depth_count > last_depth_count
                    if has_pair and ((not wait_for_new_depth) or new_depth):
                        color = self._color_img
                        depth = self._depth_img
                        depth_count = self._depth_count

                        if self.resize_depth_to_color and depth.shape[:2] != color.shape[:2]:
                            depth = cv.resize(depth, (color.shape[1], color.shape[0]), interpolation=cv.INTER_NEAREST)

                        if self.force_resize_to_config_size:
                            target_size = (int(FRAME_WIDTH), int(FRAME_HEIGHT))
                            if color.shape[1] != target_size[0] or color.shape[0] != target_size[1]:
                                color = cv.resize(color, target_size, interpolation=cv.INTER_AREA)
                            if depth.shape[1] != target_size[0] or depth.shape[0] != target_size[1]:
                                depth = cv.resize(depth, target_size, interpolation=cv.INTER_NEAREST)

                        return color, depth, depth_count

                if time.perf_counter() - t0 > timeout_sec:
                    return None, None, last_depth_count
                time.sleep(0.002)

            return None, None, last_depth_count

        def report_fps_if_needed(self, every_sec: float = 2.0):
            now = time.perf_counter()
            dt = now - self._last_report_time
            if dt < every_sec:
                return
            with self._lock:
                c = self._color_count
                d = self._depth_count
                c_shape = None if self._color_img is None else self._color_img.shape
                d_shape = None if self._depth_img is None else self._depth_img.shape
                c_enc = self._color_encoding
                d_enc = self._depth_encoding
            color_fps = (c - self._last_color_count) / dt
            depth_fps = (d - self._last_depth_count) / dt
            print(f"📷 ROS2输入 FPS RGB:{color_fps:.1f} Depth:{depth_fps:.1f} | RGB:{c_shape} {c_enc} Depth:{d_shape} {d_enc}")
            self._last_report_time = now
            self._last_color_count = c
            self._last_depth_count = d

        @staticmethod
        def _image_to_bgr(msg) -> np.ndarray:
            h, w = int(msg.height), int(msg.width)
            enc = msg.encoding.lower()
            if enc in ("bgr8", "8uc3"):
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3))
                return arr.copy()
            if enc == "rgb8":
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3))
                return cv.cvtColor(arr, cv.COLOR_RGB2BGR)
            if enc in ("mono8", "8uc1"):
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w))
                return cv.cvtColor(arr, cv.COLOR_GRAY2BGR)
            if enc in ("bgra8", "rgba8"):
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 4))
                code = cv.COLOR_BGRA2BGR if enc == "bgra8" else cv.COLOR_RGBA2BGR
                return cv.cvtColor(arr, code)
            raise ValueError(f"暂不支持 RGB encoding: {msg.encoding}")

        @staticmethod
        def _image_to_depth_mm(msg) -> np.ndarray:
            h, w = int(msg.height), int(msg.width)
            enc = msg.encoding.lower()
            if enc in ("16uc1", "mono16"):
                arr = np.frombuffer(msg.data, dtype=np.uint16).reshape((h, w))
                return arr.copy()
            if enc == "32fc1":
                arr_m = np.frombuffer(msg.data, dtype=np.float32).reshape((h, w))
                arr_mm = np.nan_to_num(arr_m, nan=0.0, posinf=0.0, neginf=0.0) * 1000.0
                return np.clip(arr_mm, 0, 65535).astype(np.uint16)
            raise ValueError(f"暂不支持 Depth encoding: {msg.encoding}")

else:
    class _Ros2CameraBridge:
        pass


class Ros2TopicCameraManager:
    """
    ROS2 topic 相机后端。接口和旧 OrbbecCameraManager 保持一致。
    """
    def __init__(self, enable_depth=True, width=640, height=480, fps=30):
        self.enable_depth = enable_depth
        self.width = width
        self.height = height
        self.fps = fps
        self.bridge = None
        self.executor = None
        self.thread = None
        self._last_depth_count = 0
        self._did_rclpy_init = False
        self.cam_param = None

    def start(self) -> bool:
        if not ROS2_AVAILABLE:
            print(f"❌ ROS2 Python 依赖不可用：{ROS2_IMPORT_ERROR}")
            print("   请用系统 Python3，并安装 ros-jazzy-rclpy / ros-jazzy-sensor-msgs。")
            return False
        try:
            try:
                rclpy.init(args=None)
                self._did_rclpy_init = True
            except RuntimeError:
                self._did_rclpy_init = False

            self.bridge = _Ros2CameraBridge(
                color_topic=ROS2_COLOR_TOPIC,
                depth_topic=ROS2_DEPTH_TOPIC,
                color_info_topic=ROS2_COLOR_INFO_TOPIC,
                depth_info_topic=ROS2_DEPTH_INFO_TOPIC,
                resize_depth_to_color=ROS2_RESIZE_DEPTH_TO_COLOR,
                force_resize_to_config_size=ROS2_FORCE_RESIZE_TO_FRAME_SIZE,
            )
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self.bridge)
            self.thread = threading.Thread(target=self.executor.spin, daemon=True)
            self.thread.start()

            print("✅ 相机后端启动：ROS2 topic 订阅模式")
            print("   请确认另一个终端已启动：ros2 launch orbbec_camera xxx.launch.py")
            return True
        except Exception as e:
            print(f"❌ ROS2 相机桥接启动失败: {e}")
            return False

    def get_frames(self):
        if self.bridge is None:
            return None, None
        color, depth, new_depth_count = self.bridge.get_latest_pair(
            wait_for_new_depth=ROS2_WAIT_FOR_NEW_DEPTH_FRAME,
            last_depth_count=self._last_depth_count,
            timeout_sec=ROS2_FRAME_TIMEOUT_SEC,
        )
        self._last_depth_count = new_depth_count
        self.cam_param = self.bridge.cam_param
        if ROS2_PRINT_INPUT_FPS:
            self.bridge.report_fps_if_needed(every_sec=ROS2_FPS_REPORT_INTERVAL_SEC)
        return color, depth

    def stop(self):
        if self.bridge is not None and self.executor is not None:
            try:
                self.executor.remove_node(self.bridge)
            except Exception:
                pass
        if self.bridge is not None:
            try:
                self.bridge.destroy_node()
            except Exception:
                pass
        if self.executor is not None:
            try:
                self.executor.shutdown(timeout_sec=0.2)
            except TypeError:
                try:
                    self.executor.shutdown()
                except Exception:
                    pass
            except Exception:
                pass
        if self.thread is not None:
            self.thread.join(timeout=0.2)
        if self._did_rclpy_init and rclpy is not None:
            try:
                rclpy.shutdown()
            except Exception:
                pass
        cv.destroyAllWindows()
        print("🔌 ROS2 相机桥接已关闭")


class SdkDirectCameraManager:
    """
    pyorbbecsdk 直连后端。

    为了不让 ROS2 模式在没有 pyorbbecsdk 时直接崩，旧 SDK 初始化放在这里按需导入。
    真正使用 SDK 后端时，会调用 camera_orbbec.py 里的 OrbbecCameraManager。
    """
    def __init__(self, enable_depth=True, width=640, height=480, fps=30):
        self.enable_depth = enable_depth
        self.width = width
        self.height = height
        self.fps = fps
        self.impl = None
        self.cam_param = None

    def start(self) -> bool:
        try:
            from camera_orbbec import OrbbecCameraManager
            self.impl = OrbbecCameraManager(
                enable_depth=self.enable_depth,
                width=self.width,
                height=self.height,
                fps=self.fps,
            )
            ok = self.impl.start()
            self.cam_param = self.impl.cam_param
            if ok:
                print("✅ 相机后端启动：pyorbbecsdk 直连模式")
            return ok
        except Exception as e:
            print(f"❌ SDK 相机后端启动失败: {e}")
            return False

    def get_frames(self):
        if self.impl is None:
            return None, None
        color, depth = self.impl.get_frames()
        self.cam_param = self.impl.cam_param
        return color, depth

    def stop(self):
        if self.impl is not None:
            self.impl.stop()


class UnifiedCameraManager:
    """
    给 main.py 使用的统一相机管理类。
    通过 config_switches.py 的 CAMERA_BACKEND 选择 ROS2 或 SDK。
    """
    def __init__(self, enable_depth=True, width=640, height=480, fps=30):
        backend = str(CAMERA_BACKEND).lower().strip()
        self.backend = backend
        if backend in ("ros2", "ros", "topic", "ros2_topic"):
            self.impl = Ros2TopicCameraManager(enable_depth, width, height, fps)
        elif backend in ("sdk", "orbbec", "pyorbbecsdk", "direct"):
            self.impl = SdkDirectCameraManager(enable_depth, width, height, fps)
        else:
            raise ValueError(f"未知 CAMERA_BACKEND={CAMERA_BACKEND}，只能填 ros2 或 sdk")

    @property
    def cam_param(self):
        return self.impl.cam_param

    def start(self) -> bool:
        print(f"📷 UnifiedCameraManager backend={self.backend}")
        return self.impl.start()

    def get_frames(self):
        return self.impl.get_frames()

    def stop(self):
        return self.impl.stop()
