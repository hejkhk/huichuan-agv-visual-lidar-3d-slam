"""
camera_orbbec.py

作用：
    Orbbec Gemini2 相机管理类：发现设备、启动 RGB+Depth、D2C 对齐、取帧、关闭。
"""

import cv2 as cv
import numpy as np
import time
from pyorbbecsdk import *

from config_switches import *

class OrbbecCameraManager:
    """
    Orbbec Gemini2 通用相机管理类。

    作用：
        1. 自动发现设备。
        2. 开启 RGB 彩色流。
        3. 开启深度流。
        4. 优先使用硬件 D2C 对齐，让 depth 像素和 RGB 像素尽量一一对应。
        5. 对彩色帧和深度帧做字节数校验，防止残帧导致程序崩溃。
    """

    def __init__(self, enable_depth=True, width=640, height=480, fps=30):
        self.enable_depth = enable_depth        # 是否启用深度图
        self.width = width                      # 图像宽度
        self.height = height                    # 图像高度
        self.fps = fps                          # 帧率

        self.ctx = Context()                    # Orbbec SDK 上下文，用来查询设备
        self.pipeline = Pipeline()              # 数据管道，用来启动和读取相机流
        self.config = Config()                  # 相机配置对象，用来启用 RGB / depth 流
        self.device = None                      # 设备对象，当前代码暂时不用
        self.cam_param = None                   # 相机内参，后面计算 XYZ 会用
        self.depth_profile = None               # 记录当前实际启用的深度 profile，方便调试
        self.depth_resize_warned = False        # depth 尺寸不一致时只打印一次提醒

        self.last_valid_time = time.time()      # 看门狗时间，记录上一次成功拿到完整图像的时间

    @staticmethod
    def _parse_profile_info(profile):
        """
        从 VideoStreamProfile 的字符串里解析 宽度 / 高度 / FPS。

        例子：
            "<VideoStreamProfile: 640x400@30>" -> (640, 400, 30)

        为什么要解析？
            Orbbec SDK 的 profile 对象在不同版本里不一定稳定暴露 get_width()/get_fps()。
            但 print(profile) 基本都会显示 640x400@30 这种格式。
            所以这里用字符串解析，兼容性更好。
        """
        text = str(profile)
        width = -1
        height = -1
        fps = -1

        try:
            if ":" in text:
                body = text.split(":", 1)[1].strip().strip(">")
            else:
                body = text.strip().strip(">")

            if "@" in body:
                size_part, fps_part = body.split("@", 1)
                fps_digits = []
                for ch in fps_part:
                    if ch.isdigit():
                        fps_digits.append(ch)
                    else:
                        break
                if fps_digits:
                    fps = int("".join(fps_digits))
            else:
                size_part = body

            if "x" in size_part:
                w_part, h_part = size_part.split("x", 1)
                w_digits = "".join(ch for ch in w_part if ch.isdigit())
                h_digits = "".join(ch for ch in h_part if ch.isdigit())
                if w_digits:
                    width = int(w_digits)
                if h_digits:
                    height = int(h_digits)
        except Exception:
            pass

        return width, height, fps

    def _choose_low_bandwidth_d2c_profile(self, d2c_list):
        """
        从硬件 D2C depth profile 列表里选择低带宽 profile。

        旧版问题：
            直接使用 d2c_list[0]。
            你的运行日志已经证明：即使 RGB 改成 640x480@30，d2c_list[0]
            仍然可能是 1280x800@30，这没有真正降低 RGB-D 带宽。

        新策略：
            1. 打印所有 D2C 候选，让你能在终端看到 SDK 到底给了哪些选项。
            2. 优先选择 D2C_DEPTH_WIDTH_PRIORITY 里面指定的低宽度，例如 640 / 320。
            3. 同一宽度下优先选择和 RGB fps 一致的 profile。
            4. 如果找不到低带宽 D2C，默认不自动退回 1280x800@30，避免又回到带宽问题。
        """
        candidates = []
        for idx in range(len(d2c_list)):
            profile = d2c_list[idx]
            w, h, fps = self._parse_profile_info(profile)
            candidates.append({
                "profile": profile,
                "width": w,
                "height": h,
                "fps": fps,
                "idx": idx,
            })
            print(f"   D2C候选[{idx}]: {profile}, parsed={w}x{h}@{fps}")

        if not candidates:
            return None

        width_priority = globals().get("D2C_DEPTH_WIDTH_PRIORITY", [640, 320])

        for prefer_w in width_priority:
            same_width = [c for c in candidates if c["width"] == prefer_w]
            if not same_width:
                continue

            exact = [c for c in same_width if c["fps"] == self.fps]
            if exact:
                chosen = exact[0]
                print(
                    f"✅ D2C选择：优先深度宽度 {prefer_w}，"
                    f"找到与 RGB {self.fps}fps 匹配的 profile: "
                    f"{chosen['width']}x{chosen['height']}@{chosen['fps']}"
                )
                return chosen["profile"]

            same_width.sort(key=lambda c: abs(c["fps"] - self.fps) if c["fps"] >= 0 else 9999)
            chosen = same_width[0]
            print(
                f"⚠️ D2C选择：深度宽度 {prefer_w} 没有 {self.fps}fps 完全匹配，"
                f"改用最接近的 {chosen['width']}x{chosen['height']}@{chosen['fps']}"
            )
            return chosen["profile"]

        if globals().get("ALLOW_HIGH_BANDWIDTH_D2C_FALLBACK", False):
            exact = [c for c in candidates if c["fps"] == self.fps]
            if exact:
                chosen = exact[0]
                print(
                    f"⚠️ D2C选择：没有低带宽 D2C，允许退回高带宽 profile: "
                    f"{chosen['width']}x{chosen['height']}@{chosen['fps']}"
                )
                return chosen["profile"]

            chosen = candidates[0]
            print(f"⚠️ D2C选择：允许退回第一个 profile: {chosen['profile']}")
            return chosen["profile"]

        print("⚠️ D2C列表里没有找到低带宽 depth profile，且配置禁止退回 1280x800 这类高带宽 D2C。")
        return None

    def _enable_low_depth_fallback(self):
        """
        启用普通低分辨率 Depth 作为兜底。

        工程含义：
            如果硬件 D2C 只有 1280x800@30 这种高带宽 profile，
            那就不要硬上 D2C，改成普通低分辨率 Depth，例如 640x400@30。

        注意：
            普通 Depth 和 RGB 的像素坐标不一定天然一一对应。
            所以后面 get_frames() 会把 depth 最近邻 resize 到 RGB 尺寸，
            先保证你的 ROI / 鼠标显示 / 避障数组不会错位或越界。
            真正上车前仍建议重新标定或确认对齐效果。
        """
        depth_profiles = self.pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
        depth_w = globals().get("DEPTH_FALLBACK_WIDTH", self.width)
        depth_h = globals().get("DEPTH_FALLBACK_HEIGHT", 400)
        depth_fps = globals().get("DEPTH_FALLBACK_FPS", self.fps)

        try:
            depth_profile = depth_profiles.get_video_stream_profile(
                depth_w,
                depth_h,
                OBFormat.Y16,
                depth_fps
            )
        except Exception as e:
            print(f"⚠️ 普通 Depth {depth_w}x{depth_h}@{depth_fps} 获取失败，退回 {self.width}x{self.height}@{self.fps}：{e}")
            depth_profile = depth_profiles.get_video_stream_profile(
                self.width,
                self.height,
                OBFormat.Y16,
                self.fps
            )

        self.config.enable_stream(depth_profile)
        self.depth_profile = depth_profile
        print(f"✅ 已开启普通低带宽 Depth: {depth_profile}")

        if globals().get("ENABLE_SOFTWARE_ALIGN_FALLBACK", True):
            try:
                self.config.set_align_mode(OBAlignMode.SW_MODE)
                print("✅ 已请求软件 D2C 对齐 SW_MODE。若 SDK 不支持，会在启动或取帧时表现出来。")
            except Exception as e:
                print(f"⚠️ 软件 D2C 对齐请求失败，继续使用普通 Depth + resize：{e}")

    def _find_best_profiles(self):
        """
        自动寻找 RGB 和 depth 的最佳配置。

        返回：
            True：配置成功。
            False：配置失败。
        """
        try:
            # 每次启动前重新创建 Config，避免上一次失败配置残留。
            self.config = Config()

            color_profiles = self.pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
            self.color_profile = color_profiles.get_video_stream_profile(
                self.width,
                self.height,
                OBFormat.RGB,
                self.fps
            )
            self.config.enable_stream(self.color_profile)
            print(f"🎥 已请求低带宽彩色流: {self.width}x{self.height}@{self.fps}, profile={self.color_profile}")

            if self.enable_depth:
                if globals().get("USE_HW_D2C", True):
                    d2c_list = self.pipeline.get_d2c_depth_profile_list(
                        self.color_profile,
                        OBAlignMode.HW_MODE
                    )

                    depth_profile = None
                    if len(d2c_list) > 0:
                        depth_profile = self._choose_low_bandwidth_d2c_profile(d2c_list)

                    if depth_profile is not None:
                        self.config.enable_stream(depth_profile)
                        self.config.set_align_mode(OBAlignMode.HW_MODE)
                        self.depth_profile = depth_profile
                        print(f"✅ 已开启硬件 D2C 对齐: 使用深度配置 {depth_profile}")
                    else:
                        print("🔁 改用普通低带宽 Depth 兜底。")
                        self._enable_low_depth_fallback()
                else:
                    print("ℹ️ 配置 USE_HW_D2C=False，直接使用普通低带宽 Depth。")
                    self._enable_low_depth_fallback()

            return True

        except Exception as e:
            print(f"❌ 配置匹配失败: {e}")
            return False

    def start(self):
        """
        启动相机。

        返回：
            True：启动成功。
            False：启动失败。
        """
        if self.ctx.query_devices().get_count() == 0:
            print("❌ 未发现 Orbbec 设备，请检查 USB 连接")
            return False

        if not self._find_best_profiles():
            return False

        self.last_valid_time = time.time()

        try:
            if globals().get("ENABLE_FRAME_SYNC", True):
                self.pipeline.enable_frame_sync()
                print("✅ 已开启 frame sync")
            else:
                print("ℹ️ 未开启 frame sync")

            self.pipeline.start(self.config)
            self.cam_param = self.pipeline.get_camera_param()
            print("🚀 相机启动成功")
            return True

        except Exception as e:
            print(f"❌ 管道启动失败: {e}")
            return False

    def get_frames(self):
        """
        获取一帧 RGB + depth 对齐数据。

        返回：
            color_img：BGR 彩色图，给 OpenCV 显示和 HSV 使用。
            depth_img：uint16 深度图，单位通常是 mm。
        """
        frames = self.pipeline.wait_for_frames(100)  # 等待一组同步帧，超时100ms

        color_img = None                            # 彩色图初始为空
        depth_img = None                            # 深度图初始为空

        if frames is not None:
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if color_frame:
                color_raw_data = color_frame.get_data()
                expected_c_bytes = color_frame.get_width() * color_frame.get_height() * 3

                if len(color_raw_data) == expected_c_bytes:
                    color_data = np.asanyarray(color_raw_data)
                    color_img = np.reshape(
                        color_data,
                        (color_frame.get_height(), color_frame.get_width(), 3)
                    )
                    color_img = cv.cvtColor(color_img, cv.COLOR_RGB2BGR)
                else:
                    color_img = None

            if depth_frame:
                depth_raw_data = depth_frame.get_data()
                expected_d_bytes = depth_frame.get_width() * depth_frame.get_height() * 2

                if len(depth_raw_data) == expected_d_bytes:
                    depth_data = np.frombuffer(depth_raw_data, dtype=np.uint16)
                    depth_img = depth_data.reshape(
                        (depth_frame.get_height(), depth_frame.get_width())
                    )
                else:
                    depth_img = None

        if color_img is not None and depth_img is not None:
            # [低带宽版保护]
            # 有些 D2C / 普通 Depth 配置下，RGB 是 640x480，但 depth 可能是 1280x800 或 640x400。
            # 后面的 ROI 标定点是按 RGB 坐标写的，所以这里必须把 depth 统一到 RGB 尺寸。
            if globals().get("RESIZE_DEPTH_TO_COLOR", True) and depth_img.shape[:2] != color_img.shape[:2]:
                if not self.depth_resize_warned:
                    print(
                        f"⚠️ depth尺寸 {depth_img.shape[1]}x{depth_img.shape[0]} 与 RGB尺寸 "
                        f"{color_img.shape[1]}x{color_img.shape[0]} 不一致，已用最近邻缩放到 RGB 尺寸。"
                    )
                    self.depth_resize_warned = True
                depth_img = cv.resize(
                    depth_img,
                    (color_img.shape[1], color_img.shape[0]),
                    interpolation=cv.INTER_NEAREST
                )

            self.last_valid_time = time.time()
            return color_img, depth_img

        if time.time() - self.last_valid_time > 5.0:
            print("💀 严重错误：连续 5 秒未收到完整图像，触发断连保护！程序结束。")
            self.stop()
            raise ConnectionError("Camera disconnected or frozen for 5 seconds.")

        return None, None

    def stop(self):
        """
        安全关闭相机和窗口。
        """
        try:
            self.pipeline.stop()
        except Exception as e:
            print(f"⚠️ 关闭相机时出现异常，可忽略：{e}")
        cv.destroyAllWindows()
        print("🔌 相机已安全关闭")
