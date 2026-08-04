# -*- coding: utf-8 -*-
"""
semi_auto_calibrate_640_print_only.py

作用：
    给 640x480 版本的小车视觉程序做“半自动标定”。

特点：
    1. 只负责标定，不会控制小车，不会打开串口。
    2. 鼠标按提示依次点 16 个关键点。
    3. 标定完成后，会在终端直接打印 calibration_640.py 里对应要填写的代码片段。
    4. 同时会在当前目录保存：
        calibration_output_640.py
        calibration_output_640.json
       你可以直接复制 calibration_output_640.py 里的内容填回 calibration_640.py。

按键：
    f：冻结 / 解除冻结画面，建议先按 f 冻结后再慢慢点
    鼠标左键：记录当前提示的标定点
    z：撤销上一个点
    r：清空全部点，重新开始
    s：保存并输出标定代码片段
    q / ESC：退出

标定点顺序：
    NEAR 近处：左外 / 中左 / 中右 / 右外
    45cm：左外 / 中左 / 中右 / 右外
    55cm：左外 / 中左 / 中右 / 右外
    95cm：左外 / 中左 / 中右 / 右外

说明：
    - NEAR 是视野最近处，不一定是车头 0cm，就是图像里最近能稳定看到路面的那一排。
    - 45/55/95cm 是你在地面上量出来的距离线。
    - 如果 95cm 当前看不到，可以先把最远能稳定看到的参考线当 95cm 点，后面再统一改距离含义。
"""

import json
import time
from pathlib import Path

import cv2 as cv
import numpy as np


# ==============================
# 1. 基本配置
# ==============================

FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
FRAME_FPS = 30

BIRD_WIDTH = 640
BIRD_HEIGHT = 480

WINDOW_NAME = "SemiAutoCalib640"
GUIDE_WINDOW_NAME = "CalibGuide_ThreeLane"
GUIDE_WIDTH = 560
GUIDE_HEIGHT = 560
OUTPUT_PY = "calibration_output_640.py"
OUTPUT_JSON = "calibration_output_640.json"

CALIBRATION_USAGE_NOTE = """
[视觉避障动态转向说明]
这次避障融合仍然使用原来的 16 个标定点，不需要新增点位。
新增的“障碍物面积/左右风险分数”来自已有 9 个 ROI 区域内的深度点统计。

需要重新标定的情况：
  1. 相机安装高度、俯仰角、左右位置变了；
  2. 画面分辨率或深度/彩色对齐方式变了；
  3. 你发现红/黄/绿避障框没有覆盖真实近地面通道。

只想改避障距离时不用重新点 16 个点，直接改 calibration_640.py：
  STOP_VALUE = 强制近障阈值(mm)
  WARN_VALUE = 中距离预警阈值(mm)
  FAR_VALUE  = 远距离提前绕障阈值(mm)
保存后重启 open_all.sh，safety_fusion_node 会重新读取这些距离。
"""


# ==============================
# 2. 标定点顺序
# ==============================

POINT_SEQUENCE = [
    # 近处：完整走廊外边界 + 中间车身碰撞通道边界
    ("ROI_NEAR_LEFT", "NEAR 近处 - 左外边界"),
    ("CENTER_NEAR_LEFT_MANUAL", "NEAR 近处 - 中间通道左边界"),
    ("CENTER_NEAR_RIGHT_MANUAL", "NEAR 近处 - 中间通道右边界"),
    ("ROI_NEAR_RIGHT", "NEAR 近处 - 右外边界"),

    # 45cm：红区和黄区分界
    ("ROI_45_LEFT", "45cm - 左外边界"),
    ("CENTER_45_LEFT_MANUAL", "45cm - 中间通道左边界"),
    ("CENTER_45_RIGHT_MANUAL", "45cm - 中间通道右边界"),
    ("ROI_45_RIGHT", "45cm - 右外边界"),

    # 55cm：黄区和绿区分界
    ("ROI_55_LEFT", "55cm - 左外边界"),
    ("CENTER_55_LEFT_MANUAL", "55cm - 中间通道左边界"),
    ("CENTER_55_RIGHT_MANUAL", "55cm - 中间通道右边界"),
    ("ROI_55_RIGHT", "55cm - 右外边界"),

    # 95cm：绿色提前观察区远端，同时作为鸟瞰图远端
    ("ROI_95_LEFT", "95cm - 左外边界"),
    ("CENTER_95_LEFT", "95cm - 中间通道左边界"),
    ("CENTER_95_RIGHT", "95cm - 中间通道右边界"),
    ("ROI_95_RIGHT", "95cm - 右外边界"),
]

POINT_NAME_TO_LABEL = dict(POINT_SEQUENCE)

ROW_POINT_KEYS = {
    "NEAR": ["ROI_NEAR_LEFT", "CENTER_NEAR_LEFT_MANUAL", "CENTER_NEAR_RIGHT_MANUAL", "ROI_NEAR_RIGHT"],
    "45cm": ["ROI_45_LEFT", "CENTER_45_LEFT_MANUAL", "CENTER_45_RIGHT_MANUAL", "ROI_45_RIGHT"],
    "55cm": ["ROI_55_LEFT", "CENTER_55_LEFT_MANUAL", "CENTER_55_RIGHT_MANUAL", "ROI_55_RIGHT"],
    "95cm": ["ROI_95_LEFT", "CENTER_95_LEFT", "CENTER_95_RIGHT", "ROI_95_RIGHT"],
}

# 副窗口示意图使用的“行 / 列”映射。
# 行代表距离线：NEAR / 45cm / 55cm / 95cm。
# 列代表四条边界：左外边界 / 中间左边界 / 中间右边界 / 右外边界。
# 标定时副窗口会把当前要点变成红色，已经点过的变成绿色，没点的保持灰色。
GUIDE_POINT_META = {
    "ROI_NEAR_LEFT": ("NEAR", "LEFT_OUTER"),
    "CENTER_NEAR_LEFT_MANUAL": ("NEAR", "CENTER_LEFT"),
    "CENTER_NEAR_RIGHT_MANUAL": ("NEAR", "CENTER_RIGHT"),
    "ROI_NEAR_RIGHT": ("NEAR", "RIGHT_OUTER"),

    "ROI_45_LEFT": ("45cm", "LEFT_OUTER"),
    "CENTER_45_LEFT_MANUAL": ("45cm", "CENTER_LEFT"),
    "CENTER_45_RIGHT_MANUAL": ("45cm", "CENTER_RIGHT"),
    "ROI_45_RIGHT": ("45cm", "RIGHT_OUTER"),

    "ROI_55_LEFT": ("55cm", "LEFT_OUTER"),
    "CENTER_55_LEFT_MANUAL": ("55cm", "CENTER_LEFT"),
    "CENTER_55_RIGHT_MANUAL": ("55cm", "CENTER_RIGHT"),
    "ROI_55_RIGHT": ("55cm", "RIGHT_OUTER"),

    "ROI_95_LEFT": ("95cm", "LEFT_OUTER"),
    "CENTER_95_LEFT": ("95cm", "CENTER_LEFT"),
    "CENTER_95_RIGHT": ("95cm", "CENTER_RIGHT"),
    "ROI_95_RIGHT": ("95cm", "RIGHT_OUTER"),
}

GUIDE_ROW_LABELS = ["95cm", "55cm", "45cm", "NEAR"]
GUIDE_COL_LABELS = ["LEFT_OUTER", "CENTER_LEFT", "CENTER_RIGHT", "RIGHT_OUTER"]

GUIDE_ROW_TEXT = {
    "NEAR": "NEAR / closest visible ground",
    "45cm": "45cm / RED-YELLOW boundary",
    "55cm": "55cm / YELLOW-GREEN boundary",
    "95cm": "95cm / far green boundary",
}

GUIDE_COL_TEXT = {
    "LEFT_OUTER": "left outer edge",
    "CENTER_LEFT": "center lane left edge",
    "CENTER_RIGHT": "center lane right edge",
    "RIGHT_OUTER": "right outer edge",
}


# ==============================
# 3. 全局鼠标状态
# ==============================

mouse_x = -1
mouse_y = -1
clicked_points = {}
click_order = []
frozen_frame = None
is_frozen = False


# ==============================
# 4. Orbbec 彩色相机管理
# ==============================

class OrbbecColorCamera:
    """
    只开启 Orbbec 的彩色流，用于标定取点。

    为什么这里只开 RGB？
        标定 ROI 点只需要 RGB 画面，不需要 depth。
        这样带宽最低，也避免 D2C / depth profile 影响标定脚本启动。
    """

    def __init__(self, width=640, height=480, fps=30):
        self.width = width
        self.height = height
        self.fps = fps
        self.pipeline = None
        self.config = None
        self.ok = False

    def start(self):
        try:
            from pyorbbecsdk import Context, Pipeline, Config, OBSensorType, OBFormat
        except Exception as e:
            print(f"❌ 未能导入 pyorbbecsdk：{e}")
            print("   将尝试使用 OpenCV VideoCapture(0) 兜底。")
            return False

        try:
            ctx = Context()
            if ctx.query_devices().get_count() == 0:
                print("❌ 未发现 Orbbec 设备。")
                return False

            self.pipeline = Pipeline()
            self.config = Config()

            color_profiles = self.pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
            color_profile = color_profiles.get_video_stream_profile(
                self.width,
                self.height,
                OBFormat.RGB,
                self.fps
            )
            self.config.enable_stream(color_profile)
            self.pipeline.start(self.config)
            self.ok = True

            print(f"🎥 标定脚本已开启 Orbbec RGB: {self.width}x{self.height}@{self.fps}, profile={color_profile}")
            return True

        except Exception as e:
            print(f"❌ Orbbec RGB 启动失败：{e}")
            print("   将尝试使用 OpenCV VideoCapture(0) 兜底。")
            self.ok = False
            return False

    def read(self):
        if not self.ok or self.pipeline is None:
            return None

        frames = self.pipeline.wait_for_frames(100)
        if frames is None:
            return None

        color_frame = frames.get_color_frame()
        if not color_frame:
            return None

        raw = color_frame.get_data()
        width = color_frame.get_width()
        height = color_frame.get_height()
        expected = width * height * 3

        if len(raw) != expected:
            return None

        data = np.asanyarray(raw)
        img_rgb = np.reshape(data, (height, width, 3))
        img_bgr = cv.cvtColor(img_rgb, cv.COLOR_RGB2BGR)
        return img_bgr

    def stop(self):
        try:
            if self.pipeline is not None:
                self.pipeline.stop()
        except Exception:
            pass


class OpenCVCameraFallback:
    """
    如果 Orbbec RGB 打不开，就用普通 OpenCV 摄像头兜底。
    一般你实际使用时还是应该走 OrbbecColorCamera。
    """

    def __init__(self, index=0, width=640, height=480, fps=30):
        self.index = index
        self.width = width
        self.height = height
        self.fps = fps
        self.cap = None

    def start(self):
        self.cap = cv.VideoCapture(self.index)
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv.CAP_PROP_FPS, self.fps)

        if not self.cap.isOpened():
            print("❌ OpenCV VideoCapture(0) 也无法打开。")
            return False

        print("⚠️ 当前使用 OpenCV VideoCapture(0) 兜底，不是 Orbbec SDK。")
        return True

    def read(self):
        if self.cap is None:
            return None
        ret, frame = self.cap.read()
        if not ret:
            return None
        return frame

    def stop(self):
        if self.cap is not None:
            self.cap.release()


# ==============================
# 5. 数学小工具
# ==============================

def point_to_int_list(p):
    return [int(round(float(p[0]))), int(round(float(p[1])))]


def midpoint(p1, p2):
    return np.array([(p1[0] + p2[0]) / 2.0, (p1[1] + p2[1]) / 2.0], dtype=np.float32)


def lerp_point(p_a, p_b, t):
    """
    两点线性插值。

    t=0 表示 p_a；t=1 表示 p_b。
    例如 65cm 在 55cm 到 95cm 之间：
        t = (65 - 55) / (95 - 55) = 0.25
    """
    a = np.array(p_a, dtype=np.float32)
    b = np.array(p_b, dtype=np.float32)
    return point_to_int_list(a + (b - a) * float(t))


def project_point(matrix, p):
    """
    用透视矩阵把原始 RGB 图坐标投影到 bird_view 坐标。
    """
    src = np.array([[[float(p[0]), float(p[1])]]], dtype=np.float32)
    dst = cv.perspectiveTransform(src, matrix)[0][0]
    return float(dst[0]), float(dst[1])


def clamp_int(v, lo, hi):
    return int(max(lo, min(hi, round(float(v)))))


def require_all_points():
    missing = [name for name, _label in POINT_SEQUENCE if name not in clicked_points]
    if missing:
        print("❌ 还有点没点完，不能输出：")
        for name in missing:
            print(f"   - {name}: {POINT_NAME_TO_LABEL[name]}")
        return False
    return True


# ==============================
# 6. 输出 calibration_640.py 可直接填写的代码片段
# ==============================

def generate_calibration_text(points):
    """
    根据 16 个手点，生成 calibration_640.py 里可以直接替换的代码片段。
    """
    p = {k: point_to_int_list(v) for k, v in points.items()}

    # 65cm 是 55cm 和 95cm 之间自动插值得到。
    # 这个点主要作为远端参考/调试点，不直接作为红黄绿分区边界。
    t65 = (65 - 55) / (95 - 55)
    p["ROI_65_LEFT"] = lerp_point(p["ROI_55_LEFT"], p["ROI_95_LEFT"], t65)
    p["ROI_65_RIGHT"] = lerp_point(p["ROI_55_RIGHT"], p["ROI_95_RIGHT"], t65)
    p["CENTER_65_LEFT_MANUAL"] = lerp_point(p["CENTER_55_LEFT_MANUAL"], p["CENTER_95_LEFT"], t65)
    p["CENTER_65_RIGHT_MANUAL"] = lerp_point(p["CENTER_55_RIGHT_MANUAL"], p["CENTER_95_RIGHT"], t65)

    # 鸟瞰图四点：远端用 95cm 外边界，近端用 NEAR 外边界。
    line_src = [
        p["ROI_95_LEFT"],
        p["ROI_95_RIGHT"],
        p["ROI_NEAR_LEFT"],
        p["ROI_NEAR_RIGHT"],
    ]

    src = np.float32(line_src)
    dst = np.float32([
        [0, 0],
        [BIRD_WIDTH, 0],
        [0, BIRD_HEIGHT],
        [BIRD_WIDTH, BIRD_HEIGHT],
    ])
    matrix = cv.getPerspectiveTransform(src, dst)

    def center_mid_at(distance_cm):
        if distance_cm == 55:
            return midpoint(p["CENTER_55_LEFT_MANUAL"], p["CENTER_55_RIGHT_MANUAL"])
        if distance_cm == 95:
            return midpoint(p["CENTER_95_LEFT"], p["CENTER_95_RIGHT"])
        t = (distance_cm - 55) / (95 - 55)
        left = np.array(p["CENTER_55_LEFT_MANUAL"], dtype=np.float32) + (
            np.array(p["CENTER_95_LEFT"], dtype=np.float32) - np.array(p["CENTER_55_LEFT_MANUAL"], dtype=np.float32)
        ) * t
        right = np.array(p["CENTER_55_RIGHT_MANUAL"], dtype=np.float32) + (
            np.array(p["CENTER_95_RIGHT"], dtype=np.float32) - np.array(p["CENTER_55_RIGHT_MANUAL"], dtype=np.float32)
        ) * t
        return midpoint(left, right)

    scan_y = {}
    raw_y_ref = {}
    for d in [55, 65, 75, 85, 95]:
        raw_mid = center_mid_at(d)
        _bx, by = project_point(matrix, raw_mid)
        scan_y[d] = clamp_int(by, 0, BIRD_HEIGHT - 1)
        raw_y_ref[int(round(float(raw_mid[1])))] = d

    # 为了复制回 calibration_640.py 顺手，这里按原文件的结构输出。
    lines = []
    lines.append("# ==============================")
    lines.append("# 半自动标定输出：640x480")
    lines.append("# 复制下面内容，替换 calibration_640.py 里对应的标定点和扫描线")
    lines.append("# ==============================\n")

    lines.append("# ---------- 巡线鸟瞰图标定点 ----------")
    lines.append("LINE_SRC_POINTS = np.float32([")
    lines.append(f"    {line_src[0]},     # 左上：95cm 左外边界")
    lines.append(f"    {line_src[1]},     # 右上：95cm 右外边界")
    lines.append(f"    {line_src[2]},     # 左下：视野最近处左外边界")
    lines.append(f"    {line_src[3]},     # 右下：视野最近处右外边界")
    lines.append("])\n")

    lines.append("RAW_Y_DISTANCE_REFERENCE_CM = {")
    for raw_y in sorted(raw_y_ref.keys(), reverse=True):
        lines.append(f"    {raw_y}: {raw_y_ref[raw_y]},")
    lines.append("}\n")

    lines.append("# ---------- 巡线扫描线参数 ----------")
    lines.append(f"SCAN_Y_55CM = {scan_y[55]}       # error1：约 55cm")
    lines.append(f"SCAN_Y_65CM = {scan_y[65]}       # error2：约 65cm")
    lines.append(f"SCAN_Y_75CM = {scan_y[75]}       # error3：约 75cm")
    lines.append(f"SCAN_Y_85CM = {scan_y[85]}       # error4：约 85cm")
    lines.append(f"SCAN_Y_95CM = {scan_y[95]}       # error5：约 95cm\n")

    lines.append("SCAN_CONFIGS = [")
    lines.append("    {\"serial_name\": \"error1\", \"label\": \"e55\", \"distance_cm\": 55, \"bird_y\": SCAN_Y_55CM},")
    lines.append("    {\"serial_name\": \"error2\", \"label\": \"e65\", \"distance_cm\": 65, \"bird_y\": SCAN_Y_65CM},")
    lines.append("    {\"serial_name\": \"error3\", \"label\": \"e75\", \"distance_cm\": 75, \"bird_y\": SCAN_Y_75CM},")
    lines.append("    {\"serial_name\": \"error4\", \"label\": \"e85\", \"distance_cm\": 85, \"bird_y\": SCAN_Y_85CM},")
    lines.append("    {\"serial_name\": \"error5\", \"label\": \"e95\", \"distance_cm\": 95, \"bird_y\": SCAN_Y_95CM},")
    lines.append("]")
    lines.append("SCAN_LINES = [cfg[\"bird_y\"] for cfg in SCAN_CONFIGS]\n")

    lines.append("# ---------- 完整可行走走廊外边界 ----------")
    outer_names = [
        "ROI_NEAR_LEFT", "ROI_NEAR_RIGHT",
        "ROI_45_LEFT", "ROI_45_RIGHT",
        "ROI_55_LEFT", "ROI_55_RIGHT",
        "ROI_65_LEFT", "ROI_65_RIGHT",
        "ROI_95_LEFT", "ROI_95_RIGHT",
    ]
    for name in outer_names:
        lines.append(f"{name} = {p[name]}")
    lines.append("")

    lines.append("# ---------- 中间车身碰撞通道边界 ----------")
    center_names = [
        "CENTER_NEAR_LEFT_MANUAL", "CENTER_NEAR_RIGHT_MANUAL",
        "CENTER_45_LEFT_MANUAL", "CENTER_45_RIGHT_MANUAL",
        "CENTER_55_LEFT_MANUAL", "CENTER_55_RIGHT_MANUAL",
        "CENTER_65_LEFT_MANUAL", "CENTER_65_RIGHT_MANUAL",
        "CENTER_95_LEFT", "CENTER_95_RIGHT",
    ]
    for name in center_names:
        lines.append(f"{name} = {p[name]}")
    lines.append("")

    lines.append("# ---------- 检查信息 ----------")
    lines.append(f"# 生成的扫描线 bird_y：55={scan_y[55]}, 65={scan_y[65]}, 75={scan_y[75]}, 85={scan_y[85]}, 95={scan_y[95]}")
    lines.append("# 如果 55/65/75/85/95 的 bird_y 挤在一起，说明你当前相机角度下这些距离线在画面里太靠近，需要重新抬高/调整相机或改距离线。")

    output_text = "\n".join(lines)

    output_json = {
        "points": p,
        "LINE_SRC_POINTS": line_src,
        "RAW_Y_DISTANCE_REFERENCE_CM": raw_y_ref,
        "SCAN_Y": {str(k): int(v) for k, v in scan_y.items()},
        "BIRD_WIDTH": BIRD_WIDTH,
        "BIRD_HEIGHT": BIRD_HEIGHT,
    }

    return output_text, output_json


# ==============================
# 7. 鼠标与显示
# ==============================


def get_current_target():
    """
    返回当前需要点击的标定点。

    返回：
        (index, name, label)：还没点完时返回当前点信息。
        None：16 个点已经全部点完。
    """
    if len(click_order) >= len(POINT_SEQUENCE):
        return None
    name, label = POINT_SEQUENCE[len(click_order)]
    return len(click_order), name, label


def print_current_target():
    """
    在终端打印当前应该点击哪个点。
    这样即使你没看副窗口，也能知道下一步点哪里。
    """
    target = get_current_target()
    print("\n" + "-" * 72)
    if target is None:
        print("✅ 16 个标定点已经全部点完。按 s 输出结果，或按 z 撤销。")
        print("-" * 72)
        return

    idx, name, label = target
    row_name, col_name = GUIDE_POINT_META[name]
    print(f"🎯 当前要点 [{idx + 1:02d}/{len(POINT_SEQUENCE)}]：{name}")
    print(f"   含义：{label}")
    print(f"   位置：{GUIDE_ROW_TEXT[row_name]}  ×  {GUIDE_COL_TEXT[col_name]}")
    print("   看副窗口 CalibGuide_ThreeLane：红色圆点就是当前要点。")
    print("-" * 72)


def draw_guide_text(img, text, org, color=(255, 255, 255), scale=0.48, thickness=1):
    """
    副窗口文字绘制。
    OpenCV 的 putText 对中文支持不好，所以副窗口尽量用英文；中文提示放终端。
    """
    x, y = org
    cv.putText(img, text, (x, y), cv.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv.LINE_AA)


def draw_three_lane_guide():
    """
    绘制三通道标定副窗口。

    作用：
        1. 用一个简化俯视图显示左借道 / 中间碰撞通道 / 右借道。
        2. 显示 NEAR / 45cm / 55cm / 95cm 四条距离线。
        3. 当前需要点击的点用红色大圆显示。
        4. 已经点过的点用绿色显示。
        5. 未点的点用灰色显示。

    注意：
        这个副窗口只是“点位顺序示意图”，不是相机真实图像。
        真正点击还是在主 RGB 窗口上点击。
    """
    guide = np.zeros((GUIDE_HEIGHT, GUIDE_WIDTH, 3), dtype=np.uint8)
    guide[:] = (28, 28, 28)

    # 示意图里的四条边界线 x 坐标。
    # 从左到右分别是：左外边界 / 中间左边界 / 中间右边界 / 右外边界。
    col_x = {
        "LEFT_OUTER": 105,
        "CENTER_LEFT": 235,
        "CENTER_RIGHT": 325,
        "RIGHT_OUTER": 455,
    }

    # 示意图里的四条距离线 y 坐标。
    # 越靠上表示越远，越靠下表示越近。
    row_y = {
        "95cm": 105,
        "55cm": 225,
        "45cm": 305,
        "NEAR": 445,
    }

    # 三个通道用半透明感的深色填充出来：左 / 中 / 右。
    # 这里颜色只是帮助看结构，不参与任何算法。
    left_poly = np.array([
        [col_x["LEFT_OUTER"], row_y["95cm"]],
        [col_x["CENTER_LEFT"], row_y["95cm"]],
        [col_x["CENTER_LEFT"], row_y["NEAR"]],
        [col_x["LEFT_OUTER"], row_y["NEAR"]],
    ], dtype=np.int32)
    center_poly = np.array([
        [col_x["CENTER_LEFT"], row_y["95cm"]],
        [col_x["CENTER_RIGHT"], row_y["95cm"]],
        [col_x["CENTER_RIGHT"], row_y["NEAR"]],
        [col_x["CENTER_LEFT"], row_y["NEAR"]],
    ], dtype=np.int32)
    right_poly = np.array([
        [col_x["CENTER_RIGHT"], row_y["95cm"]],
        [col_x["RIGHT_OUTER"], row_y["95cm"]],
        [col_x["RIGHT_OUTER"], row_y["NEAR"]],
        [col_x["CENTER_RIGHT"], row_y["NEAR"]],
    ], dtype=np.int32)

    cv.fillPoly(guide, [left_poly], (48, 44, 34))
    cv.fillPoly(guide, [center_poly], (36, 44, 54))
    cv.fillPoly(guide, [right_poly], (34, 44, 48))

    # 画边界竖线。
    for col in GUIDE_COL_LABELS:
        x = col_x[col]
        cv.line(guide, (x, row_y["95cm"]), (x, row_y["NEAR"]), (180, 180, 180), 1)

    # 画距离横线。
    row_colors = {
        "95cm": (0, 255, 0),
        "55cm": (0, 255, 255),
        "45cm": (0, 140, 255),
        "NEAR": (255, 255, 255),
    }
    for row in GUIDE_ROW_LABELS:
        y = row_y[row]
        cv.line(guide, (col_x["LEFT_OUTER"], y), (col_x["RIGHT_OUTER"], y), row_colors[row], 2)
        draw_guide_text(guide, row, (28, y + 5), row_colors[row], 0.55, 2)

    # 标出三个通道名称。
    draw_guide_text(guide, "LEFT PATH", (126, 78), (210, 210, 210), 0.48, 1)
    draw_guide_text(guide, "CENTER HIT", (215, 78), (210, 210, 210), 0.48, 1)
    draw_guide_text(guide, "RIGHT PATH", (345, 78), (210, 210, 210), 0.48, 1)

    # 顶部状态栏。
    draw_guide_text(guide, "Calibration Guide: click RED point on RGB window", (18, 28), (0, 255, 255), 0.55, 2)

    target = get_current_target()
    current_name = None if target is None else target[1]

    # 画 16 个目标点。
    for idx, (name, label) in enumerate(POINT_SEQUENCE):
        row, col = GUIDE_POINT_META[name]
        x, y = col_x[col], row_y[row]

        if name == current_name:
            color = (0, 0, 255)       # 当前点：红色
            radius = 12
            thickness = -1
        elif name in clicked_points:
            color = (0, 255, 0)       # 已点：绿色
            radius = 8
            thickness = -1
        else:
            color = (115, 115, 115)   # 未点：灰色
            radius = 6
            thickness = 1

        cv.circle(guide, (x, y), radius, color, thickness)
        cv.circle(guide, (x, y), radius + 3, (0, 0, 0), 1)
        draw_guide_text(guide, str(idx + 1), (x + 12, y - 8), color, 0.43, 1)

    # 底部详细提示。
    if target is None:
        draw_guide_text(guide, "DONE: press s to print/save, or z to undo", (18, GUIDE_HEIGHT - 52), (0, 255, 0), 0.58, 2)
    else:
        idx, name, label = target
        row, col = GUIDE_POINT_META[name]
        draw_guide_text(guide, f"NEXT {idx + 1:02d}/16: {name}", (18, GUIDE_HEIGHT - 72), (0, 0, 255), 0.58, 2)
        draw_guide_text(guide, f"ROW: {row}    COL: {col}", (18, GUIDE_HEIGHT - 48), (0, 0, 255), 0.50, 1)
        draw_guide_text(guide, "Terminal has Chinese explanation. RGB window is where you click.", (18, GUIDE_HEIGHT - 22), (230, 230, 230), 0.43, 1)

    return guide


def mouse_callback(event, x, y, flags, param):
    global mouse_x, mouse_y, clicked_points, click_order

    mouse_x = x
    mouse_y = y

    if event != cv.EVENT_LBUTTONDOWN:
        return

    if len(click_order) >= len(POINT_SEQUENCE):
        print("✅ 16 个点已经全部点完。按 s 输出标定结果，或按 z 撤销。")
        print_current_target()
        return

    name, label = POINT_SEQUENCE[len(click_order)]
    clicked_points[name] = [int(x), int(y)]
    click_order.append(name)
    print(f"✅ [{len(click_order):02d}/{len(POINT_SEQUENCE)}] {name} = [{x}, {y}]  # {label}")
    print_current_target()


def draw_text_with_bg(img, text, org, color=(255, 255, 255), scale=0.56, thickness=1):
    x, y = org
    (tw, th), baseline = cv.getTextSize(text, cv.FONT_HERSHEY_SIMPLEX, scale, thickness)
    cv.rectangle(img, (x - 4, y - th - 6), (x + tw + 4, y + baseline + 4), (0, 0, 0), -1)
    cv.putText(img, text, (x, y), cv.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv.LINE_AA)


def draw_calibration_overlay(frame):
    view = frame.copy()

    # 已点的点：画圆和名字
    for idx, name in enumerate(click_order):
        x, y = clicked_points[name]
        cv.circle(view, (x, y), 5, (0, 255, 255), -1)
        cv.circle(view, (x, y), 8, (0, 0, 0), 1)
        cv.putText(view, str(idx + 1), (x + 8, y - 8), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    # 每一行点满 4 个后，画横线，方便检查顺序有没有点反
    row_colors = {
        "NEAR": (255, 255, 255),
        "45cm": (0, 0, 255),
        "55cm": (0, 255, 255),
        "95cm": (0, 255, 0),
    }
    for row_name, keys in ROW_POINT_KEYS.items():
        if all(k in clicked_points for k in keys):
            pts = np.array([clicked_points[k] for k in keys], dtype=np.int32)
            cv.polylines(view, [pts], False, row_colors[row_name], 2)
            cv.putText(view, row_name, tuple(pts[0] + np.array([4, -8])), cv.FONT_HERSHEY_SIMPLEX, 0.55, row_colors[row_name], 2)

    # 如果全部点完，画鸟瞰图四点连线预览
    if all(k in clicked_points for k in ["ROI_95_LEFT", "ROI_95_RIGHT", "ROI_NEAR_LEFT", "ROI_NEAR_RIGHT"]):
        src_poly = np.array([
            clicked_points["ROI_95_LEFT"],
            clicked_points["ROI_95_RIGHT"],
            clicked_points["ROI_NEAR_RIGHT"],
            clicked_points["ROI_NEAR_LEFT"],
        ], dtype=np.int32)
        cv.polylines(view, [src_poly], True, (255, 0, 255), 2)

    # 顶部提示
    draw_text_with_bg(view, "Semi Auto Calibration 640x480", (12, 24), (0, 255, 255), 0.62, 2)
    draw_text_with_bg(view, "f freeze | left click point | z undo | r reset | s save/print | q exit", (12, 50), (255, 255, 255), 0.52, 1)

    if is_frozen:
        draw_text_with_bg(view, "FROZEN", (FRAME_WIDTH - 100, 24), (0, 255, 255), 0.62, 2)

    if len(click_order) < len(POINT_SEQUENCE):
        name, label = POINT_SEQUENCE[len(click_order)]
        draw_text_with_bg(view, f"Next [{len(click_order)+1:02d}/{len(POINT_SEQUENCE)}]: {name}", (12, FRAME_HEIGHT - 52), (0, 255, 255), 0.58, 2)
        draw_text_with_bg(view, label, (12, FRAME_HEIGHT - 24), (0, 255, 255), 0.58, 2)
    else:
        draw_text_with_bg(view, "All points done. Press s to print/save calibration output.", (12, FRAME_HEIGHT - 28), (0, 255, 0), 0.58, 2)

    if mouse_x >= 0 and mouse_y >= 0:
        draw_text_with_bg(view, f"mouse=({mouse_x},{mouse_y})", (FRAME_WIDTH - 160, FRAME_HEIGHT - 16), (255, 255, 255), 0.5, 1)
        cv.drawMarker(view, (mouse_x, mouse_y), (255, 255, 255), cv.MARKER_CROSS, 16, 1)

    return view


# ==============================
# 8. 主程序
# ==============================

def save_and_print_output():
    if not require_all_points():
        return

    output_text, output_json = generate_calibration_text(clicked_points)

    Path(OUTPUT_PY).write_text(output_text, encoding="utf-8")
    Path(OUTPUT_JSON).write_text(json.dumps(output_json, ensure_ascii=False, indent=2), encoding="utf-8")

    print("\n" + "=" * 80)
    print("✅ 半自动标定完成。下面这段可以复制到 calibration_640.py 对应位置：")
    print("=" * 80)
    print(output_text)
    print("=" * 80)
    print(f"✅ 已保存：{Path(OUTPUT_PY).resolve()}")
    print(f"✅ 已保存：{Path(OUTPUT_JSON).resolve()}")
    print("=" * 80 + "\n")


def main():
    global frozen_frame, is_frozen, clicked_points, click_order

    cam = OrbbecColorCamera(FRAME_WIDTH, FRAME_HEIGHT, FRAME_FPS)
    if not cam.start():
        cam = OpenCVCameraFallback(0, FRAME_WIDTH, FRAME_HEIGHT, FRAME_FPS)
        if not cam.start():
            return

    cv.namedWindow(WINDOW_NAME, cv.WINDOW_NORMAL)
    cv.resizeWindow(WINDOW_NAME, FRAME_WIDTH, FRAME_HEIGHT)
    cv.setMouseCallback(WINDOW_NAME, mouse_callback)

    cv.namedWindow(GUIDE_WINDOW_NAME, cv.WINDOW_NORMAL)
    cv.resizeWindow(GUIDE_WINDOW_NAME, GUIDE_WIDTH, GUIDE_HEIGHT)

    last_frame = None
    fps_count = 0
    fps_t0 = time.perf_counter()
    real_fps = 0.0

    print("\n====== 半自动标定开始 ======")
    print("建议：先把小车/相机固定好，让 45cm、55cm、95cm 参考线出现在画面里。")
    print("按 f 冻结画面，然后按提示从近到远依次点击 16 个点。")
    print("副窗口 CalibGuide_ThreeLane 会显示三通道示意图：红色圆点就是当前要标的点。")
    print(CALIBRATION_USAGE_NOTE)
    print("============================\n")
    print_current_target()

    try:
        while True:
            if not is_frozen:
                frame = cam.read()
                if frame is not None:
                    last_frame = frame
                    frozen_frame = frame.copy()
                    fps_count += 1

                    now = time.perf_counter()
                    if now - fps_t0 >= 1.0:
                        real_fps = fps_count / (now - fps_t0)
                        fps_count = 0
                        fps_t0 = now

            if last_frame is None:
                blank = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)
                draw_text_with_bg(blank, "Waiting for camera frame...", (20, 40), (0, 0, 255), 0.7, 2)
                cv.imshow(WINDOW_NAME, blank)
            else:
                base = frozen_frame if is_frozen and frozen_frame is not None else last_frame
                view = draw_calibration_overlay(base)
                draw_text_with_bg(view, f"cam_fps={real_fps:.1f}", (FRAME_WIDTH - 120, 50), (255, 255, 255), 0.5, 1)
                cv.imshow(WINDOW_NAME, view)

            guide_view = draw_three_lane_guide()
            cv.imshow(GUIDE_WINDOW_NAME, guide_view)

            key = cv.waitKey(1) & 0xFF

            if key in (27, ord('q')):
                break

            elif key == ord('f'):
                is_frozen = not is_frozen
                if is_frozen and last_frame is not None:
                    frozen_frame = last_frame.copy()
                print("🧊 已冻结画面" if is_frozen else "▶️ 已恢复实时画面")

            elif key == ord('z'):
                if click_order:
                    last_name = click_order.pop()
                    clicked_points.pop(last_name, None)
                    print(f"↩️ 撤销：{last_name}")
                    print_current_target()
                else:
                    print("ℹ️ 当前没有可撤销的点。")
                    print_current_target()

            elif key == ord('r'):
                clicked_points = {}
                click_order = []
                print("🧹 已清空所有标定点，请重新开始。")
                print_current_target()

            elif key == ord('s'):
                save_and_print_output()

    finally:
        cam.stop()
        cv.destroyAllWindows()
        print("🔌 标定脚本已退出。")


if __name__ == "__main__":
    main()
