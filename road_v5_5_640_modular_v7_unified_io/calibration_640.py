"""
calibration_640.py

作用：
    集中放 640x480 版本的鸟瞰图标定点、扫描线、三道路 ROI 标定点。
    后面重新鼠标精标的时候，主要改这个文件。
"""

import cv2 as cv
import numpy as np

from config_switches import *
from utils import is_point_ready, is_points_ready, draw_polygon

# ---------- 2.4 巡线鸟瞰图标定点 ----------
# [640低带宽旧标定版]
# 目的：
#     奥比中光反馈 RGB + Depth + D2C 高分辨率组合可能受 USB 带宽限制。
#     这个版本把相机输入改成 640x480@30，先用以前 640 版本跑通过的标定数据。
#
# 重要说明：
#     1. 这里是“先能跑、先验证帧率”的过渡标定。
#     2. LINE_SRC_POINTS 使用旧 road_v2 / road_v3 里能工作的 640 鸟瞰图源点。
#     3. 三道路 ROI 下面也整理成 640x480 坐标，但远端绿色区属于临时参考，后面最好重新用鼠标精标。
#     4. 因为这个版本的标定不是最终上车标定，所以默认 ENABLE_SERIAL_SEND=False。

# ---------- 巡线鸟瞰图标定点 ----------
LINE_SRC_POINTS = np.float32([
    [346, 274],     # 左上：95cm 左外边界
    [963, 285],     # 右上：95cm 右外边界
    [23, 694],     # 左下：视野最近处左外边界
    [1264, 706],     # 右下：视野最近处右外边界
])


# 旧 640 版本原始 RGB 图像中的粗略距离参考。
# 这些只用于理解/调试，不直接参与 OpenCV 计算。
RAW_Y_DISTANCE_REFERENCE_CM = {
    313: 55,
    304: 65,
    296: 75,
    287: 85,
    278: 95,
}

# [640旧版] 5 条 error 扫描线的历史物理距离。
# 串口字段仍然叫 error1~error5，只是这个 640 旧标定版的距离含义暂时恢复为旧数据。
SCAN_LINE_REAL_DISTANCE_CM = [18.0, 22.5, 28.0, 34.0, 44.5]

# 鸟瞰图输出坐标：640x480 直接输出 640x480 控制图。
BIRD_DST_POINTS = np.float32([
    [0, 0],                            # 鸟瞰图左上角
    [BIRD_WIDTH, 0],                   # 鸟瞰图右上角
    [0, BIRD_HEIGHT],                  # 鸟瞰图左下角
    [BIRD_WIDTH, BIRD_HEIGHT],         # 鸟瞰图右下角
])


# ---------- 2.5 巡线扫描线参数 ----------
# [640旧版] 恢复旧 road_v2 / road_v3 的 5 条鸟瞰图扫描线。
# bird_view 的 y 越大，越靠近车；y 越小，越远。
# ---------- 巡线扫描线参数 ----------
SCAN_Y_55CM = 71       # error1：约 55cm
SCAN_Y_65CM = 54       # error2：约 65cm
SCAN_Y_75CM = 36       # error3：约 75cm
SCAN_Y_85CM = 17       # error4：约 85cm
SCAN_Y_95CM = 0       # error5：约 95cm

SCAN_CONFIGS = [
    {"serial_name": "error1", "label": "e55", "distance_cm": 55, "bird_y": SCAN_Y_55CM},
    {"serial_name": "error2", "label": "e65", "distance_cm": 65, "bird_y": SCAN_Y_65CM},
    {"serial_name": "error3", "label": "e75", "distance_cm": 75, "bird_y": SCAN_Y_75CM},
    {"serial_name": "error4", "label": "e85", "distance_cm": 85, "bird_y": SCAN_Y_85CM},
    {"serial_name": "error5", "label": "e95", "distance_cm": 95, "bird_y": SCAN_Y_95CM},
]
SCAN_LINES = [cfg["bird_y"] for cfg in SCAN_CONFIGS]

INVALID_ERROR = 999                    # 999 表示该扫描线没识别到有效蓝线
MIN_LINE_WIDTH_PIXELS = 15             # [单条蓝色胶带巡线] 蓝色胶带在扫描线上的最小宽度；太小可能是噪声


# ---------- 2.6 避障距离参数 ----------
# 你的小车尺寸约 200mm x 150mm，相机基本在车头，所以不建议一上来用 700mm。
# 第一版推荐：
#     300mm：极限危险距离
#     400mm：强制停车距离
#     450mm：停车ROI缓冲距离
#     550mm：提前绕障距离

# [V4.5-改动] 这几个变量是避障距离阈值的“总旋钮”。
# 你要求停车阈值改成 45cm，所以 STOP_VALUE = 450。
# 后面如果发现停车太晚/太早，优先改这里，不要到处翻代码。
STOP_VALUE = 450                       # 45cm，强制停车阈值：红色危险区检测到障碍，优先停车
WARN_VALUE = 550                       # 55cm，黄色预警阈值：中间预警区有障碍，开始考虑绕障
FAR_VALUE = 950                        # 95cm，绿色提前观察阈值：远处中间通道有障碍，提前准备绕障



# ---------- 2.7 三道路避障 ROI 标定点 ----------
# [640低带宽旧标定版]
# 这里把旧 640 版本的已知点整理成当前 V5.5 的三道路红/黄/绿结构。
#
# 旧 640 已知点：
#     ROI_NEAR_LEFT / RIGHT：旧版近处走廊下边界。
#     ROI_45_LEFT / RIGHT：旧版 45cm 处走廊边界。
#     ROI_55_LEFT / RIGHT：旧版 55cm 处走廊边界，其中左点是旧版按对称关系推算的临时值。
#
# 注意：
#     1. 当前没有完整的旧 640 三道路 95cm 实测点，所以 FAR / GREEN 区域只是占位参考。
#     2. CENTER_* 点不再运行时自动比例估算，而是把旧版 30% / 70% 的结果直接写成固定坐标。
#     3. 上车前建议打开 RGB 主窗口，用鼠标重新精标：左/中/右三道路的 near/45/55/far 点。

# ---- 2.7.1 完整可行走走廊外边界：旧 640 坐标 ----
# ---------- 完整可行走走廊外边界 ----------
ROI_NEAR_LEFT = [23, 694]
ROI_NEAR_RIGHT = [1264, 706]
ROI_45_LEFT = [50, 464]
ROI_45_RIGHT = [1258, 483]
ROI_55_LEFT = [304, 303]
ROI_55_RIGHT = [1021, 326]
ROI_65_LEFT = [314, 296]
ROI_65_RIGHT = [1006, 316]
ROI_95_LEFT = [346, 274]
ROI_95_RIGHT = [963, 285]                # 旧 640：远端参考点，来自旧 LINE_SRC 右上点，不等于真实 95cm

# ---- 2.7.2 中间车身碰撞通道边界：固定写死，不再运行时比例估算 ----
# 下面这些点是按旧版中心通道 30% / 70% 位置整理出的固定坐标。
# 这样保留“可解释、可手改”的形式，避免代码运行时悄悄猜点。
# ---------- 中间车身碰撞通道边界 ----------
CENTER_NEAR_LEFT_MANUAL = [259, 698]
CENTER_NEAR_RIGHT_MANUAL = [1087, 702]
CENTER_45_LEFT_MANUAL = [350, 464]
CENTER_45_RIGHT_MANUAL = [970, 477]
CENTER_55_LEFT_MANUAL = [480, 307]
CENTER_55_RIGHT_MANUAL = [837, 319]
CENTER_65_LEFT_MANUAL = [485, 300]
CENTER_65_RIGHT_MANUAL = [824, 309]
CENTER_95_LEFT = [501, 277]
CENTER_95_RIGHT = [786, 280]                # 旧 640：远端中间通道右边界，临时占位，不等于真实 95cm


# ---- 2.7.2.1 半自动标定覆盖 ----
# 这个小段只做一件事：如果你运行 semi_auto_calibrate_640.py 生成了
# calibration_override_640.py，就用里面的新坐标覆盖上面的旧 640 临时坐标。
#
# 这样做的好处：
#     1. 原来这一整段手写注释和旧标定点都保留，方便你理解和回退。
#     2. 半自动标定结果单独放在 calibration_override_640.py，不会把这个主标定文件改乱。
#     3. 如果你删掉 calibration_override_640.py，程序会自动回到上面的旧 640 临时标定。
try:
    from calibration_override_640 import CALIBRATION_OVERRIDES
except Exception:
    CALIBRATION_OVERRIDES = {}


def _override_point(name, default):
    """
    从半自动标定结果里读取某个点。

    参数：
        name：点名，例如 "ROI_45_LEFT"。
        default：没有半自动标定时使用的旧默认点。

    返回：
        [x, y] 形式的点。
    """
    value = CALIBRATION_OVERRIDES.get(name, default)
    if not is_point_ready(value):
        return default
    return [int(value[0]), int(value[1])]


def _override_line_src(default_points):
    """
    从半自动标定结果里读取鸟瞰图 4 点。

    注意：
        LINE_SRC_POINTS 必须是 np.float32，给 cv.getPerspectiveTransform 使用。
    """
    value = CALIBRATION_OVERRIDES.get("LINE_SRC_POINTS", default_points.tolist())
    if not is_points_ready(value, min_points=4):
        return default_points
    return np.float32(value)


if CALIBRATION_OVERRIDES:
    print("✅ 已加载半自动标定覆盖: calibration_override_640.py")

    # 鸟瞰图 4 点。
    LINE_SRC_POINTS = _override_line_src(LINE_SRC_POINTS)

    # 可选：半自动标定脚本会根据 55/95cm 边界自动推算 65/75/85cm 的扫描线。
    # 如果 override 里没有 SCAN_CONFIGS，则继续使用上面旧 640 的扫描线。
    if "SCAN_CONFIGS" in CALIBRATION_OVERRIDES:
        SCAN_CONFIGS = CALIBRATION_OVERRIDES["SCAN_CONFIGS"]
        SCAN_LINES = [cfg["bird_y"] for cfg in SCAN_CONFIGS]
        SCAN_LINE_REAL_DISTANCE_CM = [cfg["distance_cm"] for cfg in SCAN_CONFIGS]

    # 可选：半自动标定脚本会把几个原图 y 坐标和距离关系保存下来，只用于理解和调试。
    if "RAW_Y_DISTANCE_REFERENCE_CM" in CALIBRATION_OVERRIDES:
        RAW_Y_DISTANCE_REFERENCE_CM = CALIBRATION_OVERRIDES["RAW_Y_DISTANCE_REFERENCE_CM"]

    # 完整可行走走廊外边界。
    ROI_NEAR_LEFT = _override_point("ROI_NEAR_LEFT", ROI_NEAR_LEFT)
    ROI_NEAR_RIGHT = _override_point("ROI_NEAR_RIGHT", ROI_NEAR_RIGHT)
    ROI_45_LEFT = _override_point("ROI_45_LEFT", ROI_45_LEFT)
    ROI_45_RIGHT = _override_point("ROI_45_RIGHT", ROI_45_RIGHT)
    ROI_55_LEFT = _override_point("ROI_55_LEFT", ROI_55_LEFT)
    ROI_55_RIGHT = _override_point("ROI_55_RIGHT", ROI_55_RIGHT)
    ROI_65_LEFT = _override_point("ROI_65_LEFT", ROI_65_LEFT)
    ROI_65_RIGHT = _override_point("ROI_65_RIGHT", ROI_65_RIGHT)
    ROI_95_LEFT = _override_point("ROI_95_LEFT", ROI_95_LEFT)
    ROI_95_RIGHT = _override_point("ROI_95_RIGHT", ROI_95_RIGHT)

    # 中间车身碰撞通道边界。
    CENTER_NEAR_LEFT_MANUAL = _override_point("CENTER_NEAR_LEFT_MANUAL", CENTER_NEAR_LEFT_MANUAL)
    CENTER_NEAR_RIGHT_MANUAL = _override_point("CENTER_NEAR_RIGHT_MANUAL", CENTER_NEAR_RIGHT_MANUAL)
    CENTER_45_LEFT_MANUAL = _override_point("CENTER_45_LEFT_MANUAL", CENTER_45_LEFT_MANUAL)
    CENTER_45_RIGHT_MANUAL = _override_point("CENTER_45_RIGHT_MANUAL", CENTER_45_RIGHT_MANUAL)
    CENTER_55_LEFT_MANUAL = _override_point("CENTER_55_LEFT_MANUAL", CENTER_55_LEFT_MANUAL)
    CENTER_55_RIGHT_MANUAL = _override_point("CENTER_55_RIGHT_MANUAL", CENTER_55_RIGHT_MANUAL)
    CENTER_65_LEFT_MANUAL = _override_point("CENTER_65_LEFT_MANUAL", CENTER_65_LEFT_MANUAL)
    CENTER_65_RIGHT_MANUAL = _override_point("CENTER_65_RIGHT_MANUAL", CENTER_65_RIGHT_MANUAL)
    CENTER_95_LEFT = _override_point("CENTER_95_LEFT", CENTER_95_LEFT)
    CENTER_95_RIGHT = _override_point("CENTER_95_RIGHT", CENTER_95_RIGHT)


# ---- 2.7.3 手动标定模式说明 ----
# 这里仍然不恢复“运行时比例估算”。
# 如果后面你觉得这个临时 640 标定不准，直接改上面的固定坐标即可。
# 点没填完时，程序会进入安全标定模式：不串口、不巡线、不避障。


def build_polygon(points):
    """
    用若干个点构造 OpenCV 多边形。

    参数：
        points：点列表，例如 [左下, 右下, 右上, 左上]。

    返回：
        np.int32 多边形数组，或者 None。
    """
    for p in points:                                  # 遍历每个点
        if not is_point_ready(p):                     # 只要有一个点没填
            return None                               # 整个 ROI 暂时不可用
    return np.array(points, dtype=np.int32)            # 所有点都可用，转成 OpenCV 需要的 int32


# ---- 2.7.4 最终 ROI 多边形 ----
# 这里不再“先赋值一次、后面再覆盖一次”。
# 每个 ROI 变量只定义一次，后面 ZONE_CONFIGS 直接使用这些变量。

# 中间红色危险区：最近可见处 ~ 45cm。
# 这是保命区；C_RED 使用它，有障碍时输出 STOP。
STOP_ROI_POINTS = build_polygon([
    CENTER_NEAR_LEFT_MANUAL,
    CENTER_NEAR_RIGHT_MANUAL,
    CENTER_45_RIGHT_MANUAL,
    CENTER_45_LEFT_MANUAL,
])

# 中间黄色预警区：45cm ~ 55cm。
CENTER_WARN_ROI_POINTS = build_polygon([
    CENTER_45_LEFT_MANUAL,
    CENTER_45_RIGHT_MANUAL,
    CENTER_55_RIGHT_MANUAL,
    CENTER_55_LEFT_MANUAL,
])

# 中间绿色提前观察区：55cm ~ 95cm。
CENTER_FAR_ROI_POINTS = build_polygon([
    CENTER_55_LEFT_MANUAL,
    CENTER_55_RIGHT_MANUAL,
    CENTER_95_RIGHT,
    CENTER_95_LEFT,
])

# 中间命中通道总区域：45cm ~ 95cm。
# 这个变量只作为语义清晰的总参考区，不再和 STOP_ROI_POINTS 重复赋值。
CENTER_HIT_ROI_POINTS = build_polygon([
    CENTER_45_LEFT_MANUAL,
    CENTER_45_RIGHT_MANUAL,
    CENTER_95_RIGHT,
    CENTER_95_LEFT,
])

# 左侧红/黄/绿借道检测区。
LEFT_STOP_ROI_POINTS = build_polygon([
    ROI_NEAR_LEFT,
    CENTER_NEAR_LEFT_MANUAL,
    CENTER_45_LEFT_MANUAL,
    ROI_45_LEFT,
])
LEFT_WARN_ROI_POINTS = build_polygon([
    ROI_45_LEFT,
    CENTER_45_LEFT_MANUAL,
    CENTER_55_LEFT_MANUAL,
    ROI_55_LEFT,
])
LEFT_FAR_ROI_POINTS = build_polygon([
    ROI_55_LEFT,
    CENTER_55_LEFT_MANUAL,
    CENTER_95_LEFT,
    ROI_95_LEFT,
])
LEFT_CLEAR_ROI_POINTS = build_polygon([
    ROI_NEAR_LEFT,
    CENTER_NEAR_LEFT_MANUAL,
    CENTER_95_LEFT,
    ROI_95_LEFT,
])

# 右侧红/黄/绿借道检测区。
RIGHT_STOP_ROI_POINTS = build_polygon([
    CENTER_NEAR_RIGHT_MANUAL,
    ROI_NEAR_RIGHT,
    ROI_45_RIGHT,
    CENTER_45_RIGHT_MANUAL,
])
RIGHT_WARN_ROI_POINTS = build_polygon([
    CENTER_45_RIGHT_MANUAL,
    ROI_45_RIGHT,
    ROI_55_RIGHT,
    CENTER_55_RIGHT_MANUAL,
])
RIGHT_FAR_ROI_POINTS = build_polygon([
    CENTER_55_RIGHT_MANUAL,
    ROI_55_RIGHT,
    ROI_95_RIGHT,
    CENTER_95_RIGHT,
])
RIGHT_CLEAR_ROI_POINTS = build_polygon([
    CENTER_NEAR_RIGHT_MANUAL,
    ROI_NEAR_RIGHT,
    ROI_95_RIGHT,
    CENTER_95_RIGHT,
])

# 65cm 完整走廊只保留为调试参考，不参与红黄绿分区决策。
FULL_ROAD_65_ROI_POINTS = build_polygon([
    ROI_NEAR_LEFT,
    ROI_NEAR_RIGHT,
    ROI_65_RIGHT,
    ROI_65_LEFT,
])

# 把 9 个红/黄/绿区域集中放进列表，后面统一检测、统一画框、统一统计。
# 注意：key 仍然保留 L_RED / C_YELLOW / C_GREEN 等名字，
# 因为后面的避障状态机和调试文字都是按这些 key 读取 zone_stats。
ZONE_CONFIGS = [
    {"key": "L_RED",    "name": "L_STOP", "lane": "LEFT",   "level": "RED",    "points": LEFT_STOP_ROI_POINTS,  "threshold": STOP_VALUE, "color": (0, 0, 255)},
    {"key": "C_RED",    "name": "C_STOP", "lane": "CENTER", "level": "RED",    "points": STOP_ROI_POINTS,       "threshold": STOP_VALUE, "color": (0, 0, 255)},
    {"key": "R_RED",    "name": "R_STOP", "lane": "RIGHT",  "level": "RED",    "points": RIGHT_STOP_ROI_POINTS, "threshold": STOP_VALUE, "color": (0, 0, 255)},
    {"key": "L_YELLOW", "name": "L_WARN", "lane": "LEFT",   "level": "YELLOW", "points": LEFT_WARN_ROI_POINTS,  "threshold": WARN_VALUE, "color": (0, 255, 255)},
    {"key": "C_YELLOW", "name": "C_WARN", "lane": "CENTER", "level": "YELLOW", "points": CENTER_WARN_ROI_POINTS,"threshold": WARN_VALUE, "color": (0, 255, 255)},
    {"key": "R_YELLOW", "name": "R_WARN", "lane": "RIGHT",  "level": "YELLOW", "points": RIGHT_WARN_ROI_POINTS, "threshold": WARN_VALUE, "color": (0, 255, 255)},
    {"key": "L_GREEN",  "name": "L_FAR",  "lane": "LEFT",   "level": "GREEN",  "points": LEFT_FAR_ROI_POINTS,   "threshold": FAR_VALUE,  "color": (0, 255, 0)},
    {"key": "C_GREEN",  "name": "C_FAR",  "lane": "CENTER", "level": "GREEN",  "points": CENTER_FAR_ROI_POINTS, "threshold": FAR_VALUE,  "color": (0, 255, 0)},
    {"key": "R_GREEN",  "name": "R_FAR",  "lane": "RIGHT",  "level": "GREEN",  "points": RIGHT_FAR_ROI_POINTS,  "threshold": FAR_VALUE,  "color": (0, 255, 0)},
]


# ---------- 2.7.5 标定完整性检查 ----------
# 这个检查用于防止“点没填完，小车却开始动”。
# 只要下面任意关键点没填好，程序就进入安全标定模式：
#     只显示 RGB 窗口和已经能画出的 ROI 边框；
#     不显示鸟瞰图；
#     不计算巡线；
#     不判断避障；
#     不打开串口；
#     不发送任何运动指令。

def collect_missing_calibration_items():
    """
    收集缺失的关键标定项。

    返回：
        missing_items：字符串列表。空列表表示标定完整，可以正常运行。
    """
    missing_items = []

    # 1. 鸟瞰图 4 点：没填就不能做巡线透视变换。
    if not is_points_ready(LINE_SRC_POINTS, min_points=4):
        missing_items.append("LINE_SRC_POINTS：鸟瞰图透视变换 4 点未完整填写")

    # 2. 原始 RGB 图上的关键 ROI 点。
    required_single_points = [
        ("ROI_NEAR_LEFT", ROI_NEAR_LEFT),
        ("ROI_NEAR_RIGHT", ROI_NEAR_RIGHT),
        ("ROI_45_LEFT", ROI_45_LEFT),
        ("ROI_45_RIGHT", ROI_45_RIGHT),
        ("ROI_55_LEFT", ROI_55_LEFT),
        ("ROI_55_RIGHT", ROI_55_RIGHT),
        ("ROI_65_LEFT", ROI_65_LEFT),
        ("ROI_65_RIGHT", ROI_65_RIGHT),
        ("ROI_95_LEFT", ROI_95_LEFT),
        ("ROI_95_RIGHT", ROI_95_RIGHT),

        ("CENTER_NEAR_LEFT_MANUAL", CENTER_NEAR_LEFT_MANUAL),
        ("CENTER_NEAR_RIGHT_MANUAL", CENTER_NEAR_RIGHT_MANUAL),
        ("CENTER_45_LEFT_MANUAL", CENTER_45_LEFT_MANUAL),
        ("CENTER_45_RIGHT_MANUAL", CENTER_45_RIGHT_MANUAL),
        ("CENTER_55_LEFT_MANUAL", CENTER_55_LEFT_MANUAL),
        ("CENTER_55_RIGHT_MANUAL", CENTER_55_RIGHT_MANUAL),
        ("CENTER_65_LEFT_MANUAL", CENTER_65_LEFT_MANUAL),
        ("CENTER_65_RIGHT_MANUAL", CENTER_65_RIGHT_MANUAL),
        ("CENTER_95_LEFT", CENTER_95_LEFT),
        ("CENTER_95_RIGHT", CENTER_95_RIGHT),
    ]

    for name, point in required_single_points:
        if not is_point_ready(point):
            missing_items.append(f"{name}：点未填写或格式不对，应为 [x, y]")

    # 3. 最终 9 个红黄绿 ROI。
    for cfg in ZONE_CONFIGS:
        if not is_points_ready(cfg["points"], min_points=3):
            missing_items.append(f"{cfg['key']} / {cfg['name']}：ROI 多边形无法构造")

    return missing_items


def print_calibration_report(missing_items):
    """
    在终端打印标定状态。
    """
    if not missing_items:
        print("✅ 标定检查通过：鸟瞰图点、避障 ROI 点均已填写，允许正常巡线/避障/串口发送。")
        return

    print("⚠️ 标定检查未通过：程序进入【安全标定模式】")
    print("   安全标定模式下：只显示 RGB 画面和已填写 ROI 边框；不显示鸟瞰图；不巡线；不避障；不打开串口；不发送运动指令。")
    print("   缺失/错误项目如下：")
    for item in missing_items:
        print(f"   - {item}")


def draw_calibration_preview(color_frame, missing_items=None, draw_text=True):
    """
    安全标定模式下的显示函数。

    作用：
        只画已经能构造出来的 ROI 线，帮助你继续标点；
        不画障碍物填充，不用 zone_stats，不触发避障逻辑。
    """
    if SHOW_ROI_POLYGONS:
        for cfg in ZONE_CONFIGS:
            draw_polygon(color_frame, cfg["points"], cfg["color"], cfg["name"], draw_label=draw_text)

    if not draw_text:
        return

    cv.putText(
        color_frame,
        "SAFE CALIB MODE: missing calibration, no serial / no motion",
        (20, 36),
        cv.FONT_HERSHEY_SIMPLEX,
        0.72,
        (0, 0, 255),
        2
    )

    if missing_items:
        cv.putText(
            color_frame,
            f"Missing items: {len(missing_items)}  Check terminal.",
            (20, 68),
            cv.FONT_HERSHEY_SIMPLEX,
            0.68,
            (0, 0, 255),
            2
        )
