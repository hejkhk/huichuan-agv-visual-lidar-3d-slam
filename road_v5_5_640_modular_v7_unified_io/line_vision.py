"""
line_vision.py

作用：
    HSV 调参、鸟瞰图透视变换、单条蓝色胶带巡线、error1~error5 计算。
"""

import cv2 as cv
import numpy as np

from config_switches import *
from calibration_640 import LINE_SRC_POINTS, BIRD_DST_POINTS, SCAN_CONFIGS, INVALID_ERROR, MIN_LINE_WIDTH_PIXELS
from utils import is_points_ready

def nothing(x):
    """
    HSV 滑动条的空回调函数。

    OpenCV 创建 trackbar 时必须传一个回调函数。
    但我们不需要滑动条变化时立刻做事，所以这里写一个空函数占位。
    """
    pass                                # 什么也不做

def create_hsv_window():
    """
    创建 HSV 调参窗口。

    作用：
        让你实时拖动 H/S/V 阈值，找到最适合蓝色胶带的范围。
    """
    cv.namedWindow("HSV_Tracker")       # 创建名为 HSV_Tracker 的窗口
    cv.resizeWindow("HSV_Tracker", 400, 300)  # 设置窗口大小，避免默认太小

    cv.createTrackbar("H_min", "HSV_Tracker", HSV_DEFAULT["H_min"], 179, nothing)
    cv.createTrackbar("H_max", "HSV_Tracker", HSV_DEFAULT["H_max"], 179, nothing)
    cv.createTrackbar("S_min", "HSV_Tracker", HSV_DEFAULT["S_min"], 255, nothing)
    cv.createTrackbar("S_max", "HSV_Tracker", HSV_DEFAULT["S_max"], 255, nothing)
    cv.createTrackbar("V_min", "HSV_Tracker", HSV_DEFAULT["V_min"], 255, nothing)
    cv.createTrackbar("V_max", "HSV_Tracker", HSV_DEFAULT["V_max"], 255, nothing)

def read_hsv_bounds():
    """
    读取 HSV 滑动条当前数值。

    返回：
        lower_bound：HSV 下限，给 cv.inRange 使用。
        upper_bound：HSV 上限，给 cv.inRange 使用。
    """
    h_min = cv.getTrackbarPos("H_min", "HSV_Tracker")  # 读取 H 下限
    h_max = cv.getTrackbarPos("H_max", "HSV_Tracker")  # 读取 H 上限
    s_min = cv.getTrackbarPos("S_min", "HSV_Tracker")  # 读取 S 下限
    s_max = cv.getTrackbarPos("S_max", "HSV_Tracker")  # 读取 S 上限
    v_min = cv.getTrackbarPos("V_min", "HSV_Tracker")  # 读取 V 下限
    v_max = cv.getTrackbarPos("V_max", "HSV_Tracker")  # 读取 V 上限

    lower_bound = np.array([h_min, s_min, v_min], dtype=np.uint8)  # 打包 HSV 下限
    upper_bound = np.array([h_max, s_max, v_max], dtype=np.uint8)  # 打包 HSV 上限

    return lower_bound, upper_bound     # 返回两个阈值数组

def build_perspective_matrix():
    """
    根据 LINE_SRC_POINTS 构建鸟瞰图透视变换矩阵。

    返回：
        matrix：可以给 cv.warpPerspective 使用的 3x3 透视变换矩阵。
        None：如果 LINE_SRC_POINTS 还没填，就返回 None。
    """
    if not is_points_ready(LINE_SRC_POINTS, min_points=4):
        return None                     # 没有4个点就不能做透视变换

    src = np.float32(LINE_SRC_POINTS)   # 确保源点是 float32，OpenCV 要求这种格式
    matrix = cv.getPerspectiveTransform(src, BIRD_DST_POINTS)  # 计算透视变换矩阵
    return matrix                       # 返回矩阵

def get_bird_view(frame, matrix):
    """
    把原始 RGB 图像变成鸟瞰图。

    参数：
        frame：原始 BGR 彩色图。
        matrix：透视变换矩阵。

    返回：
        bird_view：640x480 的鸟瞰图。
    """
    return cv.warpPerspective(frame, matrix, (BIRD_WIDTH, BIRD_HEIGHT))

def apply_color_dominance_filter(bird_view, hsv_mask):
    """
    可选的“通用颜色通道优势”过滤。      返回通道大于阈值的 mask。

    这一步放在 HSV 之后。

    HSV 的任务：
        按 H/S/V 选出你想找的颜色范围。

    颜色优势过滤的任务：
        再额外要求某个 B/G/R 通道明显比另外两个通道强，
        用来过滤某些被光照/反光误选进来的区域。

    COLOR_DOMINANCE_CHANNEL：
        False：关闭，不做这个过滤。
        0：要求 B 通道优势，适合蓝色线。
        1：要求 G 通道优势，适合绿色线。
        2：要求 R 通道优势，适合红色线。

    注意：
        黄线/白线/黑线不建议启用这个单通道优势过滤。
        黄线一般 R 和 G 都强；白线三个通道都强；黑线三个通道都弱。
        这些颜色请设为 False，只用 HSV + 几何过滤。
    """
    # 必须用 “is False” 判断。
    # 因为 Python 里 False == 0，如果写成 if not COLOR_DOMINANCE_CHANNEL，
    # 那么你设置 0 表示 B 通道时，也会被误认为关闭。
    if COLOR_DOMINANCE_CHANNEL is False:
        return hsv_mask

    if COLOR_DOMINANCE_CHANNEL not in (0, 1, 2):
        print("⚠️ COLOR_DOMINANCE_CHANNEL 只能是 False/0/1/2，当前值无效，已跳过颜色优势过滤")
        return hsv_mask

    # OpenCV 图像通道顺序是 BGR，不是 RGB。
    # channels[0] = B，channels[1] = G，channels[2] = R。
    channels = [ch.astype(np.int16) for ch in cv.split(bird_view)]

    dominant = channels[COLOR_DOMINANCE_CHANNEL]
    other_indices = [idx for idx in (0, 1, 2) if idx != COLOR_DOMINANCE_CHANNEL]

    # 优势条件：
    # 1. 目标通道本身不能太暗；
    # 2. 目标通道必须比另外两个通道都至少高 COLOR_DOMINANCE_MARGIN。
    dominance_mask = (
        (dominant >= COLOR_MIN_ABSOLUTE_VALUE) &
        (dominant >= channels[other_indices[0]] + COLOR_DOMINANCE_MARGIN) &
        (dominant >= channels[other_indices[1]] + COLOR_DOMINANCE_MARGIN)
    )

    return cv.bitwise_and(hsv_mask, hsv_mask, mask=dominance_mask.astype(np.uint8) * 255)

def filter_line_components(mask):
    """
    对 HSV mask 做连通域过滤。      返回面积、长宽大于阈值后的 mask。
    
    这一步与颜色无关：
        你 HSV 选蓝色，它过滤蓝色碎片；
        你 HSV 选红色，它过滤红色碎片；
        你 HSV 选其他颜色，它也一样工作。

    主要删除：面积小、高度短、不能跨越多条扫描线的反光块/杂物。
    """
    if not ENABLE_LINE_COMPONENT_FILTER:
        return mask

                                    # connectedComponentsWithStats直接数白色像素面积
    num_labels, labels, stats, _ = cv.connectedComponentsWithStats(mask, connectivity=8)  #connectivity=8 是什么意思？
                                                                                          # 它表示 8 邻域连接。
                                                                                          # 也就是一个白色像素的上下左右、四个斜角，只要碰到另一个白色像素，就认为它们连在一起：
                                                                                          # 左上  上  右上
                                                                                          #  左   我   右
                                                                                          # 左下  下  右下
    filtered = np.zeros_like(mask)

    for label in range(1, num_labels):  # 0 是背景，从 1 开始才是目标
        x = stats[label, cv.CC_STAT_LEFT]
        y = stats[label, cv.CC_STAT_TOP]
        w = stats[label, cv.CC_STAT_WIDTH]
        h = stats[label, cv.CC_STAT_HEIGHT]
        area = stats[label, cv.CC_STAT_AREA]

        if area < LINE_MIN_COMPONENT_AREA:
            continue
        if w < LINE_MIN_COMPONENT_WIDTH:
            continue
        if h < LINE_MIN_COMPONENT_HEIGHT:
            continue

        filtered[labels == label] = 255

    return filtered

def choose_scanline_edges(mask, y, expected_center=None):
    """
    [单条蓝色胶带巡线-回退旧版]
    从某条扫描线里找目标颜色胶带的左右边缘。            返回胶带的左右边缘 x 坐标。

    逻辑：
        1. 取出 mask 的第 y 行。
        2. 找到这一行所有白色像素，也就是 HSV/颜色过滤后认为是“目标线”的像素。
        3. 最左边白色像素作为胶带左边缘 x_left。
        4. 最右边白色像素作为胶带右边缘 x_right。
        5. 后面用 (x_left + x_right) / 2 得到胶带中心。

    注意：
        expected_center 参数保留只是为了兼容原来的调用接口。
        现在不再使用“双边界道路配对”的中心先验逻辑。
    """
    row = mask[y, :]                         # 取出鸟瞰图 mask 中第 y 行扫描线
    white_pixels = np.where(row == 255)[0]    # 找到这一行所有目标颜色像素的 x 坐标

    if len(white_pixels) < 2:                 # 少于 2 个像素，无法形成一段胶带宽度
        return None

    x_left = int(white_pixels.min())          # 胶带/色带最左边
    x_right = int(white_pixels.max())         # 胶带/色带最右边
    return x_left, x_right

def detect_blue_line_errors(color_frame, perspective_matrix):
    """
    在鸟瞰图中检测 HSV 选中的目标颜色胶带/线，输出 5 条扫描线的 error。         返回鸟瞰图、鸟瞰图处理后的 mask、error。

    参数：
        color_frame：原始 BGR 彩色图。
        perspective_matrix：透视变换矩阵。

    返回：
        bird_view：鸟瞰图，用于显示调试。
        mask：目标颜色掩码，用于显示调试。
        errors：长度为5的列表，对应 error1~error5。
                V4.4 中物理含义为：
                    error1 = 55cm 处误差
                    error2 = 65cm 处误差
                    error3 = 75cm 处误差
                    error4 = 85cm 处误差
                    error5 = 95cm 处误差
                有效值：center_x - BIRD_WIDTH/2。
                无效值：999。
    """
    errors = [INVALID_ERROR] * len(SCAN_CONFIGS)   # 先默认 5 条距离扫描线全丢失
    trajectory_points = []                         # 存储识别到的中心点，方便后续画轨迹

    if perspective_matrix is None:
        mask = np.zeros((BIRD_HEIGHT, BIRD_WIDTH), dtype=np.uint8)
        return None, mask, errors

    lower_bound, upper_bound = read_hsv_bounds()   # 读取当前 HSV 滑动条阈值

    bird_view = get_bird_view(color_frame, perspective_matrix)  # 原图转鸟瞰图

    hsv = cv.cvtColor(bird_view, cv.COLOR_BGR2HSV) # BGR 转 HSV，便于按颜色提取蓝色

    mask = cv.inRange(hsv, lower_bound, upper_bound)  # HSV 粗筛：蓝色范围内为255，其他为0

    # [V5.5.2-说明] 可选蓝色优势过滤。默认关闭，避免把算法写死成只能寻蓝线。
    mask = apply_color_dominance_filter(bird_view, mask)

    kernel = np.ones((5, 5), np.uint8)             # 5x5形态学核，用来消除噪声和补洞

    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)   # 开运算：去掉小白噪点
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)  # 闭运算：补上蓝线内部小黑洞

    # [V5.5.2-改动] 连通域过滤：删除面积/高度太小的颜色碎片。
    mask = filter_line_components(mask)

    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        cv.drawContours(bird_view, [contour], -1, (255, 0, 0), 2)

    # expected_center 用来做扫描线连续性约束。
    # 从最近的 55cm 开始，优先相信画面中心；后面扫描线优先跟随上一条有效中心，
    # 这样单个反光块不容易把某一行突然拉歪。
    expected_center = BIRD_WIDTH // 2

    for i, scan_cfg in enumerate(SCAN_CONFIGS):
        y = int(scan_cfg["bird_y"])                   # 当前距离扫描线在 bird_view 里的 y 坐标
        label = scan_cfg["label"]                     # 显示标签，例如 e55 / e65 / e75
        distance_cm = scan_cfg["distance_cm"]         # 当前扫描线对应的真实地面距离，单位 cm

        # 防御性保护：如果以后你手动改 y 改出边界，就直接跳过，避免数组越界。
        if y < 0 or y >= BIRD_HEIGHT:
            continue

        cv.line(bird_view, (0, y), (BIRD_WIDTH, y), (80, 80, 80), 1)  # 先画灰色扫描线

        # 在扫描线左侧标出真实距离，方便你看这一条线对应车前多少 cm。
        cv.putText(
            bird_view,
            f"{distance_cm}cm",
            (10, max(20, y - 6)),
            cv.FONT_HERSHEY_SIMPLEX,
            0.55,
            (80, 80, 80),
            1
        )

        # [单条蓝色胶带巡线-回退旧版]
        # 对这一行 mask 直接取最左/最右目标颜色像素，得到胶带左右边缘。
        edge_pair = choose_scanline_edges(mask, y, expected_center=expected_center)
        if edge_pair is None:
            continue                                  # 没找到足够的目标颜色像素，说明这一行没识别到胶带

        x_left, x_right = edge_pair

        if x_right - x_left <= MIN_LINE_WIDTH_PIXELS:
            continue                                  # 胶带宽度太窄，可能是噪声，不算有效线

        center_x = (x_left + x_right) // 2            # 单条胶带左右边缘的中点，就是胶带中心
        expected_center = center_x                    # 保留变量更新，方便以后需要连续性判断时扩展
        error = center_x - BIRD_WIDTH // 2            # 胶带中心相对鸟瞰图中心的偏差

        errors[i] = int(error)                        # 写入对应序号，保证 error1~error5 不错位
        trajectory_points.append((center_x, y))       # 记录轨迹点

        cv.line(bird_view, (x_left, y), (x_right, y), (0, 255, 0), 2)
        cv.circle(bird_view, (center_x, y), 5, (0, 0, 255), -1)
        cv.putText(
            bird_view,
            f"{label}:{error}",
            (center_x + 8, y - 8),
            cv.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 255),
            1
        )

    for i in range(1, len(trajectory_points)):
        cv.line(bird_view, trajectory_points[i - 1], trajectory_points[i], (0, 0, 255), 2)

    return bird_view, mask, errors
