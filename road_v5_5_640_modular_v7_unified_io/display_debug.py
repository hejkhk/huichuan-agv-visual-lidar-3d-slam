"""
display_debug.py

作用：
    所有和调试显示有关的函数：鼠标深度、ROI/障碍显示、状态文字、FPS 文字。
"""

import cv2 as cv
import numpy as np

from config_switches import *
from calibration_640 import ZONE_CONFIGS, STOP_VALUE, WARN_VALUE, FAR_VALUE
from utils import draw_polygon, get_xyz
from obstacle_vision import combine_stats

# ==============================
# 3. 鼠标调试变量
# ==============================

mouse_x = -1                           # 当前鼠标在图像中的 x 坐标；-1 表示还没进入窗口
mouse_y = -1                           # 当前鼠标在图像中的 y 坐标；-1 表示还没进入窗口


def mouse_move_callback(event, x, y, flags, param):
    """
    鼠标移动回调函数。

    作用：
        当你把鼠标放到 RGB 图像窗口上时，实时记录鼠标所在像素坐标。
        后面你标定 ROI 点的时候，就靠它读取 x/y 坐标。

    参数：
        event：OpenCV 传进来的鼠标事件类型。
        x：鼠标当前所在像素的 x 坐标。
        y：鼠标当前所在像素的 y 坐标。
        flags：鼠标按键状态，本程序暂时不用。
        param：额外参数，本程序暂时不用。
    """
    global mouse_x, mouse_y             # 声明要修改全局变量 mouse_x 和 mouse_y

    if event == cv.EVENT_MOUSEMOVE:     # 只有鼠标移动时才更新坐标
        mouse_x = x                     # 保存鼠标 x 坐标
        mouse_y = y                     # 保存鼠标 y 坐标

def draw_obstacle_debug(
    color_frame,
    zone_stats,
    draw_text=True,
    show_roi_polygons=None,
    show_obstacle_fill=None,
    show_zone_debug_text=None,
):
    """
    [V4.5-改动] 在 RGB 图上画出三道路红/黄/绿分段 ROI，并把障碍像素涂红。

    颜色含义：
        红色框：最近可见处~45cm，危险停车区。
        黄色框：45~55cm，预警区。
        绿色框：55~95cm，提前观察区。

    注意：
        这里的红色“填充”不是区域背景，而是程序判定出来的障碍像素。
    """
    # [V4.8-新增] 每类显示都可以单独开关：ROI 边框、障碍红色填充、调试文字。
    roi_enabled = SHOW_ROI_POLYGONS if show_roi_polygons is None else bool(show_roi_polygons)
    fill_enabled = SHOW_OBSTACLE_FILL if show_obstacle_fill is None else bool(show_obstacle_fill)
    text_enabled = SHOW_ZONE_DEBUG_TEXT if show_zone_debug_text is None else bool(show_zone_debug_text)

    if roi_enabled:
        for cfg in ZONE_CONFIGS:
            draw_polygon(color_frame, cfg["points"], cfg["color"], cfg["name"], draw_label=draw_text)

    if fill_enabled:
        combined_obstacle_mask = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype=np.uint8)
        for stats in zone_stats.values():
            raw = stats.get("obstacle_mask")
            if raw is None:
                continue
            mask = np.asarray(raw, dtype=np.uint8)
            if mask.shape == combined_obstacle_mask.shape:
                combined_obstacle_mask |= mask
        color_frame[combined_obstacle_mask > 0] = (0, 0, 255)

    # [V4.8.1-修正] 详细调试文字也属于文字层，只在 draw_text=True 的帧绘制。
    # 这样视频/ROI/障碍红色填充仍然每帧刷新，只有 putText 降频。
    if (not text_enabled) or (not draw_text):
        return

    # 调试文字：重点显示中间三段和左右整体堵塞程度。
    left_total = combine_stats([zone_stats["L_RED"], zone_stats["L_YELLOW"], zone_stats["L_GREEN"]], "LEFT_TOTAL")
    center_total = combine_stats([zone_stats["C_RED"], zone_stats["C_YELLOW"], zone_stats["C_GREEN"]], "CENTER_TOTAL")
    right_total = combine_stats([zone_stats["R_RED"], zone_stats["R_YELLOW"], zone_stats["R_GREEN"]], "RIGHT_TOTAL")

    debug_lines = [
        f"C_RED c:{zone_stats['C_RED']['obstacle_count']} raw:{int(zone_stats['C_RED'].get('raw_is_obstacle', zone_stats['C_RED']['is_obstacle']))} stable:{int(zone_stats['C_RED']['is_obstacle'])} min:{zone_stats['C_RED']['min_depth']} STOP<{STOP_VALUE}",
        f"C_YEL c:{zone_stats['C_YELLOW']['obstacle_count']} raw:{int(zone_stats['C_YELLOW'].get('raw_is_obstacle', zone_stats['C_YELLOW']['is_obstacle']))} stable:{int(zone_stats['C_YELLOW']['is_obstacle'])} min:{zone_stats['C_YELLOW']['min_depth']} WARN<{WARN_VALUE}",
        f"C_GRN c:{zone_stats['C_GREEN']['obstacle_count']} raw:{int(zone_stats['C_GREEN'].get('raw_is_obstacle', zone_stats['C_GREEN']['is_obstacle']))} stable:{int(zone_stats['C_GREEN']['is_obstacle'])} min:{zone_stats['C_GREEN']['min_depth']} FAR<{FAR_VALUE}",
        f"L_TOTAL c:{left_total['obstacle_count']}  C_TOTAL c:{center_total['obstacle_count']}  R_TOTAL c:{right_total['obstacle_count']}",
        f"FILTER depth:ON hold:{DEPTH_HOLD_MAX_FRAMES} zone:ON on/off:{ZONE_ON_CONFIRM_FRAMES}/{ZONE_OFF_CONFIRM_FRAMES}",
    ]

    for i, line in enumerate(debug_lines):
        cv.putText(color_frame, line, (10, 155 + i * 22), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)


def put_status_text(image, mode, obs_flag, nearest_dist, avoid_dir, baseline_ready, cmd):
    """
    在图像左上角显示当前系统状态。

    参数：
        image：要显示文字的 BGR 图像。
        mode：当前运动模式。
        obs_flag：是否有障碍。
        nearest_dist：最近障碍距离。
        avoid_dir：绕障方向。
        baseline_ready：是否已经采集地面基准。
        cmd：当前发送给 STM32 的命令。
    """
    mode_name = {
        MODE_TRACE: "TRACE",
        MODE_STOP: "STOP",
        MODE_AVOID_LEFT: "AVOID_LEFT",
        MODE_AVOID_RIGHT: "AVOID_RIGHT",
        MODE_LINE_LOST: "LINE_LOST",
        MODE_SPIN_LEFT: "SPIN_LEFT",
        MODE_SPIN_RIGHT: "SPIN_RIGHT",
    }.get(mode, "UNKNOWN")

    cv.putText(image, f"MODE: {mode_name}", (10, 25), cv.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
    cv.putText(image, f"OBS:{obs_flag} DIST:{nearest_dist}mm DIR:{avoid_dir}", (10, 50), cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv.putText(image, f"BASELINE:{'YES' if baseline_ready else 'NO'}", (10, 75), cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv.putText(image, f"Mouse: ({mouse_x},{mouse_y})", (10, 100), cv.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

    short_cmd = cmd.strip()
    if len(short_cmd) > 85:
        short_cmd = short_cmd[:85] + "..."

    cv.putText(image, short_cmd, (10, FRAME_HEIGHT - 15), cv.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)


def put_fps_text(image, fps_value):
    """
    [V4.7-新增] 在 RGB 调试窗口右上角显示帧率。

    参数：
        image：要写字的 BGR 图像。
        fps_value：当前平滑后的 FPS 数值，单位是 帧/秒。

    为什么要单独写成函数：
        1. 主循环里只负责算 FPS。
        2. 这个函数专门负责“怎么把 FPS 漂亮地画到右上角”。
        3. 以后你想改颜色、字号、位置，只改这里即可。
    """
    if not SHOW_FPS:                                                       # 如果关闭 FPS 显示，就直接退出，不画文字
        return                                                              # return 表示这个函数到此结束

    fps_text = f"FPS:{fps_value:.1f}"                                      # 把 FPS 数值格式化成一位小数，例如 FPS:24.6
    font = cv.FONT_HERSHEY_SIMPLEX                                          # OpenCV 内置字体，和前面状态文字保持一致
    font_scale = 0.8                                                        # 字号大小；1280x720 下 0.8 比较清楚
    thickness = 2                                                           # 字体线条粗细；2 比较醒目

    text_size, baseline = cv.getTextSize(fps_text, font, font_scale, thickness) # 计算这串文字实际占多少像素宽高
    text_width, text_height = text_size                                     # 拆出文字宽度和高度

    x = image.shape[1] - text_width - FPS_TEXT_MARGIN                       # 右上角 x：图像宽度 - 文字宽度 - 右边距
    y = FPS_TEXT_MARGIN + text_height                                       # 右上角 y：上边距 + 文字高度，避免文字顶到窗口边缘

    # 先画一层黑色粗字作为描边，防止白色文字在亮背景上看不清。
    cv.putText(image, fps_text, (x, y), font, font_scale, (0, 0, 0), thickness + 2)

    # 再画白色正文，这样右上角 FPS 在亮地面/暗背景上都比较清楚。
    cv.putText(image, fps_text, (x, y), font, font_scale, (255, 255, 255), thickness)


def draw_mouse_depth_info(color_frame, depth_frame, cam_param, filtered_depth_frame=None):
    """
    在图像上显示鼠标所在点的 depth 和 XYZ。

    参数：
        color_frame：BGR 彩色图。
        depth_frame：深度图。
        cam_param：相机内参。
    """
    if mouse_x < 0 or mouse_y < 0:
        return

    if mouse_x >= depth_frame.shape[1] or mouse_y >= depth_frame.shape[0]:
        return

    cv.circle(color_frame, (mouse_x, mouse_y), 5, (0, 0, 255), -1)

    raw_depth_val = int(depth_frame[mouse_y, mouse_x])

    filtered_depth_val = None
    if filtered_depth_frame is not None:
        filtered_depth_val = int(filtered_depth_frame[mouse_y, mouse_x])

    # XYZ 计算优先使用滤波后的有效深度；如果滤波后仍无效，再尝试 raw depth。
    depth_val_for_xyz = filtered_depth_val if filtered_depth_val and filtered_depth_val > 0 else raw_depth_val

    if depth_val_for_xyz <= 0:
        cv.putText(color_frame, f"RawDepth:{raw_depth_val}  FiltDepth:invalid", (10, 130), cv.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
        return

    if filtered_depth_val is None:
        depth_text = f"Depth:{raw_depth_val}mm"
    else:
        depth_text = f"Raw:{raw_depth_val}mm Filt:{filtered_depth_val}mm"

    xyz = get_xyz(mouse_x, mouse_y, depth_val_for_xyz, cam_param)

    # ROS2 桥接启动早期可能还没收到 /camera/color/camera_info。
    # 这时先显示 depth，不强行显示 XYZ，避免空内参导致调试窗口没文字。
    if xyz is None:
        cv.putText(
            color_frame,
            f"{depth_text}  XYZ:waiting camera_info",
            (10, 130),
            cv.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2
        )
        return

    x_m, y_m, z_m = xyz

    cv.putText(
        color_frame,
        f"{depth_text}  XYZ:{x_m:.3f},{y_m:.3f},{z_m:.3f}m",
        (10, 130),
        cv.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2
    )
