

"""
road_v5_5_3_single_line_minmax_640x480_old_calib.py

作用：
    这是 road_v2 / road_v3 的升级版。
    V2 只做“蓝色胶带巡线”，V3 加入“深度相机避障”。
    V4 重点升级“三道路避障”：左可通行区 / 中间碰撞区 / 右可通行区。
    V4 还新增“反光地面深度丢失补偿”：如果空地面 baseline 在某些像素无效，但障碍物出现后有有效近距离深度，也允许判成障碍。
    V4.3 重点升级“1280x720 实测标定版”：
        1. 原始相机分辨率改为 1280x720，扩大视野。
        2. 鸟瞰图 LINE_SRC_POINTS 按你原来的策略：近点取图像左右下角，远点取 y=514，并由边界斜率 k 外推得到。
        3. 三道路 ROI 已填入你当前测量的数据：左可借道区 / 中间碰撞区 / 右可借道区。
        4. V4.4 重点调整 5 条 error 扫描线：error1~error5 现在对应 55/65/75/85/95cm。
    V4.5 重点升级避障显示和阈值：
        1. 停车阈值改成 STOP_VALUE = 450mm。
        2. 前方三条道路都分成红色危险区、黄色预警区、绿色提前观察区。
        3. 障碍物像素仍然用红色填充显示。
    V4.6 重点升级深度抗抖滤波：
        1. 增加 DepthVoidTemporalFilter：让 depth 在 valid / void 之间横跳时，不要立刻变成无效。
        2. 增加 ZoneStableFilter：让障碍判断必须连续确认，避免 mode 在 TRACE/STOP/AVOID 间乱跳。
        3. 鼠标深度显示同时显示 raw_depth 和 filtered_depth，方便确认滤波效果。
    V4.8 重点优化速度：
        1. 原始相机仍然采集 1280x720，保留大视野。
        2. 鸟瞰图输出降为 640x480，相当于把大视野压缩到小控制图，降低 HSV/mask/扫描线计算量。
        3. 【V4.8.1修正】视频画面每帧刷新；只有画面上的文字/标签每 TEXT_EVERY_N_FRAMES 帧绘制一次。
        4. 增加显示开关：RGB窗口、鸟瞰图、MASK、ROI框、障碍填充、文字、鼠标深度都能单独开关。
        5. 深度时间滤波只在避障 ROI 联合区域内工作，避免对整张 1280x720 深度图做无意义计算。
    V4.9 重点升级性能诊断：
        1. 每隔 PROFILE_PRINT_EVERY_N_FRAMES 帧打印 camera/line/obstacle/display 等模块耗时。
        2. 新增 CAMERA_ONLY_TEST，用来测试相机取帧本身的极限 FPS。
        3. 深度时间滤波改成 bbox 裁剪级别，只处理避障 ROI 总包围框。
    V5.5.3 重点改动：
        1. 将蓝色优势过滤改成通用 B/G/R 通道优势过滤。
        2. COLOR_DOMINANCE_CHANNEL=False/0/1/2 分别表示关闭/B/G/R。
        3. 不改变 HSV、避障、串口和状态机逻辑，只改颜色二次过滤。
    V5.5 重点升级避障状态机：
        1. 不再“每一帧独立决定是否绕障/回正”。
        2. 新增 AVOID 状态记忆：中间黄/绿区看到障碍后锁定绕障方向。
        3. 障碍短暂消失不会立刻恢复 TRACE，而是保持 AVOID_HOLD_TIME_SEC。
        4. 绕障偏置不再一帧突变，而是按斜坡逐渐加大/逐渐回正。
        5. 新增 RETURN_TO_LINE 内部状态：障碍消失后慢慢减小偏置，避免车头一转开就立刻直走。
        6. 目前还没有真正接入 IMU yaw，只预留了结构；先用视觉状态机解决“避障后不稳定回线”的问题。

整车架构：
    Gemini2 双目深度相机 + 树莓派：
        负责看路、看障碍、计算 error1~error5、判断 mode。
    STM32F103C8T6：
        负责接收串口数据、执行 PID、通过 CAN 控制步进电机。
    CAN 总线步进电机：
        负责真正让车轮动起来。

V3 相比 V2 的核心改动：
    [V3-改动1] 保留 V2 的 HSV 蓝色胶带识别和 5 条扫描线 error 输出。
    [V4-改动1] 保留 V2/V3 的 HSV 蓝色胶带识别和 5 条扫描线 error 输出。
    [V4-改动2] 把避障区域升级成“三道路”：LEFT_CLEAR / CENTER_HIT / RIGHT_CLEAR。
    [V4-改动3] STOP_ROI 不再默认看整个路面大梯形，而是看“车身会撞到的中间通道”。
    [V4-改动4] 新增反光地面补偿：baseline 无效但当前深度有效且很近时，也算候选障碍。
    [V3-改动5] 新增 mode 状态：
              0 = 正常巡线
              1 = 障碍停车
              2 = 向左绕障
              3 = 向右绕障
              4 = 丢线找线
    [V3-改动6] 串口协议保持兼容旧 STM32：
              开头仍然是 error1~error5，旧版 sscanf 仍然能解析前 5 个误差。
              后面追加 mode / obs / dist / dir，方便你后续升级 STM32 代码。
    [V3-改动7] 所有标定点先留空，你后面按实际相机画面填进去。

重要说明：
    1. 巡线的 src_points 是“鸟瞰图透视变换”的标定点。
    2. 避障的 ROI_POINTS 是“原始 RGB 图像 / 对齐深度图”上的标定点。
    3. 因为你开启了 D2C 硬件对齐，RGB 图上的像素坐标可以直接对应 depth 图坐标。
    4. 如果标定点没填，程序不会崩溃，只是对应功能会暂时关闭。
"""

# ==============================
# 1. 导入库
# ==============================

import cv2 as cv                      # OpenCV：负责图像显示、HSV识别、透视变换、画ROI
import numpy as np                    # NumPy：负责矩阵、数组、像素批量计算


# ==============================
# 模块化导入区
# ==============================

import cv2 as cv
import numpy as np
import time

from config_switches import *
from calibration_640 import *
# 相机后端在 main() 里根据 CAMERA_BACKEND 动态导入。
# CAMERA_BACKEND="ros2" 时不会 import pyorbbecsdk，避免树莓派 ROS2 环境缺旧 SDK 直接报错。
from line_vision import *
from obstacle_vision import *
from navigation import *
from serial_comm import *
from display_debug import *
from web_video_stream import start_web_video_server, stop_web_video_server, publish_web_frame
from profile_tools import *

def main():
    """
    主程序入口。

    程序主循环每一帧做的事情：
        1. 读取 Gemini2 的 RGB + depth。
        2. 用 HSV + 鸟瞰图计算 error1~error5。
        3. 用 depth + ROI 判断是否有障碍。
        4. 决定 mode。
        5. 如果绕障，给 error 加偏置。
        6. 通过 UART 发送给 STM32。
        7. 显示 RGB、鸟瞰图、mask 调试窗口。
    """
    missing_calibration_items = collect_missing_calibration_items()
    calibration_safe_mode = len(missing_calibration_items) > 0
    print_calibration_report(missing_calibration_items)

    # 关键安全逻辑：
    # 只要有任何关键标定点没填，就不打开串口。
    # 这样即使 STM32 已经接着，小车也不会收到任何运动指令。
    ser = None
    if not calibration_safe_mode:
        ser = open_serial_port()
        start_serial_rx_monitor(ser)
    else:
        print("🔒 安全标定模式：已跳过串口打开。")

    create_hsv_window()

    perspective_matrix = build_perspective_matrix()
    bird_view_enabled = (not calibration_safe_mode) and (perspective_matrix is not None)

    if perspective_matrix is None:
        print("⚠️ LINE_SRC_POINTS 未完整填写：本次不显示鸟瞰图。")

    # ==============================
    # 相机输入后端选择
    # ==============================
    # V7 统一入口：camera_input.py
    #     CAMERA_BACKEND="ros2"：订阅 Orbbec ROS2 Wrapper 的 RGB + Depth topic。
    #     CAMERA_BACKEND="sdk" ：回退到旧 pyorbbecsdk Pipeline() 直连。
    # main.py 不再直接 import ros2_camera_bridge / camera_orbbec，避免两套入口越写越乱。
    from camera_input import UnifiedCameraManager as CameraManager

    cam = CameraManager(
        enable_depth=True,
        width=FRAME_WIDTH,
        height=FRAME_HEIGHT,
        fps=FRAME_FPS
    )

    baseline_depth = None                           # 空地面深度基准图；None 表示还没有采集
    baseline_ready = False                          # baseline 是否已经准备好
    baseline_collecting = False                     # 是否正在连续采集 baseline 帧
    baseline_frames = []                            # 用来暂存 baseline 采集过程中的多帧 depth

    # [V4.8-新增] 预计算 9 个避障 ROI 的 mask，并生成一个总的避障区域 mask。
    # 安全标定模式下不创建任何避障逻辑对象，避免点没填完时误触发。
    zone_roi_masks = {}
    obstacle_logic_mask = None
    zone_roi_metas = {}
    depth_filter = None
    zone_stable_filter = None
    avoid_state_machine = None

    if not calibration_safe_mode:
        zone_roi_masks, obstacle_logic_mask, zone_roi_metas = build_zone_roi_metas((FRAME_HEIGHT, FRAME_WIDTH))

        # [V4.6/V5.5-整理] 深度时间滤波、ROI 防抖、避障状态机现在固定启用。
        # 不再保留 ENABLE_DEPTH_TEMPORAL_FILTER / ENABLE_ZONE_STABLE_FILTER / ENABLE_AVOID_STATE_MACHINE 旧开关。
        depth_filter = DepthVoidTemporalFilter(
            image_shape=(FRAME_HEIGHT, FRAME_WIDTH),
            alpha=DEPTH_EMA_ALPHA,
            hold_max_frames=DEPTH_HOLD_MAX_FRAMES,
            logic_mask=obstacle_logic_mask
        )

        zone_stable_filter = ZoneStableFilter(
            zone_keys=[cfg["key"] for cfg in ZONE_CONFIGS],
            on_frames=ZONE_ON_CONFIRM_FRAMES,
            off_frames=ZONE_OFF_CONFIRM_FRAMES
        )

        avoid_state_machine = AvoidanceStateMachine()

    last_send_time = 0.0

    # [V4.7-新增] FPS 计算变量。
    # fps_last_time：上一帧结束时的时间戳，用来计算两帧之间隔了多久。
    # fps_smooth：经过指数滑动平均后的平滑 FPS，避免数字疯狂跳动。
    fps_last_time = time.perf_counter()
    fps_smooth = 0.0
    frame_index = 0                           # 总帧计数
    profile_sum, profile_count = make_profile_accumulator()  # [V4.9-新增] 性能诊断累计器

    window_name = "RGB + Depth + Obstacle ROI"
    if SHOW_RGB_WINDOW:
        cv.namedWindow(window_name)
        cv.setMouseCallback(window_name, mouse_move_callback)

    if ENABLE_WEB_VIDEO_STREAM:
        start_web_video_server(
            host=WEB_VIDEO_HOST,
            port=WEB_VIDEO_PORT,
            fps=WEB_VIDEO_FPS,
            quality=WEB_VIDEO_JPEG_QUALITY,
        )

    if not cam.start():
        stop_web_video_server()
        return

    try:
        while True:
            # [V4.9-新增] 一帧性能计时开始。
            t_frame_start = time.perf_counter()

            color, depth = cam.get_frames()
            t_after_camera = time.perf_counter()

            if color is None or depth is None:
                continue

            # [V4.7-新增] 计算当前处理帧率。
            # time.perf_counter() 是 Python 里适合做计时的高精度时钟。
            now_time = time.perf_counter()
            frame_dt = now_time - fps_last_time                              # 当前帧和上一帧之间的时间差，单位秒
            fps_last_time = now_time                                         # 更新时间戳，给下一帧使用

            if frame_dt > 0:                                                  # 防止极端情况下除以 0
                instant_fps = 1.0 / frame_dt                                  # 瞬时 FPS = 1 / 每帧耗时
                if fps_smooth <= 0.0:                                         # 第一帧还没有历史平均值
                    fps_smooth = instant_fps                                  # 直接用瞬时 FPS 初始化
                else:
                    fps_smooth = (                                           # 指数滑动平均，让 FPS 显示更稳
                        FPS_EMA_ALPHA * instant_fps
                        + (1.0 - FPS_EMA_ALPHA) * fps_smooth
                    )

            frame_index += 1
            # [V4.8.1-修正] 画面每帧显示；只有文字/标签每 TEXT_EVERY_N_FRAMES 帧绘制一次。
            # 之前 v4.8 是 should_display 控制 imshow，导致整个画面 5 帧才刷新一次，这是不符合你的需求的。
            should_draw_text = (frame_index % max(1, TEXT_EVERY_N_FRAMES) == 0)

            # [V4.9-新增] 相机极限测试模式：
            # 只测试 cam.get_frames() + 最小显示，不跑巡线、不跑避障、不发串口。
            # 如果这个模式 FPS 也不高，说明瓶颈主要在相机/SDK/USB/D2C，而不是你的算法。
            if CAMERA_ONLY_TEST:
                t_display_start = time.perf_counter()
                if SHOW_RGB_WINDOW or ENABLE_WEB_VIDEO_STREAM:
                    display_color = color.copy()
                    if should_draw_text:
                        put_fps_text(display_color, fps_smooth)
                    publish_web_frame(display_color)
                    if SHOW_RGB_WINDOW:
                        cv.imshow(window_name, display_color)
                update_serial_rx_window()
                key = cv.waitKey(1) & 0xFF
                t_frame_end = time.perf_counter()

                if PROFILE_MODE:
                    timings = {
                        "camera": (t_after_camera - t_frame_start) * 1000.0,
                        "depth_filter": 0.0,
                        "line": 0.0,
                        "baseline": 0.0,
                        "obstacle": 0.0,
                        "decision": 0.0,
                        "display": (t_frame_end - t_display_start) * 1000.0,
                        "waitkey": 0.0,
                        "total": (t_frame_end - t_frame_start) * 1000.0,
                    }
                    add_profile_sample(profile_sum, timings)
                    profile_count += 1
                    if profile_count >= PROFILE_PRINT_EVERY_N_FRAMES:
                        maybe_print_profile(profile_sum, profile_count)
                        profile_sum, profile_count = make_profile_accumulator()

                if key == 27:
                    break
                continue

            # ============================================================
            # 安全标定模式：
            #     有任意关键标定点缺失时，只显示正常 RGB 画面和已填写 ROI 线。
            #     不显示鸟瞰图，不计算巡线，不判断避障，不采集 baseline，不发送串口。
            # ============================================================
            if calibration_safe_mode:
                t_display_start = time.perf_counter()

                if SHOW_RGB_WINDOW or ENABLE_WEB_VIDEO_STREAM:
                    display_color = color.copy()
                    draw_calibration_preview(
                        display_color,
                        missing_items=missing_calibration_items,
                        draw_text=should_draw_text
                    )

                    if should_draw_text:
                        if SHOW_MOUSE_DEPTH:
                            draw_mouse_depth_info(display_color, depth, cam.cam_param, depth)
                        put_fps_text(display_color, fps_smooth)

                    publish_web_frame(display_color)
                    if SHOW_RGB_WINDOW:
                        cv.imshow(window_name, display_color)

                update_serial_rx_window()
                key = cv.waitKey(1) & 0xFF
                t_frame_end = time.perf_counter()

                if PROFILE_MODE:
                    timings = {
                        "camera": (t_after_camera - t_frame_start) * 1000.0,
                        "depth_filter": 0.0,
                        "line": 0.0,
                        "baseline": 0.0,
                        "obstacle": 0.0,
                        "decision": 0.0,
                        "display": (t_frame_end - t_display_start) * 1000.0,
                        "waitkey": 0.0,
                        "total": (t_frame_end - t_frame_start) * 1000.0,
                    }
                    add_profile_sample(profile_sum, timings)
                    profile_count += 1
                    if profile_count >= PROFILE_PRINT_EVERY_N_FRAMES:
                        maybe_print_profile(profile_sum, profile_count)
                        profile_sum, profile_count = make_profile_accumulator()

                if key == 27:
                    break

                if key == ord("b"):
                    print("🔒 安全标定模式：标定未完整，baseline 采集已禁用。请先补齐终端提示的标定点。")

                if key == ord("r"):
                    print("🔒 安全标定模式：当前没有启用避障/baseline 逻辑，无需重置。")

                continue

            # [V4.6-新增] 先对 depth 做时间滤波。
            # raw depth 仍然保留给鼠标显示对比；避障和 baseline 使用 filtered depth，减少 void 横跳。
            if depth_filter is not None:
                depth_for_logic = depth_filter.update(depth)
            else:
                depth_for_logic = depth
            t_after_depth_filter = time.perf_counter()

            # ENABLE_LINE_FOLLOW=True：正常做蓝线寻线/巡线。
            # ENABLE_LINE_FOLLOW=False：关闭寻线，只保留深度近地面避障；error 输出为 0，避免误触发 LINE_LOST。
            if ENABLE_LINE_FOLLOW:
                bird_view, mask, raw_errors = detect_blue_line_errors(color, perspective_matrix)
            else:
                bird_view = None
                mask = np.zeros((BIRD_HEIGHT, BIRD_WIDTH), dtype=np.uint8)
                raw_errors = [0] * len(SCAN_CONFIGS)
            t_after_line = time.perf_counter()

            if baseline_collecting:                                              # 如果用户刚按下 b，开始采集空地面 baseline
                baseline_frames.append(depth_for_logic.copy())                       # 保存滤波后的深度帧副本，减少 baseline 里的临时 void
                print(f"📷 baseline 采集中: {len(baseline_frames)}/{BASELINE_CAPTURE_FRAME_COUNT}") # 打印采集进度
                if len(baseline_frames) >= BASELINE_CAPTURE_FRAME_COUNT:            # 如果已经采够指定帧数
                    baseline_stack = np.stack(baseline_frames, axis=0).astype(np.float32) # 把多帧 depth 堆成三维数组
                    baseline_depth = np.median(baseline_stack, axis=0).astype(np.uint16)  # 对每个像素取中位数，抗噪更强
                    baseline_frames.clear()                                         # 清空临时列表，释放内存
                    baseline_collecting = False                                     # 结束采集状态
                    baseline_ready = True                                           # 标记 baseline 可用
                    print("✅ baseline_depth 采集完成：已使用多帧中位数，反光地面会自动启用 hole recovery 补偿。")
            t_after_baseline = time.perf_counter()

            # [V5.0-优化] 统一统计 9 个区域：左/中/右 × 红/黄/绿。
            # 旧版 calculate_obstacle_stats 在 9 个 ROI 里反复做 astype 和全图布尔运算，
            # 这是你 profile 里 obstacle:52ms 的核心原因之一。
            # 现在一帧只把 depth/baseline 转成 int32 一次，然后每个 ROI 只处理自己的 bbox 小图。
            depth_int_global = depth_for_logic.astype(np.int32)

            baseline_int_global = None
            if baseline_depth is not None:
                baseline_int_global = baseline_depth.astype(np.int32)

            zone_stats = {}
            for cfg in ZONE_CONFIGS:
                zone_stats[cfg["key"]] = calculate_obstacle_stats_fast(
                    depth_int_global,
                    baseline_int_global,
                    cfg,
                    zone_roi_metas.get(cfg["key"])
                )

            # [V4.6-新增] 对每个 ROI 的障碍判断再做一次“连续帧确认”。
            if zone_stable_filter is not None:
                zone_stats = zone_stable_filter.update(zone_stats)
            t_after_obstacle = time.perf_counter()

            # [V5.5-新增] 第一步：先得到“当前这一帧”的瞬时视觉判断。
            instant_mode, instant_avoid_dir, obs_flag, nearest_dist = decide_mode_and_direction(
                raw_errors,
                zone_stats
            )

            # 第二步：把瞬时判断交给新的避障状态机。
            # 状态机会记住：是否正在绕障、是否处于紧急停车等待、是否正在原地转向找空路。
            mode, avoid_dir, current_avoid_bias, nav_state_name = avoid_state_machine.update(
                instant_mode,
                instant_avoid_dir,
                raw_errors,
                time.perf_counter()
            )
            final_errors = apply_navigation_bias(raw_errors, current_avoid_bias, mode)

            cmd = build_serial_command(
                final_errors,
                mode,
                obs_flag,
                nearest_dist,
                avoid_dir
            )

            last_send_time = send_command_if_needed(ser, cmd, last_send_time)
            t_after_decision = time.perf_counter()

            # [V4.8.1-修正] 显示逻辑：
            # 1. RGB / Bird / MASK 画面每帧 imshow，保证视频是连续的。
            # 2. ROI 边框和障碍红色填充每帧绘制，保证避障可视化跟手。
            # 3. 只有文字类内容（ROI名字、MODE、C_RED调试信息、鼠标深度、FPS、串口命令）每 TEXT_EVERY_N_FRAMES 帧绘制一次。
            if SHOW_RGB_WINDOW or ENABLE_WEB_VIDEO_STREAM:
                display_color = color.copy()
                draw_obstacle_debug(display_color, zone_stats, draw_text=should_draw_text)

                if should_draw_text:
                    if SHOW_MOUSE_DEPTH:
                        draw_mouse_depth_info(display_color, depth, cam.cam_param, depth_for_logic)
                    if SHOW_STATUS_TEXT:
                        put_status_text(display_color, mode, obs_flag, nearest_dist, avoid_dir, baseline_ready, cmd)
                    put_fps_text(display_color, fps_smooth)

                publish_web_frame(display_color)
                if SHOW_RGB_WINDOW:
                    cv.imshow(window_name, display_color)

            if SHOW_BIRD_WINDOW and bird_view_enabled and ENABLE_LINE_FOLLOW and bird_view is not None:
                # 鸟瞰图画面每帧显示，巡线延迟不会因为调试文字降频而变大。
                cv.imshow("Bird's Eye View", bird_view)

            if SHOW_MASK_WINDOW and bird_view_enabled and ENABLE_LINE_FOLLOW:
                cv.imshow("MASK", mask)
            update_serial_rx_window()
            t_after_display = time.perf_counter()

            key = cv.waitKey(1) & 0xFF
            t_after_waitkey = time.perf_counter()

            # [V4.9-新增] 累计并定期打印各模块平均耗时。
            if PROFILE_MODE:
                timings = {
                    "camera": (t_after_camera - t_frame_start) * 1000.0,
                    "depth_filter": (t_after_depth_filter - t_after_camera) * 1000.0,
                    "line": (t_after_line - t_after_depth_filter) * 1000.0,
                    "baseline": (t_after_baseline - t_after_line) * 1000.0,
                    "obstacle": (t_after_obstacle - t_after_baseline) * 1000.0,
                    "decision": (t_after_decision - t_after_obstacle) * 1000.0,
                    "display": (t_after_display - t_after_decision) * 1000.0,
                    "waitkey": (t_after_waitkey - t_after_display) * 1000.0,
                    "total": (t_after_waitkey - t_frame_start) * 1000.0,
                }
                add_profile_sample(profile_sum, timings)
                profile_count += 1

                if profile_count >= PROFILE_PRINT_EVERY_N_FRAMES:
                    maybe_print_profile(profile_sum, profile_count)
                    profile_sum, profile_count = make_profile_accumulator()

            if key == 27:
                break

            if key == ord("b"):
                baseline_frames.clear()                                             # 清空旧的 baseline 临时帧
                baseline_collecting = True                                          # 开始连续采集 baseline
                baseline_ready = False                                              # 采集完成前先认为 baseline 未就绪
                zone_stable_filter.reset()
                avoid_state_machine.reset()
                print(f"📷 开始采集空地面 baseline：请确保前方 ROI 内没有瓶子/手/障碍物，将连续采 {BASELINE_CAPTURE_FRAME_COUNT} 帧。")

            if key == ord("r"):
                baseline_depth = None                                               # 清除空地面基准图
                baseline_ready = False                                              # 标记 baseline 不可用
                baseline_collecting = False                                         # 如果正在采集，也立刻停止
                baseline_frames.clear()                                             # 清空临时帧
                if depth_filter is not None:
                    depth_filter.reset()
                zone_stable_filter.reset()
                avoid_state_machine.reset()
                print("♻️ 已清除 baseline_depth，并已重置 V4.6 深度滤波/区域防抖/V5.5避障状态机记忆。")

    finally:
        stop_web_video_server()
        stop_serial_rx_monitor()
        cam.stop()

        if ser is not None:
            ser.close()
            print("🔌 串口已关闭")


if __name__ == "__main__":
    main()
