"""
obstacle_vision.py

作用：
    避障 ROI mask/bbox 预计算、深度时间滤波、区域防抖、障碍统计和区域合并。
"""

import cv2 as cv
import numpy as np

from config_switches import *
from calibration_640 import ZONE_CONFIGS
from utils import make_polygon_mask

def build_zone_roi_metas(image_shape):
    """
    [V5.0-新增] 为每个避障 ROI 预计算 mask、bbox 和 bbox 内的小 mask。              返回每个区域的mask(zone_masks)、整个图像的区域布尔数组((union_mask > 0))、

    为什么要这么做？
        你的 obstacle 模块耗时 52ms，主要不是算法复杂，而是 9 个 ROI
        每次都在 1280x720 全图上做布尔运算、astype、形态学。
        现在每个 ROI 只处理自己的外接矩形 bbox，比如几百乘几十像素，
        这样 CPU 不再反复扫整张大图。

    返回：
        zone_masks：整图 mask，主要用于调试显示。
        obstacle_logic_mask：所有 ROI 并集，给 depth 时间滤波使用。
        zone_metas：每个 ROI 的加速元信息。
    """
    height, width = image_shape[:2]
    zone_masks = {}
    union_mask = np.zeros((height, width), dtype=np.uint8)
    zone_metas = {}

    for cfg in ZONE_CONFIGS:
        key = cfg["key"]
        mask = make_polygon_mask(image_shape, cfg["points"])
        zone_masks[key] = mask
        union_mask = cv.bitwise_or(union_mask, mask)

        ys, xs = np.where(mask > 0)
        if len(xs) == 0 or len(ys) == 0:
            zone_metas[key] = {
                "enabled": False,
                "x1": 0, "x2": 0, "y1": 0, "y2": 0,
                "roi_mask_crop": np.zeros((1, 1), dtype=bool),
                "roi_area": 0,
                "full_mask": mask,
            }
            continue

        x1 = int(xs.min())
        x2 = int(xs.max()) + 1
        y1 = int(ys.min())
        y2 = int(ys.max()) + 1                          # 外接矩形坐标。
        roi_mask_crop = (mask[y1:y2, x1:x2] > 0)        # 九个区域的分别 外接矩形 mask。
        roi_area = int(np.count_nonzero(roi_mask_crop)) # 计算 ROI 内的像素数量/面积。

        zone_metas[key] = {
            "enabled": roi_area > 0,
            "x1": x1, "x2": x2, "y1": y1, "y2": y2,
            "roi_mask_crop": roi_mask_crop,
            "roi_area": roi_area,
            "full_mask": mask,
        }

        print(
            f"⚡ [V5.0] ROI {key} bbox: "
            f"x={x1}:{x2}, y={y1}:{y2}, "
            f"size={x2-x1}x{y2-y1}, area={roi_area}"
        )

    return zone_masks, (union_mask > 0), zone_metas

class DepthVoidTemporalFilter:
    """
    [V4.6-新增 / V4.9-优化] 深度图时间滤波器。

    它解决什么问题？
        深度相机在反光地面、黑色物体边缘、物体边缘处，经常会出现：
            这一帧 depth = 260mm
            下一帧 depth = 0，也就是 void / 无效
            再下一帧 depth = 255mm
        如果直接拿原始 depth 做避障，障碍物就会一会儿存在、一会儿消失。

    V4.8 的优化：
        只在避障 ROI 并集内滤波，不在整张 1280x720 里做无意义计算。

    V4.9 的进一步优化：
        不再把整张 depth 转成 float。
        先根据避障 ROI 并集求一个外接矩形 bbox，只处理这个小矩形里的 depth。
        这样可以判断瓶颈到底是不是深度滤波。
    """
    def __init__(self, image_shape, alpha=0.45, hold_max_frames=4, logic_mask=None):
        self.alpha = float(alpha)                                      # EMA 中当前帧的权重
        self.hold_max_frames = int(hold_max_frames)                    # 无效后最多保持几帧旧值
        self.last_depth = np.zeros(image_shape, dtype=np.float32)       # 记住上一次可靠深度
        self.age = np.full(image_shape, hold_max_frames + 1, dtype=np.uint8) # age 越小，说明记忆越新

        if logic_mask is None:
            self.logic_mask = np.ones(image_shape, dtype=bool)          # 没传 mask 时，退回整图滤波
        else:
            self.logic_mask = logic_mask.astype(bool)                  # 只在避障 ROI 并集内滤波

        # [V4.9-新增] 求避障 ROI 并集的外接矩形 bbox。
        # 后续 update() 只处理这个 bbox 裁剪出来的小图，而不是整张 1280x720。
        ys, xs = np.where(self.logic_mask)
        if len(xs) == 0 or len(ys) == 0:
            self.x1, self.x2 = 0, image_shape[1]
            self.y1, self.y2 = 0, image_shape[0]
        else:
            self.x1 = int(xs.min())
            self.x2 = int(xs.max()) + 1
            self.y1 = int(ys.min())
            self.y2 = int(ys.max()) + 1

        self.logic_mask_roi = self.logic_mask[self.y1:self.y2, self.x1:self.x2]
        print(
            f"⚡ [V4.9] 深度滤波 bbox: "
            f"x={self.x1}:{self.x2}, y={self.y1}:{self.y2}, "
            f"size={self.x2-self.x1}x{self.y2-self.y1}"
        )

    def reset(self):
        """清空滤波器记忆。换相机角度、重新开始测试时可以调用。"""
        self.last_depth.fill(0)
        self.age.fill(self.hold_max_frames + 1)

    def update(self, raw_depth):
        """
        输入原始 depth，输出滤波后的 depth。

        参数：
            raw_depth：uint16 深度图，单位 mm，0 通常表示无效。

        返回：
            filtered_depth：uint16 深度图，单位 mm。
        """
        # [V4.9-优化] 先复制原始深度，ROI 外保持原样。
        # 这样不需要整张图 astype(float32)，只对 bbox 小区域做 float 运算。
        filtered = raw_depth.copy()

        raw_roi_float = raw_depth[self.y1:self.y2, self.x1:self.x2].astype(np.float32)
        last_roi = self.last_depth[self.y1:self.y2, self.x1:self.x2]
        age_roi = self.age[self.y1:self.y2, self.x1:self.x2]

        current_valid = (
            self.logic_mask_roi
            & (raw_roi_float >= MIN_VALID_DEPTH_MM)
            & (raw_roi_float <= MAX_VALID_DEPTH_MM)
        )

        memory_valid = (
            self.logic_mask_roi
            & (last_roi >= MIN_VALID_DEPTH_MM)
            & (last_roi <= MAX_VALID_DEPTH_MM)
            & (age_roi <= self.hold_max_frames)
        )

        filtered_roi_float = raw_roi_float.copy()

        smooth_mask = current_valid & memory_valid
        filtered_roi_float[smooth_mask] = (
            self.alpha * raw_roi_float[smooth_mask]
            + (1.0 - self.alpha) * last_roi[smooth_mask]
        )

        fill_mask = (~current_valid) & memory_valid
        filtered_roi_float[fill_mask] = last_roi[fill_mask]

        invalid_no_memory = self.logic_mask_roi & (~current_valid) & (~memory_valid)
        filtered_roi_float[invalid_no_memory] = 0

        update_memory_mask = current_valid
        last_roi[update_memory_mask] = filtered_roi_float[update_memory_mask]
        age_roi[update_memory_mask] = 0

        hold_memory_mask = (~current_valid) & memory_valid
        age_roi[hold_memory_mask] = np.minimum(
            age_roi[hold_memory_mask] + 1,
            self.hold_max_frames + 1
        )

        forget_mask = self.logic_mask_roi & (~current_valid) & (~memory_valid)
        last_roi[forget_mask] = 0
        age_roi[forget_mask] = self.hold_max_frames + 1

        # 把 bbox 内滤波后的结果写回整张图对应区域。
        filtered[self.y1:self.y2, self.x1:self.x2] = filtered_roi_float.astype(np.uint16)

        return filtered

class ZoneStableFilter:
    """
    [V4.6-新增] ROI 障碍判断防抖器。

    它解决什么问题？
        就算 depth 做了滤波，ROI 的 obstacle_count 仍然可能在阈值附近抖动：
            第 1 帧：C_RED 有障碍
            第 2 帧：C_RED 没障碍
            第 3 帧：C_RED 又有障碍
        如果直接用这个 raw 判断，mode 会在 TRACE / STOP / AVOID 之间横跳。

    它怎么做？
        1. 连续 ZONE_ON_CONFIRM_FRAMES 帧检测到障碍，才把该区稳定置为 True。
        2. 连续 ZONE_OFF_CONFIRM_FRAMES 帧检测不到障碍，才把该区稳定置为 False。

    你可以把它理解成：
        给“障碍存在/消失”加了一个确认过程。
    """
    def __init__(self, zone_keys, on_frames=2, off_frames=4):
        self.zone_keys = list(zone_keys)                                # 所有 ROI 的 key，比如 C_RED / L_YELLOW
        self.on_frames = int(on_frames)                                 # 开启确认帧数
        self.off_frames = int(off_frames)                               # 关闭确认帧数
        self.state = {key: False for key in self.zone_keys}             # 当前稳定状态
        self.on_count = {key: 0 for key in self.zone_keys}              # 连续检测到障碍的帧数
        self.off_count = {key: 0 for key in self.zone_keys}             # 连续检测不到障碍的帧数

    def reset(self):
        """清空所有 ROI 的稳定状态。"""
        for key in self.zone_keys:
            self.state[key] = False
            self.on_count[key] = 0
            self.off_count[key] = 0

    def update(self, zone_stats):
        """
        输入当前帧每个 ROI 的原始 stats，输出带稳定 is_obstacle 的 stats。

        注意：
            stats["raw_is_obstacle"] 会保留原始判断。
            stats["is_obstacle"] 会被替换成防抖后的稳定判断。
        """
        for key, stats in zone_stats.items():
            raw_detected = bool(stats.get("is_obstacle", False))        # 当前帧原始判断
            stats["raw_is_obstacle"] = raw_detected                    # 保存原始判断，方便调试

            if raw_detected:
                self.on_count[key] += 1
                self.off_count[key] = 0
                if self.on_count[key] >= self.on_frames:
                    self.state[key] = True
            else:
                self.off_count[key] += 1
                self.on_count[key] = 0
                if self.off_count[key] >= self.off_frames:
                    self.state[key] = False

            stats["stable_on_count"] = self.on_count[key]              # 调试：连续触发帧数
            stats["stable_off_count"] = self.off_count[key]            # 调试：连续消失帧数
            stats["is_obstacle"] = self.state[key]                     # 用稳定判断替代原始判断

        return zone_stats

def calculate_obstacle_stats_fast(depth_int, baseline_int, cfg, roi_meta):
    """
    [V5.0-新增] 高速版 ROI 障碍统计函数。

    和旧版 calculate_obstacle_stats 的区别：
        1. depth_int / baseline_int 在主循环里一帧只转换一次，这里不再 astype。
        2. 这里只裁剪 ROI 的 bbox 小图计算，不再每个 ROI 扫 1280x720 全图。
        3. 形态学开运算只对 bbox 小图做。
        4. SHOW_OBSTACLE_FILL=False 时，不生成整图 obstacle_mask。

    参数：
        depth_int：当前滤波后的深度图 int32，单位 mm。
        baseline_int：baseline 的 int32；None 表示没有 baseline。
        cfg：ZONE_CONFIGS 里的一个区域配置。
        roi_meta：build_zone_roi_metas 预计算出来的该 ROI 元信息。
    """
    roi_name = cfg["name"]
    distance_threshold_mm = cfg["threshold"]

    stats = {
        "name": roi_name,
        "enabled": False,
        "is_obstacle": False,
        "valid_count": 0,
        "baseline_valid_count": 0,
        "roi_area": 0,
        "obstacle_count": 0,
        "obstacle_ratio": 0.0,
        "obstacle_area_ratio": 0.0,
        "baseline_count": 0,
        "hole_recovery_count": 0,
        "absolute_count": 0,
        "min_depth": 9999,
        "median_depth": 9999,
        "mask": roi_meta.get("full_mask", None),
        "obstacle_mask": None,
        "baseline_mask": None,
        "hole_recovery_mask": None,
    }

    if (roi_meta is None) or (not roi_meta.get("enabled", False)):
        return stats

    x1, x2 = roi_meta["x1"], roi_meta["x2"]
    y1, y2 = roi_meta["y1"], roi_meta["y2"]
    roi_pixels = roi_meta["roi_mask_crop"]
    roi_area = int(roi_meta["roi_area"])

    stats["enabled"] = True
    stats["roi_area"] = roi_area

    depth_roi = depth_int[y1:y2, x1:x2]

    valid_current = (
        roi_pixels
        & (depth_roi >= MIN_VALID_DEPTH_MM)
        & (depth_roi <= MAX_VALID_DEPTH_MM)
    )

    valid_count = int(np.count_nonzero(valid_current))
    stats["valid_count"] = valid_count

    current_close = valid_current & (depth_roi <= distance_threshold_mm)
    stats["absolute_count"] = int(np.count_nonzero(current_close))

    if baseline_int is not None:
        baseline_roi = baseline_int[y1:y2, x1:x2]

        baseline_valid = (
            roi_pixels
            & (baseline_roi >= MIN_VALID_DEPTH_MM)
            & (baseline_roi <= MAX_VALID_DEPTH_MM)
        )
        stats["baseline_valid_count"] = int(np.count_nonzero(baseline_valid))

        baseline_obstacle = (
            valid_current
            & baseline_valid
            & ((baseline_roi - depth_roi) > BASELINE_NEARER_THAN_MM)
            & (depth_roi <= distance_threshold_mm)
        )

        # baseline 无效补偿现在默认启用，不再用 ENABLE_HOLE_RECOVERY 开关。
        # 含义不是“洞就是障碍”，而是：
        #     baseline 里这个位置没有可靠地面深度；
        #     但当前帧这个位置出现了有效且足够近的深度；
        #     那就把它作为低矮障碍候选，补偿 2D 雷达和 baseline 差分都看不到的近地面障碍。
        baseline_hole = roi_pixels & (~baseline_valid)
        recovery_threshold = max(
            MIN_VALID_DEPTH_MM,
            distance_threshold_mm - BASELINE_NEARER_THAN_MM
        )
        hole_recovery_obstacle = (
            baseline_hole
            & valid_current
            & (depth_roi <= recovery_threshold)
        )
    else:
        baseline_obstacle = current_close
        hole_recovery_obstacle = np.zeros_like(valid_current, dtype=bool)

    raw_obstacle = baseline_obstacle | hole_recovery_obstacle

    if OBSTACLE_MORPH_KERNEL_SIZE > 1:
        # [V5.0-优化] 只在 bbox 小图上做形态学，不再对整张 1280x720 图做。
        obstacle_mask_crop = raw_obstacle.astype(np.uint8) * 255
        kernel = np.ones((OBSTACLE_MORPH_KERNEL_SIZE, OBSTACLE_MORPH_KERNEL_SIZE), np.uint8)
        obstacle_mask_crop = cv.morphologyEx(obstacle_mask_crop, cv.MORPH_OPEN, kernel)
        obstacle_pixels = obstacle_mask_crop > 0
    else:
        obstacle_pixels = raw_obstacle

    obstacle_count = int(np.count_nonzero(obstacle_pixels))
    obstacle_ratio = obstacle_count / max(valid_count, 1)
    obstacle_area_ratio = obstacle_count / max(roi_area, 1)

    stats["obstacle_count"] = obstacle_count
    stats["obstacle_ratio"] = float(obstacle_ratio)
    stats["obstacle_area_ratio"] = float(obstacle_area_ratio)
    stats["baseline_count"] = int(np.count_nonzero(baseline_obstacle))
    stats["hole_recovery_count"] = int(np.count_nonzero(hole_recovery_obstacle))

    if obstacle_count > 0:
        obstacle_depths = depth_roi[obstacle_pixels]
        stats["min_depth"] = int(np.min(obstacle_depths))
        stats["median_depth"] = int(np.median(obstacle_depths))

    stats["is_obstacle"] = (
        obstacle_count >= MIN_OBSTACLE_PIXELS
        and obstacle_ratio >= MIN_OBSTACLE_RATIO
        and obstacle_area_ratio >= MIN_OBSTACLE_AREA_RATIO
    )

    # [V5.0-优化] 只有需要在画面上涂红障碍像素时，才生成整图 obstacle_mask。
    if SHOW_OBSTACLE_FILL:
        full_obstacle_mask = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype=np.uint8)
        full_obstacle_mask[y1:y2, x1:x2][obstacle_pixels] = 255
        stats["obstacle_mask"] = full_obstacle_mask

    return stats

def combine_stats(stats_list, name="COMBINED"):
    """
    把多个 ROI 的统计结果合并成一个“虚拟统计结果”。

    [V5.0-优化]
    旧版这里每次 combine 都会创建一张 1280x720 的 combined_mask，
    但决策逻辑其实只需要 is_obstacle / obstacle_count / min_depth。
    所以在 SHOW_OBSTACLE_FILL=False 时，不再做大图 mask 合并，避免隐藏开销。
    """
    if not stats_list:
        return {
            "name": name,
            "enabled": False,
            "is_obstacle": False,
            "valid_count": 0,
            "obstacle_count": 0,
            "hole_recovery_count": 0,
            "min_depth": 9999,
            "median_depth": 9999,
            "obstacle_mask": None,
        }

    enabled = any(s["enabled"] for s in stats_list)
    is_obstacle = any(s["is_obstacle"] for s in stats_list)
    valid_count = sum(s["valid_count"] for s in stats_list)
    roi_area = sum(s.get("roi_area", 0) for s in stats_list)
    obstacle_count = sum(s["obstacle_count"] for s in stats_list)
    hole_recovery_count = sum(s["hole_recovery_count"] for s in stats_list)
    obstacle_ratio = obstacle_count / max(valid_count, 1)
    obstacle_area_ratio = obstacle_count / max(roi_area, 1)

    depths = [s["min_depth"] for s in stats_list if s["is_obstacle"] and s["min_depth"] != 9999]
    min_depth = min(depths) if depths else 9999

    combined_mask = None
    if SHOW_OBSTACLE_FILL:
        combined_mask = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype=np.uint8)
        for s in stats_list:
            m = s.get("obstacle_mask", None)
            if m is not None:
                combined_mask |= m

    return {
        "name": name,
        "enabled": enabled,
        "is_obstacle": is_obstacle,
        "valid_count": valid_count,
        "roi_area": roi_area,
        "obstacle_count": obstacle_count,
        "obstacle_ratio": float(obstacle_ratio),
        "obstacle_area_ratio": float(obstacle_area_ratio),
        "hole_recovery_count": hole_recovery_count,
        "min_depth": min_depth,
        "median_depth": min_depth,
        "obstacle_mask": combined_mask,
    }

def lane_obstacle_score(stats_list):
    """
    给一条借道区域算一个“堵塞分数”。

    分数越大，说明这边越不适合绕。
    现在先用障碍像素数量作为分数，后面可以加入距离权重。
    """
    return sum(s["obstacle_count"] for s in stats_list)
