#!/usr/bin/env python3
"""
ROS2 depth obstacle publisher.

This node reuses the existing Gemini2 near-ground obstacle logic and publishes a
small summary for the LiDAR/Nav2 safety fusion node.

/depth_obstacle std_msgs/Int32MultiArray data:
  [level, preferred_dir, nearest_mm, center_danger, center_far,
   left_blocked, right_blocked, seq,
   center_area_x1000, total_area_x1000,
   left_score_x1000, right_score_x1000,
   center_offset_x1000, center_min_mm]

level:
  0 = clear
  1 = far center obstacle, slow/avoid
  2 = center red/yellow danger, stop/spin

preferred_dir:
  -1 = turn/avoid left, 1 = turn/avoid right, 0 = no preference
"""

from __future__ import annotations

import time
import json

import cv2 as cv
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String
from std_msgs.msg import Bool

from camera_input import UnifiedCameraManager
from config_switches import (
    FRAME_WIDTH,
    FRAME_HEIGHT,
    FRAME_FPS,
    TEXT_EVERY_N_FRAMES,
    ENABLE_WEB_VIDEO_STREAM,
    WEB_VIDEO_HOST,
    WEB_VIDEO_PORT,
    WEB_VIDEO_FPS,
    WEB_VIDEO_JPEG_QUALITY,
    SHOW_RGB_WINDOW,
    DEPTH_EMA_ALPHA,
    DEPTH_HOLD_MAX_FRAMES,
    ZONE_ON_CONFIRM_FRAMES,
    ZONE_OFF_CONFIRM_FRAMES,
    BASELINE_CAPTURE_FRAME_COUNT,
)
from calibration_640 import FAR_VALUE, ZONE_CONFIGS, collect_missing_calibration_items
from obstacle_vision import (
    DepthVoidTemporalFilter,
    ZoneStableFilter,
    build_zone_roi_metas,
    calculate_obstacle_stats_fast,
    combine_stats,
    lane_obstacle_score,
)
import display_debug
from display_debug import draw_obstacle_debug
from web_video_stream import start_web_video_server, stop_web_video_server, publish_web_frame


def _choose_better_side(zone_stats) -> int:
    left_path = combine_stats(
        [zone_stats["L_RED"], zone_stats["L_YELLOW"], zone_stats["L_GREEN"]],
        name="LEFT_PATH",
    )
    right_path = combine_stats(
        [zone_stats["R_RED"], zone_stats["R_YELLOW"], zone_stats["R_GREEN"]],
        name="RIGHT_PATH",
    )

    if (not left_path["is_obstacle"]) and right_path["is_obstacle"]:
        return -1
    if left_path["is_obstacle"] and (not right_path["is_obstacle"]):
        return 1

    left_score = lane_obstacle_score(
        [zone_stats["L_RED"], zone_stats["L_YELLOW"], zone_stats["L_GREEN"]]
    )
    right_score = lane_obstacle_score(
        [zone_stats["R_RED"], zone_stats["R_YELLOW"], zone_stats["R_GREEN"]]
    )
    if left_score < right_score:
        return -1
    if right_score < left_score:
        return 1
    return -1


def _area_x1000(stats) -> int:
    return int(round(1000.0 * float(stats.get("obstacle_area_ratio", 0.0))))


def _score_x1000(stats, far_mm: int = 950) -> int:
    area = float(stats.get("obstacle_area_ratio", 0.0))
    ratio = float(stats.get("obstacle_ratio", 0.0))
    distance_boost = 0.0
    min_depth = int(stats.get("min_depth", 9999))
    if 0 < min_depth < 9999:
        distance_boost = max(0.0, min(1.0, (float(far_mm) - min_depth) / max(1.0, float(far_mm) - 250.0)))
    return int(round(1000.0 * (0.70 * area + 0.25 * ratio + 0.05 * distance_boost)))


def _center_offset_x1000(left_score: int, right_score: int) -> int:
    return int(max(-1000, min(1000, right_score - left_score)))


class DepthObstacleNode(Node):
    def __init__(self):
        super().__init__("depth_obstacle_node")

        self.declare_parameter("publish_rate", 15.0)
        self.declare_parameter("topic", "/depth_obstacle")

        self.pub = self.create_publisher(
            Int32MultiArray, self.get_parameter("topic").value, 10
        )
        self.web_control_sub = self.create_subscription(
            String, "/robot/web_control", self._on_web_control, 10
        )
        self.baseline_ready_pub = self.create_publisher(
            Bool, "/depth/baseline_ready", 10
        )
        self.show_obstacle_fill = bool(getattr(display_debug, "SHOW_OBSTACLE_FILL", False))
        self.show_roi_polygons = bool(getattr(display_debug, "SHOW_ROI_POLYGONS", False))
        self.show_rgb_debug_text = bool(getattr(display_debug, "SHOW_ZONE_DEBUG_TEXT", False))

        missing = collect_missing_calibration_items()
        if missing:
            self.get_logger().error("Depth ROI calibration is incomplete; not starting.")
            for item in missing:
                self.get_logger().error(f"  - {item}")
            raise RuntimeError("incomplete depth calibration")

        self.zone_masks, self.logic_mask, self.zone_metas = build_zone_roi_metas(
            (FRAME_HEIGHT, FRAME_WIDTH)
        )
        self.depth_filter = DepthVoidTemporalFilter(
            image_shape=(FRAME_HEIGHT, FRAME_WIDTH),
            alpha=DEPTH_EMA_ALPHA,
            hold_max_frames=DEPTH_HOLD_MAX_FRAMES,
            logic_mask=self.logic_mask,
        )
        self.zone_stable_filter = ZoneStableFilter(
            zone_keys=[cfg["key"] for cfg in ZONE_CONFIGS],
            on_frames=ZONE_ON_CONFIRM_FRAMES,
            off_frames=ZONE_OFF_CONFIRM_FRAMES,
        )

        self.cam = UnifiedCameraManager(
            enable_depth=True,
            width=FRAME_WIDTH,
            height=FRAME_HEIGHT,
            fps=FRAME_FPS,
        )
        if not self.cam.start():
            raise RuntimeError("failed to start unified camera")

        if ENABLE_WEB_VIDEO_STREAM:
            start_web_video_server(
                host=WEB_VIDEO_HOST,
                port=WEB_VIDEO_PORT,
                fps=WEB_VIDEO_FPS,
                quality=WEB_VIDEO_JPEG_QUALITY,
            )

        self.window_name = "RGB + Depth + Obstacle ROI"
        if SHOW_RGB_WINDOW:
            cv.namedWindow(self.window_name)

        # Baseline capture state (web-triggered, same as main.py 'b' key)
        self.baseline_depth = None
        self.baseline_ready = False
        self.baseline_collecting = False
        self.baseline_frames = []

        self.seq = 0
        self.last_report_time = time.perf_counter()
        period = 1.0 / max(1.0, float(self.get_parameter("publish_rate").value))
        self.timer = self.create_timer(period, self._tick)
        self.get_logger().info("Depth obstacle node started")

    def _on_web_control(self, msg: String):
        try:
            data = json.loads(msg.data or "{}")
        except json.JSONDecodeError:
            return
        if data.get("command") == "baseline_capture":
            self.baseline_depth = None
            self.baseline_ready = False
            self.baseline_collecting = True
            self.baseline_frames.clear()
            self.get_logger().info(
                f"Baseline capture started: collecting {BASELINE_CAPTURE_FRAME_COUNT} frames"
            )
            return

        if data.get("command") != "runtime_options":
            return

        if "show_obstacle_fill" in data:
            self.show_obstacle_fill = bool(data.get("show_obstacle_fill"))
        if "show_roi_polygons" in data:
            self.show_roi_polygons = bool(data.get("show_roi_polygons"))
        if "show_rgb_debug_text" in data:
            self.show_rgb_debug_text = bool(data.get("show_rgb_debug_text"))

        display_debug.SHOW_OBSTACLE_FILL = self.show_obstacle_fill
        display_debug.SHOW_ROI_POLYGONS = self.show_roi_polygons
        display_debug.SHOW_ZONE_DEBUG_TEXT = self.show_rgb_debug_text

        # Also sync config_switches for obstacle_vision module
        import config_switches as _cs
        _cs.SHOW_OBSTACLE_FILL = self.show_obstacle_fill
        _cs.SHOW_ROI_POLYGONS = self.show_roi_polygons

    def _tick(self):
        color, depth = self.cam.get_frames()
        if color is None or depth is None:
            return

        depth_for_logic = self.depth_filter.update(depth)
        depth_int = depth_for_logic.astype(np.int32)

        # Baseline capture logic (web-triggered, replaces main.py 'b' key)
        if self.baseline_collecting:
            self.baseline_frames.append(depth.copy())  # raw depth, not filtered
            if len(self.baseline_frames) >= BASELINE_CAPTURE_FRAME_COUNT:
                stack = np.stack(self.baseline_frames, axis=0).astype(np.float32)
                self.baseline_depth = np.median(stack, axis=0).astype(np.uint16)
                self.baseline_frames.clear()
                self.baseline_collecting = False
                self.baseline_ready = True
                self.get_logger().info("Baseline capture complete")

        baseline_int = self.baseline_depth.astype(np.int32) if self.baseline_ready else None

        zone_stats = {}
        for cfg in ZONE_CONFIGS:
            zone_stats[cfg["key"]] = calculate_obstacle_stats_fast(
                depth_int,
                baseline_int,
                cfg,
                self.zone_metas.get(cfg["key"]),
            )
        zone_stats = self.zone_stable_filter.update(zone_stats)

        if ENABLE_WEB_VIDEO_STREAM:
            display_color = color.copy()
            draw_text = (self.seq % max(1, TEXT_EVERY_N_FRAMES)) == 0
            draw_obstacle_debug(display_color, zone_stats, draw_text=draw_text,
                show_roi_polygons=self.show_roi_polygons,
                show_obstacle_fill=self.show_obstacle_fill,
                show_zone_debug_text=self.show_rgb_debug_text)
            publish_web_frame(display_color)
        elif SHOW_RGB_WINDOW:
            display_color = color.copy()
            draw_text = (self.seq % max(1, TEXT_EVERY_N_FRAMES)) == 0
            draw_obstacle_debug(display_color, zone_stats, draw_text=draw_text,
                show_roi_polygons=self.show_roi_polygons,
                show_obstacle_fill=self.show_obstacle_fill,
                show_zone_debug_text=self.show_rgb_debug_text)

        if SHOW_RGB_WINDOW:
            cv.imshow(self.window_name, display_color)
            key = cv.waitKey(1) & 0xFF
            if key == 27:
                self.get_logger().info("ESC pressed in RGB window, shutting down.")
                rclpy.shutdown()

        center_danger = int(
            zone_stats["C_RED"]["is_obstacle"]
            or zone_stats["C_YELLOW"]["is_obstacle"]
        )
        center_far = int(zone_stats["C_GREEN"]["is_obstacle"])
        center_total = combine_stats(
            [zone_stats["C_RED"], zone_stats["C_YELLOW"], zone_stats["C_GREEN"]],
            name="CENTER_TOTAL",
        )
        left_total = combine_stats(
            [zone_stats["L_RED"], zone_stats["L_YELLOW"], zone_stats["L_GREEN"]],
            name="LEFT_TOTAL",
        )
        right_total = combine_stats(
            [zone_stats["R_RED"], zone_stats["R_YELLOW"], zone_stats["R_GREEN"]],
            name="RIGHT_TOTAL",
        )
        left_blocked = int(left_total["is_obstacle"])
        right_blocked = int(right_total["is_obstacle"])
        all_total = combine_stats(
            [
                zone_stats["L_RED"], zone_stats["L_YELLOW"], zone_stats["L_GREEN"],
                zone_stats["C_RED"], zone_stats["C_YELLOW"], zone_stats["C_GREEN"],
                zone_stats["R_RED"], zone_stats["R_YELLOW"], zone_stats["R_GREEN"],
            ],
            name="ALL_TOTAL",
        )

        obstacle_distances = [
            s["min_depth"]
            for s in zone_stats.values()
            if s["is_obstacle"] and s["min_depth"] != 9999
        ]
        nearest = int(min(obstacle_distances)) if obstacle_distances else 9999

        if center_danger:
            level = 2
            preferred_dir = _choose_better_side(zone_stats)
        elif center_far:
            level = 1
            preferred_dir = _choose_better_side(zone_stats)
        else:
            level = 0
            preferred_dir = 0

        self.seq += 1
        left_score = _score_x1000(left_total, FAR_VALUE)
        right_score = _score_x1000(right_total, FAR_VALUE)
        msg = Int32MultiArray()
        msg.data = [
            int(level),
            int(preferred_dir),
            int(nearest),
            int(center_danger),
            int(center_far),
            int(left_blocked),
            int(right_blocked),
            int(self.seq),
            _area_x1000(center_total),
            _area_x1000(all_total),
            left_score,
            right_score,
            _center_offset_x1000(left_score, right_score),
            int(center_total.get("min_depth", 9999)),
        ]
        self.pub.publish(msg)

        # Publish baseline ready status
        self.baseline_ready_pub.publish(Bool(data=self.baseline_ready))

        now = time.perf_counter()
        if now - self.last_report_time > 2.0:
            self.last_report_time = now
            self.get_logger().info(
                f"depth level={level} dir={preferred_dir} nearest={nearest} "
                f"L/C/R={left_blocked}/{center_danger or center_far}/{right_blocked}"
            )

    def destroy_node(self):
        try:
            stop_web_video_server()
            self.cam.stop()
            if SHOW_RGB_WINDOW:
                try:
                    cv.destroyWindow(self.window_name)
                except Exception:
                    pass
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DepthObstacleNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
