"""
semi_auto_calibrate_640.py

作用：
    640x480 低带宽版的半自动标定工具。

为什么单独放一个文件？
    主程序 main.py 负责跑车；
    calibration_640.py 负责保存“当前正在使用的标定”；
    这个文件只负责“用鼠标重新采点并生成覆盖文件”。

使用方式：
    1. 先把车放在赛道上，摄像头固定好，不要再动相机支架。
    2. 运行：python semi_auto_calibrate_640.py
    3. 按 f 冻结当前画面，避免实时画面晃动影响点选。
    4. 按提示依次左键点击 16 个点。
    5. 点完后按 s 保存，会生成 calibration_override_640.py。
    6. 重新运行 main.py，calibration_640.py 会自动加载这个覆盖文件。

按键：
    f：冻结 / 解除冻结画面。
    z：撤销上一个点。
    r：清空所有点，重新开始。
    s：保存标定结果。必须 16 个点全部点完才能保存。
    q / ESC：退出。

点选顺序的核心思想：
    每一条距离线从左到右点 4 个点：
        左外边界、中心通道左边界、中心通道右边界、右外边界。

    一共 4 条距离线：
        NEAR：视野最近处。
        45cm：红区 / 黄区分界。
        55cm：黄区 / 绿区分界。
        95cm：绿区远端，也作为鸟瞰图远端。

    所以一共是：
        4 条距离线 × 4 个边界点 = 16 个点。

    65cm 点不需要你手点，脚本会在 55cm 和 95cm 之间插值出来。
    75cm / 85cm 也不需要保存为 ROI 点，只用于自动推算鸟瞰图扫描线 y 坐标。
"""

import json
import time
from pathlib import Path

import cv2 as cv
import numpy as np

from camera_orbbec import OrbbecCameraManager
from config_switches import FRAME_WIDTH, FRAME_HEIGHT, FRAME_FPS, BIRD_WIDTH, BIRD_HEIGHT


# ==============================
# 1. 标定点顺序
# ==============================

POINT_SEQUENCE = [
    # NEAR：视野最近处，从左到右四个点。
    ("ROI_NEAR_LEFT", "NEAR 近处：完整走廊 左外边界"),
    ("CENTER_NEAR_LEFT_MANUAL", "NEAR 近处：中间车身通道 左边界"),
    ("CENTER_NEAR_RIGHT_MANUAL", "NEAR 近处：中间车身通道 右边界"),
    ("ROI_NEAR_RIGHT", "NEAR 近处：完整走廊 右外边界"),

    # 45cm：红区 / 黄区分界，从左到右四个点。
    ("ROI_45_LEFT", "45cm：完整走廊 左外边界"),
    ("CENTER_45_LEFT_MANUAL", "45cm：中间车身通道 左边界"),
    ("CENTER_45_RIGHT_MANUAL", "45cm：中间车身通道 右边界"),
    ("ROI_45_RIGHT", "45cm：完整走廊 右外边界"),

    # 55cm：黄区 / 绿区分界，从左到右四个点。
    ("ROI_55_LEFT", "55cm：完整走廊 左外边界"),
    ("CENTER_55_LEFT_MANUAL", "55cm：中间车身通道 左边界"),
    ("CENTER_55_RIGHT_MANUAL", "55cm：中间车身通道 右边界"),
    ("ROI_55_RIGHT", "55cm：完整走廊 右外边界"),

    # 95cm：绿色提前观察区远端，从左到右四个点。
    ("ROI_95_LEFT", "95cm：完整走廊 左外边界 / 鸟瞰左上点"),
    ("CENTER_95_LEFT", "95cm：中间车身通道 左边界"),
    ("CENTER_95_RIGHT", "95cm：中间车身通道 右边界"),
    ("ROI_95_RIGHT", "95cm：完整走廊 右外边界 / 鸟瞰右上点"),
]

WINDOW_NAME = "Semi Auto Calibration 640"
OUTPUT_PY = Path(__file__).with_name("calibration_override_640.py")
OUTPUT_JSON = Path(__file__).with_name("calibration_override_640.json")

clicked_points = {}
mouse_xy = (-1, -1)


# ==============================
# 2. 通用小工具
# ==============================

def current_index():
    """
    返回当前应该点第几个点。
    """
    return len(clicked_points)


def is_complete():
    """
    判断 16 个关键点是否已经全部点完。
    """
    return len(clicked_points) >= len(POINT_SEQUENCE)


def as_int_point(p):
    """
    把点转换成 [int, int]，保证写入 Python 文件后干净好读。
    """
    return [int(round(float(p[0]))), int(round(float(p[1])))]


def lerp_point(p1, p2, t):
    """
    在两个点之间线性插值。

    参数：
        p1：起点。
        p2：终点。
        t：插值比例，0 表示 p1，1 表示 p2。

    返回：
        [x, y]。
    """
    p1 = np.asarray(p1, dtype=np.float32)
    p2 = np.asarray(p2, dtype=np.float32)
    p = p1 + (p2 - p1) * float(t)
    return as_int_point(p)


def average_y(*points):
    """
    计算若干个点的平均 y 坐标，用于生成 RAW_Y_DISTANCE_REFERENCE_CM。
    """
    return int(round(sum(p[1] for p in points) / max(len(points), 1)))


def build_scan_configs(overrides):
    """
    根据 55cm 和 95cm 的原图边界点，自动推算 55/65/75/85/95cm 在鸟瞰图里的 y 坐标。

    为什么要这么做？
        你点的是原始 RGB 图坐标；
        巡线算法用的是 bird_view 里的水平扫描线。
        所以这里用 cv.getPerspectiveTransform + cv.perspectiveTransform，
        把原图中的距离参考线变换到 bird_view 坐标。
    """
    src = np.float32(overrides["LINE_SRC_POINTS"])
    dst = np.float32([
        [0, 0],
        [BIRD_WIDTH, 0],
        [0, BIRD_HEIGHT],
        [BIRD_WIDTH, BIRD_HEIGHT],
    ])
    matrix = cv.getPerspectiveTransform(src, dst)

    distances = [55, 65, 75, 85, 95]
    configs = []

    for index, distance in enumerate(distances):
        # 55 到 95 之间插值。95cm 是远端，55cm 是绿区近端。
        t = (distance - 55.0) / (95.0 - 55.0)
        left_raw = lerp_point(overrides["ROI_55_LEFT"], overrides["ROI_95_LEFT"], t)
        right_raw = lerp_point(overrides["ROI_55_RIGHT"], overrides["ROI_95_RIGHT"], t)

        raw_pts = np.float32([[left_raw, right_raw]])
        bird_pts = cv.perspectiveTransform(raw_pts, matrix)[0]
        bird_y = int(round(float((bird_pts[0][1] + bird_pts[1][1]) / 2.0)))

        # 95cm 如果刚好贴着鸟瞰图最上边，容易吃边缘噪声，所以保留一点点边距。
        if distance == 95:
            bird_y = max(bird_y, 13)

        bird_y = max(0, min(BIRD_HEIGHT - 1, bird_y))

        configs.append({
            "serial_name": f"error{index + 1}",
            "label": f"e{distance}",
            "distance_cm": float(distance),
            "bird_y": int(bird_y),
        })

    # 为了保证从近到远的 error1~error5，按距离升序保存。
    return configs


# ==============================
# 3. 根据 16 个点击点生成覆盖配置
# ==============================

def build_overrides():
    """
    把鼠标点选结果整理成 calibration_640.py 能读取的覆盖字典。
    """
    if not is_complete():
        raise RuntimeError("标定点还没点完，不能保存。")

    overrides = {name: as_int_point(clicked_points[name]) for name, _ in POINT_SEQUENCE}

    # 65cm 点：在 55cm 和 95cm 之间按距离插值。
    # t = (65 - 55) / (95 - 55) = 0.25。
    t65 = (65.0 - 55.0) / (95.0 - 55.0)
    overrides["ROI_65_LEFT"] = lerp_point(overrides["ROI_55_LEFT"], overrides["ROI_95_LEFT"], t65)
    overrides["ROI_65_RIGHT"] = lerp_point(overrides["ROI_55_RIGHT"], overrides["ROI_95_RIGHT"], t65)
    overrides["CENTER_65_LEFT_MANUAL"] = lerp_point(overrides["CENTER_55_LEFT_MANUAL"], overrides["CENTER_95_LEFT"], t65)
    overrides["CENTER_65_RIGHT_MANUAL"] = lerp_point(overrides["CENTER_55_RIGHT_MANUAL"], overrides["CENTER_95_RIGHT"], t65)

    # 鸟瞰图 4 点：沿用你原来的策略。
    # 远端用 95cm 左右外边界，近端直接用图像左下角 / 右下角。
    overrides["LINE_SRC_POINTS"] = [
        overrides["ROI_95_LEFT"],
        overrides["ROI_95_RIGHT"],
        [0, FRAME_HEIGHT],
        [FRAME_WIDTH, FRAME_HEIGHT],
    ]

    # 原图 y 坐标和距离的粗略参考，只用于终端/代码理解。
    overrides["RAW_Y_DISTANCE_REFERENCE_CM"] = {
        average_y(overrides["ROI_45_LEFT"], overrides["ROI_45_RIGHT"]): 45.0,
        average_y(overrides["ROI_55_LEFT"], overrides["ROI_55_RIGHT"]): 55.0,
        average_y(overrides["ROI_95_LEFT"], overrides["ROI_95_RIGHT"]): 95.0,
    }

    # 自动推算 bird_view 里的 55/65/75/85/95cm 扫描线。
    overrides["SCAN_CONFIGS"] = build_scan_configs(overrides)

    return overrides


def write_python_override(overrides):
    """
    生成 calibration_override_640.py。

    这个文件会被 calibration_640.py 自动导入。
    """
    text = """# calibration_override_640.py\n#\n# 这个文件由 semi_auto_calibrate_640.py 自动生成。\n# 不建议手改；如果要重新标定，重新运行半自动标定脚本即可。\n#\n# 如果想回到 calibration_640.py 里的旧 640 临时标定，直接删除本文件即可。\n\nCALIBRATION_OVERRIDES = """
    text += repr(overrides)
    text += "\n"
    OUTPUT_PY.write_text(text, encoding="utf-8")

    OUTPUT_JSON.write_text(
        json.dumps(overrides, ensure_ascii=False, indent=2),
        encoding="utf-8"
    )

    print(f"✅ 已保存 Python 覆盖标定: {OUTPUT_PY}")
    print(f"✅ 已保存 JSON 备份标定: {OUTPUT_JSON}")

    print("\n自动推算出的鸟瞰图扫描线：")
    for cfg in overrides["SCAN_CONFIGS"]:
        print(
            f"  {cfg['serial_name']} / {cfg['label']} / "
            f"{cfg['distance_cm']}cm -> bird_y={cfg['bird_y']}"
        )


# ==============================
# 4. 鼠标与显示
# ==============================

def mouse_callback(event, x, y, flags, param):
    """
    鼠标回调：左键记录当前点。
    """
    global mouse_xy
    mouse_xy = (x, y)

    if event != cv.EVENT_LBUTTONDOWN:
        return

    idx = current_index()
    if idx >= len(POINT_SEQUENCE):
        print("✅ 16 个点已经点完。按 s 保存，或按 z 撤销。")
        return

    name, desc = POINT_SEQUENCE[idx]
    clicked_points[name] = [int(x), int(y)]
    print(f"[{idx + 1:02d}/{len(POINT_SEQUENCE)}] {name} = [{x}, {y}]  # {desc}")

    if is_complete():
        print("\n✅ 16 个点已经全部点完。检查画面连线没问题后，按 s 保存。")


def undo_last_point():
    """
    撤销上一个点。
    """
    idx = current_index()
    if idx <= 0:
        print("ℹ️ 没有可以撤销的点。")
        return

    name, _ = POINT_SEQUENCE[idx - 1]
    clicked_points.pop(name, None)
    print(f"↩️ 已撤销：{name}")


def draw_existing_points(image):
    """
    在画面上画出已经点过的点和辅助连线。
    """
    # 已点的点。
    for i, (name, desc) in enumerate(POINT_SEQUENCE):
        if name not in clicked_points:
            continue
        x, y = clicked_points[name]
        cv.circle(image, (x, y), 5, (0, 255, 255), -1)
        cv.putText(image, str(i + 1), (x + 6, y - 6), cv.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

    # 每 4 个点为一条距离线，画横向辅助线。
    for start in range(0, len(POINT_SEQUENCE), 4):
        names = [POINT_SEQUENCE[start + j][0] for j in range(4)]
        if all(name in clicked_points for name in names):
            pts = np.array([clicked_points[name] for name in names], dtype=np.int32)
            cv.polylines(image, [pts], False, (255, 255, 0), 2)

    # 左外边界、中间通道左右边界、右外边界纵向辅助线。
    columns = [
        ["ROI_NEAR_LEFT", "ROI_45_LEFT", "ROI_55_LEFT", "ROI_95_LEFT"],
        ["CENTER_NEAR_LEFT_MANUAL", "CENTER_45_LEFT_MANUAL", "CENTER_55_LEFT_MANUAL", "CENTER_95_LEFT"],
        ["CENTER_NEAR_RIGHT_MANUAL", "CENTER_45_RIGHT_MANUAL", "CENTER_55_RIGHT_MANUAL", "CENTER_95_RIGHT"],
        ["ROI_NEAR_RIGHT", "ROI_45_RIGHT", "ROI_55_RIGHT", "ROI_95_RIGHT"],
    ]
    for names in columns:
        ready = [name for name in names if name in clicked_points]
        if len(ready) >= 2:
            pts = np.array([clicked_points[name] for name in ready], dtype=np.int32)
            cv.polylines(image, [pts], False, (0, 255, 0), 1)


def draw_instruction(image, frozen):
    """
    在画面上画当前提示。
    """
    idx = current_index()
    y0 = 28

    cv.putText(image, "Semi Auto Calibration 640", (12, y0), cv.FONT_HERSHEY_SIMPLEX, 0.72, (0, 255, 0), 2)
    y0 += 30

    status = "FROZEN" if frozen else "LIVE"
    cv.putText(image, f"Mode: {status}   Points: {idx}/{len(POINT_SEQUENCE)}", (12, y0), cv.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 255), 2)
    y0 += 28

    if idx < len(POINT_SEQUENCE):
        name, desc = POINT_SEQUENCE[idx]
        cv.putText(image, f"Next: {idx + 1}. {name}", (12, y0), cv.FONT_HERSHEY_SIMPLEX, 0.58, (0, 0, 255), 2)
        y0 += 24
        cv.putText(image, desc, (12, y0), cv.FONT_HERSHEY_SIMPLEX, 0.50, (0, 0, 255), 2)
    else:
        cv.putText(image, "All points done. Press s to save.", (12, y0), cv.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)

    mx, my = mouse_xy
    if mx >= 0 and my >= 0:
        cv.putText(image, f"mouse=({mx},{my})", (12, FRAME_HEIGHT - 16), cv.FONT_HERSHEY_SIMPLEX, 0.56, (255, 255, 255), 2)

    cv.putText(image, "f freeze | z undo | r reset | s save | q quit", (12, FRAME_HEIGHT - 44), cv.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2)


# ==============================
# 5. 主函数
# ==============================

def main():
    print("=" * 72)
    print("640x480 半自动标定工具")
    print("点选顺序：每条距离线从左到右点 4 个点：左外 / 中左 / 中右 / 右外")
    print("距离线顺序：NEAR -> 45cm -> 55cm -> 95cm")
    print("按 f 冻结画面后再点，会更稳。")
    print("=" * 72)

    cam = OrbbecCameraManager(enable_depth=True, width=FRAME_WIDTH, height=FRAME_HEIGHT, fps=FRAME_FPS)
    if not cam.start():
        print("❌ 相机启动失败，无法标定。")
        return

    cv.namedWindow(WINDOW_NAME)
    cv.setMouseCallback(WINDOW_NAME, mouse_callback)

    frozen_frame = None

    try:
        while True:
            color, depth = cam.get_frames()
            if color is None:
                key = cv.waitKey(1) & 0xFF
                if key in (ord('q'), 27):
                    break
                continue

            if frozen_frame is None:
                show = color.copy()
                frozen = False
            else:
                show = frozen_frame.copy()
                frozen = True

            draw_existing_points(show)
            draw_instruction(show, frozen)

            cv.imshow(WINDOW_NAME, show)
            key = cv.waitKey(1) & 0xFF

            if key in (ord('q'), 27):
                break

            if key == ord('f'):
                if frozen_frame is None:
                    frozen_frame = color.copy()
                    print("🧊 已冻结当前画面，可以慢慢点。")
                else:
                    frozen_frame = None
                    print("▶️ 已解除冻结，回到实时画面。")

            elif key == ord('z'):
                undo_last_point()

            elif key == ord('r'):
                clicked_points.clear()
                print("🔄 已清空所有点，重新开始。")

            elif key == ord('s'):
                if not is_complete():
                    print(f"⚠️ 还没点完：当前 {current_index()}/{len(POINT_SEQUENCE)}，不能保存。")
                    continue
                overrides = build_overrides()
                write_python_override(overrides)
                print("✅ 保存完成。现在可以退出，然后重新运行 main.py。")

    finally:
        cam.stop()


if __name__ == "__main__":
    main()
