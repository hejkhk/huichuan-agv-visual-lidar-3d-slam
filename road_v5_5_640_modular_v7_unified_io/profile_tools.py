"""
profile_tools.py

作用：
    性能诊断统计工具：累计 camera/line/obstacle/display 等模块耗时并定期打印。
"""

from config_switches import PROFILE_MODE

def make_profile_accumulator():
    """
    [V4.9-新增] 创建性能诊断统计器。

    返回：
        profile_sum：每个模块累计耗时，单位 ms。
        profile_count：累计了多少帧。
    """
    keys = [
        "camera",
        "depth_filter",
        "line",
        "baseline",
        "obstacle",
        "decision",
        "display",
        "waitkey",
        "total",
    ]
    return {key: 0.0 for key in keys}, 0


def add_profile_sample(profile_sum, timings_ms):
    """
    [V4.9-新增] 把当前帧各模块耗时累加进统计器。
    """
    for key, value in timings_ms.items():
        if key in profile_sum:
            profile_sum[key] += float(value)


def maybe_print_profile(profile_sum, profile_count):
    """
    [V4.9-新增] 每隔若干帧打印一次平均耗时。

    打印结果单位是 ms：
        camera      相机取帧耗时
        depth_filter 深度滤波耗时
        line        鸟瞰图/HSV/扫描线耗时
        obstacle    9 个 ROI 障碍统计耗时
        display     画面显示耗时
        total       一整帧总耗时
    """
    if not PROFILE_MODE:
        return
    if profile_count <= 0:
        return

    avg = {key: profile_sum[key] / profile_count for key in profile_sum}
    fps_est = 1000.0 / avg["total"] if avg["total"] > 0 else 0.0

    print(
        "⏱️ [V5.5 PROFILE] "
        f"camera:{avg['camera']:.1f}ms | "
        f"depth_filter:{avg['depth_filter']:.1f}ms | "
        f"line:{avg['line']:.1f}ms | "
        f"baseline:{avg['baseline']:.1f}ms | "
        f"obstacle:{avg['obstacle']:.1f}ms | "
        f"decision:{avg['decision']:.1f}ms | "
        f"display:{avg['display']:.1f}ms | "
        f"waitkey:{avg['waitkey']:.1f}ms | "
        f"total:{avg['total']:.1f}ms | "
        f"est_fps:{fps_est:.1f}"
    )
