"""
utils.py

作用：
    放项目里通用的小工具函数。
    这里尽量不要放具体业务逻辑，避免和巡线/避障/相机耦合在一起。
"""

import cv2 as cv
import numpy as np

def is_point_ready(point):
    """
    判断一个像素点是否已经填写。

    参数：
        point：正常应该是 [x, y]，例如 [462, 585]。

    返回：
        True：点可用。
        False：点还是 None、格式不对，或者里面有 None / NaN。
    """
    if point is None:                                  # None 表示这个点还没标定
        return False
    try:
        if len(point) != 2:                            # 一个点必须是 [x, y] 两个数
            return False
        x = float(point[0])                            # 尝试转成数字，避免填了字符串/None
        y = float(point[1])
        return np.isfinite(x) and np.isfinite(y)       #np.isfinite() 是 NumPy 中用于判断数值是否为有限数的核心函数，专门用来检测数组 / 数值中是否存在无穷大（inf）或非数值（NaN）。
                                                       # NaN / inf 都认为没填好
    except Exception:
        return False

def is_points_ready(points, min_points=3):
    """
    判断 ROI / 标定点是否已经填写。

    参数：
        points：你配置的点集，可以是 None，也可以是 np.array。
        min_points：至少需要几个点；多边形至少 3 个点，透视变换至少 4 个点。

    返回：
        True：说明点集可用。
        False：说明你还没填标定数据，或者里面有 None / NaN。
    """
    if points is None:                                  # None 代表你还没填
        return False

    try:
        arr = np.asarray(points, dtype=np.float32)      # 转成数字数组；如果有非法内容会失败
    except Exception:
        return False

    if arr.ndim != 2:                                   # 正常点集应该是二维数组，比如 [[x,y], [x,y]]
        return False

    if arr.shape[0] < min_points:                       # 点数量不够
        return False

    if arr.shape[1] != 2:                               # 每个点必须有 x 和 y 两个坐标
        return False

    if not np.all(np.isfinite(arr)):                    # 只要有 NaN / inf，就认为没标定好
        return False

    return True                                         # 通过所有检查，说明点集可用

def make_polygon_mask(image_shape, points):
    """
    根据多边形点集生成 ROI 掩码。

    参数：
        image_shape：图像形状，可以传 depth.shape 或 color.shape[:2]。
        points：多边形点，例如 STOP_ROI_POINTS。

    返回：
        mask：uint8 单通道图，ROI 内为 255，ROI 外为 0。
    """
    height, width = image_shape[:2]     # 取图像高度和宽度
    mask = np.zeros((height, width), dtype=np.uint8)  # 创建一张全黑掩码

    if not is_points_ready(points, min_points=3):
        return mask                     # 如果点没填，直接返回全黑掩码

    pts = np.asarray(points, dtype=np.int32)  # OpenCV 画多边形需要 int32 坐标
    cv.fillPoly(mask, [pts], 255)       # 把多边形内部填成白色 255
    return mask                         # 返回 ROI 掩码

def draw_polygon(image, points, color, name, draw_label=True):
    """
    在图像上画出 ROI 多边形。           给对应的多边形画框并标名字。

    参数：
        image：要画的 BGR 图像。
        points：ROI 多边形点。
        color：BGR 颜色，例如 (0, 0, 255) 是红色。
        name：区域名字，会显示在第一个点旁边。
        draw_label：是否绘制区域名字；V4.8.1 中文字可以降频绘制。
    """
    if not is_points_ready(points, min_points=3):
        return                          # 如果点没填，就不画

    pts = np.asarray(points, dtype=np.int32)  # 转成 OpenCV 需要的 int32
    cv.polylines(image, [pts], True, color, 2)  # 画封闭多边形边框

    # [V4.8.1-修正] ROI 边框每帧都画，但 ROI 名字属于文字，按 TEXT_EVERY_N_FRAMES 降频绘制。
    if draw_label:
        first_x, first_y = pts[0]       # 取第一个点坐标，用来放文字
        cv.putText(image, name, (first_x, first_y - 6), cv.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

def get_xyz(x_pixel, y_pixel, depth_val, intr_param):
    """
    把某个像素点 + 深度值转换成相机坐标系下的 X/Y/Z。           返回对应点相对于相机的XYZ坐标。

    参数：
        x_pixel：像素 x 坐标。
        y_pixel：像素 y 坐标。
        depth_val：该像素的深度值，单位通常是 mm。
        intr_param：Orbbec SDK 返回的相机内参。

    返回：
        (X, Y, Z)：单位是米。
    """
    if depth_val <= 0:                  # 深度为 0 表示无效深度
        return None                     # 无效就返回 None

    # ROS2 桥接刚启动时，camera_info 可能还没收到；此时先不显示 XYZ，只显示 depth。
    if intr_param is None:
        return None

    fx_rgb = intr_param.rgb_intrinsic.fx  # RGB 相机 x 方向焦距
    fy_rgb = intr_param.rgb_intrinsic.fy  # RGB 相机 y 方向焦距
    cx_rgb = intr_param.rgb_intrinsic.cx  # RGB 相机光心 x
    cy_rgb = intr_param.rgb_intrinsic.cy  # RGB 相机光心 y

    Z = depth_val / 1000.0              # 深度从 mm 转成 m
    X = (x_pixel - cx_rgb) * Z / fx_rgb # 小孔成像模型反算 X
    Y = (y_pixel - cy_rgb) * Z / fy_rgb # 小孔成像模型反算 Y

    return X, Y, Z                      # 返回相机坐标系下的三维坐标
