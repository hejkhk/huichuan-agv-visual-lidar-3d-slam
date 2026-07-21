# V6 ROS2 RGB-D 输入版使用说明

这一版是在 `road_v5_5_640_modular_v5_avoid_opt` 基础上加的 ROS2 RGB-D 图像输入层。

## 核心变化

旧版本：

```text
main.py -> camera_orbbec.py -> pyorbbecsdk Pipeline() -> Gemini2
```

新版本默认：

```text
Orbbec ROS2 Wrapper 负责打开 Gemini2
    ↓
/camera/color/image_raw
/camera/depth/image_raw
/camera/color/camera_info
/camera/depth/camera_info
    ↓
ros2_camera_bridge.py
    ↓
main.py 原来的巡线 / 深度避障逻辑
```

好处：

1. 避开旧 Python SDK 直连时的 pipeline 开启失败、D2C profile 乱选、USB 带宽问题。
2. 后面可以自然接 ROS2 雷达导航、/cmd_vel、safety_fusion。
3. 原来的避障、baseline、ROI、寻线逻辑基本保留。

## 默认配置位置

在 `config_switches.py` 里：

```python
CAMERA_BACKEND = "ros2"
ROS2_FORCE_RESIZE_TO_FRAME_SIZE = True
ROS2_WAIT_FOR_NEW_DEPTH_FRAME = True
```

含义：

- `CAMERA_BACKEND="ros2"`：main.py 不再直接 import pyorbbecsdk，而是订阅 ROS2 图像。
- `ROS2_FORCE_RESIZE_TO_FRAME_SIZE=True`：如果你启动 1280×720 相机流，但旧标定还是 640×480，会先自动缩放到 640×480，保证代码能跑。
- `ROS2_WAIT_FOR_NEW_DEPTH_FRAME=True`：主循环跟着 depth 的新帧走，避免 10Hz depth 被重复计算几十遍。

## 启动步骤

### 1. 安装依赖

```bash
cd road_v5_5_640_modular_v6_ros2_rgbd
bash tools/install_rgbd_bridge_deps.sh
```

### 2. 复制 launch 文件到 Orbbec 包

```bash
cp launch/gemini2_rgbd_640.launch.py \
$(ros2 pkg prefix orbbec_camera)/share/orbbec_camera/launch/

cp launch/gemini2_rgbd_1280_color_safe.launch.py \
$(ros2 pkg prefix orbbec_camera)/share/orbbec_camera/launch/

cp launch/gemini2_rgbd_1280_full_test.launch.py \
$(ros2 pkg prefix orbbec_camera)/share/orbbec_camera/launch/
```

### 3. 先启动相机

推荐先用 640 稳定版：

```bash
ros2 launch orbbec_camera gemini2_rgbd_640.launch.py
```

如果你要继续测试 1280 full：

```bash
ros2 launch orbbec_camera gemini2_rgbd_1280_full_test.launch.py
```

注意：1280 full 默认 RGB 1280×720@30，Depth 1280×800@10，点云、IR、IMU 都关。

### 4. 检查 topic

```bash
ros2 topic list | grep camera
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
```

至少要有：

```text
/camera/color/image_raw
/camera/depth/image_raw
/camera/color/camera_info
/camera/depth/camera_info
```

### 5. 单独测试 ROS2 图像桥接

另开终端：

```bash
cd road_v5_5_640_modular_v6_ros2_rgbd
source /opt/ros/jazzy/setup.bash
python3 ros2_camera_bridge.py
```

能看到 `ROS2 RGB` 和 `ROS2 Depth(mm)` 窗口，就说明 ROS2 图像输入没问题。

### 6. 运行原主程序

```bash
cd road_v5_5_640_modular_v6_ros2_rgbd
source /opt/ros/jazzy/setup.bash
python3 main.py
```

## 1280 和旧 640 标定的关系

如果你用 `gemini2_rgbd_1280_full_test.launch.py`，ROS2 相机实际输出可能是 1280×720 / 1280×800。

但当前主程序还是旧 640 标定，所以默认会把 ROS2 图像缩放到：

```text
FRAME_WIDTH × FRAME_HEIGHT = 640 × 480
```

这能保证代码先跑起来，不会因为 ROI mask 尺寸不一致崩溃。

真正要做 1280 精准避障/巡线时，需要：

1. 把 `config_switches.py` 里的 `FRAME_WIDTH / FRAME_HEIGHT` 改成 1280×720；
2. 把 `ROS2_FORCE_RESIZE_TO_FRAME_SIZE` 改成 `False`；
3. 重新做 1280 标定。

## 后续接统一底盘接口

这版只是先把原来的视觉/深度算法跑在 ROS2 图像输入上。

下一步建议再改输出：

```text
旧文本串口 error1/mode/obs
    ↓
ROS2 /cmd_vel 或 /safety/depth_obstacle
    ↓
safety_fusion_node
    ↓
chassis_node
    ↓
AA55 二进制四轮速度协议
    ↓
STM32F407
```
