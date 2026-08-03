# STEP10V2：直接读取深度图生成低延迟局部三维点云

## 1. 为什么重做 STEP10

原 STEP10 的实测链路是：

```text
Gemini2
  → 约 27～30 万点的完整 PointCloud2
  → DDS 传输
  → Python/NumPy 解析、TF、裁剪、体素化
  → /local_highres_cloud
```

实测约为：

```text
原始点云：约 274K～305K 点
输出点云：约 22K～33K 点
处理时间：约 72～122 ms
总延迟：约 216～389 ms
输出频率：约 5～10 Hz
```

所以原 STEP10 的精度能看，但延迟不适合实时碰撞避障。

STEP10V2 改为：

```text
Gemini2 原生深度图
  → C++ 节点直接读取像素
  → 先抽样、距离过滤
  → 利用 CameraInfo 投影 XYZ
  → TF 到 base_link
  → 局部空间/车体/地面过滤
  → 3 cm 体素去重
  → /local_highres_cloud_v2
```

最关键的变化是：**Orbbec 不再生成完整原始 PointCloud2**，也不再让 Python 处理几十万 XYZ 点。

---

## 2. 分辨率说明：为什么默认是 1280×800，不是 1280×720

Gemini2 的原生高分辨率深度模式是 1280×800。RGB 可以是 1280×720。

STEP10V2 默认使用：

```text
Depth：1280×800 @ 30 FPS
Color：关闭
D2C：关闭
```

这是为了：

1. 使用真正的高分辨率深度测量，而不是把低分辨率深度插值到 RGB 网格；
2. 保留 Gemini2 深度相机完整的宽视场，左右三条道路不会被像素 ROI 裁掉；
3. 关闭 RGB、D2C、完整点云生成，先测出最低延迟基线。

需要同时开启 1280×720 RGB 识别时，可在验证完低延迟基线后改为：

```bash
STEP10V2_ENABLE_COLOR=true
STEP10V2_COLOR_WIDTH=1280
STEP10V2_COLOR_HEIGHT=720
STEP10V2_COLOR_FPS=30
STEP10V2_DEPTH_REGISTRATION=true
STEP10V2_ENABLE_FRAME_SYNC=true
```

开启 D2C 后，要检查统计中的 `image_width/image_height` 和 `camera_info_width/camera_info_height`。若驱动将深度对齐到 RGB 内参，可把：

```bash
STEP10V2_CAMERA_INFO_TOPIC=/camera/color/camera_info
```

默认测试阶段不要开启 D2C，先确认 V2 本身是否把延迟降下来。

---

## 3. 新增文件

```text
STEP10V2_DEPTH_IMAGE_LOCAL_CLOUD_TEST.sh

lidar/chapt1_ws/src/local_depth_cloud_cpp/
├── CMakeLists.txt
├── package.xml
└── src/
    └── depth_image_to_local_cloud_node.cpp

lidar/chapt1_ws/src/lidar_py/rviz/
└── local_highres_cloud_v2_test.rviz

视觉实时局部3D避障_STEP10V2说明.md
```

修改的已有文件：

```text
visual_laser_slam/run_visual_slam_step.sh
visual_laser_slam/visual_laser_slam.env
lidar/chapt1_ws/src/lidar_py/launch/visual_laser_slam.launch.py
lidar/chapt1_ws/src/lidar_py/setup.py
lidar/chapt1_ws/src/lidar_py/package.xml
```

原 STEP1～STEP10 保留，STEP10V2 是独立 profile：

```text
local_highres_v2
```

---

## 4. 运行

```bash
cd /home/hejkhk/下载/huichuan-agv-ros2-foxy-visual-lidar-slam-step/huichuan-agv-ros2-foxy-main

chmod +x STEP10V2_DEPTH_IMAGE_LOCAL_CLOUD_TEST.sh
./STEP10V2_DEPTH_IMAGE_LOCAL_CLOUD_TEST.sh
```

V2 会自动构建：

```text
lidar_py
local_depth_cloud_cpp
```

它只启动：

```text
Gemini2 Depth Image
相机 TF
C++ 深度图投影节点
RViz
```

不会启动：

```text
STM32
2D 雷达
RTAB-Map
Cartographer
OctoMap
Nav2
Orbbec 原始 PointCloud2
```

验证原始点云确实关闭：

```bash
ros2 topic list | grep '/camera/depth/points'
```

正常情况下不应出现 `/camera/depth/points`。

---

## 5. 输出话题

```text
深度输入：/camera/depth/image_raw
相机内参：/camera/depth/camera_info
局部点云：/local_highres_cloud_v2
性能统计：/local_highres_cloud_v2/stats
裁剪标记：/local_highres_cloud_v2/crop_markers
```

查看统计：

```bash
ros2 topic echo /local_highres_cloud_v2/stats
```

查看频率：

```bash
ros2 topic hz /camera/depth/image_raw
ros2 topic hz /local_highres_cloud_v2
```

查看带宽：

```bash
ros2 topic bw /camera/depth/image_raw
ros2 topic bw /local_highres_cloud_v2
```

---

## 6. 统计字段

```text
image_width/image_height
    节点真正收到的深度图分辨率。

camera_info_width/camera_info_height
    CameraInfo 对应分辨率。

intrinsics_scaled
    两者尺寸不一致时，节点是否按比例缩放了内参。

input_pixels
    输入图像总像素数。

sampled_pixels
    经过 pixel_stride 后实际读取的像素数。

valid_depth_pixels
    距离有效的深度像素数。

output_points
    局部裁剪、车体过滤、地面过滤、体素化后的点数。

input_age_ms
    深度图进入 C++ 回调时已经有多旧。

process_ms
    C++ 本帧处理时间。

output_age_ms
    点云发布完成时，原深度图总年龄。

input_hz/output_hz
    深度输入与局部点云输出频率。

tf_wait_ms
    等待相机到 base_link TF 的时间。
```

---

## 7. 第一轮验收目标

先以独立测试为准，不接避障控制：

```text
深度输入：接近 30 Hz，至少稳定高于 20 Hz
点云输出：至少 15 Hz，目标 20～30 Hz
process_ms：目标低于 30～40 ms
output_age_ms：平均低于 100～120 ms
95% 帧延迟：尽量低于 160 ms
连续 5 分钟：延迟不能不断累积
```

动态测试：

1. 在相机前快速左右摆动纸板；
2. 观察点云是否紧跟纸板，而不是拖着半秒前的位置；
3. 放置 5～8 cm 桌腿和细杆；
4. 放置低矮木板、门槛；
5. 放置悬空纸板；
6. 确认左右三路区域都在点云范围内。

---

## 8. 调参顺序

### 情况 A：处理仍超过 40 ms

先把：

```bash
STEP10V2_PIXEL_STRIDE=3
```

不要先增大体素。像素抽样发生在 XYZ 投影之前，省下的计算更多。

### 情况 B：点云太稀，细杆缺失

改回：

```bash
STEP10V2_PIXEL_STRIDE=2
STEP10V2_VOXEL_SIZE=0.025
```

仍不够再测试 `PIXEL_STRIDE=1`，但它会读取完整 102.4 万像素。

### 情况 C：点很多但只需要前方 3 米

```bash
STEP10V2_MAX_RANGE=3.0
STEP10V2_X_MAX=3.2
```

### 情况 D：RViz 导致频率下降

先关闭 RViz 做纯性能测试：

```bash
USE_RVIZ=false
./STEP10V2_DEPTH_IMAGE_LOCAL_CLOUD_TEST.sh
```

RViz 配置默认没有显示原始 PointCloud2；Depth Image 显示也默认关闭。

### 情况 E：1280×800@30 启动失败

先查看相机支持模式：

```bash
ros2 run orbbec_camera list_camera_profile_mode_node
```

再退到：

```bash
STEP10V2_DEPTH_WIDTH=640
STEP10V2_DEPTH_HEIGHT=400
STEP10V2_DEPTH_FPS=60
STEP10V2_PIXEL_STRIDE=1
```

该模式点数少，但帧率高，可用于对比延迟。

---

## 9. 当前还没有做的功能

STEP10V2 只负责：

```text
新鲜、局部、高精度、base_link坐标系点云
```

尚未加入：

```text
URDF精确自过滤
局部VoxelLayer/STVL
完整URDF三维碰撞检测
未来轨迹碰撞预测
MPPI联合避障
```

只有 STEP10V2 的延迟验收通过后，才继续 STEP11。
