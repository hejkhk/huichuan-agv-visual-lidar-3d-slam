# STEP10：实时局部高精度三维点云测试

## 1. STEP10 是干什么的

STEP1～STEP9 已经完成：

- 视觉里程计 + 轮速 + IMU 的 EKF 融合；
- Cartographer 2D 激光 SLAM；
- OctoMap 全局三维体素地图；
- 2D 与 3D 地图统一到同一套 TF。

STEP10 **不修改也不替代 STEP1～STEP9**。它新增一条独立的实时局部感知链：

```text
Gemini2 原始深度点云
        ↓
最新帧优先（QoS depth=1）
        ↓
转换到 base_link
        ↓
距离裁剪 + 三维空间裁剪
        ↓
车体自身盒子过滤（可开关）
        ↓
地面高度带过滤（可开关）
        ↓
2.5 cm 体素降采样
        ↓
/local_highres_cloud
```

这份点云以后用于：

- STEP11 局部三维体素障碍地图；
- STEP12 URDF 模型与自过滤；
- STEP13 三维碰撞检测；
- STEP14 未来轨迹碰撞预测；
- STEP15 Nav2 + MPPI 联合避障。

全局 OctoMap 可以继续保持 5～8 cm 的较低分辨率；实时避障使用 STEP10 的 2.5 cm 局部点云。

---

## 2. 新增文件

```text
STEP10_HIGH_RES_LOCAL_CLOUD_TEST.sh
视觉实时局部3D避障_STEP10说明.md

lidar/chapt1_ws/src/lidar_py/lidar_py/local_highres_cloud_node.py
lidar/chapt1_ws/src/lidar_py/rviz/local_highres_cloud_test.rviz
```

同时更新：

```text
visual_laser_slam/visual_laser_slam.env
visual_laser_slam/run_visual_slam_step.sh
lidar/chapt1_ws/src/lidar_py/launch/visual_laser_slam.launch.py
lidar/chapt1_ws/src/lidar_py/setup.py
```

---

## 3. 运行前准备

1. 关闭 `open_all.sh`、STEP1～STEP9 和其他占用相机的程序。
2. 插好 Gemini2。
3. 确认系统已经安装并能找到 Orbbec ROS 2 驱动。
4. 确认相机外参仍是 STEP4～STEP9 实车测试成功的参数：

```bash
nano visual_laser_slam/visual_laser_slam.env
```

至少检查：

```bash
CAMERA_X=0.3
CAMERA_Y=0.0
CAMERA_Z=0.4
CAMERA_ROLL=0.0
CAMERA_PITCH=0.0
CAMERA_YAW=0.0
CAMERA_TF_CONFIRMED=true
```

上面的数值只是当前源码保留值，最终应以实车精确测量为准。

如果 Orbbec 驱动不在系统环境中，设置：

```bash
ORBBEC_SETUP=/你的Orbbec工作空间/install/setup.bash
```

源码中删除 `OrbbecSDK_ROS2` 文件夹不影响本测试，只要电脑上已有可用的驱动安装环境。

---

## 4. 一键启动

```bash
cd /你的路径/huichuan-agv-ros2-foxy-main
chmod +x STEP10_HIGH_RES_LOCAL_CLOUD_TEST.sh
./STEP10_HIGH_RES_LOCAL_CLOUD_TEST.sh
```

脚本会自动：

- 隔离编译 `lidar_py`；
- 启动 Gemini2 原始深度点云；
- 发布 `base_link → camera_link` 外参；
- 启动高分辨率局部点云节点；
- 打开专用 RViz；
- 保存运行日志。

停止：

```text
Ctrl + C
```

---

## 5. 话题

### 原始点云

```text
/camera/depth/points
```

### 实时局部高精度点云

```text
/local_highres_cloud
```

该点云已经转换到：

```text
base_link
```

并保留原始点云时间戳，方便后续判断实际延迟。

### 性能统计

```text
/local_highres_cloud/stats
```

查看：

```bash
ros2 topic echo /local_highres_cloud/stats
```

统计字段包括：

- `input_points`：输入点数；
- `output_points`：输出点数；
- `input_age_ms`：收到点云时，数据已经有多旧；
- `output_age_ms`：处理完成发布时，总数据年龄；
- `process_ms`：单帧处理耗时；
- `tf_wait_ms`：等待 TF 的时间；
- `input_hz`：节点实际收到的频率；
- `output_hz`：局部点云输出频率；
- `known_dropped_frames`：节点明确知道的丢弃帧数；
- `rate_dropped_frames`：因为限频跳过的帧；
- `tf_dropped_frames`：因为 TF 不可用丢弃的帧；
- `empty_dropped_frames`：过滤后无有效点的帧。

### 裁剪区域标记

```text
/local_highres_cloud/crop_markers
```

RViz 中：

- 淡蓝色盒子：局部感知范围；
- 红色盒子：车体自身过滤范围；
- 黄色薄盒子：地面过滤高度带（开启后才显示）。

---

## 6. 默认参数

配置位于：

```text
visual_laser_slam/visual_laser_slam.env
```

### 分辨率与频率

```bash
LOCAL_MAX_RATE_HZ=12.0
LOCAL_SAMPLE_STRIDE=1
LOCAL_VOXEL_SIZE=0.025
```

含义：

- 最多处理 12 Hz；Gemini2 当前约 10 Hz，因此正常情况下不会主动限频；
- 不跳过原始点；
- 用 2.5 cm 体素保留局部障碍细节。

### 距离范围

```bash
LOCAL_MIN_RANGE=0.20
LOCAL_MAX_RANGE=4.0
```

### base_link 局部窗口

```bash
LOCAL_X_MIN=0.20
LOCAL_X_MAX=4.00
LOCAL_Y_MIN=-2.50
LOCAL_Y_MAX=2.50
LOCAL_Z_MIN=-0.50
LOCAL_Z_MAX=2.00
```

车体坐标：

```text
x：向前
 y：向左
 z：向上
```

### 车体自身过滤

```bash
LOCAL_REMOVE_SELF=true
LOCAL_SELF_X_MIN=-0.36
LOCAL_SELF_X_MAX=0.36
LOCAL_SELF_Y_MIN=-0.36
LOCAL_SELF_Y_MAX=0.36
LOCAL_SELF_Z_MIN=-0.10
LOCAL_SELF_Z_MAX=0.90
```

当前按约 66.5 cm 见方底盘设置，并留了少量余量。STEP12 加入 URDF 后，会用更准确的机器人模型替代这个简单盒子。

### 地面过滤

第一轮默认关闭：

```bash
LOCAL_GROUND_FILTER_ENABLED=false
```

先在 RViz 中观察真实地面在 `base_link` 下的 Z 高度。确认后再开启，例如：

```bash
LOCAL_GROUND_FILTER_ENABLED=true
LOCAL_GROUND_Z_MIN=-0.06
LOCAL_GROUND_Z_MAX=0.08
```

不要在不知道 `base_link` 高度基准时直接开启，否则可能把低矮障碍一起删掉。

---

## 7. STEP10 测试流程

### 测试 A：静止性能

启动后静止 30 秒，执行：

```bash
ros2 topic hz /camera/depth/points
ros2 topic hz /local_highres_cloud
ros2 topic echo /local_highres_cloud/stats
```

建议目标：

```text
输入：约 10 Hz
输出：至少 8～10 Hz
process_ms：尽量低于 80～100 ms
output_age_ms：尽量低于 150～200 ms
```

最重要的是：连续运行时 `output_age_ms` 不能不断增长。

### 测试 B：延迟测试

在相机前 0.5～2 m 范围内，快速左右摆动一块纸板或纸箱。

同时观察 RViz：

- `Raw Gemini2 Cloud`；
- `STEP10 Local High-Resolution Cloud`。

通过标准：

- 两幅点云动作基本同步；
- 局部点云不会落后约半秒；
- 停止摆动后不会继续显示多帧旧位置；
- 运行越久，延迟不会越来越大。

### 测试 C：细障碍

分别放置：

- 5 cm 左右桌腿或圆柱；
- 低矮木板；
- 20 cm 纸箱；
- 悬空纸板。

检查这些物体在 `/local_highres_cloud` 中是否仍有连续点。

### 测试 D：五分钟稳定性

持续运行至少 5 分钟，观察：

- 输出频率是否稳定；
- `output_age_ms` 是否稳定；
- TF 丢帧是否持续增长；
- CPU 是否异常满载；
- 点云是否出现越来越明显的拖影。

---

## 8. 如何调参数

### 输出频率不足或处理时间太高

先改：

```bash
LOCAL_SAMPLE_STRIDE=2
```

仍然过慢，再改：

```bash
LOCAL_VOXEL_SIZE=0.03
```

不建议一开始就把体素增大到 5～8 cm，因为 STEP10 的目的就是保留局部细节。

### 点太多、远处噪声多

```bash
LOCAL_MAX_RANGE=3.0
LOCAL_X_MAX=3.5
LOCAL_Y_MIN=-2.0
LOCAL_Y_MAX=2.0
```

局部避障一般不需要保存整个房间。

### 地面点过多

先确认地面 Z 值，再开启：

```bash
LOCAL_GROUND_FILTER_ENABLED=true
```

一次只把过滤带扩大 1～2 cm，避免把门槛和低矮障碍误删。

### 车体点仍出现

根据 RViz 红色盒子调整：

```bash
LOCAL_SELF_X_MIN
LOCAL_SELF_X_MAX
LOCAL_SELF_Y_MIN
LOCAL_SELF_Y_MAX
LOCAL_SELF_Z_MIN
LOCAL_SELF_Z_MAX
```

---

## 9. STEP10 通过标准

满足以下条件后再进入 STEP11：

```text
1. /local_highres_cloud 稳定输出至少 8 Hz 左右；
2. output_age_ms 不随运行时间持续增加；
3. 快速摆动物体时延迟明显小于原 STEP7 的约 0.5 秒；
4. 5 cm 左右桌腿/细杆仍能被看到；
5. 低矮和悬空障碍都有点云；
6. 裁剪区域、自身过滤和地面过滤参数都能正常控制；
7. 连续运行 5 分钟不崩溃、不出现严重 TF 错误。
```

STEP10 只验证实时点云，**不启动 OctoMap、Cartographer、Nav2、MPPI 或底盘电机**。
