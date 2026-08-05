# 双分辨率 2D/3D SLAM 修改记录

## 2026-08-05 V6.52：GitHub x86 预检边界修正

- 托管的 Ubuntu 22.04 runner 是 x86 且没有 Gemini2，不能代替 Jetson 链接、运行 ARM64 相机 SDK。
- `CAR_VALIDATION_SKIP_EXTERNAL=1` 现在会从 hosted `colcon build` 中明确排除
  `orbbec_camera`、`orbbec_camera_msgs` 和 `orbbec_description`，避免把硬件或架构环境错误误判为项目失败。
- CI 仍逐项验证三个 Orbbec ROS 包、ARM64 `libOrbbecSDK.so.2.9.3` 和 ARM64 深度引擎均已上传；
  项目自身的 Humble 包、定制行为树及 Nav2 lifecycle/pluginlib 测试仍完整执行。
- CI 使用 `--packages-ignore` 而不是 `--packages-skip`：后者仍会让 `lidar_py` 等待未构建相机包的
  ament 环境钩子，前者才会让硬件包完整退出 x86 runner 的依赖图。

## 2026-08-05 V6.53：Jetson 导航入口负载修正

- 删除 `START_DUAL_2D_3D_NAVIGATION.sh` 中遗留的 RTX 3060 专用 `RTAB-Map 2 Hz` 强制覆盖。
- 建图版和导航版现在都由统一 runner 自动判断平台：Jetson 使用 `1 Hz / 1 OMP线程`，
  x86 PC 使用 `2 Hz / 2 OMP线程`。
- 该频率只影响长期 RTAB-Map/OctoMap 更新；`640x400 @ 15 Hz`、3 cm 的实时碰撞点云保持不变。

## 2026-08-05 V6.54：首次下载后的 Orbbec 自动构建

- 修复重新下载或修改项目绝对路径后，构建指纹清理旧缓存，启动器却在重新编译之前报
  `Missing ROS package: orbbec_camera` 的启动顺序错误。
- 当 `orbbec_camera` 不可见时，一键启动器现在自动使用仓库内的官方 Wrapper，单线程编译
  `orbbec_camera_msgs` 和 `orbbec_camera` 到隔离缓存，并在成功加载新 overlay 后继续检查。
- 已存在且可用的 Orbbec overlay 不会重复编译；系统 Orbbec SDK、udev 规则和相机参数不变。

## 一、修改目标

本次修改针对 `STEP11_CLEAN_VISUAL_LIDAR_SLAM_TEST.sh` 启动后地图立即旋转、发散和乱飘的问题，并将系统整理为三层感知结构：

1. 2D 激光雷达负责稳定定位、2D 建图和全局路径规划。
2. Gemini2 建立可保存、可重新加载并继续扩展的低分辨率全局 3D 地图。
3. Gemini2 同时输出近场高分辨率实时点云，用于精细避障和后续 URDF 碰撞检测。

## 二、旧 STEP11 的问题

旧链路为：

```text
视觉里程计 + 轮速里程计 + IMU
                |
                v
               EKF
                |
                v
        odom -> base_link
                |
                v
          Cartographer
```

视觉里程计在弱纹理、反光地面、旋转模糊或 RGB-D 时间不同步时会产生错误位姿。EKF 接收错误位姿后继续发布 `odom -> base_link`，Cartographer 因而把整张 2D 地图跟着错误姿态旋转，最终出现：

- 原地转向后地图放射状发散；
- 墙体重复、房间旋转；
- RViz 中多组里程计箭头乱飞；
- 视觉故障直接影响底盘定位。

## 三、新架构

```text
                         +-------------------------------+
STM32 0x07 NAVI -------->| /odom + /imu_cartographer     |
                         |                               |
2D LiDAR ---------------->| Cartographer V13              |
                         | map -> odom -> base_link       |
                         | 唯一平面定位与TF权威           |
                         +---------------+---------------+
                                         |
                  +----------------------+----------------------+
                  |                                             |
                  v                                             v
       RTAB-Map低分辨率全局3D图                         2D地图/Nav2全局规划
       读取RGB-D、2D扫描和/odom
       publish_tf=false
       保存rtabmap.db

Gemini2 Depth -> STEP10V2.1 -> /local_highres_cloud_v21
                                |
                                v
                     Nav2局部VoxelLayer/未来URDF碰撞避障
```

关键原则：

- `map -> odom` 只由 Cartographer 发布。
- `odom -> base_link` 只由稳定底盘里程计发布。
- RTAB-Map 设置 `publish_tf=false`，没有权力改变小车定位。
- 新 STEP11 不启动 `rgbd_odometry`、`robot_localization` 或 `/odometry/filtered`。
- 3D 建图失败时最多影响 3D 地图，不会再带飞 2D 地图。

## 四、具体修改

### 1. 新增一键启动入口

```text
START_DUAL_2D_3D_SLAM.sh
```

该脚本可从任意目录执行，会自动切换到项目根目录并调用完整启动器。

### 2. 重写 STEP11 入口

```text
STEP11_CLEAN_VISUAL_LIDAR_SLAM_TEST.sh
```

旧入口原来启动 `slam_clean` EKF 链，现在改为启动新的双分辨率链。

### 3. 新增统一 ROS 2 Launch

```text
lidar/chapt1_ws/src/lidar_py/launch/dual_resolution_3d_slam.launch.py
```

此 Launch 同时启动：

| 模块 | 作用 | 主要输出 |
|---|---|---|
| 稳定版 Cartographer V13 | 2D 建图和定位 | `/map`、`map -> odom` |
| `chassis_node` | 接收 STM32 NAVI 帧并提供里程计 | `/odom`、`odom -> base_link` |
| 2D 雷达节点 | 发布固定角度扫描 | `/scan_timed_v2` |
| Gemini2 | 同步 RGB-D 图像 | `/camera/color/image_raw`、`/camera/depth/image_raw` |
| RTAB-Map | 低分辨率持久化 3D SLAM | `/rtabmap_3d/mapData`、数据库 |
| STEP10V2.1 C++ 点云节点 | 近场高分辨率实时点云 | `/local_highres_cloud_v21` |
| RViz2 | 同时查看 2D 图、3D 图和局部点云 | 图形界面 |

### 4. 新增独立启动器

```text
visual_laser_slam/run_dual_resolution_3d_slam.sh
```

启动器负责：

- 检查 ROS 2 Jazzy；
- 自动识别 STM32 和 2D 雷达串口；
- 自动申请串口权限；
- 检查 Orbbec、Cartographer 和 RTAB-Map 依赖；
- 使用独立缓存目录编译，不覆盖正式工作空间；
- 阻止与旧 SLAM/open_all 同时抢占相机、串口和 TF；
- 检查 `/map`、RGB、Depth、局部点云及 RTAB-Map 是否启动；
- `Ctrl+C` 时先让 RTAB-Map 正常写完数据库，再结束全部子进程；
- 将终端日志保存到 `SLAM_Log/dual_3d_时间/runtime.log`。

### 5. 新增统一参数文件

```text
visual_laser_slam/dual_resolution_3d.env
```

默认值：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `RTABMAP_RATE` | 2 Hz | 全局低分辨率 3D 图更新频率 |
| `GLOBAL_3D_VOXEL` | 0.08 m | 全局 3D 体素尺寸 |
| `LOCAL_RATE` | 15 Hz | 近场点云最大输出频率 |
| `LOCAL_VOXEL` | 0.03 m | 近场点云体素尺寸 |
| `LOCAL_X_MIN/MAX` | 0.15/4.0 m | 前向检测范围 |
| `LOCAL_Y_MIN/MAX` | -2.5/2.5 m | 左、中、右三条路径视野 |
| `LOCAL_Z_MIN/MAX` | -0.5/2.0 m | 高度检测范围 |
| `CAMERA_X` | 0.30 m | 相机到车体中心的前向距离 |

### 6. 新增持久化 3D 地图

数据库位置：

```text
maps/rtabmap_3d/rtabmap.db
```

正常退出后数据库会保留。下一次启动会读取已有节点并继续建图。

新建地图时执行：

```bash
./RESET_STEP11_3D_MAP.sh
```

旧数据库不会被直接删除，而是重命名为带时间戳的备份。

### 7. 新增 RViz 配置

```text
lidar/chapt1_ws/src/lidar_py/rviz/dual_resolution_3d_slam.rviz
```

同时显示：

- Cartographer 2D 地图；
- 2D 激光扫描；
- RTAB-Map 低分辨率持久化 3D 图；
- STEP10V2.1 高分辨率局部点云；
- Gemini2 RGB 图像；
- TF 树和局部裁剪范围。

### 8. 新增 Nav2 三维局部代价层配置

```text
lidar/chapt1_ws/src/lidar_py/config/nav2_local_3d_voxel_layer.yaml
```

该配置使用 Nav2 `VoxelLayer` 读取 `/local_highres_cloud_v21`：

- 只影响局部 costmap；
- 支持三维障碍标记与射线清除；
- 全局 costmap 继续使用 2D 激光雷达和 2D 地图；
- 后续加入 URDF 后可继续完善真实车体碰撞几何。

当前 STEP11 是 SLAM 和感知验收入口，不发送电机运动指令。点云验收稳定后，再把该配置合入正式 Nav2 启动链。

### 9. 更新安装清单

修改了：

```text
lidar/chapt1_ws/src/lidar_py/setup.py
lidar/chapt1_ws/src/lidar_py/package.xml
```

加入新 Launch、RViz、VoxelLayer 配置及 RTAB-Map 运行依赖。

## 五、一键启动方法

首次运行先赋予权限：

```bash
cd ~/huichuan-agv-ros2-foxy-visual-lidar-slam-step/huichuan-agv-ros2-foxy-main
chmod +x START_DUAL_2D_3D_SLAM.sh \
  STEP11_CLEAN_VISUAL_LIDAR_SLAM_TEST.sh \
  RESET_STEP11_3D_MAP.sh \
  visual_laser_slam/run_dual_resolution_3d_slam.sh
```

以后只需要：

```bash
./START_DUAL_2D_3D_SLAM.sh
```

结束：

```text
Ctrl+C
```

按一次后等待终端出现数据库路径和 `Complete`，不要直接关闭终端或断电。

## 六、首轮验收标准

1. 静止 10 秒，地图和箭头不能自行旋转或平移。
2. 直行约 1 m，2D 地图方向和实际方向一致。
3. 低速原地旋转 90 度，不得出现放射状地图或整个房间重建。
4. 左、中、右三处分别放障碍，局部点云必须全部覆盖。
5. 按 `Ctrl+C` 退出并重新启动，低分辨率 3D 地图应被重新加载。
6. 执行下面命令时不应出现 EKF 或视觉里程计节点：

```bash
ros2 node list | grep -E 'ekf|rgbd_odometry'
```

7. TF 应只有一条稳定的平面链：

```bash
ros2 run tf2_ros tf2_echo map base_link
```

## 七、未修改的稳定内容

本次没有修改：

- `cartographer_2d_v9_tightened.lua`；
- 已稳定的 Cartographer V13 参数；
- `open_all.sh`；
- `open_all_log.sh`；
- STM32 工程；
- STEP10V2.1 的三通道空间裁剪范围和核心 C++ 点云算法。

因此本次修改和正式运行链隔离，可以单独验证，不会直接破坏现有稳定版本。

## 八、2026-07-22 点云方向与显示修正

### 1. 修正 RTAB-Map 与 Cartographer 的显示坐标

第一版使用独立的 `map_3d`，再通过静态零变换叠加到 Cartographer 的 `map`。Cartographer 在建图过程中会更新 `map -> odom`，静态 `map -> map_3d` 无法表达这项修正，因此 2D 激光、实时局部点云和 RTAB-Map 彩色历史点云可能看起来不在同一方向。

现在 RTAB-Map 的 `map_frame_id` 直接使用 `map`，同时继续保持：

```text
publish_tf=false
```

因此两种地图使用同一个显示坐标，但 TF 定位权仍然只属于 Cartographer。

### 2. 留出 Gemini2 外参标定参数

外参位于：

```text
visual_laser_slam/dual_resolution_3d.env
```

位置单位为米，角度单位为度：

```bash
CAMERA_X=0.30
CAMERA_Y=0.0
CAMERA_Z=0.40

CAMERA_ROLL_DEG=0.0
CAMERA_PITCH_DEG=0.0
CAMERA_YAW_DEG=0.0
CAMERA_EXTRINSIC_CALIBRATED=false
```

角度约定：

| 参数 | 正方向 |
|---|---|
| `CAMERA_ROLL_DEG` | 相机左侧向上抬 |
| `CAMERA_PITCH_DEG` | 相机视线向下俯视 |
| `CAMERA_YAW_DEG` | 相机视线向左转 |

如果相机相对车体向下倾斜约 15 度，可先试：

```bash
CAMERA_PITCH_DEG=15.0
```

标定完成后改为：

```bash
CAMERA_EXTRINSIC_CALIBRATED=true
```

启动器会自动把角度转换成 ROS 使用的弧度。未标定时终端会持续给出明确警告，不会再把零角度误认为已完成标定。

### 3. 推荐标定步骤

1. 把车停在水平地面，车头正对一面平直墙壁。
2. 测量相机光心相对 `base_link` 的前、左、上距离，填写 `X/Y/Z`。
3. 用角度尺测量相机向下倾角，先填写 `CAMERA_PITCH_DEG`。
4. 启动一键脚本，仅观察实时彩色点云和局部 AxisColor 点云。
5. 调整 pitch，直到地面点云与 RViz 的 XY 平面平行。
6. 调整 yaw，直到正前方墙面与车体 Y 轴平行、左右位置对称。
7. 最后调整 roll，直到同一水平墙边的左右高度一致。
8. 每次修改参数后重新启动；确认后设置 `CAMERA_EXTRINSIC_CALIBRATED=true`。

不要用 RTAB-Map 已经累计的历史点判断实时外参。标定阶段重点观察 `/local_highres_cloud_v21`，因为它只显示当前帧。

### 4. 外参变化后必须重建 3D 数据库

旧点云已经按旧外参写入数据库，修改 `CAMERA_*` 后必须先停止程序并执行：

```bash
./RESET_STEP11_3D_MAP.sh
```

该脚本会归档旧数据库，再启动一键脚本建立新图。否则旧点和新点会形成两套方向不同的墙面。

### 5. RViz 清晰度调整

本次只调整显示和 RTAB-Map 深度栅格采样，不改变 STEP10V2.1 的稳定视野：

- RTAB-Map `Grid/DepthDecimation` 从 4 调为 2；
- RViz MapCloud 显示 decimation 从 4 调为 2；
- RViz MapCloud 显示体素从 8 cm 调为 4 cm；
- 实时局部点云点尺寸从 2 px 调为 3 px；
- 2D 地图透明度降低，避免遮挡 3D 点云；
- 粉色裁剪盒和地面网格默认关闭，可在 Displays 中手动打开。

全局 3D 数据库仍然使用 8 cm 低分辨率体素，近场避障点云仍然使用 3 cm 体素。显示更清楚不等于把全局地图改成高负载模式。

## 九、2026-07-22 AGV URDF 与相机倾角

### 1. 车体坐标和尺寸

新增：

```text
lidar/chapt1_ws/src/lidar_py/urdf/agv_box.urdf.xacro
```

URDF 使用一个外接长方体表示整车：

| 项目 | 数值 |
|---|---:|
| 长度 X | 0.665 m |
| 宽度 Y | 0.665 m |
| 高度 Z | 0.321 m |
| `base_link` | 地面上的车体中心 |
| 车体模型中心高度 | 0.1605 m |

URDF 同时包含 visual 和 collision。由于目前没有提供整车质量和质心数据，本次没有虚构惯性参数；后续接入 Nav2/URDF 碰撞检测时可以直接使用车体 collision box。

### 2. 雷达和相机位置

以下计算假设“在车上面”表示从车体上表面继续向上测量：

```text
雷达：X=+0.20 m，Y=0，Z=0.321+0.1025=0.4235 m
相机：X=+0.30 m，Y=0，Z=0.321+0.05=0.3710 m
```

参数仍留在 `visual_laser_slam/dual_resolution_3d.env`：

```bash
LIDAR_X=0.20
LIDAR_Y=0.0
LIDAR_Z=0.4235

CAMERA_X=0.30
CAMERA_Y=0.0
CAMERA_Z=0.371
```

如果给出的 10.25 cm 和 5 cm 是传感器离地高度，而不是高出车体上表面的距离，应把 `LIDAR_Z`、`CAMERA_Z` 直接改成 `0.1025`、`0.05`。

### 3. 避免重复 TF

功能性传感器坐标仍由原节点发布：

```text
base_link -> laser_frame
base_link -> camera_link -> camera optical frames
```

URDF 使用单独的显示/碰撞链接：

```text
base_link -> lidar_model_link
base_link -> camera_model_link
base_link -> chassis_body_link
```

因此 RobotModel 不会与雷达、相机节点重复发布同名 TF。

### 4. 加速度估算相机角度

截图中的静止平均加速度为：

```text
ax = -0.102163 m/s^2
ay = -8.770055 m/s^2
az = -4.098008 m/s^2
|a| = 9.6808 m/s^2
```

按照 Orbbec 相机 IMU 常用坐标约定（X 向右、Y 向下、Z 向前），估算：

```text
俯视角 = atan2(|az|, sqrt(ax^2 + ay^2)) = 25.04 deg
侧倾角约 = atan2(ax, |ay|) = -0.67 deg
```

X 分量很小，侧倾可能来自安装误差、桌面不水平或传感器零偏，所以默认仍使用：

```bash
CAMERA_ROLL_DEG=0.0
CAMERA_PITCH_DEG=25.04
CAMERA_YAW_DEG=0.0
CAMERA_EXTRINSIC_CALIBRATED=false
```

静止加速度无法确定 yaw。必须通过车头正对墙面、观察点云左右方向来标定 `CAMERA_YAW_DEG`。

25.04 度只是初值。最终应让实时地面点云与 RViz XY 平面平行，并确认正前方墙面与车体 Y 轴平行。完成后设置 `CAMERA_EXTRINSIC_CALIBRATED=true`。

### 5. RViz 显示

新的 Launch 启动 `robot_state_publisher`，RViz 增加 `AGV URDF Collision Model`。默认显示半透明车体 visual；需要检查碰撞盒时，在 RobotModel 中关闭 `Visual Enabled`、打开 `Collision Enabled`。

外参改变后，URDF 模型和实际 `camera_link` 会同时读取同一组参数，因此模型、局部点云和 RGB-D 建图保持一致。

## 十、2026-07-22 恢复 STEP10 宽视场并统一 RGB-D 坐标

STEP11 最初使用硬件 D2C（深度对齐到彩色）。即使深度话题仍显示 `640x400`，有效深度视场也会被裁切到彩色相机视场，所以明显小于 `STEP10V21_STABLE_LOCAL_CLOUD_TEST.sh`。

现改为：

```text
depth_registration=true
align_mode=SW
align_target_stream=DEPTH
```

即使用软件 C2D，把 RGB 对齐到原生深度坐标。局部避障云保留 STEP10V2.1 的原生深度宽视场，RTAB-Map 同时接收与深度坐标一致的 RGB-D 数据。

启动器会记录相机位置、角度、对齐模式和图像规格的配置签名。配置变化时自动备份旧 `rtabmap.db` 并建立新库，防止不同外参生成的墙面混在一起。

本链路仍不启动 EKF 或视觉里程计：

```text
STM32 NAVI -> /odom -> odom -> base_link
2D LiDAR + /odom -> Cartographer -> map -> odom
RTAB-Map publish_tf=false
```

曾根据截图估计加入第一轮水平朝向修正：

```text
CAMERA_YAW_DEG=-45.0  # 已由后续实测撤销
```

ROS 从车顶向下看时正 yaw 为逆时针，因此顺时针 45 度填写 `-45`。后续实测证明该截图估计不成立，详见第十二节。

## 十一、2026-07-22 STEP11 时间戳同步优化

本次没有修改 Cartographer 参数、TF 权限或传感器外参，只统一三条传感器链路的时间基准。

### 1. Gemini2 RGB-D

- 继续使用设备时间戳并启用主机时间同步、帧同步。
- `CAMERA_TIME_SYNC_PERIOD=10.0`：每 10 秒重新校正设备时钟与 ROS 主机时钟。
- `RGBD_SYNC_MAX_INTERVAL=0.030`：RTAB-Map 只配对时间差不超过 30 ms 的 RGB/Depth。
- 队列从 20/20 收紧为 10/10，减少旧图像排队后参与建图。
- 新增 `/dual_3d/rgbd_timestamp_stats`，持续报告配对差、p95、重复帧、倒退帧和未配对帧。

实车检查：

```bash
ros2 topic echo /dual_3d/rgbd_timestamp_stats --once
```

正常标准：`p95_ms <= 25`，`color_backward=0`、`depth_backward=0`，重复帧为 0 或极少。若软件 C2D 在当前机器上持续超过 25 ms，先确认 CPU 未满载，再把 `RGBD_SYNC_MAX_INTERVAL` 调至 `0.040`，不要直接扩大到原来的 `0.080`。

### 2. STM32 NAVI tick

0x07 上行帧 `spd[3]` 的 `HAL_GetTick()` 仍是采样时间来源。STEP11 新增滑动最小延迟映射：

```bash
NAVI_ADAPTIVE_CLOCK_SYNC=true
NAVI_CLOCK_WINDOW_SAMPLES=250
NAVI_CLOCK_MAX_ADJUSTMENT_NS=20000
```

50 Hz 下 250 帧约等于 5 秒。算法用窗口内最小串口延迟估计 MCU 与主机的时钟偏移，并把每帧修正限制为 20 us，既能追踪晶振慢漂，又不会因为 Linux 调度延迟造成 odom/IMU 时间戳跳动。时间戳始终严格递增，也不会晚于串口帧接收时刻。

该功能在通用 Cartographer 启动中默认关闭，只由 STEP11 显式开启，因此不会无意改变已经验证过的二维稳定启动方式。

### 3. 2D LiDAR

雷达继续使用 LD14P 帧内毫秒 tick、30 秒回绕展开和每包 0.1 ms 限幅校正。该链路已有设备时钟映射，本次没有改动，避免破坏 V13 已验证的二维建图效果。

### 4. 启动后验证

```bash
./START_DUAL_2D_3D_SLAM.sh
ros2 topic echo /dual_3d/rgbd_timestamp_stats --once
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic hz /odom
ros2 topic hz /scan_timed_v2
```

预期约为 RGB 15 Hz、Depth 15 Hz、odom 50 Hz，雷达按实际转速稳定输出。若建图异常，先保存上述统计和 `SLAM_Log/dual_3d_*/runtime.log`，不要先扩大同步窗口或更改 Cartographer 权重。

## 十二、2026-07-22 两轮实测日志修复

### 1. 日志结论

两份日志中的 RGB-D 同步正常：平均约 5.6 ms、p95 约 5.7 ms、最大约 15.6~20 ms，RGB 和 Depth 均约 14.4~14.5 Hz，未发现时间倒退或重复帧。

第二次运行在约 `19:22:40` 后停止收到 `/scan_timed_v2`。Cartographer 从 `19:22:50` 开始持续等待 scan，并反复请求固定在 `1784719359.994625` 的最后一帧雷达数据；与此同时 `/odom` 已继续前进近 40 秒。该时间差造成：

- Cartographer 无法查询扫描时刻的 `odom -> base_link`；
- RViz 被大量 TF 警告拖慢，表现为图像和点云卡住、消失；
- 扫描恢复后旧数据与新姿态错配，地图旋转、重建或乱飞。

这不是 Gemini2 时间同步失败，也不是 STM32 自适应 tick 映射造成。日志中 `/odom` 一直稳定为 50 Hz。

### 2. 雷达停流保护

- `/scan_timed` 和 `/scan_timed_v2` 改用 `BEST_EFFORT + KEEP_LAST(5)` 传感器 QoS，避免可靠发布反压堵住单线程串口读取定时器。
- `/scan` 仍保留 RELIABLE，兼容原有工具。
- 连续 1 秒没有有效 LD14P 数据包时，节点会关闭并重开雷达串口。
- 重连时清空半帧、未完成扫描和旧设备时钟映射，避免恢复后的第一圈与停流前数据拼接。
- 日志会明确打印 `LiDAR stream stalled` 和重连次数，不再静默卡死。

### 3. 撤销错误的 45 度外参

日志确认雷达 TF 始终为 `yaw=0.0deg`。歪 45 度的是我们此前依据截图猜测设置的摄像头：

```bash
CAMERA_YAW_DEG=-45.0
```

该数值同时正确作用于功能 TF 和 URDF 显示模型，因此两者一起歪 45 度恰好说明配置生效，并非被重复旋转。现已恢复 STEP9 验证过的前向约定：

```bash
CAMERA_YAW_DEG=0.0
```

外参签名改变后，启动器会自动归档使用 `-45°` 建立的旧 `rtabmap.db` 并新建数据库。旧数据库中的黑色历史点云不能继续复用，否则会与实时彩色局部云方向不一致。

两次日志还显示旧库启动时产生约 7000~15000 条 `VWDictionary::addWordRef(): Not found word ... (dict size=0)`。启动器现会先执行 SQLite `quick_check`，并检查 `Map_Node_Word` 存在引用但 `Word` 表为空的语义损坏。发现异常时自动归档旧库并创建新库，避免错误视觉词典继续制造假回环。

### 4. RViz

`START_DUAL_2D_3D_SLAM.sh` 和内部 runner 现在都强制 `USE_RVIZ=true`，执行一键脚本后自动打开 `dual_resolution_3d_slam.rviz`，不再依赖终端之前导出的环境变量。

若 launch 中的 RViz 节点在 15 秒内仍未出现，runner 会自动使用安装后的同一份 RViz 配置兜底启动，并在系统退出时回收该进程。

## 十三、2026-07-22 螺旋点云根因与单权威修复

### 1. 新日志结论

两次新日志中雷达没有再次停流，RGB-D 的 p95 约为 5.6 ms。螺旋点云不是时间戳问题。

旧链路让 RTAB-Map 直接使用下位机 `/odom`。实测在 `vx=0`、多数帧 `vz=0` 时，绝对 yaw 仍以约 15~20 度的台阶改变。RTAB-Map 同时启用了自己的视觉/激光邻接优化和空间闭环，随后出现 15~24 度约束误差并连续拒绝闭环。结果是 Cartographer 和 RTAB-Map 各自维护一套互相冲突的位姿图。

### 2. 新链路

```text
STM32 /odom + 2D LiDAR
          |
          v
Cartographer V13 -- map->base_link --> /cartographer_pose_odom (30 Hz)
                                           |
Gemini2 RGB-D ------------------------------+
                                           v
                              RTAB-Map 持久化低分辨率 3D 图
```

- `robot_pose_publisher` 新增 `/cartographer_pose_odom`，直接发布未经网页滤波的 Cartographer `map->base_link`。
- RTAB-Map 不再读取原始 `/odom`，也不再订阅 2D scan。
- 禁用 RTAB-Map 独立视觉闭环、空间邻近闭环和邻接边 ICP 修正。
- Cartographer 成为唯一全局位姿权威；RTAB-Map 只保存 RGB-D 关键帧和低分辨率 3D 地图。
- 实时高分辨率 `/local_highres_cloud_v21` 保持原 STEP10V2.1 链路，不受影响。
- 配置签名升级为 `cartographer_map_pose_v2_no_independent_loops`，首次启动会自动归档旧的螺旋数据库。
# 2026-07-22 V3：旧图迁移、RTAB RGB-D 几何与漂移诊断

- 新日志确认 RTAB-Map 仍加载了旧的 `rtabmap.db (38 MB)`；旧螺旋节点会随持久化地图继续显示，不能用于验证新位姿链。启动器新增 `cartographer_pose_v3` 一次性迁移标记：首次启动会把旧库重命名备份，之后仍正常续建新库。
- Gemini2 在 `DEPTH` 对齐模式下输出 RGB 640x480、Depth 640x400。新增 `rgb_depth_canvas_node`，只为 RTAB-Map 中心裁剪 RGB 到 640x400，并使用深度 CameraInfo；近场避障继续直接读取原生 640x400 深度，不缩小视野。
- `/cartographer_pose_odom` 增加 `CARTOGRAPHER_POSE_JUMP` 诊断。若下一次仍漂，日志将直接显示 Cartographer 的瞬时平移/偏航突跳，避免把 2D 位姿问题误判成 RTAB-Map 回环。
- RViz 的黑色持久云主要来自旧数据库和 RGB/Depth 几何不一致；本版同时处理这两个来源。
- 持久图输入增加 2 倍预降采样，RViz MapCloud 改为 8 cm / decimation 4，降低数据库增长和全图重建负载；15 Hz、3 cm 的近场避障点云保持不变。
- 一键启动固定使用独立的 `maps/rtabmap_3d/rtabmap_v3.db`。旧的 `rtabmap.db` 永远不会被稳定链加载；V3 库首次创建、以后自动续建。
- RViz 改由外层 runner 在编译和 ROS 传感器栈之前直接启动，launch 内部禁用第二个 RViz。窗口会先出现并等待 `/map` 和 TF，退出时仍由一键脚本统一回收。
# 2026-07-23 RTX 3060：OctoMap 全局三维占用层

## 本次结论

- 当前运行平台是 RTX 3060 电脑，不再采用树莓派的保守点云频率。
- Cartographer 继续作为唯一的二维全局位姿权威。
- RTAB-Map 继续保存低分辨率 RGB-D 关键帧，但不再默认显示黑色历史 MapCloud。
- 新增 OctoMap 全局三维占用层，默认 `5 Hz`、`0.05 m` 分辨率、`4 m` 最大范围。
- V21 局部高分辨率点云保持 `15 Hz`、`0.03 m`，继续负责实时近场避障。

## 黑点原因与处理

D2C 以 DEPTH 为目标时，深度视场中没有 RGB 对应像素的位置会被填黑。
这些黑色仅影响 RGB 着色点云，不代表深度几何无效。本次 OctoMap 只消费
`PointCloud2` 的 XYZ 几何，不消费 RGB，因此不会产生整片黑色占用体素。
RViz 的 RGB 窗口继续显示 `/camera/color/image_raw`，RTAB 对齐画布仅供 RTAB 使用。

## 新增链路

```text
/camera/depth/image_raw
  -> depth_image_to_local_cloud_v21_node
  -> /local_highres_cloud_v21        (15 Hz / 0.03 m，局部避障)
  -> global_cloud_relay              (限频并转换为 Reliable QoS)
  -> /global_3d/cloud_in             (5 Hz)
  -> octomap_server_global_3d
  -> /occupied_cells_vis_array       (RViz 三维占用体素)
  -> /octomap_binary /octomap_full   (保存接口)
```

`global_cloud_relay` 的 QoS 转换是必要的：V21 使用传感器 Best Effort QoS，
`octomap_server` 使用 Reliable 订阅；直接连接在部分 DDS 配置下收不到点云。

## 默认参数

参数位于 `visual_laser_slam/dual_resolution_3d.env`：

```bash
USE_OCTOMAP=true
OCTOMAP_RATE=5.0
OCTOMAP_RESOLUTION=0.05
OCTOMAP_MAX_RANGE=4.0
OCTOMAP_CLOUD_TOPIC=/global_3d/cloud_in
SAVE_OCTOMAP_ON_EXIT=true
OCTOMAP_SAVE_PATH=maps_3d/octomap_latest.bt
```

正常按 `Ctrl+C` 结束一键脚本时，会先调用 `octomap_saver_node` 保存
`maps_3d/octomap_latest.bt`，随后再结束 OctoMap 与 RTAB-Map。强制断电或
`kill -9` 无法触发保存。

需要更细的体素时可在 3060 电脑上测试 `OCTOMAP_RESOLUTION=0.03`，但应先确认
CPU 与内存余量。OctoMap 和大多数 ROS 点云节点主要使用 CPU，并不会因为存在
RTX 3060 自动获得 CUDA 加速。

## 启动与验证

首次缺少依赖时执行：

```bash
./STEP0_INSTALL_VISUAL_SLAM_DEPS.sh
```

正常启动：

```bash
./START_DUAL_2D_3D_SLAM.sh
```

启动完成后应看到：

```bash
ros2 topic hz /local_highres_cloud_v21
ros2 topic hz /global_3d/cloud_in
ros2 topic echo /occupied_cells_vis_array --once
```

预期频率分别约为 `15 Hz`、`5 Hz`，并且占用体素话题可以收到数据。
RViz 默认打开 `OctoMap 3D Occupied Voxels`，默认关闭
`RTAB-Map Persistent Low Resolution 3D`。

## 未改动的稳定链路

- 没有修改 Cartographer V13 参数。
- 没有修改 2D 雷达方向、时间戳或底盘里程计逻辑。
- OctoMap 与 RTAB-Map 均设置为不发布 TF，不能覆盖 Cartographer 位姿。
- 没有降低 V21 局部点云分辨率或频率。

## 实时避障接入

`nav2_auto_mapping_jazzy.yaml` 的实际 local costmap 已接入 V21 点云：

```text
V21 15 Hz 点云
  -> Nav2 local_costmap/depth_voxel_layer (10 Hz)
  -> 三维高度过滤、障碍标记与射线清除
  -> 二维局部代价地图
  -> Nav2 控制器实时减速、改向或停车
```

全局 costmap 仍只使用静态 Cartographer 地图与 2D 雷达，不将深度点云长期写入
全局规划层。这样低矮临时障碍只影响局部绕行，不会污染已建好的全局地图。

有效障碍高度默认是 `0.03..1.40 m`，Z 体素为 `0.10 m × 16`，深度有效距离是
`0.15..3.50 m`，清除射线最远 `4.0 m`。local costmap 更新为 `10 Hz`、发布为
`5 Hz`。

注意：VoxelLayer 只在 Nav2 controller/local_costmap 正在运行时参与自动导航。
手柄或网页直接发送轮速绕开了 Nav2 controller，因此还需要独立的硬急停安全层，
不能只依赖 VoxelLayer。

# 2026-07-23 V4：真实彩色 RTAB、完整滤波、实测外参与 MPPI 双版本

## 本轮目标

这一版保留已稳定的 `cartographer_2d_v9_tightened.lua`，没有改动 Cartographer
参数、雷达方向、底盘里程计和时间戳。改动只位于双分辨率 2D/3D 分支。

提供两个独立入口：

```bash
# 只建图：Cartographer + 实时局部3D + OctoMap + 按需RTAB
./START_DUAL_2D_3D_MAPPING.sh

# 建图并导航：在上面基础上增加Nav2 MPPI和深度VoxelLayer实时避障
./START_DUAL_2D_3D_NAVIGATION.sh
```

原来的 `START_DUAL_2D_3D_SLAM.sh` 保留为“只建图”兼容入口。

## 1. 黑色 RGB 与彩色持久点云

不再让 Orbbec 驱动把 RGB 硬塞到深度画布，也不再使用
`rgb_depth_canvas_node`。现在保持两个原生流不变：

```text
/camera/color/image_raw       640x480，真实RGB
/camera/depth/image_raw       640x400，完整深度视野
```

仅为 RTAB-Map 增加独立软件注册：

```text
原生Depth + Depth CameraInfo + Color CameraInfo
  -> depth_image_proc/RegisterNode
  -> /camera/rtabmap/depth_registered/image_raw（640x480）
  + /camera/color/image_raw（真实RGB）
  -> /rtabmap_3d
```

因此注册后没有深度的像素是“无几何点”，不会再生成大片黑色彩色点云。
RViz 的 `Gemini2 RGB` 也直接显示原生 RGB。

`RTAB-Map Persistent Low Resolution 3D` 默认关闭。勾选后
`rtabmap_demand_manager` 自动调用 `/rtabmap_3d/resume`；取消勾选并空闲 3 秒后
自动调用 `/rtabmap_3d/pause`，减少数据库、特征提取和全图重建压力。
实时避障点云和 OctoMap 不依赖这个开关。

新管线固定使用 `maps/rtabmap_3d/rtabmap_v4_color.db`。它不会加载 V3 及更早
数据库中的黑色 C2D 关键帧；旧数据库不会删除，仍保留在原路径或自动归档备份中。

## 2. 完整实时点云滤波

`depth_image_to_local_cloud_v21_node` 现在按以下顺序处理：

```text
深度范围 -> 8邻域空间一致性 -> 轻量时间滤波
-> base_link ROI裁剪 -> 车体自滤波 -> 可选地面滤波
-> 3cm体素降采样 -> 26邻域孤立体素剔除
```

默认参数位于 `visual_laser_slam/dual_resolution_3d.env`：

```bash
LOCAL_SPATIAL_FILTER=true
LOCAL_SPATIAL_THRESHOLD_M=0.08
LOCAL_SPATIAL_THRESHOLD_RATIO=0.025
LOCAL_SPATIAL_MIN_NEIGHBORS=2
LOCAL_TEMPORAL_FILTER=true
LOCAL_TEMPORAL_ALPHA=0.65
LOCAL_TEMPORAL_MAX_DELTA_M=0.06
LOCAL_VOXEL_OUTLIER_FILTER=true
LOCAL_VOXEL_MIN_NEIGHBORS=1
```

时间滤波只平滑小于 6 cm 的连续变化；人或箱子突然离开时会立即接受新深度，
不会因为平均旧帧而留下长时间残影。

节点同时发布：

```text
/local_highres_cloud_v21         base_link坐标，供RViz观察
/local_highres_cloud_v21/sensor  深度光学坐标，供OctoMap和Nav2射线清除
```

第二个话题保留真实相机原点。OctoMap/VoxelLayer 不再错误地从车体中心发射清除射线。

## 3. 不依赖支架量角的两阶段外参标定

第一步将车停在平整地面，相机下半视野不能被遮挡：

```bash
./START_DUAL_2D_3D_MAPPING.sh
# 另开终端
./CALIBRATE_CAMERA_EXTRINSIC.sh
```

把输出的 `CAMERA_ROLL_DEG`、`CAMERA_PITCH_DEG`、`CAMERA_Z` 写入
`visual_laser_slam/dual_resolution_3d.env`，重启建图。

第二步把车停在距一面大平墙约 1 至 3 米处，保证 Gemini2 与 2D 雷达同时看见
同一面墙；墙不需要与车身垂直：

```bash
./CALIBRATE_CAMERA_YAW.sh
```

工具分别拟合深度墙面与激光墙线，输出 `CAMERA_YAW_DEG`。写回 env 后设置：

```bash
CAMERA_EXTRINSIC_CALIBRATED=true
```

地面只能观测 roll、pitch 和高度，不能真实观测 yaw；因此不能把加速度计计算出的
yaw 当作标定结果。第二步才是相机相对激光雷达的真实水平夹角标定。

## 4. MPPI 导航与实时三维避障

导航版链路如下：

```text
Cartographer 2D地图 -> Smac全局路径
                              \
15Hz滤波深度点云 -> local_costmap/depth_voxel_layer
                              -> MPPI FollowPath -> /cmd_vel_nav
                              -> safety_fusion_node -> /cmd_vel_safe
                              -> chassis_node -> AA55轮速帧
```

MPPI 使用四轮差速车的 `DiffDrive` 模型，速度二档为 `vx_max=0.20 m/s`，
最大角速度约 `12 deg/s`。车体 footprint 为 `0.666 m x 0.666 m`。
局部代价地图以 10 Hz 更新，深度障碍高度为 `0.03..1.40 m`，会对临时低矮障碍
进行标记和射线清除。

本分支不再要求旧视觉 Baseline 才允许 Nav2 输出；`require_depth_baseline=false`。
但网页/手柄的接管状态和下位机安全逻辑仍然有效。

RViz 新增 `/plan` 全局路径和 `/local_plan` MPPI 局部路径显示。建图版没有这些
话题时只会保持空白，不影响建图。

## 5. 首次安装与验证

依赖安装：

```bash
./STEP0_INSTALL_VISUAL_SLAM_DEPS.sh
```

该脚本现在包含 `depth_image_proc`、`sensor_msgs_py`、Nav2 与 MPPI。

导航版启动后检查：

```bash
ros2 topic hz /local_highres_cloud_v21/sensor
ros2 topic echo /local_highres_cloud_v21/stats --once
ros2 topic hz /local_costmap/costmap
ros2 lifecycle get /controller_server
ros2 topic echo /cmd_vel_nav
```

预期点云约 15 Hz、local costmap 约 5 Hz 发布，`controller_server` 为 `active`。
先架空驱动轮测试方向和急停，再在低速、空旷区域测试 MPPI。
# V5：自动外参写入与 2D/3D 避障闭环（2026-07-23）

本节建立在 V4 原生 RGB、注册深度、完整点云滤波和双分辨率 3D 地图之上。

## 自动标定写入

- 新增 `lidar_py/calibration_env.py`，保留 env 注释并使用原子替换。
- 地面标定通过质量检查后自动写入 `CAMERA_ROLL_DEG`、
  `CAMERA_PITCH_DEG`、`CAMERA_Z`。
- 墙面标定通过质量检查后自动写入 `CAMERA_YAW_DEG`，并设置
  `CAMERA_EXTRINSIC_CALIBRATED=true`。
- 每次改写前自动生成 `dual_resolution_3d.env.bak.*`。
- 质量不合格、目标文件缺失或写入失败时不会覆盖原配置。

## 2D/3D 导航协同

- Cartographer 继续作为唯一 2D 地图和全局平面位姿权威。
- 过滤后的 `/local_highres_cloud_v21/sensor` 同时进入：
  - 局部 VoxelLayer：10 Hz 更新，供 MPPI 20 Hz 轨迹碰撞预测。
  - 全局 VoxelLayer：5 Hz 更新，供全局规划器绕开持续堵塞的低矮障碍。
- `planner_server` 现在也会加载双分辨率覆盖参数，修复过去只有控制器收到
  覆盖文件、全局 costmap 收不到 3D 障碍层的问题。
- 局部和全局 costmap 增加 0.02 m footprint padding。
- 车体实际 footprint 仍来自 0.665 m 方形车体参数，URDF 只负责模型和 TF。

完整交接、验证命令和故障边界见：

```text
交接文档_双分辨率3DSLAM与导航.md
```

## V5.1：Jazzy 首次实机编译兼容修正

- 修复 `local_depth_cloud_cpp` 在 GCC 13 下将 ROS 2 Jazzy 整数参数与
  `std::max(int, long)` 混用导致的编译失败。
- `spatial_min_neighbors` 和 `voxel_min_neighbors` 现在显式转换为 `int`。
- 修复两个标定脚本在 `set -u` 下加载 `/opt/ros/jazzy/setup.bash`
  时报 `AMENT_TRACE_SETUP_FILES: 未绑定的变量`。
- 新增根目录完整教程：
  `双分辨率3D建图导航_完整使用教程.md`。

## V5.2：三次旋转漂移日志修正（2026-07-23）

### 日志结论

三次测试均加载同一份 `cartographer_2d_v9_tightened.lua`，MAPPING 与兼容
SLAM 启动入口最终也进入同一个双分辨率 launch，因此问题不是两套
Cartographer 参数不一致。

三次日志共同记录到：

- `/NAVI` 的 `vx=0`、`vz=0`；
- 绝对 yaw 以 45、55、85 度等大台阶变化；
- `/cartographer_pose_odom` 随后在 20 至 40 ms 内跳变 40 至 89 度；
- 跳变时平移量接近 0。

这说明上游绝对 yaw 与角速度互相矛盾。直接把阶跃 yaw 同时写入 odom，会让
Cartographer 在原地旋转时产生螺旋或多层地图。

### 上位机修正

- 新增 `NAVI_MAX_YAW_RATE_DEG_S=120.0`，只限制物理上不可能的单帧姿态跳变。
- 初始绝对 yaw 仍直接作为起始朝向，不改变方向约定。
- 后续 yaw 先做 `-180..180` 连续展开，再按 MCU 时间戳限速。
- Cartographer IMU 的 `angular_velocity.z` 改为限速后姿态的真实导数，避免
  “yaw 正在变化但 vz 永远为 0”。
- 触发保护时日志输出 `NAVI_YAW_RATE_LIMIT`，包含原始阶跃、接受阶跃、dt
  和累计次数。
- 不修改 STM32 工程，不修改定版 Cartographer 参数。

### RGBD QoS 修正

Gemini2 的 RGB、Depth 和两路 CameraInfo 改为 `DEFAULT` 可靠 QoS，与 Jazzy
的 `depth_image_proc::RegisterNode` 匹配。修正前日志明确出现
`RELIABILITY_QOS_POLICY` 不兼容，RTAB-Map 因此一直收不到注册深度。

### 标定脚本诊断

两个标定脚本现在会分别检查 Jazzy、缓存工作区、配置文件和必要 ROS 话题。
失败时直接打印缺失路径或话题，不再只给模糊的启动失败提示。

## V5.3：五次实测日志闭环修正（2026-07-23）

### 五份日志的明确结论

- `20-45-49`、`20-47-11` 是导航版，均因隔离缓存工作区没有编译
  `frontier_exploration_ros2` 而立即退出。
- 标定脚本在新终端使用默认 ROS domain 0，主程序使用 domain 88，因此即使
  相机正在发布，标定仍会误报缺少 `/camera/depth/image_raw`。
- `20-38-43` 运行中出现雷达 `/dev/ttyACM0` 的 `Errno 5` 并短暂消失，
  这是独立的 USB/供电/接触问题，需要实车继续观察。
- `20-50-14` 在车辆静止时记录到下位机 yaw 和 `vz` 异常阶跃，随后
  Cartographer 在数百毫秒内连续跳变约 `+6.56/+9.60/+9.60/-11.51/-17.85`
  度，和截图中的双墙位置一致。
- 本地高分辨率点云通常约 14.4 Hz、单帧处理约 9 至 10 ms；实时避障源本身
  没有“几秒级”延迟。OctoMap 方块和 RTAB 持久地图属于低频/历史层，不能
  用它们的显示延迟判断碰撞避障延迟。

### 本次代码修正

- 导航版隔离构建加入 `frontier_exploration_ros2`，构建完成后再次检查包是否
  可见，避免进入 launch 后才报错。
- 两个外参标定脚本统一读取 `dual_resolution_3d.env`，使用与主程序相同的
  `ROS_DOMAIN_ID=88` 和 CycloneDDS，并等待话题最多 10 秒。
- yaw 保护由“把错误角度限制为每帧 2.4 度并继续追赶”改为
  “丢弃不可能的单帧跳变，同时把原始参考重设到新样本”。这样后续相同的
  错误绝对偏置不会再被逐帧注入里程计。
- 被丢弃的 yaw 帧不会再把原始 `vz` 发布给 Cartographer IMU；里程计和 IMU
  使用同一个实际接受角速度。
- RTAB-Map 彩色注册深度容器始终随 RTAB 启动，不再错误地依赖 RViz
  按需暂停选项。
- 默认 `RTABMAP_ON_DEMAND_PAUSE=false`：即使 RViz 不显示 MapCloud，
  RGB 特征、关键帧和视觉回环仍在后台运行；RViz 取消勾选只减少渲染负载。
- RTAB 输入恢复原始 RGB 分辨率与完整深度采样，视觉回环阈值由禁用状态
  恢复为 `0.11`，但 `publish_tf=false`，不会抢占 Cartographer 的主 TF。
- OctoMap 输入从 5 Hz 提高到 10 Hz；真正的导航碰撞输入仍是约 15 Hz 的
  `/local_highres_cloud_v21/sensor`。

### 持久地图和动态障碍的边界

- `RTAB-Map Persistent Low Resolution` 是历史关键帧地图。人走开后，它
  不保证立即删除过去看到的人，这是预期行为。
- `OctoMap 3D Occupied` 是长期占用记忆，也不是碰撞控制的唯一依据。
- 导航时的动态人物、箱子和低矮障碍由 Nav2 VoxelLayer 直接订阅
  `/local_highres_cloud_v21/sensor`，`observation_persistence=0` 且开启
  clearing；这是应当用来验收“放入/移走后是否实时更新”的图层。
- RTAB 视觉回环主要依赖 RGB 特征与关键帧匹配，不是依赖 RViz 中远处点云
  看起来有多密。点云密度影响 3D 展示和几何细节，但不是回环的唯一指标。

## V5.4：RViz 世界坐标观察与实时点云失联停车（2026-07-23）

### 六份 22 点实测日志结论

- `22-19-01` 在 `22:20:14` 记录到一次真实的 Cartographer 位姿跳变：
  平移仅 `0.004 m`，但单帧偏航变化达到 `+5.06°`。
- 该次运行中下位机上行的 `yaw/vx/vz` 从开始到结束均为零。如果实车当时确实在移动，
  说明 0x07 上行里程被冻结；Cartographer 同时收到“静止里程”和“变化激光”会造成双墙、
  墙体加厚或错误转角。上位机不能凭空恢复缺失的真实运动量。
- 2D 雷达在异常前后仍约为 `6 Hz`、设备时延约 `3~8 ms`、正常运行段丢帧为零，
  因而最后两次异常不属于雷达带宽不足。
- STEP10V2.1 实时点云通常约 `14 Hz`，处理约 `9~12 ms`，输入帧龄约 `117~126 ms`，
  偶发输出间隔约 `0.20~0.33 s`。OctoMap 方块和 RTAB 历史点云的显示延迟不能代表
  Nav2 实时避障延迟。
- 地面外参标定已通过：平面内点 `89.2%`、RMSE `3.2 mm`，并成功自动写入 roll、pitch、z。
  地面无法标定 yaw；重启建图后仍需执行 `CALIBRATE_CAMERA_YAW.sh` 完成相机与雷达水平对齐。

### 本次代码修改

- `dual_resolution_3d_slam.rviz` 的观察目标由 `base_link` 改为 `map`。现在 RViz 中墙体固定、
  车辆移动，不再因相机视角跟随车辆而产生“车不动、墙在动”的显示错觉。
- 双分辨率导航启用 `/local_highres_cloud_v21/sensor` 心跳看门狗，超时阈值 `0.35 s`。
  实时点云断流时，`safety_fusion_node` 立即输出零速并显示
  `local_cloud_timeout_lock`，不会继续依赖陈旧 VoxelLayer 障碍物行驶。
- 看门狗只在双分辨率导航入口启用；纯 2D 建图和定版 Cartographer V9 参数完全不变。
- Cartographer V9 配置 SHA256 仍为
  `00DFD1C721F0FE8C61AC6F2B417001920694E4FC77E895FB4A1F194330C910D9`。

### 下一次实车验证要求

1. 重启建图后，对准一面同时能被相机和雷达看到的大平墙，执行
   `./CALIBRATE_CAMERA_YAW.sh`。
2. 驱动车辆前进和原地转动时，确认终端 `[NAVI]` 的 `vx`、`yaw`、`vz` 不能持续全为零。
3. RViz 的 `Fixed Frame` 和 `Current View / Target Frame` 均应为 `map`。
4. 导航时临时遮断相机数据超过 `0.35 s`，状态应出现
   `local_cloud_timeout_lock` 且发送零速；恢复点云后才允许继续。

## V5.5：外参标定误写保护与 07-24 三次日志结论（2026-07-24）

### 三次日志结论

- 第一次在 `CALIBRATE_CAMERA_YAW.sh` 真正启动前约 `0.54 s`，整套 mapping
  launch 已收到 `SIGINT/SIGTERM`。Cartographer 是正常收尾，不是算法崩溃。
  标定程序是独立 ROS 进程，不会主动关闭主 launch；最常见原因是在主终端按了
  `Ctrl+C`、复用了主终端或终端进程组被一起中断。
- 第二次地面结果为 `roll=-0.749 deg`、`pitch=25.772 deg`、
  `z=0.3666 m`；YAW 工具随后写入 `-8.197 deg`。
- 第三次启动确实加载了 `-8.197 deg` 相机 yaw，但 2D 主定位同时发生两次
  `CARTOGRAPHER_POSE_JUMP`：分别约 `-20.08 deg` 和 `-6.88 deg`。
  这类 2D 地图/车体共同跳变不是相机外参造成的。
- 三次日志持续出现下位机绝对 yaw 的不可能单帧阶跃，典型值为
  `+/-5 deg / 20 ms`，上位机保护已将其丢弃。第三次总计两次 Cartographer
  跳变，后续实车仍需同时核对 0x07 的 yaw 与 `vz`。

### 标定安全修改

- 两个标定入口用独立 `setsid` 进程组执行，并明确要求在第二终端运行，降低
  终端信号误伤 mapping launch 的风险。
- 地面标定新增法向与相机高度检查。即使某个大墙面拟合内点很多，也不能再被
  当成地面写入。
- 新增 `CAMERA_GROUND_CALIBRATED` 状态。未成功完成本轮地面标定时，YAW
  脚本拒绝执行。
- YAW 标定新增同墙距离一致性、前后两半数据稳定性和单次最大 `15 deg`
  修正限制，避免相机与雷达各自拟合到不同墙面后仍自动写入。
- YAW 完成后不需要再次运行地面标定。正确顺序只有：
  `启动 -> 地面标定 -> 停止并重启 -> YAW 标定 -> 停止并重启 -> 验证`。
- Cartographer V9 参数和 STM32 工程均未修改。

## V5.6：07-24 三次复测后的定位与三维地图链路修正

### 日志结论

1. 第一次测试的地面外参虽然已经写入 `dual_resolution_3d.env`，但运行中的
   `base_link -> camera_link` 是静态 TF，不会热更新。该次继续行驶及执行 YAW
   标定时仍使用旧的 `pitch=25.04 deg, yaw=0 deg`，所以这段数据不能用于比较
   视觉墙和 2D 雷达墙。
2. 第二、三次重启后才加载新外参，分别约为
   `roll=0.028 deg, pitch=25.834 deg, yaw=-6.986 deg`。
3. 三次日志均存在下位机绝对 yaw 的物理不可能跳变。第三次更换 STM32 程序后
   仍出现 `-10/-20/-40.32 deg / 20 ms`，累计触发 96 次保护。该问题与相机
   外参无关，也是转弯时 2D SLAM 漂移的直接风险源。
4. 原独立 `octomap_server` 把实时点云按当时的 `map` 位姿永久累计。Cartographer
   后续修正 `map->odom` 时，旧方块无法重新排列，因此会产生墙体加厚、视觉墙
   跑到当前 2D 墙外和历史残影。这不是实时避障点云延迟。

### 代码修正

- 标定成功后新增
  `visual_laser_slam/.camera_calibration_restart_required` 标记。
- 地面标定写入后，未重启建图时 `CALIBRATE_CAMERA_YAW.sh` 会直接拒绝执行。
  新建图进程读取最新 env 后自动确认并清除标记；YAW 写入后再次产生标记，
  强制最后一次重启。
- 绝对 yaw 改为“`vz/gz` 短时预测 + 绝对角创新门限”：
  - 正常绝对 yaw 继续作为长期航向；
  - 不可能跳变被隔离；
  - 隔离期间只按受限角速度预测；
  - 异常值缓慢返回时不会再被逐帧注入里程计。
- 删除双分辨率主链路中的原始点云累计 `global_cloud_relay + octomap_server`。
- RViz 方块地图改为 RTAB-Map 根据优化后关键帧位姿生成的
  `/rtabmap_3d/octomap_occupied_space`。它可以随图优化重新排列历史节点。
- 最终 `.bt` 保存改为读取 `/rtabmap_3d/octomap_binary`。
- RTAB 按需管理同时监控彩色 MapCloud 和优化 OctoMap；两个显示都关闭时才允许
  暂停。Nav2 仍直接使用 `/local_highres_cloud_v21/sensor` 做实时碰撞避障。
- 定版 Cartographer 文件未改，SHA256 仍为
  `00DFD1C721F0FE8C61AC6F2B417001920694E4FC77E895FB4A1F194330C910D9`。
- STM32 工程未改。

### 下一次实车验证（必须重新开始）

```bash
# 终端 1
./START_DUAL_2D_3D_MAPPING.sh

# 终端 2：平整、哑光、宽敞地面
./CALIBRATE_CAMERA_EXTRINSIC.sh

# 停止终端 1，再重新启动
./START_DUAL_2D_3D_MAPPING.sh

# 终端 2：同一面平墙同时进入相机和 2D 雷达视野
./CALIBRATE_CAMERA_YAW.sh

# 再停止并重启终端 1，然后才开始正式建图
```

验证时先清理本轮测试数据库，低速完成：静止 10 秒、直行 2 m、左右各原地
转 90 度、回到起点。重点检查：

```bash
grep -E "NAVI_YAW_RATE_LIMIT|CARTOGRAPHER_POSE_JUMP" SLAM_Log/dual_3d_*/runtime.log
ros2 topic hz /local_highres_cloud_v21/sensor
ros2 topic hz /rtabmap_3d/octomap_occupied_space
```

只要 `NAVI_YAW_RATE_LIMIT` 在正常转弯中连续增长，就说明 0x07 的 yaw 与 `vz/gz`
仍不连续；上位机现在会隔离它，但下位机数据源仍需继续排查。正式判断外参时，
只比较重启后当前彩色点云、优化 OctoMap 与 2D 墙，不能使用标定写入后未重启的
同一进程数据。

## V5.7：0x07 上行接收链复核与异常原始帧取证

- 重新检查 `chassis_node.py` 的串口接收链：
  - 每 5 ms 读取串口缓冲区；
  - 按 `AA 55` 帧头同步；
  - 仅接受长度为 20 字节、`cmd=0x07` 且累加校验正确的 NAVI 帧；
  - `yaw/vx/vz` 按小端有符号 `int32` 解码；
  - `tick_ms` 按小端无符号 `uint32` 解码；
  - 单个 ROS 回调内完成读取和拆帧，没有并发线程同时修改接收缓冲区。
- 当前协议解码与 `ReadMe.md` 中的 0x07 定义一致，没有发现固定字节偏移或大小端
  错误。
- 旧运行日志只记录了解码结果，没有保存触发航向异常的完整 20 字节帧，因此不能
  仅凭旧日志最终判定是 STM32 写入异常、USB 串口数据异常还是上位机误拆帧。
- `NAVI_YAW_RATE_LIMIT` 现在额外打印：
  - 完整 20 字节十六进制原始帧；
  - `yaw_raw/vx_raw/vz_raw/tick_raw`；
  - 接收校验字节和重新计算的校验值。
- 下一次出现异常后直接检查：

```bash
grep "NAVI_YAW_RATE_LIMIT" SLAM_Log/dual_3d_*/runtime.log
```

如果原始 hex 中 `yaw_raw` 已经跳变且校验正确，说明上位机确实收到了一帧内容完整
但 yaw 数值异常的数据，应继续检查 STM32 生成 0x07 帧时的共享变量原子性、任务
并发和发送缓冲生命周期。如果原始 hex 连续而日志解码值不连续，才属于上位机解析
问题。

当前航向策略不是完全丢弃下位机 yaw，而是：

```text
正常绝对 yaw -----------------------> 长期航向参考
vz/gz × dt --------------------------> 两帧之间的短时角度预测
绝对 yaw 超过物理速率/创新门限 ----> 拒绝该帧角度，仅采用 vz/gz 预测
2D 激光 scan matching + 回环 --------> map 层长期位姿修正
```

因此正常下位机绝对角仍会使用，异常帧不会直接改变 `odom -> base_link`，激光 SLAM
继续负责全局纠偏。

外参复核时必须先关闭历史显示，只比较
`STEP10V2.1 Local High Resolution` 与 `Filtered 2D LiDAR`：

- 两条墙线存在夹角：继续检查 `CAMERA_YAW_DEG`；
- 墙线平行但视觉距离按比例偏大/偏小：检查 Gemini2 深度尺度和 D2C 注册；
- 墙线平行且始终是固定平移：检查相机 `CAMERA_X/Y` 安装尺寸；
- 当前点云对齐、只有历史方块错位：检查 RTAB 图优化/历史地图，不再改外参。

地面 EXTRINSIC 只标定 roll、pitch、z，墙面 YAW 只标定水平旋转，两者都不会自动
估计相机 x/y，也不会校正深度传感器的比例误差。

## V5.8：07-24 下午标定入口与 OctoMap 显示修复

### 日志结论

- `14-27-18` 日志中 `/camera/depth/image_raw` 实际持续工作，STEP10V2.1
  深度点云约为 12～15 Hz，RTAB-Map 也持续接收 RGB-D。终端显示
  `Missing /camera/depth/image_raw` 是标定脚本依赖 ROS CLI 话题列表造成的
  假阴性，不是相机没有启动。
- `14-30-49` 后半段同时出现深度输入停止以及 `/dev/ttyACM0` 消失。该段属于
  USB 设备掉线，和前面的 CLI 假阴性是两个独立问题；发生此现象时应先检查
  USB 供电、线缆、接头和设备枚举，不能继续做外参标定。
- RViz 中原来的独立 `octomap_server` 已在 V5.6 被替换。当前方块来自
  `/rtabmap_3d/octomap_occupied_space`，能够按 RTAB-Map 优化后的关键帧位姿
  重排历史体素。截图里的 `RTAB-Map Optimized ...` 就是该占用图，并未被删除。

### 代码修改

- `CALIBRATE_CAMERA_EXTRINSIC.sh` 不再用
  `ros2 topic list --no-daemon` 阻断标定。它直接启动订阅节点，以收到的真实
  Depth、CameraInfo 和 TF 作为有效性依据。
- `camera_ground_calibrator.py` 增加 15 秒输入看门狗。失败时会分别打印
  `depth_messages`、`camera_info_messages` 和 `accepted_frames`，可直接区分
  ROS 图发现、CameraInfo 缺失以及地面有效点不足。
- RViz 显示名称改为 `OctoMap 3D Occupied (RTAB-Map Optimized)`，让左侧列表
  即使宽度较窄也能直接看到 OctoMap 字样。
- Cartographer V13 配置、二维 SLAM 参数和 STM32 工程均未修改。

## V5.9：07-24 15:37～15:50 三维显示与导航链修复

### 三次日志结论

- `15-43-52` 已正确加载本次标定值，二维漂移没有复现。RTAB-Map 数据库从空库
  增长到约 195 MB，说明彩色持久图确实在生成。
- 当前持久体素曾错误使用 `Grid/CellSize=0.08 m`，而原定架构是持久地图
  `0.05 m`、实时避障 `0.03 m`。本次将持久地图的实际分辨率恢复为 5 cm，
  RViz 同样按 5 cm 绘制，避免仅修改显示尺寸掩盖配置错误。
- STEP10V2.1 是约 15 Hz 的当前帧点云，RTAB-Map OctoMap 是按关键帧和图优化更新
  的长期地图。车辆未达到 5 cm / 0.05 rad 关键帧阈值时，当前彩色点云区域没有新增
  持久方块属于预期行为，不能把持久 OctoMap 当实时避障输入。
- `15-50-06` 中 Nav2、MPPI、Smac Lattice 和 costmap 均启动，`/map` 也被
  global costmap 接收并调整到 `128 x 203`；“没有二维地图”不是 Cartographer
  未发布，而是旧 RTAB 三维数据库与本次新二维坐标原点混合显示造成的遮挡/错位。
- 自动探索节点因命令行出现两次 `--ros-args`，并传入 Jazzy 不支持的
  `--log-level none` 而立即崩溃。
- RViz 通用 `2D Goal Pose` 发布 `/goal_pose`，原桥接节点只订阅
  `/web/nav_goal`，所以 RViz 标点不会调用 Nav2 action。
- 三维启动没有运行旧的 `depth_obstacle_node`，因此旧 baseline 永远不会 ready，
  原 chassis 的 PS2 归还检查会永久拒绝手柄。三维导航实际使用的是 STEP10V2.1
  点云、Nav2 VoxelLayer 和点云断流锁，不能再依赖一个未启动的旧 baseline 节点。

### 代码修改

- frontier launch 改为 `ros_arguments=["--log-level", ...]`，由 launch_ros
  统一添加一次 `--ros-args`；导航启动后还会检查 `/control_exploration` 服务，
  失败时直接给出明确错误。
- `web_goal_nav_node.py` 同时订阅 `/web/nav_goal` 与 `/goal_pose`，两种入口都
  转成 `/navigate_to_pose` action，并在日志中标明目标来自 `web` 或 `rviz`。
- 三维导航脚本改用独立的 `rtabmap_nav_live.db`，每次启动先归档上一导航会话并
  创建新库，禁止“旧三维数据库 + 新二维 Cartographer 地图”混用。普通建图脚本
  继续使用原来的持久数据库。
- chassis 新增 `require_depth_baseline_for_ps2`，默认仍为 `true`，所以原二维
  系统的 baseline 安全逻辑不变；只有三维导航入口将它设为 `false`，改由实时
  点云断流锁和 Nav2 局部 VoxelLayer 保护，网页归还 PS2 可以正常执行。
- RViz 将 STEP10V2.1 改为 `Live Local 3D Voxels (15 Hz, Navigation Input)`，
  使用 3 cm Boxes 实时显示；RTAB-Map `Grid/CellSize` 与持久 OctoMap 显示统一
  恢复为 5 cm。
- 数据库配置签名新增 `global_3d_voxel`。首次使用 5 cm 配置时会自动归档旧的
  8 cm 数据库并创建新库，避免两种体素分辨率混在同一持久地图中。
- 启动自检增加 `/local_highres_cloud_v21/sensor`、`/navigate_to_pose` action
  和 `/control_exploration` service，任何一段缺失都不会再假装“导航已就绪”。
- safety 日志新增 `local_cloud_alive` 及三个 `require_*` 实际参数，下一次可直接
  区分“没有导航速度”“点云断流”“baseline 锁”。
- 修复 goal/path/frontier 三个 Python 节点在 ROS context 已关闭后重复
  `rclpy.shutdown()` 的退出报错。

### 未修改范围

- `cartographer_2d_v9_tightened.lua`、二维 SLAM 参数、雷达方向和 STM32 工程
  均未修改。
- 本次只修复显示尺寸和导航控制链，没有用视觉点云反向修改二维位姿。

## V6.0：07-24 19:05 Nav2 BT Navigator 崩溃修复

### 日志结论

- 六次测试中只有 `19-05-28` 是导航启动会话。二维地图、RGB-D、STEP10V2.1
  实时点云和 Nav2 costmap 都能启动。
- safety 日志持续显示 `local_cloud_alive=True`、`require_cloud=True`，说明导航不能
  动不是点云断流锁或 baseline 锁造成的。
- `bt_navigator` 配置阶段明确报错：

```text
Could not load library: libshort_goal_behind_bt_node.so:
cannot open shared object file: No such file or directory
```

- 自定义插件源码位于 `lidar/chapt1_ws/src/short_goal_bt`，但 STEP11 隔离工作区原来
  只构建 `lidar_py`、`local_depth_cloud_cpp` 和 `frontier_exploration_ros2`，遗漏了
  `short_goal_bt`。配置失败后 Jazzy 的 `bt_navigator` 在清理
  `BehaviorTreeEngine::resetRootMonitor()` 时又触发空指针 SIGSEGV；系统弹窗是这个
  二次崩溃，不是显卡、RTAB-Map 或底盘问题。

### 代码修改

- 三维导航构建清单加入 `short_goal_bt`，与 frontier、lidar_py 一起安装到同一个
  隔离工作区，确保动态链接器能够找到自定义 BT 插件。
- source 隔离工作区后同时检查 ROS 包和
  `lib/libshort_goal_behind_bt_node.so`。插件缺失时启动脚本会立即给出明确错误并退出，
  不再进入会触发 SIGSEGV 的 Nav2 生命周期配置。
- Cartographer V13、标定值、二维地图参数、MPPI 参数和 STM32 均未修改。

## V6.1：07-24 20:39 无网页导航与 PS2 控制链修复

### 日志结论

- 本次不是旧深度 baseline 锁车。`safety_fusion_node` 明确记录
  `require_baseline=False`，实时点云稳定后也持续显示
  `local_cloud_alive=True`、`source=none`、`nav=(0,0)`。
- Nav2 在启动阶段加载了系统自带的
  `navigate_to_pose_w_replanning_and_recovery.xml`，但当前行为服务器没有注册
  `backup` 动作，因此报错：

```text
"backup" action server not available after waiting for 1.00s
Exception when loading BT: Action server backup not available
Failed to bring up all requested nodes. Aborting bringup.
```

- `bt_navigator` 没有进入 Active，RViz 发布目标后自然不会产生
  `/cmd_vel_nav`。这才是 RViz 选点导航不能动的直接原因。
- PS2 不能动是另一条独立问题：底盘节点启动时发送
  `STARTUP_MOVE_TAKEOVER`，无网页可用时没有入口再发送 `PS2` 归还控制权。

### 代码修改

- 三维导航不再强制覆盖成 Jazzy 系统恢复树，恢复使用项目自己的
  `navigate_to_pose_jazzy.xml` 和 `navigate_through_poses_jazzy.xml`。两棵树均不依赖
  未注册的 `backup` 动作，并保留 Smac、MPPI、路径重规划和清图/旋转/等待恢复。
- 三维导航模式启用 `auto_nav_ps2_handoff`：
  - 没有活动导航任务时，下发 `PS2` 并把控制权交给手柄；
  - `/navigate_to_pose` 进入 Accepted/Executing/Canceling 时，自动下发零速
    `MOVE` 接管，然后允许 `/cmd_vel_safe` 驱动车辆；
  - 任务结束或取消后等待 0.75 秒，再自动归还 PS2；
  - 导航任务执行期间即使 MPPI 因障碍短暂停车，也不会错误归还 PS2。
- 三维导航仍设置 `require_depth_baseline_for_ps2=false`。旧网页 baseline 逻辑没有
  删除，只在其他旧启动方式中保持兼容；当前无网页导航不依赖它。

### Ubuntu 验证标志

启动 `START_DUAL_2D_3D_NAVIGATION.sh` 后应依次看到：

```text
STARTUP_PS2_RELEASE
Nav2 goal active: MOVE takeover enabled
Nav2 idle: control returned to PS2
```

同时不能再出现：

```text
Action server backup not available
Failed to bring up all requested nodes
```

本次未修改 Cartographer V13、MPPI 参数、代价地图参数、相机标定、雷达方向和
STM32 协议。

## V6.2：07-24 21:52 路径存在但控制器不输出修复

### 日志结论

- RViz 目标、`/navigate_to_pose`、Smac State Lattice 和全局路径均已正常工作。
- 车辆不动的直接原因是行为树在预旋转安全检查失败后选择了旧控制器
  `FollowPathNoShim`，而三维导航 MPPI override 明确只注册 `FollowPath`：

```text
Pre-rotation is unsafe or failed; skipping Spin and selecting controller 'FollowPathNoShim'
FollowPath called with controller name FollowPathNoShim, which does not exist.
Available controllers are: FollowPath.
```

- controller server 因此在跟踪路径前立即拒绝任务，日志中的 `/cmd_vel_nav` 和
  `/cmd_vel_safe` 始终为零。这不是 baseline、点云看门狗、串口或 STM32 锁车。

### 代码修改

- `navigate_to_pose_jazzy.xml` 的“不允许原地预旋转”回退路径改为继续使用唯一的
  MPPI 控制器 `FollowPath`。
- 删除仅服务于旧 DWB `FollowPathNoShim` 的倒车后重新对准分支，避免行为树再次
  请求不存在的控制器。正常 MPPI 跟踪、全局路径有效性检查和恢复动作保留。
- frontier 自动探索启动时显式使用 Jazzy 支持的 `warn` 日志级别，修复
  `Couldn't parse log level: '--log-level none'` 导致的节点退出。

### 转弯半径复核

- 当前 State Lattice 明确使用差速模型 `motion_model=diff`。
- 格点包含 64 组原地转向 primitive（半径 0）和 64 组行驶圆弧；圆弧设计半径约
  `0.50 m`，地图分辨率为 `0.05 m`。
- 对 `0.665 x 0.665 m` 方形车体，半对角线约 `0.47 m`，`0.50 m` 行驶圆弧属于
  保守但合理的初始值。截图中的弧线不是汽车式“不能原地转”，而是规划器在附近
  障碍导致整车原地旋转不安全时选择的行驶转向轨迹。
- 本轮不缩小圆弧半径。应先验证车辆实际开始跟踪路径；只有确认空旷区域仍明显
  绕大圈，再生成较小半径的新 lattice，不能只修改 YAML 数字而不重建 primitive。

本次仍未修改 Cartographer V13、MPPI 速度、代价地图、相机标定和 STM32 协议。

## V6.3：四轮滑移转向实测半径补偿

### 实测与反算

- 原二档理想模型目标半径约 `0.85 m`，实车测得约 `1.60 m`，欠转向倍率约为：

```text
1.60 / 0.85 = 1.882
```

- 在轮胎、地面和载荷近似不变的第一轮补偿中，要得到约 `0.85 m` 的实际半径，
  规划/控制侧应请求约：

```text
0.85 / 1.882 = 0.452 m
```

### 新导航参数

- 使用 Nav2 Jazzy 官方 `nav2_smac_planner/lattice_primitives` 生成器新增
  `lattice_diff_slip_compensated_45cm_5cm.json`，不是手工缩放旧 JSON。
- 新 lattice 参数：
  - `motion_model=diff`
  - 最小指令半径 `0.45 m`
  - 分辨率 `0.05 m`
  - 16 个朝向
  - 112 条官方生成的运动原语
  - 非零圆弧半径范围约 `0.452～0.689 m`
  - 保留半径为零的原地转向原语
- 三维导航默认切换到新 lattice；旧 `0.50 m` 文件保留作对照，不再由
  `START_DUAL_2D_3D_NAVIGATION.sh` 默认加载。
- MPPI `wz_max` 从 `0.209` 调整为 `0.218 rad/s`（约 `12.5°/s`），直行上限仍为
  `0.20 m/s`，圆弧外侧轮上限仍为 `0.16 m/s`。
- 在最紧圆弧附近，理论左右侧速度约为：

```text
外侧轮 0.160 m/s
内侧轮 0.037 m/s
```

  内侧轮保持正转，不使用单侧轮停转的硬拐方式。

### 复测标准

- 在空旷、干燥、摩擦一致的地面进行连续 90° 圆弧测试，不用原地转向测试代替。
- 从车体中心轨迹测半径，不从外侧轮或车壳外缘测量。
- 目标实际车体中心半径先接受 `0.75～1.00 m`；若仍超过 `1.10 m`，应记录日志中的
  `/cmd_vel_nav`、`/cmd_vel_safe` 和 `/wheel_speed_sent` 后再做第二轮补偿。
- 地面、轮胎状态和载荷会改变滑移倍率，不能继续仅凭一次结果无限缩小 lattice。

本次未修改 Cartographer V13、二维 SLAM、相机标定、代价地图和 STM32 协议。

## V6.4：视觉假障碍、MPPI 低速抽搐与原地预旋转修复

### 22:50 实车日志结论

- 截图中的全局路径绕开大片空地，不是 State Lattice 转弯半径造成的。全局代价地图
  同时接收 `/local_highres_cloud_v21/sensor`，而点云中的地面起伏被
  `min_obstacle_height=0.03`、`mark_threshold=1` 标成障碍，青色路径因此绕开整片
  相机视锥区域。
- MPPI 的 20 Hz 控制循环多次降到约 9～14 Hz；原配置把同一份高密度点云同时送入
  global/local 两个 VoxelLayer，并使用 1800 组采样，计算负载过高。
- MPPI 连续输出约 `0.004～0.018 m/s` 的微小速度，低于实车底盘可执行门槛，最终
  触发 `Failed to make progress`，这就是中途极慢和抽搐的直接原因。
- 第一个目标开始后，行为树曾连续约 12 秒输出 `wz=-0.209 rad/s` 做原地预旋转。
  随后 Cartographer 约束连续出现约 `2.5～3.4°` 的角度修正。四轮差速车原地旋转
  打滑会放大 odom 航向误差，是这次二维地图再次变歪的主要触发因素。

### 导航配置修改

- 全局代价地图移除 `depth_global_voxel_layer`。全局规划继续使用 Cartographer 静态图、
  2D 雷达以及原有投影扫描；Gemini2 高分辨率点云仍保留在 local costmap，继续参与
  MPPI 实时碰撞避障，并未关闭视觉融合。
- local VoxelLayer 将 `min_obstacle_height` 从 `0.03 m` 提高到 `0.08 m`，
  `mark_threshold` 从 `1` 提高到 `2`，抑制瓷砖反光、地面起伏和孤立深度噪声。
- `expected_update_rate` 改为 `0.20 s`，匹配日志中的偶发帧延迟，避免正常的
  13～14 Hz 点云被反复误报断流；安全节点自己的点云超时停车看门狗仍然保留。
- MPPI `batch_size` 从 `1800` 降至 `1200`，预测时域仍是 `56 x 0.05 = 2.8 s`，
  优先恢复实时控制频率，不改变点云分辨率、车体 footprint 或碰撞检查。
- MPPI 改为 `vx_min=0.0`，与当前前进型 lattice 的 `allow_reverse_expansion=false`
  保持一致；新增 Jazzy 原生 `VelocityDeadbandCritic`，线速度死区 `0.04 m/s`、角速度
  死区 `0.03 rad/s`，防止控制器长时间输出底盘无法执行的毫米级速度。

### 行为树修改

- `InitialPathPreRotate` 的普通路径触发角从 `0.20 rad` 提高到 `pi/2`。小于 90° 的
  起步转向交给 MPPI 沿圆弧完成，减少四轮原地打滑；大于 90° 的反向目标和恢复 Spin
  能力仍保留。

### 未修改范围

本次未修改 `cartographer_2d_v9_tightened.lua`、Cartographer V13 参数、二维雷达方向、
相机标定、持久化 RTAB-Map/OctoMap、STM32 工程和通信协议。

## V6.5：地面标定与实时导航地面剔除链修复

### 截图判断

- OctoMap 地面方块高度不一致包含三类现象：相机 roll/pitch/height 外参误差会让整片
  地面倾斜或偏离 `z=0`；Gemini2 深度噪声会造成邻近方块上下跳动；光滑瓷砖反光或
  无效深度会形成孔洞。标定只能修正第一类，不能凭空补出相机没有测到的深度。
- 当前仓库仍标记 `CAMERA_GROUND_CALIBRATED=false`，`CAMERA_PITCH_DEG=25.04` 是根据
  加速度估算的临时值，不是地面 RANSAC 标定结果。
- STEP10V2.1 的 C++ 节点原本已有地面带过滤，但双分辨率 launch 将
  `ground_filter_enabled` 固定为 `false`，因此标定后也不会真正剔除实时避障点云中的
  地面。

### 修改

- 新增 `LOCAL_GROUND_FILTER`、`LOCAL_GROUND_Z_MIN`、`LOCAL_GROUND_Z_MAX`，默认地面带
  为 `-0.10～+0.08 m`。
- 只有 `CAMERA_GROUND_CALIBRATED=true` 时启动脚本才会打开地面剔除；尚未标定时自动
  保持关闭并给出明确警告，避免错误外参把斜墙或低矮障碍误删。
- 地面剔除只作用于 `/local_highres_cloud_v21` 及其 `/sensor` 实时碰撞点云，减少
  local VoxelLayer 和 MPPI 的假障碍及计算量；RTAB-Map/OctoMap 持久化建图输入独立，
  不会因为 RViz 隐藏地面或局部过滤而停止视觉回环。
- Nav2 V6.4 的全局/局部分层、MPPI 速度死区和原地预旋转限制保持不变。

### 正确使用顺序

1. 启动建图版，保持车辆静止。
2. 第二终端执行 `./CALIBRATE_CAMERA_EXTRINSIC.sh`；只接受自动写入成功且提示重启的结果。
3. 完全结束建图版并重新启动，使新 roll/pitch/z 生效。
4. 再执行 `./CALIBRATE_CAMERA_YAW.sh`，成功后再次完整重启。
5. 启动横幅必须显示 `Ground calibrated: true`、`Local ground filter: true`。

本次未修改 Cartographer V13、二维 SLAM、STM32、相机驱动分辨率和持久化地图分辨率。

## V6.6：2 cm 低障碍保留与微小地面噪点阈值

- 用户确认 `8 cm` 盲区过大，实际环境中存在低矮障碍，因此取消 V6.4 的 8 cm
  障碍高度门槛。
- 标定成功后的地面剔除上界从 `+0.08 m` 改为 `+0.02 m`；只消除地面平面及 2 cm
  以内的深度残余。
- local VoxelLayer 的 `min_obstacle_height` 从 `0.08 m` 改为 `0.02 m`，
  `mark_threshold` 从 `2` 恢复为 `1`，确保高度超过 2 cm 的真实物体可以进入 MPPI
  footprint 碰撞检查。
- 小于约 `2 x 2 cm` 的孤立噪点由 STEP10V2.1 已有的空间邻域过滤和体素邻居过滤
  删除。当前点云体素为 `3 cm`，因此这是保实时性的近似实现，不能声称具有 1 cm
  级精确连通域测量能力。
- 不把全链路体素强行改成 `1 cm`：这会显著增加点数、VoxelLayer 更新和 MPPI 控制
  负载，并可能重新造成控制循环低于 20 Hz。

本次未修改 Cartographer V13、二维 SLAM、STM32、RTAB-Map 回环和相机标定值。

## V6.7：07-25 实车导航、实时避障与退出链路修复

### 日志结论

- 首个导航目标连续约 12 秒输出 `wz=-0.209 rad/s` 做原地 Spin，并以超时结束；后续恢复树又多次执行原地 Spin。四轮滑移底盘在原地转向时打滑，会放大轮速里程计误差。
- `00:14:04` Cartographer 完成一批约束后产生一次 `-5.52 deg` 全局航向修正；后续约束出现约 `0.14 rad` 的旋转差。导航原地打滑是本次地图变歪的重要触发因素，但没有证据要求改动已经定版的 V13 参数。
- MPPI 目标频率为 20 Hz，实测多次降到 `3.38~5 Hz`；这会表现为龟速、停顿和 `Failed to make progress`。RTX 3060 不会自动加速 Nav2 的 CPU MPPI 采样。
- 持久 OctoMap 是长期记忆地图，不保证人物离开后立刻删除；实时碰撞依据应查看 `/local_costmap/costmap` 和 STEP10V2.1 点云。旧 RViz 默认显示持久方块，容易把它误认为实时避障层。
- 退出保存调用的 `/rtabmap_3d/octomap_binary` 并不存在，因此 `octomap_saver_node` 必然超时。长期彩色三维图实际持续保存在 RTAB-Map `.db`。

### 修改

- `navigate_to_pose_jazzy.xml` 删除起步预旋转和恢复 Spin；`navigate_through_poses_jazzy.xml` 同步删除恢复 Spin。单目标和多目标导航都只允许 MPPI 沿路径做运动圆弧，恢复动作改为清代价地图和短暂等待，避免连续原地打滑。
- 三维导航全局规划器由差速 lattice 切换为 `SmacPlanner2D`。差速底盘不再被 45 cm lattice 圆弧强制绕行；Smac 2D 生成更直接的全局路径，MPPI 负责局部平滑和动态避障。
- MPPI 控制频率改为 15 Hz，`batch_size` 从 1200 降为 800，预测时域仍为 `56 x 0.05 = 2.8 s`。导航版 RTAB-Map 检测率单独降为 1 Hz，建图版仍保持 2 Hz。
- VoxelLayer 的 `expected_update_rate` 设为 0，停止把实测 0.25~0.39 秒偶发间隔误报为持续断流；`0.35 s` 点云看门狗仍会在真正断流时停车。
- `safety_fusion_node` 新增独立实时点云碰撞门：读取 base_link 下的滤波点云，在车前 `x=0.20~0.58 m`、`|y|<=0.38 m`、`z=0.02~1.40 m` 内至少 6 个点时立即停止前进。人物离开并收到新点云后自动解除，且不依赖持久 OctoMap 清除。
- RViz 默认关闭持久 OctoMap 方块，新增并默认显示 `REAL-TIME Nav2 Local Costmap`。长期地图仍可手动勾选查看，但不能作为动态人物是否已清除的判断依据。
- 默认关闭无效的外部 `.bt` 保存；RTAB-Map `.db` 继续自动持久化。Python 节点统一处理 `ExternalShutdownException`，仅在 ROS context 有效时调用 shutdown/发布，减少 Ctrl+C 的重复 shutdown 报错。

### 复测重点

1. 使用 `START_DUAL_2D_3D_NAVIGATION.sh`，确认日志显示 `SmacPlanner2D`、MPPI `batch size 800`，且不再出现 `Spin safety clear` 或 `Exceeded time allowance before reaching the Spin goal`。
2. 在直走目标前放置纸箱，观察 `REAL-TIME Nav2 Local Costmap` 出现障碍并停车；移走后应在新点云到达后清除并恢复。日志应出现 `local_cloud_collision_lock`，不应穿越障碍。
3. 持久 OctoMap 默认不显示。需要检查三维长期地图时再勾选，它保留曾经出现的人属于长期地图语义，不代表 local costmap 仍阻挡。
4. 记录控制循环是否仍低于 10 Hz，以及是否仍出现大于 5 度的 `CARTOGRAPHER_POSE_JUMP`。若不再原地 Spin 后仍发生跳变，再单独分析二维 SLAM，不提前改 V13。

本次未修改 Cartographer V13、二维雷达方向、STM32、相机标定值、2 cm 地面过滤阈值和 RTAB-Map 回环策略。

## V6.8：C++动态避障、STVL、视觉里程计EKF与可观测视觉回环

### 最终分层

```text
STM32原始/odom绝对yaw+vx ─┐
RGB-D视觉里程计vx/vy ─────┼─> robot_localization EKF ─> odom -> base_link
                           │                            │
2D LiDAR + /imu_cartographer + /odometry/filtered ────┴─> Cartographer
                                                        └─> map -> odom

RGB + registered depth + Cartographer位姿 ─> RTAB-Map长期3D图和内部视觉回环

STEP10V2.1实时点云 ─┬─> C++ STVL 1秒衰减 ─> local costmap ─> MPPI
                    └─> C++近碰撞门 ─> safety_fusion ─> /cmd_vel_safe
```

### C++碰撞门

- 新增 `local_cloud_collision_gate_node.cpp`，逐点碰撞检查从 Python 迁移到 C++。
- 输入 `/local_highres_cloud_v21`，检测车前 `x=0.20~0.58 m`、`|y|<=0.38 m`、`z=0.02~1.40 m`；至少 6 点触发，保持 0.25 秒。
- 输出 `/local_cloud_collision_stop` 和 `/local_cloud_collision_status`。Python `safety_fusion_node`只订阅结果并仲裁速度，不再解析 PointCloud2。
- 点云超过 0.35 秒断流时，原有点云看门狗仍独立停车。

### STVL动态层

- 新增 `nav2_dual_3d_stvl_override.yaml`，只替换 Nav2 local costmap 的旧 VoxelLayer。
- 体素 3 cm、线性衰减 1秒，同时使用深度相机视锥清除；人物或移动箱子离开后不再依赖长期 OctoMap删除。
- RTAB-Map长期3D图和 Cartographer二维全局图不接收 STVL，不会被动态衰减破坏。
- `ENABLE_STVL=true`为默认；通过 `DUAL_3D_ENABLE_STVL=false`可回退旧 VoxelLayer做对照。

### 可切换视觉EKF

- 稳定脚本 `START_DUAL_2D_3D_NAVIGATION.sh`不启用视觉EKF，继续由底盘发布 `odom -> base_link`。
- 新增 `START_DUAL_2D_3D_NAVIGATION_VISUAL_FUSION.sh`：
  - `rgbd_odometry`发布 `/visual_odom`，但 `publish_tf=false`；
  - 底盘仍发布原始 `/odom`消息，但 `publish_tf=false`；
  - `robot_localization`独占 `odom -> base_link`，输出 `/odometry/filtered`；
  - Cartographer只读取 `/odometry/filtered`。
  - RTAB-Map也读取连续的 `/odometry/filtered`，不再接收可能因二维回环发生离散修正的 `/cartographer_pose_odom`；RTAB-Map用自己的视觉回环优化长期3D图。
- EKF融合视觉里程计的车体坐标 `vx/vy`，STM32提供绝对 yaw和前向 `vx`。视觉速度与轮速共同估计打滑后的真实平移；不直接融合视觉绝对 `x/y`，避免车辆以非零绝对 yaw启动时发生坐标轴错位。第一版也故意不融合视觉 yaw，不重复融合相关的 `/imu_cartographer` gz，防止旧版螺旋漂移再次出现。
- 视觉丢失时 `publish_null_when_lost=true`，EKF依靠 STM32速度和绝对 yaw继续短时预测；视觉位姿带异常拒绝门限。

### 视觉回环

- RTAB-Map继续 `publish_tf=false`，视觉回环只优化其长期彩色3D图，不与 Cartographer争抢 `map -> odom`。
- 视觉融合脚本使用独立持久数据库 `rtabmap_nav_visual_fusion.db`，不在每次启动时重置，允许跨会话识别旧场景。
- 新增 `rtabmap_loop_monitor`，日志明确输出：

```text
VISUAL_LOOP_ACCEPTED
VISUAL_PROXIMITY_ACCEPTED
VISUAL_LOOP_STATUS global=... proximity=...
```

- 本版没有把视觉回环跳变直接送入 EKF。EKF负责连续局部里程计，RTAB-Map负责内部全局3D图优化，避免离散回环修正破坏连续 `odom`。

### 安装和测试顺序

```bash
cd ~/视频/huichuan-agv-ros2-foxy-visual-lidar-slam-step/huichuan-agv-ros2-foxy-main
chmod +x STEP0_INSTALL_VISUAL_SLAM_DEPS.sh START_DUAL_2D_3D_NAVIGATION*.sh
./STEP0_INSTALL_VISUAL_SLAM_DEPS.sh
```

1. 先执行 `./START_DUAL_2D_3D_NAVIGATION.sh`，验证 STVL、C++碰撞门和原有 Cartographer稳定。
2. 在直行路径放置纸箱：`COLLISION_GATE blocked=true`时必须停车；移走后 local costmap约1秒内清除。
3. 稳定版通过后再执行 `./START_DUAL_2D_3D_NAVIGATION_VISUAL_FUSION.sh`，第一次只做低速直行和缓慢圆弧，不做原地转向。
4. 确认 `/visual_odom`、`/odometry/filtered`持续发布，TF树中只有一个 `odom -> base_link`。
5. 第二次经过第一次看过的纹理场景，检查是否出现 `VISUAL_LOOP_ACCEPTED`；没有该日志不能声称实测已成功回环。

本次仍未修改 Cartographer V13、STM32协议、相机标定值和二维雷达方向。

### V6.8.1：Jazzy C++整型参数编译修复

- Ubuntu 24.04 / ROS 2 Jazzy 的 `rclcpp::Node::declare_parameter`整型参数返回 `int64_t`。
- C++碰撞门原代码把该返回值与 `int`常量直接传入 `std::max`，导致 `min_points`和 `sample_stride`两处模板类型推导失败。
- 两处参数现统一以 `int64_t`声明并显式转换为内部 `int`，不改变参数值和碰撞逻辑。

## V6.9：二维雷达整圈误丢弃与 Jazzy MPPI 启动修复

### 2026-07-25 三次日志结论

- 三次运行都不是相机或 RTAB-Map 首先失效。LD14P 串口正常打开，每圈稳定产生约 `629~632` 个原始点，但固定栅格构建器仍使用旧的 `300~480` 点合法窗口，因此所有整圈均被记录为 `Dropped malformed fixed-grid revolution`。
- `/scan_timed_v2`没有任何有效帧后，Cartographer持续报告 `Queue waiting for data: (0, scan)`，无法建立 `map -> base_link`，二维地图自然为空。
- STEP10V2.1 实时视觉点云实际以约 `14~15 Hz`正常生成；但 RViz Fixed Frame 为 `map`，缺少 map TF 时不能把 base_link 下的点云变换到画面中，所以看起来只剩不依赖 map TF 的 RGB。
- RTAB-Map等待的 `/cartographer_pose_odom`同样依赖 Cartographer位姿，因此其 `Did not receive data since 5 seconds`是上游二维雷达断链的连锁结果，不是本次先调 RTAB-Map 参数能解决的问题。

### 修复

- 固定栅格整圈合法范围由 `300~480`放宽为 `300~720` 点。
- 最大整圈时间由 `0.25 s`放宽为 `0.35 s`。日志中的真实雷达当前约 `0.26~0.27 s/圈`，仍保留最短 `0.10 s`、最少 300 点等畸形圈保护。
- `fixed_scan_grid.py`默认值、Cartographer统一 launch、旧视觉融合 launch 三处同步，避免不同启动入口再次加载旧阈值。
- 新增 630 点、0.27 秒整圈回归测试，验证真实低转速数据能够生成固定 360 格 LaserScan。
- Nav2控制频率为 `15 Hz`时，Jazzy MPPI要求 `model_dt`不得短于 `1/15 s`。将 `model_dt: 0.05`、`time_steps: 56`改为 `model_dt: 0.067`、`time_steps: 42`，预测时域仍约 `2.81 s`，消除 `Controller period more then model dt`启动失败。

本次没有修改 Cartographer V13 参数、二维雷达角度方向、STM32代码、相机标定值、视觉EKF融合策略或 RTAB-Map 回环参数。

### V6.9.1：完整链路复核补充

- 三份日志逐一统计后，稳定阶段每圈原始点数分别为 `601~635`、`631~633`、`630~632`，相邻整圈日志中位间隔分别为 `0.2687 s`、`0.2691 s`、`0.2693 s`。因此 `720 点 / 0.35 s`不是猜测值：它覆盖三次真实数据并保留约 13%点数余量和26%时间余量；最少300点和最短0.10秒保护保持不变。
- 确认一键脚本默认 `AUTO_BUILD=true`，每次都会把 `lidar_py`和C++点云节点重新构建到隔离缓存工作空间，不会正常情况下继续使用改动前的旧install。RViz二维雷达显示订阅 `/scan_timed_v2`，与当前Cartographer稳定入口一致。
- 原`wait_topic`只检查ROS图中是否存在话题名称，不能证明已经发布过消息。现在改为以真实Bash截止时间等待一帧数据，并使用best-effort订阅兼容传感器QoS和可靠QoS。
- 启动就绪检查新增 `/scan_timed_v2`实际数据门槛。20秒内没有完整雷达帧时直接报告 `LiDAR did not publish a valid /scan_timed_v2 revolution`；只有收到雷达帧后才继续等待真实 `/map`消息，不再把空地图发布者误判为建图成功。
- 修复Jazzy统一关闭ROS context时，`lidar_node`和`robot_pose_publisher`可能抛出 `RCLError`并以退出码1死亡的问题；只有context仍有效的真实运行异常才会继续抛出。`chassis_node`也不再在context失效后向rosout写“串口已关闭”。
- 启动横幅中的二维里程计输入改为显示真实的 `$CARTOGRAPHER_ODOM_TOPIC`，稳定版显示 `/odom`，视觉融合试验版显示 `/odometry/filtered`。

完整复核仍未发现需要修改Cartographer V13、STM32接收解析、雷达方向、相机外参或RTAB-Map参数的证据。

## V6.10：导航目标横跳与正常退出误报修复

### 三次实车日志结论

- 三次测试的固定栅格雷达均正常，`dropped=0`，Cartographer扫描频率约为`5.09~5.43 Hz`。本轮不修改已经定版的Cartographer V13、雷达方向或扫描阈值。
- 第二次和第三次测试中，一次RViz点击连续出现两次`Begin navigating`，中间紧跟`Received goal preemption request`。原因是RViz已经直接调用Nav2，`web_goal_nav_node`又订阅`/goal_pose`并重复提交同一个目标。
- 第二次车辆长时间合法原地修正方向，但旧`SimpleProgressChecker`只检查平移；30秒后误报`Failed to make progress`，恢复流程重新规划并改变转向方向，形成左右横跳。
- 第一次没有收到导航目标，约21秒后所有节点同时收到`SIGINT/SIGTERM`，属于外部统一结束，不是SLAM自行崩溃。
- 第三次虽然中途出现一次进度误报，但随后日志明确记录`Goal succeeded`；之后才收到统一退出信号，也不是导航进程自行崩溃。

### 修改

- `web_goal_nav_node`新增`bridge_rviz_goal`参数，默认并在统一launch中显式设为`false`。网页`/web/nav_goal`仍由该节点发送Nav2 action；RViz的`2D Goal Pose`只走Jazzy Nav2原生action，不再重复提交和立即抢占自己。
- Nav2进度检查器改为Jazzy的`nav2_controller::PoseProgressChecker`：平移`5 cm`或旋转约`6度`都视为真实进度，允许四轮差速底盘先低速调整朝向，不再因为“只旋转未平移”触发错误恢复。
- `safety_fusion_node`在ROS context已被统一关闭时不再把Jazzy的`RCLError`当成进程崩溃；运行期间的真实异常仍然继续抛出。
- RViz的`REAL-TIME Nav2 Local Costmap`透明度由`0.55`降为`0.25`。青蓝区域是局部代价地图中的自由空间，红紫边缘是障碍膨胀代价，不是OctoMap或建图漂移。

### 下一次复测判据

1. 只运行`./START_DUAL_2D_3D_NAVIGATION.sh`，在RViz中点击一次`2D Goal Pose`。
2. 每次点击只能出现一次`Begin navigating`，后面不应立即出现`Received goal preemption request`。
3. 车辆低速转向时不应再在20~30秒处出现`Failed to make progress`并突然反向。
4. 正常到达应出现`Goal succeeded`；Ctrl+C后不应再出现`safety_fusion_node process has died`。

本轮没有修改Cartographer V13、STM32、相机标定、RTAB-Map、雷达方向或建图滤波参数。

## V6.11：MPPI实车龟速、直线停顿和轻微摆动优化

### 日志证据

- 第三次导航的48个运动采样全部低于`0.04 m/s`，典型输出为`0.017~0.022 m/s`；这些采样中`source=nav2`、`collision=false`，并且`safe`与`nav`完全一致。
- 因此该次龟速和近似停车不是C++碰撞门、点云看门或安全融合节点截停，而是MPPI主动选择了低于实车可靠运动区的速度。
- 旧`PathFollowCritic.threshold_to_consider=1.2`会在距离目标小于1.2米时停止路径推进评价。第三次目标总距离只有约0.96米，因此整段导航都主要依靠近目标评价，路径推进约束没有发挥作用。
- 旧`VelocityDeadbandCritic`同时设置线速度和角速度死区。Jazzy源码会把两个死区缺口相加，因此直线路径的`wz=0`也会被惩罚，容易产生没有必要的正负小角速度。

### 参数修改

- 保持实车二档上限`vx_max=0.20 m/s`、`wz_max=0.218 rad/s`、15 Hz控制频率和2.81秒预测域不变。
- `vx_std`由`0.10`增至`0.14`，让800条轨迹能覆盖更多有效前进速度；保持采样数不变，避免重新增加CPU负载。
- `temperature`由`0.30`降至`0.25`，减少大量轨迹平均后产生的极小命令；`regenerate_noises=false`，降低每个控制周期重新随机采样造成的命令抖动。
- `PathFollowCritic`在离目标45厘米以外持续工作，权重由6增至10，前视偏移由4增至8；最后60厘米再由增强后的`GoalCritic`平滑接管。
- 外层膨胀软代价权重由5.0降至3.5，减少在紫色外层区域无障碍时长时间爬行。`near_collision_cost=253`、`collision_cost=1000000`和完整footprint碰撞检查保持不变，不会因此允许车体穿墙。
- 实车无效线速度死区改为`0.05 m/s`、权重增至60；角速度死区改为0，避免直行时为了满足死区评价而左右摆动。
- 路径角度、目标角度和前向评价的接管范围同步收紧，让长距离由路径评价主导，接近目标后才切换到精确到点评价。

### 复测要求

1. 先选择正前方`1.5~2.0 m`、无障碍的目标。稳定直行阶段`FUSION_STATUS`中的`nav.x`应明显高于`0.05 m/s`，不应连续数十秒保持`0.017~0.022 m/s`。
2. 再选择带一个缓弯的目标，检查路径不再左右反复，并确认一次点击只有一次`Begin navigating`。
3. 最后在路径侧面放置障碍物。外层紫色区域允许规划器权衡通过，但footprint接近实体障碍时仍必须绕行；进入C++近距门则应显示`local_cloud_collision_lock`并停车。
4. 若再次停车，必须记录该时刻`FUSION_STATUS source`：`nav2`且`nav=(0,0)`是控制器主动停车，`local_cloud_collision_lock`是实体点云停车，`local_cloud_timeout_lock`是点云断流停车，不能混为同一问题。

本轮仍未修改Cartographer V13、STM32、雷达、相机标定、RTAB-Map或车体footprint。

## V6.12：RViz段错误不得关闭导航和建图

### 两次日志结论

- `13:28:56`和`13:31:37`两次运行均在核心链路健康时收到统一`SIGINT`。退出前雷达约6 Hz、RGB-D点云约14 Hz、碰撞门持续更新，未发现Nav2、MPPI、Cartographer、相机或串口节点先行崩溃。
- 终端明确报告`rviz2`发生`段错误（核心已转储）`。旧脚本把RViz作为普通后台子进程；其异常状态使顶层脚本进入EXIT清理，随后向整个ROS launch进程组发送SIGINT，因此看起来像“导航运行一段时间后报错结束”。
- 第二次末尾的`web_goal_nav_node RCLError: context is not valid`发生在统一ROS context已经关闭之后，是关停连锁现象，不是最初触发源。

### 修复

- 新增独立`rviz_supervisor`。RViz异常退出时核心ROS launch保持运行，监督器等待2秒后自动重启RViz，最多重启3次；连续3次失败后只转为无界面运行，不再关闭建图、导航、相机和底盘。
- 用户正常关闭RViz窗口时不自动重开，核心系统继续无界面运行；Ctrl+C仍由主脚本统一停止所有节点。
- 主launch的`wait`显式关闭errexit并独立保存退出码，防止任何无关后台图形进程状态触发EXIT清理。
- RViz实时局部点云默认由`Boxes`改为`Points`、只保留当前帧、帧率由25降至20，并默认关闭全TF箭头/文字绘制。点云数据、STVL、MPPI和碰撞门完全不变，只降低RViz/Ogre渲染负载。
- `web_goal_nav_node`补充Jazzy统一关停时的无效context处理，不再把正常连锁关停显示成节点崩溃。

### 复测判据

1. 启动`./START_DUAL_2D_3D_NAVIGATION.sh`并连续运行至少5分钟。
2. 若RViz再次段错误，终端应显示`RViz exited ... core ROS stack is still running`和`Restarting RViz`；底盘、雷达、Cartographer和Nav2不得退出。
3. 新终端执行`ros2 node list`和`ros2 topic hz /scan_timed_v2`，确认RViz重启期间核心节点和雷达数据仍在。
4. 动态避障尚未由本轮日志覆盖，需在稳定运行通过后单独测试，不能根据这两次记录判定通过。

本轮没有修改MPPI运动参数、Cartographer V13、STM32、相机标定、RTAB-Map数据库或动态避障阈值。

## V6.13：主脚本被后台子进程状态提前唤醒

### 14:08:03日志结论

- 本次没有出现`rviz2段错误`，也没有ROS节点在统一关停之前报告`process has died`、Traceback、OOM或串口断开。
- 14:10:01前雷达约6 Hz、RGB-D点云约14 Hz、Cartographer、RTAB-Map和底盘数据均继续发布；14:10:02所有节点同时收到SIGINT，说明仍是顶层脚本先进入cleanup，而不是核心节点先崩溃。
- 终端只出现`[stop] Gracefully stopping...`，没有启动检查失败信息。旧版末尾直接执行`wait "$LAUNCH_PID"`；后台RViz监督器、RViz子进程或tail子进程发生状态变化时，Bash的wait可能提前返回，随后脚本无条件exit并触发cleanup，误杀仍在运行的ROS launch进程组。
- 日志中的`Failed to make progress`发生在碰撞门持续检测到前方约47厘米、70~123个点期间，只会触发Nav2恢复，不会关闭进程；它不是本次整套系统退出的原因。

### 修复

- 不再直接阻塞`wait`。主脚本现在每秒只检查真正的ROS launch PID，并读取`ps`进程状态；只有该PID消失或成为僵尸进程时才进入最终回收。
- 轮询中的`sleep`即使被任意SIGCHLD打断也不会触发errexit，因此RViz重启、tail变化或其他后台子进程结束都不能关闭核心系统。
- 真正的ROS launch退出后会明确打印`[monitor] ROS launch process exited with status ...`，下一次若仍退出可以直接区分核心launch死亡和外部Ctrl+C。
- 保留上一版RViz监督器和低负载显示配置，没有继续关闭任何SLAM、导航、点云或动态避障模块。

### 复测判据

1. 连续运行至少10分钟；正常运行期间不应出现`[monitor] ROS launch process exited`或`[stop]`。
2. 手动关闭RViz窗口后，ROS节点和底盘仍应继续运行；终端只应提示进入headless模式。
3. 若RViz崩溃，允许自动重启，但不应触发统一SIGINT。
4. 本次日志已意外触发C++碰撞门，但尚不能算完整动态避障测试；待进程稳定后再做“放入障碍、停车/绕行、移走障碍、恢复”的完整验证。

本轮没有修改Cartographer、MPPI、STVL、碰撞阈值、STM32、相机标定或RTAB-Map参数。
# 2026-07-25 V6.14：异常统一退出、真实地面平面滤波与底盘曲率统一

## 1. 本轮日志结论

检查 `14-30-28`、`14-33-11`、`14-35-40`、`14-37-08` 四次 `runtime.log` 后，
没有发现相机、雷达、Cartographer、RTAB-Map 或 Nav2 子进程先崩溃。每次都是所有
ROS 节点同时收到 `SIGINT`，因此“运行一段时间后卡死结束”的直接原因仍是顶层启动器
进入 cleanup，而不是 3060 性能不足或某个 SLAM 节点 OOM。

导航日志同时说明原配置存在模型不一致：`SmacPlanner2D` 生成不受曲率约束的栅格路径，
MPPI 再用 `DiffDrive` 去追踪，最终底盘还会对左右轮速度二次限幅。RViz 中画出的路径、
MPPI 期望轨迹和实车能够执行的圆弧并不是同一个模型，因此会出现原地左右抽动、慢速
转向和“路径看起来能走但车追不上”。

## 2. 车体几何与速度模型

根据最新 `车尺寸.txt`：

| 项目 | 数值 | 代码含义 |
|---|---:|---|
| 车体长/宽 | `0.665 / 0.665 m` | footprint 外接方形 |
| 车轮外侧/内侧总宽 | `0.620 / 0.510 m` | 单轮宽约 `0.055 m` |
| 轮胎中心距 | `0.565 m` | 半轮距 `wheel_track_w=0.2825 m`，原值正确 |
| 轮径 | `0.151~0.152 m` | 当前轮半径 `0.0755 m` 正确 |
| 前后轮最外沿距离 | `0.580 m` | 按该含义推算轴距约 `0.429 m` |

四轮差速理论半径可为 0，但原地转向会产生明显轮胎侧滑。常规导航统一使用最小中心
转弯半径 `0.60 m`：直线仍为速度二档 `0.20 m/s`；紧弯目标约为
`v=0.12 m/s, w=0.20 rad/s`。终点对角和恢复仍可低速原地转，但 `TwirlingCritic`
会抑制普通路径中的无意义自转。

## 3. 导航修改

- 全局规划器从 `SmacPlanner2D` 改为 Jazzy `SmacPlannerHybrid`。
- 搜索模型使用前进型 `DUBIN`，`minimum_turning_radius=0.60 m`、72 个角度桶；RViz
  全局路径从源头就满足实车曲率，不再把直角栅格折线交给 MPPI 硬追。
- MPPI 开启路径朝向约束并加入 `TwirlingCritic`，降低左右反复改向和原地抽动。
- 日志中实际出现过 `vx=0.004 m/s, wz=0.03 rad/s` 的无效反向微动；MPPI 角速度
  死区设为 `0.08 rad/s`，最终融合层也会把低于该阈值且几乎无前进速度的命令归零。
- MPPI 最大速度保持 `0.20 m/s`，最大角速度为 `0.22 rad/s`。
- `safety_fusion_node` 在最终下发前再次执行 `|w| <= |v| / 0.60`，确保控制命令
  不会比规划路径转得更急；圆弧外侧轮上限调整为 `0.18 m/s`。
- 没有修改 STM32 协议、轮速换算方向、Cartographer V13 或相机外参。

## 4. 地面滤波修改

原 V2.1 C++ 节点只删除固定 `z` 高度带。深度噪声只要高于固定上限 2 cm，就会被
误认为凸起障碍。现在每帧先对标定后地面附近的点做两阶段稳健最小二乘平面拟合：

```text
候选点 -> 初次拟合 z=ax+by+c -> 2.5cm 残差内点 -> 再拟合
       -> 法向/截距/内点数检查 -> 删除平面上下小范围地面噪声
```

仅当内点不少于 120、比例不少于 30%、坡度和截距合理时使用动态平面；置信度不足会
自动回退已验证的固定 Z 带，不会拿墙面当地面。`/local_highres_cloud_v21/stats` 新增
`ground_plane_valid/a/b/c/candidates/inliers/ground_removed_points`，便于实车确认。

## 5. OctoMap 与实时避障

当前小方块由 RTAB-Map 内置 OctoMap 直接生成，并不存在独立 `octomap_server`，因此
环境变量 `OCTOMAP_RATE` 不能改变该显示频率，本轮没有用这个无效参数冒充优化。
RTAB-Map 持久图显式开启法向地面分割：`NormalK=20`、最大地面坡角 15 度、最大地面
高度 5 cm，并删除不足 10 点的孤立簇。这样改善长期图的地面凸点，同时独立实时层仍
保留 2.5 cm 以上低障碍。OctoMap 是长期 3D 记忆和 RViz 显示，约 0.5~1 秒视觉延迟
是 2 Hz RTAB 图优化链路的正常代价；碰撞避障不等它。实时链路仍为：

```text
Gemini2 深度 15Hz -> V2.1 C++滤波点云 -> STVL局部动态层 -> MPPI
                                      -> C++近碰撞门 -> safety_fusion
```

## 6. 启动器修改

- `launcher.pid`、`ros_launch.pid` 和所有 `[launcher]` 原因写入本次 `runtime.log`。
- 忽略终端/RViz 短暂产生的 `SIGHUP`，只有 Ctrl+C、SIGTERM、明确启动失败或 ROS launch
  真正退出才执行统一 cleanup。
- 如果启动器 shell 自身异常退出但 ROS launch 仍健康，不再误发 SIGINT 杀死整套节点；
  日志会给出进程组 PID。正常 Ctrl+C 仍会完整停止并保存数据库/OctoMap。
- 修复 `frontier_web_bridge` 在 Jazzy context 已关闭后重复销毁节点产生的次生异常。

## 7. Ubuntu 实车验证顺序

先编译新 C++ 节点和 Python 包：

```bash
cd ~/视频/huichuan-agv-ros2-foxy-visual-lidar-slam-step/huichuan-agv-ros2-foxy-main/lidar/chapt1_ws
colcon build --symlink-install --packages-select local_depth_cloud_cpp lidar_py
```

先运行 `./START_DUAL_2D_3D_MAPPING.sh`，静止面对普通走廊地面，检查：

```bash
ros2 topic echo /local_highres_cloud_v21/stats --once
```

正常应为 `ground_plane_valid=true`，`inliers >= 120`，且地面凸点明显减少。若长期为
`false`，不要提高删除高度，保留日志和当时 RViz 截图用于调整候选范围。

再运行 `./START_DUAL_2D_3D_NAVIGATION.sh`，依次测试 2 m 直线、90 度转弯、掉头和
动态障碍。日志应显示 `SmacPlannerHybrid`、`minimum_turning_radius 0.60`；90 度路径
应为连续圆弧，实车不得在普通转弯中左右交替原地抽动。若启动器再次意外结束，提交
完整 `runtime.log`，重点保留末尾 `[launcher] [stop] Trigger` 和两个 PID 文件。
# 2026-07-25 V6.15：撤销错误车式规划并修复启动探针误杀

> 本节覆盖 V6.14 中的 Hybrid-A* / `0.60 m` 最小曲率方案。实车截图证明该方案不适合
> 能够原地转向的四轮差速底盘，已经从运行配置完全撤销。

## 最新四次日志的确定结论

| 运行 | 实际结束原因 |
|---|---|
| `15-18-33` | 启动时 `/dev/ttyUSB0` 不存在 |
| `15-20-53` | 启动器没有在 30 秒内 echo 到 `/cartographer_pose_odom`，主动误杀健康进程 |
| `15-22-20` | 明确收到 `Ctrl+C`，属于人工结束 |
| `15-23-44` | 启动器没有在 90 秒内 echo 到 `/map`，主动误杀健康进程 |

后两种自动结束不是 ROS 节点崩溃。ROS 2 CLI 在高负载启动阶段创建临时订阅可能超过
原来的 2 秒单次超时；车静止时修正里程计也不保证立刻产生新消息。旧探针把这两种情况
错误当成致命故障，然后 cleanup 给所有节点发送 SIGINT。

## 启动逻辑修复

- `/map` 和 `/cartographer_pose_odom` 改为检查 publisher 注册，不再依赖必须收到一帧。
- 两项检查即使超时也只写 WARNING，不再结束仍健康的 SLAM/Nav2 栈。
- 需要真实数据的雷达、相机和实时安全点云仍保留数据级检查，避免无传感器时误放行。
- 单次 `ros2 topic echo` 容限从 2 秒增加到 6 秒。
- 串口缺失时等待 15 秒并重新执行 USB 产品号识别，处理开机枚举和端口号变化。
- RTAB 地面分割版本加入数据库配置签名；下一次建图会自动归档旧数据库，避免旧噪点
  节点继续混进新参数生成的 OctoMap。

## 导航回退与重新调整

- 删除 `SmacPlannerHybrid + DUBIN + minimum_turning_radius=0.60`。截图中的巨大 U 形绕行
  正是 Dubin 车式路径造成，不是实车应该执行的路线。
- 恢复 `SmacPlanner2D`，它负责生成直接的二维中心路径；`DiffDrive MPPI` 负责圆弧、
  原地对角和动态避障。
- 删除安全融合层中的最小转弯半径夹紧。左右轮仍按真实半轮距 `0.2825 m` 使用
  `v_left=v-w*0.2825`、`v_right=v+w*0.2825`，圆弧限速同时缩放两侧，不改变曲率。
- 全局代价权重降为 `1.20`，减少空旷区域无意义绕行；路径数据权重提高，减少平滑器
  把直线路径拉弯。
- 保留 `0.05 m/s` 线速度和 `0.08 rad/s` 角速度无效死区，Twirling 权重降至 2，
  只抑制左右小幅抽动，不阻止差速底盘需要的原地转向。

## 地面噪点第二阶段过滤

V6.14 的平面带只删除平面上方 2.5 cm 内点，深度相机产生的 2.5～6 cm 小凸点仍会
进入 STVL。现在对该高度范围增加 3D 体素邻域检查：少于 4 个相邻体素的小簇删除，
大于约 2x2 体素的真实低矮物体继续保留。每秒日志新增：

```text
ground valid=true plane=(a,b,c) inliers=有效/候选 removed=平面点+小簇点
```

若 `ground valid=false`，表示当前画面没有足够可信地面，节点会回退固定高度带；不再需要
猜测滤波是否运行。RTAB-Map 长期图的法向地面分割保持启用，实时碰撞仍只使用 C++
局部点云和 STVL，不等待 OctoMap。

# 2026-07-25 V6.16：全局 2D/3D 障碍统一与误停车修复

## 本次日志的确定结论

`dual_3d_2026-07-25_16-09-17/runtime.log` 没有再次异常结束，最后是用户主动按下
`Ctrl+C`。C++ 地面拟合持续为 `ground valid=true`，每帧约 4.1 万个地面点被正确删除，
因此这次绕路和停车不能继续归因于地面噪点。

第二个导航目标期间，MPPI 连续输出有效的小角速度 `nav=(0.000,-0.052)`，同时日志明确为
`collision=false`、`local_cloud_alive=true`；旧安全融合代码却把它改成
`safe=(0.000,0.000)`，持续约 20 秒后 Nav2 才报告 `Failed to make progress`。这就是
“人和障碍物很远但车停住”的直接原因。

## 修改内容

1. 删除 `safety_fusion_node` 对 Nav2 小角速度的二次硬归零。安全层现在只在急停、点云
   断流或 C++ 碰撞门真正触发时停车，不再篡改 MPPI 的合法转向命令。
2. MPPI 角速度死区由 `0.08` 降为 `0.04 rad/s`；保留 `0.05 m/s` 的实车线速度死区。
3. 启用 `laser_filters`，Cartographer、Nav2 和 RViz 统一使用
   `/scan_timed_v2_filtered`。滤波链只剔除 0.10～8.0 m 之外数据和孤立跳点，避免旧
   `/scan` 的瞬时散点在代价地图上生成不存在的障碍。
4. 全局规划器从此也加载 STVL 参数文件。旧 launch 只把该文件交给 controller server，
   所以局部 MPPI 能看见 3D 障碍，而全局规划器完全看不见；这是全局路径朝视觉障碍物
   规划的结构性原因。
5. 全局代价地图新增 `depth_global_stvl_layer`：5 cm 体素、至少 2 点才标记、1 秒衰减；
   以 2 Hz 更新；局部地图继续使用 3 cm、1 点标记的 STVL，并保留 C++ 近碰撞门。
6. 全局和局部 2D 障碍源均改为过滤后的雷达。3D 障碍不再重复走旧
   `/depth_obstacle_scan`，由 STVL 负责标记和清除。
7. 车体 footprint 不变。膨胀半径由 `0.58` 调为 `0.52 m`、衰减系数由 `5.0` 调为
   `6.0`，减少空旷区域被软代价过度推弯；Smac 2D 代价权重由 `1.20` 调为 `2.0`，
   使真实障碍仍有足够绕行权重。
8. 一键启动增加过滤雷达和全局代价地图数据级就绪检查。脚本仍默认自动编译，不需要
   手工执行 `colcon build`。

## 实车验证顺序

只运行：

```bash
./START_DUAL_2D_3D_NAVIGATION.sh
```

启动日志必须出现 `/scan_timed_v2_filtered` 已收到数据、`/global_costmap/costmap` 已收到
数据，并显示 `SmacPlanner2D`、`depth_stvl_layer` 和 `depth_global_stvl_layer` 已初始化。

1. 正前方 1.5～2.0 m 无障碍目标：青色全局路径应基本为直线，不应绕过空地散点。
2. 在直线路径中放置大于 10x10 cm 的箱子并等待 1 秒：全局路径应重新绕行，局部 MPPI
   路径不得穿过箱子；移走后重新扫到原位置，约 1～2 秒内代价应清除并恢复直线。
3. 选择需要原地修正约 10～30 度的目标：当日志出现约 `0.04～0.08 rad/s` 的角速度时，
   `safe.angular.z` 必须保持同方向且不为零，不得再因 20 秒未平移报无进展。
4. 若仍停车，只按 `FUSION_STATUS source` 分类：`nav2` 且 `nav=(0,0)` 属于 MPPI 决策；
   `local_cloud_collision_lock` 属于近碰撞门；`local_cloud_timeout_lock` 属于点云断流。
   三者不能再混为同一问题。

本轮没有修改 Cartographer V13 参数、STM32、相机标定、RTAB-Map 回环参数、雷达方向、
车体尺寸或 3D 地面拟合算法。

# 2026-07-25 V6.17：实时重规划、实车曲率补偿与全方向脱困安全链

## 本次两份实车日志结论

分析 `dual_3d_2026-07-25_17-09-40` 和 `dual_3d_2026-07-25_17-17-19` 后，确认并非单一 MPPI
权重问题：

1. 单目标行为树仅以 `0.2 Hz` 检查旧路径，而且只要 `IsPathValid` 仍返回真就不重新规划。新障碍物进入后，
   全局路径可以继续穿过障碍；旧路径时间戳超过 TF 的 10 秒缓存后还会产生
   `Requested time ... but earliest data ...`，最终无法取得机器人位姿。
2. 实车移动弧线明显欠转。例如命令约为 `(v=0.124,w=0.128)` 时，实测约为
   `(v=0.126,w=0.079)`，实际半径约 `1.59 m`，而命令半径约 `0.97 m`。因此 RViz 路径能转过去，
   实车却转不到，随后控制器持续用差速弧线追赶。
3. 旧 C++ 近碰撞门只检查相机前方 `x=0.20～0.58 m`，而安全融合又只在正向行驶时采用它。
   原地转向、车侧扫掠和倒车没有最终硬保护；相机太近看不到墙角时就可能碰撞。
4. 局部 STVL 代价地图没有静态层。靠近已经扫描过的墙、传感器短暂看不到墙时，局部控制器会遗失历史墙体。
5. 恢复树只有清图和等待，无法从死路或错误朝向中受控退出。

## 行为树与路径更新

- `NavigateToPose` 与 `NavigateThroughPoses` 都改为无条件 `1.0 Hz` 重新执行全局规划，不再复用一条“仍被判定有效”
  但已经过时的路径。
- 每次重规划都会纳入最新全局 2D 雷达障碍和全局 STVL 障碍；新障碍出现后不再等待旧路径失效。
- 恢复顺序改为：清理两张代价地图、后退 `0.35 m @ 0.06 m/s`、左转约 30 度、右转约 30 度、短暂等待，
  每一步之后重新规划。
- `behavior_server` 新增 Jazzy 原生 `nav2_behaviors::BackUp`。倒车同时受 Nav2 footprint 碰撞预测和本文新增的
  后向 2D 雷达硬门保护，不允许盲倒。

## 转向控制

- MPPI 外层新增 Jazzy `RotationShimController`。路径方向误差超过 `0.50 rad` 时先停车并以
  `0.16 rad/s` 原地对齐，低于 `0.18 rad` 后才交回 DiffDrive MPPI，避免在狭窄空间用大半径弧线硬追目标。
- MPPI 仍负责普通弧线、动态避障和路径跟踪，没有替换导航算法。
- 根据本次日志的实车欠转比例，Nav2 移动弧线在最终下发前增加 `1.45` 倍角速度补偿，最大限制为
  `0.32 rad/s`；随后仍执行速度二档的外侧轮 `0.16 m/s` 限幅，并等比例缩放两侧轮速以保持曲率。
- 角速度无效区由 `0.04` 调到 `0.06 rad/s`，避免控制器反复选择实车几乎不响应的龟速角速度。
- 补偿只作用于 Nav2 的移动弧线，不修改网页/PS2 命令，不修改 STM32，也不修改 Cartographer 使用的里程计。

## 历史墙与实时障碍

- 局部代价地图新增 Cartographer `static_layer`，所以已经建入 2D 地图的墙会继续参与局部碰撞预测，
  不要求相机在贴墙时再次看见它。
- 3 cm STVL 仍负责实时 RGB-D 人物、箱子和低矮障碍；Cartographer 静态层只补长期墙体。
- 没有把 RTAB-Map OctoMap 直接接入 Nav2。当前 OctoMap 仍可能含有地面体素，直接作为代价地图会把可走地面
  误判为障碍。OctoMap 继续用于长期 3D 记忆和 RViz 显示。

## C++ 全方向碰撞门

`local_cloud_collision_gate_node` 现在同时订阅实时 RGB-D 点云和 `/scan_timed_v2_filtered`，输出状态数组为：

```text
[前方阻挡, 前方点数, 最近前方毫米,
 旋转扫掠阻挡, 后方阻挡, 旋转点数, 后方点数, 2D雷达存活]
```

- 前进：RGB-D 低矮障碍和 2D 雷达任一命中即停车。
- 倒车：检查车后 `x=-0.62～-0.20 m` 走廊；2D 雷达断流也禁止倒车。
- 原地转向/紧弧线：按车体外接圆加余量后的 `0.52 m` 扫掠圆检查；雷达断流也禁止转向。
- 2D 雷达至少连续命中 2 个过滤点才阻挡，减少单点噪声误停；阻挡保持 `0.25 s`，避免边界抖动。
- 前三个状态字段保持旧接口兼容。

## 实车复测

只运行：

```bash
./START_DUAL_2D_3D_NAVIGATION.sh
```

一键脚本会自动重编译 `local_depth_cloud_cpp` 和 `lidar_py`。启动日志必须出现：

```text
C++ collision gate: RGB-D front ... 2D scan=/scan_timed_v2_filtered
COLLISION_GATE front=... rotation=... rear=... scan_alive=true
```

按以下顺序验证：

1. 空旷处设置正前方目标：全局路径应每秒更新，实车直行不应左右抽动。
2. 路径上放置箱子：约 1 秒内全局路径应改变，局部轨迹和车体都不得穿过箱子。
3. 选择侧后方目标：应先停止、原地对齐，再沿新路径前进；靠墙导致 `rotation=true` 时必须拒绝旋转。
4. 模拟死路：常规跟踪失败后应先清图，再在后方安全时低速退约 35 cm，然后尝试小角度转向并重新规划。
5. 查看 `FUSION_STATUS`：正常移动弧线应显示 `arc_gain=true`；安全锁应明确区分
   `front_collision_lock`、`rotation_scan_collision_lock`、`rear_scan_collision_lock` 和
   `local_cloud_timeout_lock`。

本轮仍未修改 Cartographer V13、STM32、相机标定、RTAB-Map 回环/数据库和地面拟合参数。

# 2026-07-26 V6.18：RTAB-Map 优化 OctoMap 延迟修正

## 确认的问题

- 旧启动横幅显示 `OctoMap 10 Hz`，但当前双分辨率 launch 中没有独立
  `octomap_server`；`OCTOMAP_RATE=10.0`、`OCTOMAP_RESOLUTION` 和
  `OCTOMAP_CLOUD_TOPIC` 都是未连接到节点的残留变量。
- RViz 的 `/rtabmap_3d/octomap_occupied_space` 实际跟随
  `Rtabmap/DetectionRate`。导航脚本又把它从公共配置的 `2 Hz` 覆盖成
  `1 Hz`，因此正常情况下也至少有约 1 秒更新间隔，地图增大后整图发布还会更慢。
- 不能简单恢复旧 `global_cloud_relay + octomap_server`：它会按当时
  `map->base_link` 位姿永久累计原始点云，Cartographer 后续回环修正时历史方块
  无法重新排列，之前已经因此产生过厚墙、重影和视觉墙偏离 2D 墙。

## 修正

1. 导航版 RTAB-Map 恢复为 `2.0 Hz`，所以新关键帧和优化 OctoMap 的目标更新周期
   约为 `0.5 s`。
2. 明确启用 `RGBD/CreateOccupancyGrid=true`。
3. 关闭项目不使用且体积很大的 `publish_octomap_full`；保留 RViz 使用的
   `/rtabmap_3d/octomap_occupied_space`。
4. 设置官方 `GridGlobal/UpdateError=0.05 m`。毫米级图优化不再触发昂贵的全局
   OctoMap 重建；超过 5 cm 的真实校正和回环仍会重建并重新排列历史体素。
5. 删除三个无效的独立 OctoMap 环境变量，启动横幅现在显示真实来源、真实目标频率
   和 `GLOBAL_3D_VOXEL=0.05 m`。

## RViz 验证

启动 `./START_DUAL_2D_3D_NAVIGATION.sh` 后勾选
`OctoMap 3D Occupied (RTAB-Map Optimized)`。正常预期：

- 新建小地图时约 `0.5～1.5 s` 出现新方块，而不是 10 Hz；这是长期优化地图，
  不是 15 Hz 实时避障层。
- 地图范围增大后，完整占用点云的序列化时间仍会增长；若达到数秒，应保留日志中的
  RTAB-Map `total_time/pub_time`，不能通过重复发布旧消息伪装成实时更新。
- 实时碰撞避障继续使用 `Live Local 3D Voxels`、3 cm STVL 和 C++ 碰撞门，
  不等待该 OctoMap。

检查实际频率：

```bash
ros2 topic hz /rtabmap_3d/octomap_occupied_space
ros2 topic hz /rtabmap_3d/mapData
```

本轮没有恢复会造成回环残影的独立 OctoMap，也没有修改 Cartographer、STM32、
相机外参、导航代价地图或 RTAB-Map 回环阈值。

# 2026-07-26 V6.19：导航平滑转向与 Cartographer 错误闭环防护

## 本次实车日志结论

`dual_3d_2026-07-25_20-24-45/runtime.log` 中存在两种不同问题：

1. MPPI 多次输出 `v=0`、`|w|=0.02~0.07 rad/s` 的极小纯旋转。该角速度落在
   实车有效死区附近，下位机经常反馈 `used_wz=0`，因此车表现为原地慢转、
   停顿和左右反复修正。
2. `20:28:15` 底盘没有发生对应突变，但 Cartographer 在一次优化中加入 6 条
   回环约束，随后 `map->base_link` 单帧跳变 `+6.08 deg`。参与匹配的分数集中在
   `75.1%~76.9%`，说明短墙和狭窄空间产生了低置信度歧义闭环。

## 平滑导航修改

- Rotation Shim 的触发角由 `0.50 rad` 提高到 `1.75 rad`（约 100 度）。
  普通直角拐弯和路径微调由 DiffDrive MPPI 以移动圆弧完成；只有新目标真正位于
  车后方时才允许碰撞检查后的原地调头。
- `rotate_to_heading_once=true`，1 Hz 全局重规划产生新路径时不再反复重新触发
  Rotation Shim；`rotate_to_goal_heading=false`，到达 XY 容差后不再强制原地对齐
  RViz 目标箭头。
- MPPI 的 `TwirlingCritic` 从 2 提高到 10，`PreferForwardCritic` 从 7 提高到 9，
  `PathAngleCritic` 从 4 提高到 6，减少无意义原地扭转并优先连续前进。
- 实车角速度死区恢复为 `0.08 rad/s`。若 Nav2 仍输出
  `0.02 < |w| < 0.08 rad/s` 的纯旋转，安全融合层会提升到恰好
  `0.08 rad/s`，使修正真正完成；旋转扫掠碰撞门仍可将其拦停。
- `1.45` 倍欠转补偿现在只作用于 `|v| >= 0.10 m/s` 且原始曲率半径不小于
  `0.70 m` 的行进圆弧。MPPI 已经给出紧弯时不再二次放大角速度，降低四轮差速
  打滑和建图姿态扰动。

## 导航专用闭环防护

- 新增 `cartographer_2d_v9_nav_guarded.lua`。它完整继承定版 V9，只把导航期间
  的近距离约束门槛从 `0.75` 提高到 `0.78`，全局重定位门槛从 `0.80` 提高到
  `0.82`，拒绝本次日志中已经确认会造成 6 度跳变的分数带。
- `START_DUAL_2D_3D_NAVIGATION.sh` 自动加载该防护配置。
- `START_DUAL_2D_3D_MAPPING.sh` 明确继续加载原
  `cartographer_2d_v9_tightened.lua`，所以纯建图定版参数没有被替换。

## 保留的地图与避障链路

- Cartographer 全局 2D 静态地图、过滤后 2D 雷达、全局 5 cm STVL、局部 3 cm
  STVL、C++ 前后/旋转碰撞门、SmacPlanner2D、1 Hz 全局重规划和 MPPI 全部保留。
- `publish_octomap_full=false` 只关闭未使用的巨大完整 OctoMap 消息；
  `/rtabmap_3d/octomap_occupied_space`、RTAB-Map 长期数据库、视觉回环和退出时
  `.bt` 保存仍然保留。Nav2 实时避障继续使用 STVL，不等待低频长期 OctoMap。

复测只需运行：

```bash
./START_DUAL_2D_3D_NAVIGATION.sh
```

启动横幅应显示：

```text
2D SLAM config  : cartographer_2d_v9_nav_guarded.lua
```

# 2026-07-26 V6.20：平滑决策树、实车曲率闭环和视觉墙体避障

## 本次日志确认的两个根因

1. `dual_3d_2026-07-25_20-24-45/runtime.log` 实际加载的是
   `cartographer_2d_v9_tightened.lua`，不是导航专用的
   `cartographer_2d_v9_nav_guarded.lua`。随后 Cartographer 接受了多批
   `75.1%~77.8%` 的走廊歧义约束，约束旋转修正逐步增加到约
   `0.08 rad`，因此地图再次发生可见歪斜。
2. 同一日志中 `rotation=true` 几乎持续存在，旧安全层又把
   `|v| <= 0.04 m/s` 的低速移动弧线当作原地旋转。MPPI 命令在该阈值
   两侧波动时，安全输出会在正常移动和 `rotation_scan_collision_lock`
   之间反复切换，形成停顿、慢转和左右犹豫。

## 新决策链

```text
SmacPlanner2D（1 Hz，最新全局代价图）
  -> SimpleSmoother（碰撞复检后的平滑路径）
  -> Rotation Shim（仅大角度初始对齐）
  -> DiffDrive MPPI（局部轨迹和动态避障）
  -> Nav2 Velocity Smoother（30 Hz 闭环速度/加速度平滑）
  -> safety_fusion（曲率补偿、接近减速、方向性硬停车）
  -> STM32
```

- 网页/视觉安全逻辑不再给 Nav2 叠加左右转向偏置。Nav2 已经通过
  MPPI 和代价地图选择方向，安全层只按比例降低 `v/w`、保持曲率，或在
  碰撞门触发时停车。
- RViz 蓝线改为 `/plan_smoothed`，它就是 MPPI 实际跟随的路径。
  原始 `/plan` 保留为默认隐藏的灰线，便于排查平滑前后的差异。
- 控制器和脱困行为先输出 `/cmd_vel_nav_raw`，再由
  `nav2_velocity_smoother` 以 30 Hz 闭环输出 `/cmd_vel_nav`。加减速
  限制同时作用于线速度和角速度，`scale_velocities=true` 保持弧线曲率。

## 路径规划和转弯

- `NavigateToPose` 与 `NavigateThroughPoses` 都无条件以 `1 Hz`
  重新规划；每条新路径都经过 `SimpleSmoother` 和碰撞复检。
- MPPI 路径跟随、路径角度和防原地扭转权重提高，减少明明可以直行却
  绕空地，以及左右交替修正。
- Rotation Shim 只在初始路径方向误差超过 `1.20 rad`（约 69 度）时
  工作，低于 `0.25 rad` 后交回 MPPI。普通转角使用连续移动弧线，大角度
  错向才执行一次受保护的停转。
- 新增 `AdaptiveArcGain`。以日志中实车欠转的 `1.45` 为初值，只在
  `|v| >= 0.10 m/s` 的稳定宽弧中比较期望半径与 `/odom` 实测半径，
  在 `1.10~1.80` 内缓慢学习。它不修改 STM32、PS2 和 Cartographer
  里程计，只补偿 Nav2 的移动弧线。
- 前方 `0.62~1.20 m` 新增连续减速区，线速度和角速度同比缩放；进入
  `0.62 m` 硬保护区才停车，避免临近障碍突然急停。

## 脱困逻辑

旧行为树的 `BackUp backup_dist="-0.35"` 符号错误，已经改为正的
`0.25 m`。新顺序为：

1. 清除局部和全局代价地图。
2. 等待 1 秒，让行人等动态障碍先离开。
3. 后方 2D 雷达确认安全后，以 `0.06 m/s` 受控倒车 25 cm。
4. 分别尝试左右约 20 度的碰撞检测旋转。
5. 等待 2 秒并重新规划。

倒车、原地旋转、前进分别由后向走廊、车体扫掠圆和前向 2D/3D 门保护；
雷达断流时禁止倒车和原地旋转。

## 视觉 SLAM 墙体进入避障

新增 C++ `persistent_visual_wall_filter_node`：

- 输入 `/rtabmap_3d/octomap_occupied_space`。
- 按 5 cm 的 XY 柱分组，仅保留同柱点数至少 5、垂直跨度至少
  35 cm、且高度在 `0.08~1.40 m` 的结构。
- 输出 `/rtabmap_3d/navigation_walls`。平坦地面和小片地面方块不会
  作为长期墙体送给 Nav2。
- 局部和全局代价地图新增 5 cm 视觉墙 STVL，衰减时间 3 秒。RTAB-Map
  回环移动墙体后，旧位置会衰减，新位置重新写入，不会永久留下双墙。
- 15 Hz、3 cm 的实时 RGB-D STVL 仍负责人物、箱子和低矮障碍；视觉
  长期墙层负责相机近距离盲区中已经扫过的墙。
- RViz 的 `OctoMap 3D Occupied (RTAB-Map Optimized)` 完整保留，可继续
  勾选检查 3D 地图；另新增默认关闭的
  `RTAB-Map Vertical Walls (Nav2 Input)` 用于核对真正进入导航的墙体。

“厘米碰撞级”表示代价地图使用 3 cm 实时体素、最终硬门直接检查原始
2D/3D 点；它不能承诺物理上零厘米误差。最终误差仍受 Gemini2 深度噪声、
外参、2D 雷达盲区、轮胎侧滑和制动距离限制，实车必须保留安全余量。

## 启动与验证

只执行：

```bash
./START_DUAL_2D_3D_NAVIGATION.sh
```

一键脚本会自动编译新增 C++ 节点。启动日志必须同时出现：

```text
2D SLAM config  : cartographer_2d_v9_nav_guarded.lua
[dual3d] Cartographer config: cartographer_2d_v9_nav_guarded.lua
Persistent visual wall filter:
Created smoother : SmoothPath of type nav2_smoother::SimpleSmoother
velocity_smoother lifecycle node launched
```

若 Cartographer 再打印 `Found ... cartographer_2d_v9_tightened.lua`，立即
停止测试并保留日志，因为这表示启动的不是当前导航脚本。正常测试顺序为：
2 m 直线、90 度转角、路径中放入并移走箱子、侧后方目标、死路脱困。
同时保留完整 `runtime.log`，重点检查 `arc_gain_value`、
`approach_scale`、`front/rotation/rear_collision_lock` 和
`VISUAL_WALL_FILTER`。

# 2026-07-26 V6.21：启动误杀与 DDS 参与者耗尽修复

## 两次自动退出的真实原因

`dual_3d_2026-07-26_11-44-07` 和
`dual_3d_2026-07-26_11-45-08` 都不是 Cartographer、Nav2、MPPI 或
C++ 避障节点崩溃。退出前同一时刻的日志明确显示：

```text
COLLISION_GATE ... scan_age=13.1ms scan_alive=true
FUSION_STATUS ... scan_alive=True
[launcher] [ERROR] LiDAR did not publish a valid /scan_timed_v2 revolution
```

雷达数据实际正常，外层启动器却把临时 `ros2 topic echo` 的 DDS 连接失败
误判成雷达断流，随后主动向全部 ROS 节点发送 SIGINT。第二次日志中的
`Failed to find a free participant index for domain 88` 进一步确认了
CycloneDDS 参与者编号耗尽。

## 修正

1. 新增 `visual_laser_slam/cyclonedds_dual_3d.xml`，该一键链路固定使用
   `ParticipantIndex=none`。每个 ROS 进程使用系统分配的端口，不再受默认
   参与者编号上限影响。
2. 启动前停止旧的 ROS 2 daemon，使新 daemon 继承本次 DDS 配置。
3. `wait_topic` 从“每秒重新执行一次 topic list 和 echo”改成单个
   DDS 订阅者持续等待。探针不会再在启动高负载阶段反复创建参与者。
4. ROS 2 CLI 禁用后台 daemon，所有启动检查直接使用本次 DDS 配置；导航版
   还会接受 C++ 碰撞门的 `scan_alive=true` 作为雷达数据正常的进程内证据。
5. 启动横幅新增 `DDS profile`，方便从日志确认配置确实生效。
6. 路径平滑器若把原始安全路径推入膨胀区，行为树立即保留并跟随原始路径；
   不再把可选平滑失败升级成清图和导航失败。
7. 补齐 Jazzy 行为树使用的 planner、controller、smoother、backup 和
   spin 错误码键，消除缺少错误码上下文的警告。

本轮未修改 Cartographer V13/导航防误闭环参数、STM32、相机外参、MPPI
动力学、STVL、视觉墙体提取及 RTAB-Map/OctoMap。

## 复测

只运行：

```bash
./START_DUAL_2D_3D_NAVIGATION.sh
```

启动横幅应包含：

```text
DDS profile     : file:///.../visual_laser_slam/cyclonedds_dual_3d.xml
```

随后应逐项出现 `[ready] data ...`，且不能再出现
`Failed to find a free participant index` 或在 `scan_alive=true` 时报告
`LiDAR did not publish`。如真实雷达确实没有输出，单次探针仍会在 20 秒后
终止启动，原有硬件安全检查没有被关闭。

# 2026-07-26 V6.22：导航零输出与地面假障碍修复

## 本次日志结论

`dual_3d_2026-07-26_12-53-39/runtime.log` 中 RViz 目标、全局规划、平滑路径和
`FollowPath` 均已成功下发，但 `safety_fusion_node` 连续收到
`nav=(+0.000,+0.000)`。这说明不是网页/手柄接管、安全融合或 STM32 把速度清零，
而是 Nav2 控制器没有生成可执行速度。

两个测试目标与车头的初始夹角约为 106～120 度，旧
`angular_dist_threshold=1.20 rad` 会强制 Rotation Shim 原地旋转。与此同时，
2D 雷达扫掠圆持续检测到约 20～40 个点位于车体 0.52 m 旋转包络内。这个保护拒绝
贴墙原地旋转是正确的，但旧决策把 MPPI 的移动弧线也挡在了 Rotation Shim 后面，
最终表现为点击目标后始终不动。

深度地面拟合本身工作正常：每帧约 3.95 万个候选点中已去除约 3.6 万个地面点。
RViz 中大片青色禁行岛主要来自少量反光地面残点被局部 STVL 按“单点占用”写入，
再被 0.52 m 膨胀层放大，并不是地面滤波没有启动。

## 修改

1. Rotation Shim 触发角由 `1.20 rad` 提高为 `2.80 rad`。普通侧向和侧后方目标交给
   DiffDrive MPPI 以连续移动弧线完成；仅目标几乎位于正后方时才执行碰撞检查后的
   原地调头。
2. 局部 3 cm 深度 STVL 的 `voxel_min_points` 从 `1` 提高为 `2`。同一体素至少有
   两个深度回波才写入动态代价地图，抑制瓷砖反光形成的孤立假障碍。
3. 全局 5 cm STVL 原本已经采用两个点，本轮保持不变。
4. C++ 原始 RGB-D/2D 雷达前进、后退和旋转碰撞闸、Cartographer 静态墙、
   视觉长期墙层、1 秒动态衰减和 MPPI 足迹碰撞检查全部保留。此次没有降低硬停车
   检测，也没有修改 Cartographer、相机标定、STM32 或 RTAB-Map 数据库。

## 实车复测

只运行：

```bash
./START_DUAL_2D_3D_NAVIGATION.sh
```

先在车头侧前方选择 1～2 m 的空旷目标。正常结果应为 MPPI 直接输出带线速度的弧线，
`FUSION_STATUS` 中 `nav` 不再长期为 `(0,0)`。随后再选择侧后方目标；只要不是几乎
正后方，车辆也应先沿安全弧线移动，而不是贴墙原地旋转。

观察 RViz 的局部代价地图，孤立青色圆岛应在约 1 秒内明显减少或衰减。真实箱子、墙
和成片低矮障碍仍应形成连续占用区。若原始彩色点云中还能看到少量散点，但青色代价区
已经消失，这是预期现象：原始显示保留诊断细节，导航只接受具有空间密度的障碍。

# 2026-07-26 V6.23：MPPI 全轨迹无效与速度链可观测性修复

## V6.22 复测结论

`dual_3d_2026-07-26_13-28-24/runtime.log` 证明 V6.22 的 Rotation Shim 角度修改没有
解决零速度问题。该次目标方向仅比车头偏约 25 度，本来就不会触发 Rotation Shim；
目标下发后共记录 51 次 Nav2 状态，仍没有一次非零速度。

更关键的证据是：

```text
Optimizer fail to compute path
```

并且在大部分零速度时段，C++ 闸同时报告
`front=false rotation=false rear=false`。所以不是 STM32、安全融合或旋转扫掠闸
拦截，而是 MPPI 的局部代价地图判定没有可执行轨迹。

## 根因与修改

1. 局部深度 STVL 原来使用 `track_unknown_space=true`。Gemini2 只看车前方，因此
   侧面和后方未观测体素会成为 unknown；MPPI 足迹碰撞检查把这些区域当作不可通行，
   转弯采样很容易全部失效。现将**局部**深度 STVL 改为
   `track_unknown_space=false`，只标记相机实际看到的障碍。
2. **全局**深度 STVL 继续保持 `track_unknown_space=true`；360 度 2D 雷达、
   Cartographer 静态地图和全局规划的未知空间限制均未放开。
3. 明确设置 controller server 与 velocity smoother 均使用 Jazzy 的非 stamped
   `Twist` 接口，避免升级版本后两端消息类型默认值不一致。
4. velocity smoother 的输出死区由 `[0.03, 0, 0.04]` 改为零。它仍保留 30 Hz
   加减速平滑和速度上限，但不再把 MPPI 已经算出的低速命令静默替换成 `(0,0)`。
   MPPI 的 `VelocityDeadbandCritic`、安全融合和下位机限制仍然存在。
5. `safety_fusion_node` 新增订阅 `/cmd_vel_nav_raw`。状态日志现在同时输出：

```text
raw=(MPPI 原始线速度, MPPI 原始角速度) raw_alive=...
nav=(平滑后线速度, 平滑后角速度)
safe=(最终安全速度)
```

这能直接定位后续任何零速度首次产生于 MPPI、velocity smoother 还是安全层，避免
再次用最终零速度反推上游。

本轮没有抬高地面高度阈值，2 cm 低矮障碍检测、C++ 点云硬停车、2D 雷达、视觉墙体、
RTAB-Map 和 Cartographer 参数全部保留。

## 复测

```bash
./START_DUAL_2D_3D_NAVIGATION.sh
```

在空旷前方选择约 1～2 m 目标。正常状态应出现
`raw_alive=true`，并且 `raw`、`nav`、`safe` 至少在一个周期内为非零。若仍不动，
保留新的 `runtime.log`；新日志已经可以直接指出首次归零层，不再需要重复猜测。

# 2026-07-26 V6.24：完整导航链审计与视觉障碍链修正

## 完整链路结论

本轮按以下顺序逐层复核：

```text
RViz 2D Goal
  -> bt_navigator
  -> SmacPlanner2D
  -> SimpleSmoother
  -> RotationShim + DiffDrive MPPI
  -> /cmd_vel_nav_raw
  -> velocity_smoother
  -> /cmd_vel_nav
  -> safety_fusion_node
  -> /cmd_vel_safe
  -> chassis_node
  -> STM32 AA55 MOVE
```

`dual_3d_2026-07-26_13-28-24/runtime.log` 中，目标、全局路径、平滑路径、Nav2
动作接管和串口 MOVE 接管均成功；碰撞门也曾连续报告
`front=false rotation=false rear=false`，但速度仍为零。由此排除网页/PS2 接管、
串口发送、STM32 和安全融合层是本次零速度的第一原因。第一处异常位于 MPPI
局部轨迹计算或其输入的局部代价地图。

## 修正内容

1. Gemini2 是前向相机，局部和全局 RGB-D STVL 均改为
   `track_unknown_space=false`。相机看不到的侧面和后方不再写成
   `NO_INFORMATION` 并使 MPPI 全部采样轨迹失效。未知空间仍由 Cartographer
   静态层和 360 度 2D 雷达负责。
2. 撤销 V6.22 中错误的 `voxel_min_points=2`，局部 3 cm 和全局 5 cm 深度 STVL
   均恢复为 `1`。C++ 点云生产器已经按 3 cm 体素执行 first-hit 去重，每个体素
   最多只发布一个代表点；下游再要求两个点会随机删除真实低矮障碍物。
3. 修复 RTAB-Map 长期视觉墙始终输出空点云的问题。旧算法只统计单个 5 cm
   XY 柱，日志持续为 `VISUAL_WALL_FILTER ... output=0`。新算法汇总当前柱周围
   `3x3` 邻域后判断垂直结构，阈值调整为至少 4 点、垂直跨度至少 25 cm，并在
   日志中新增 `accepted_columns`。平地仍因没有足够垂直跨度而不会进入视觉墙层。
4. MPPI 的 `model_dt` 保持 `0.067`。15 Hz 控制周期约为 `0.0667 s`，Jazzy
   要求 `model_dt` 不得短于控制周期；向上取整才能通过控制器配置。
5. 保留 V6.23 的三段速度诊断：

```text
raw=(MPPI 原始速度)
nav=(velocity_smoother 输出)
safe=(安全融合最终速度)
```

下一份日志可直接区分 MPPI 没有输出、速度平滑器归零或安全门拦截。

## 未修改项

- Cartographer V13 及 `cartographer_2d_v9_nav_guarded.lua`
- STM32 程序、串口协议、NAVI 里程计和绝对偏航角
- 相机外参和地面标定结果
- 2 cm 低矮障碍物阈值
- C++ RGB-D 前向硬停车门、2D 雷达后退/旋转扫掠圆保护
- RTAB-Map 彩色回环和 RViz 完整 OctoMap 显示
- BT 的 1 Hz 动态重规划、清图、受控倒车及左右脱困旋转

## 下一次验证

只运行：

```bash
./START_DUAL_2D_3D_NAVIGATION.sh
```

启动后必须确认：

```text
Persistent visual wall filter: ... neighborhood=1
Controller frequency set to 15.0000Hz
```

点击前方 1～2 m 的无障碍目标后检查：

1. `raw` 非零：MPPI 已产生速度。
2. `nav` 非零：速度平滑器已正确转发。
3. `safe` 非零：安全门允许运动。
4. `VISUAL_WALL_FILTER` 的 `accepted_columns` 和 `output` 在扫到墙后应大于零。
5. 不应出现 `Controller period more then model dt`；`less then` 只是已有版本的
   建议性警告，不会阻止生命周期激活。

若 `raw` 仍始终为零，保留整份新 `runtime.log`。此时可直接根据局部代价地图和
MPPI 诊断继续处理，不再改 Cartographer、STM32 或相机外参。

# 2026-07-26 V6.25：恢复 Nav2 生命周期

`dual_3d_2026-07-26_14-44-25/runtime.log` 明确记录：

```text
Original error: Controller period more then model dt, set it equal to model dt
Failed to change state for node: controller_server
Failed to bring up all requested nodes. Aborting bringup.
```

V6.24 将 `model_dt` 错误改成 `0.066`，短于 15 Hz 的实际控制周期约
`0.0667 s`，导致 MPPI 在 configure 阶段拒绝启动。RViz 仍记录了三次
`Setting goal pose`，但 Nav2 生命周期未激活，因此目标没有被导航动作服务器接收。

现已恢复已经验证能启动的 `model_dt=0.067`。本次故障与 RViz 标点工具、
Cartographer、STM32、串口、相机外参和碰撞门无关。

同时，一键启动脚本现在会明确检查 `/controller_server` 和 `/bt_navigator`
必须处于 `active [3]`。过去只检查 `/navigate_to_pose` 名字是否出现在 ROS 图中，
即使动作服务器所属生命周期节点没有激活也可能误报启动成功；现在会直接停止并打印
第一条生命周期错误，不再让无效的导航栈继续显示成“可以标点”。

# 2026-07-26 V6.26：导航栈重构为 SmacPlanner2D + RPP

## 最新日志根因

`dual_3d_2026-07-26_15-47-58/runtime.log` 中已经确认了两个互相独立的问题：

1. 旧行为树每秒执行一次 `ComputePathToPose + SmoothPath`，运行期间累计出现
   587 次 `FollowPath` action abort 和 566 次新路径覆盖。MPPI 还记录了 82 次
   `Optimizer fail to compute path`。车并不是缺少全局路线，而是控制任务被持续重启，
   再叠加大量低于底盘有效死区的速度，最终反复绕行且到不了目标。
2. 15:58:02 左右，Cartographer 在同一轮计算了 1281 个历史候选并接纳 657 条约束。
   重复走廊对旧 submap 的大量相似匹配一起进入 pose graph，导致整张地图突然旋转。
   这不是 Nav2 路径线把地图转歪，也不是 RTAB-Map 在发布 TF。

参考项目 `nav3_localization` 使用 Navfn + Regulated Pure Pursuit，证明确定性路径跟踪比
当前随机采样控制更适合这台低速四轮差速底盘。这里保留了 RPP 思路，但没有照搬其硬编码
路径、圆形 footprint 和仅 2D 代价层；全局规划改用 Jazzy 官方 SmacPlanner2D，并继续
融合本项目的 2D LiDAR、3D STVL、视觉长期墙和最终 C++ 碰撞门。

## 新导航决策链

```text
RViz / web NavigateToPose
  -> SmacPlanner2D（5 cm 全局中心路径，内部平滑）
  -> Regulated Pure Pursuit（20 Hz 连续闭环路径跟踪）
  -> velocity_smoother（30 Hz OPEN_LOOP 加减速限制）
  -> 固定 1.45 四轮差速欠转补偿（停止在线学习）
  -> safety_fusion_node
  -> C++ RGB-D + 360° LiDAR 前进/后退/旋转碰撞门
  -> chassis_node
  -> STM32 AA55 MOVE
```

默认导航不再加载 MPPI、Rotation Shim、State Lattice 或独立 SimpleSmoother。
旧 `nav2_dual_3d_mppi_override.yaml` 和 Lattice JSON 只保留作历史对照，不参与
一键启动，也不再由 `setup.py` 复制到正式安装空间。

所有启动入口已统一：

- `nav2_auto_mapping_jazzy.yaml` 已删除旧 DWB、Rotation Shim、State Lattice 和
  Constrained Smoother 参数，只保留公共 Nav2/代价地图配置；
- `open_all.sh` 和 `open_all_log.sh` 改为检查并加载 SmacPlanner2D、RPP 和
  `velocity_smoother`，不再要求旧 Lattice JSON、`short_goal_bt` 或
  `smoother_server`；
- 即使诊断时设置 `ENABLE_STVL=false`，也只回退到基础 2D 代价层，不会暗中重新
  加载历史 MPPI 参数。
- 冻结基线中的 `cartographer_scan_v2_launch.py` 哈希已同步为当前稳定链实际文件
  `ee90d9...e5894`。该文件本身没有在本轮回退或改写；旧哈希会让 `open_all.sh`
  和预检脚本错误拒绝当前已使用的时间同步、yaw 限速和外参入口。

## 规划和控制参数

- 直线最大速度 `0.20 m/s`，对应 `速度.txt` 的导航二档。
- RPP 在约 `0.85 m` 半径弯道自动降到约 `0.12 m/s + 0.14 rad/s`。
- 普通转角连续走弧线；路径初始方向误差超过 `1.10 rad`（约 63°）才原地校正。
- 原地转速 `0.157 rad/s`，并保留 2D 雷达 `0.52 m` 旋转扫掠圆硬保护。
- RPP 对完整 `0.665 m x 0.665 m` footprint 做前向碰撞模拟。
- 目标容差为位置 `0.15 m`、角度 `0.18 rad`。这比传感器和四轮滑移底盘的可重复
  精度更实际，避免到点附近无限左右修正。
- 关闭 RPP 的代价速度缩放，避免在正常膨胀区长期龟速；曲率和到点距离仍会平滑降速。

## 行为树与脱困

- 每 `0.5 s` 检查路径是否仍有效。
- 障碍物使路径失效时立即重规划；路径正常时每 `3 s` 刷新一次。
- 删除行为树中的外部 `SmoothPath`，SmacPlanner2D 只在规划内部平滑一次。
- 失败恢复顺序：清局部/全局代价地图、等待 `0.5 s`、受控后退 `0.25 m @ 0.06 m/s`、
  尝试左右各 `0.35 rad` 小角度旋转、重新规划。
- 倒车同时受 Nav2 footprint、后方 2D 雷达和 scan 超时停车三重约束，不允许盲倒。

## 2D/3D 避障输入

局部代价地图继续融合：

- Cartographer 已扫描静态墙；
- `/scan_timed_v2_filtered` 360° 2D 雷达；
- Gemini2 3 cm 实时深度 STVL；
- `/rtabmap_3d/navigation_walls` 长期视觉墙；
- `0.665 m` 车体 footprint 和 `0.56 m` 膨胀层。

全局代价地图继续融合 Cartographer、2D 雷达、5 cm 深度 STVL 和 RTAB-Map 视觉墙，
因此视觉 SLAM 扫描出的有效垂直墙仍会参与全局绕障。动态深度体素衰减改为局部
`0.6 s`、全局 `0.8 s`，人物离开重新看见空地后能更快从实时导航层消失。

`/rtabmap_3d/octomap_occupied_space` 没有关闭，RViz 的
`OctoMap 3D Occupied (RTAB-Map Optimized)` 仍可手动勾选验证 3D 长期地图。
关闭的只是未使用且体积巨大的完整 OctoMap 序列化消息；已占用点云、RTAB 回环和
视觉墙提取都继续运行。

## 地面滤波

旧算法每帧独立拟合地面，反光点或墙边可以把平面短暂拉斜。新 C++ 地面模型增加：

- 上一帧平面作为下一帧拟合锚点；
- `6 cm` 种子带限制；
- 单帧斜率/高度突变拒绝；
- `0.18` 时间低通；
- 总斜率限制 `0.06`；
- 地面上方直接删除带收紧到 `1.8 cm`。

`1.8-3.5 cm` 的孤立小块只有在邻域不足时才清理；成片的 2 cm 以上真实障碍仍进入
3 cm STVL 和 C++ 碰撞门。

## 导航模式地图防跳变

纯建图文件 `cartographer_2d_v9_tightened.lua` 未修改。只有导航专用
`cartographer_2d_v9_nav_guarded.lua` 改为：

```text
constraint sampling_ratio     0.80 -> 0.15
max_constraint_distance       3.00 -> 2.00 m
min_score                     0.75 -> 0.78
global_min_score              0.80 -> 0.82
constraint match verbose log  true -> false
```

这样仍允许可信回环，但不会让重复走廊在一次优化中产生数百票约束。

## 实车复测

只运行：

```bash
./START_DUAL_2D_3D_NAVIGATION.sh
```

启动横幅必须出现：

```text
Navigation      : true (SmacPlanner2D + Regulated Pure Pursuit)
```

RViz 中：

- 蓝线 `Nav2 Global Path (Smac Internal Smoothing)` 是全局目标路径；
- 橙线 `Nav2 RPP Collision Lookahead Arc` 是当前控制器检查的短期可行弧；
- `REAL-TIME Nav2 Local Costmap` 是实际动态避障层；
- OctoMap 可手动勾选检查，但它不是 15 Hz 最终碰撞停车输入。

建议依次测试：

1. 前方空旷处 2 m 目标：应连续前进，不再每秒停止或重启 FollowPath。
2. 侧前方目标：应平滑弧线跟踪，只有大于约 63°时先原地校正。
3. 路径中临时放人或箱子：蓝线应在约 3 s 内重规划，局部层和硬碰撞门立即保护。
4. 人离开并重新看见空地：局部障碍应在约 1 s 内明显消退。
5. 死路：等待进度超时后应清图、受控后退、小角度转向并重新规划。

测试日志中不应再连续出现 `FollowPath ... Aborting handle`、`Optimizer fail to compute path`
或单轮数百条 Cartographer additional constraints。

### 最终链路审计补充：空点云也是有效观测

`depth_image_to_local_cloud_v21_node` 过去会在地面和噪点全部被过滤后直接丢弃空帧。
这会产生两个错误结果：安全融合把空旷视野误判为点云断流；STVL 收不到“此处已经变空”的
观测，人物或箱子离开后可能继续残留。现在零障碍帧仍会发布合法的空 `PointCloud2`：

- `/local_highres_cloud_v21` 和 `/local_highres_cloud_v21/sensor` 心跳不会因场景空旷而中断；
- STVL 可以利用同一帧的传感器视锥清除旧动态体素；
- 统计字段新增 `empty_published_frames`，这些帧不再计入 `known_dropped_frames`。

这项修改不放宽 2 cm 低矮障碍阈值，也不绕过 2D 雷达、RPP footprint 碰撞检查或 C++ 硬停车门。

# 2026-07-26 V6.27：修复 Nav2 No Map Received 与地面残点

## 日志结论

`dual_3d_2026-07-26_20-46-43/runtime.log` 在启动第 1 秒已经给出直接根因：

```text
bt_navigator terminate called after throwing InvalidParameterValueException
parameter_value_from failed for parameter 'plugin_lib_names': No parameter value set
```

Cartographer、`cartographer_occupancy_grid_node` 和 `/map` 链没有崩溃。真正的问题是
Jazzy 的参数解析器把 `plugin_lib_names: []` 变成了“存在但没有值”的参数，
`bt_navigator` 随即退出；生命周期管理器一直等待它，controller、planner 和两张
Nav2 costmap 都无法进入 active，因此 RViz 的 Nav2 显示 `No Map Received`。

## 修改

1. 从 `nav2_auto_mapping_jazzy.yaml` 完全删除 `plugin_lib_names`。Jazzy 会自动加载内置
   BT 插件，不需要显式空列表。
2. `validate_auto_mapping_jazzy.sh` 新增回归检查，今后只要再次写入该键，预检会直接失败，
   避免同类启动崩溃。
3. 地面拟合日志持续稳定在约 `plane=(0.0025, 0.0164, -0.0025)`，并且每帧已删除约
   `3.67 万` 个地面点，因此没有重做标定或改变平面模型。
4. 仅将平面上方直接删除带由 `1.8 cm` 调整到 `2.0 cm`，与既定低矮噪点边界一致；
   孤立地面残点检查上限由 `3.5 cm` 调整到 `4.0 cm`，仍要求邻域不足才删除。
   成片箱体、墙面和低矮障碍不会仅因高度小于 4 cm 被整片清除。

## 复测

```bash
./validate_auto_mapping_jazzy.sh --build
./START_DUAL_2D_3D_NAVIGATION.sh
```

正常启动必须出现 `bt_navigator` configure/activate、`Managed nodes are active`，
并且 global/local costmap 的 static layer 收到 `/map`。不应再出现
`parameter_value_from failed for parameter 'plugin_lib_names'`。

# 2026-07-26 V6.28：终点判定、即时倒车脱困与导航地图防跳

## 本次日志结论

`dual_3d_2026-07-26_21-09-36/runtime.log` 给出了三条直接证据：

1. 第一个目标为 `(1.21, -2.51)`，下一次标点时车辆已位于 `(1.32, -2.48)`，
   位置误差约 `0.11 m`，已经小于 `0.15 m` 目标容差；任务仍未结束，是因为旧配置还
   强制车辆把 RViz 目标箭头的朝向误差收敛到 `0.18 rad`，所以车辆在目标附近持续转向。
2. 最后一次受困时，Nav2 连续输出 `nav=(0.000,-0.025)`，360 度雷达持续报告
   `rotation_collision=true`，安全层正确将其改成 `safe=(0.000,0.000)`。旧进度检查
   每 15 秒才失败一次，而且旧恢复树把倒车排在清图和等待之后，因此测试结束前看起来
   完全没有倒车。日志中较早的一次 `backup completed successfully` 也证明行为服务器、
   后向运动和安全链本身是可用的。
3. `21:15:19` 下位机里程计静止时仍出现
   `CARTOGRAPHER_POSE_JUMP ... yaw=+5.49deg`。该跳变紧跟在 pose graph 优化完成之后，
   是导航专用 Cartographer 在线历史约束把实时地图整体旋转，并非 RPP 路径显示、
   RTAB-Map 渲染或下位机在该时刻输出了错误角度。

## 导航决策修正

1. 单点导航在车体中心进入目标 `0.15 m` 范围后立即成功，不再强制追随 RViz 箭头朝向。
2. SmacPlanner2D 按 `1 Hz` 使用最新全局/局部代价地图无条件重规划，不再复用最长 3 秒
   的旧路径。2D 激光、Cartographer 静态地图、3 cm/5 cm STVL、视觉墙和实时深度障碍
   均继续参与代价地图。
3. 进度判定改为 8 秒窗口；小于 7 cm 的平移和小于 0.35 rad 的原地摆动不再被误当成
   有效进展。
4. FollowPath 失败后立即清理局部代价地图，并先尝试
   `0.22 m @ 0.06 m/s` 的受控倒车，然后重新规划。
5. 外层恢复顺序改为：`0.30 m` 受控倒车、左转 `0.35 rad`、右转 `0.35 rad`、清图等待。
   倒车始终受 Nav2 footprint、后方 360 度激光、scan 存活检测和最终安全融合层约束，
   没有开放盲倒。
6. RPP 只在路径初始方向误差超过约 77 度时原地对向，普通弯道继续平滑跟踪。

## 导航地图防跳

纯建图脚本继续使用冻结的 `cartographer_2d_v9_tightened.lua`，参数未改变。只有导航版
`cartographer_2d_v9_nav_guarded.lua` 将在线 pose graph 优化延后到正常结束时，并把
最终历史约束限制为 `1.5 m` 内、局部得分不低于 `0.82`、全局得分不低于 `0.87`。
实时局部 SLAM、占用地图发布和 Nav2 均继续运行，但单次错误历史匹配不能再在导航途中
把整张地图突然旋转数度。

复测运行：

```bash
./validate_auto_mapping_jazzy.sh --build
./START_DUAL_2D_3D_NAVIGATION.sh
```

# 2026-07-27 V6.29：动态路径保真与窄门通行

## 两次实车日志结论

`dual_3d_2026-07-26_22-11-00` 的导航和避障整体正常。最新的
`dual_3d_2026-07-27_07-37-58/runtime.log` 进一步表明：

1. 第一段目标从 `(-0.17, -2.20)` 到 `(1.40, 2.39)` 正常成功，证明当前
   SmacPlanner2D、RPP、差速补偿和 2D/3D 避障主链可以继续保留。
2. 07:44 的极限目标中，规划器在同一目标上反复报告
   `no valid path found`。旧行为树每次失败都会立即清空全局代价地图；动态障碍刚被
   清掉时规划可能短暂成功，于是 RViz 蓝线会在障碍重新写回前穿过该区域。
3. 返回门口时，C++ 碰撞门长期报告约 `800` 个 approach 点，安全层把部分有效命令
   从 `0.20 m/s` 缩放到约 `0.018 m/s`。这低于底盘可靠运动区，随后进度检查触发
   倒车和旋转恢复，形成门口徘徊。
4. 最后停车阶段只有 `2` 个 2D 激光点在约 `0.60 m` 处进入前向硬停车框。旧框半宽
   `0.39 m`、减速框半宽 `0.50 m`，均大于 `0.333 m` 车体半宽和 Nav2 实际 padded
   footprint，门框侧边因此可能在车体仍可通过时提前锁车。
5. 日志中没有新的 `CARTOGRAPHER_POSE_JUMP`。07:44 与 07:54 截图中的房间、走廊
   仍保持相同夹角，看到的是整张 `map` 相对 RViz 屏幕坐标的固定旋转，而不是运行中
   某一段地图再次折弯或跳转。因此没有改动冻结的 V9 local SLAM 参数。

## 修改

1. 单次 `ComputePathToPose` / `ComputePathThroughPoses` 失败后先等待 `0.5 s`
   再重试，不再立即清空全局代价地图。只有受控倒车改变车位后，外层恢复流程才允许
   清图，避免真实 2D/3D 障碍在重规划瞬间消失。
2. 全局实时 RGB-D STVL 衰减时间由 `0.8 s` 调整为 `1.5 s`，覆盖一次完整的
   `1 Hz` 重规划周期和短暂相机漏帧；局部动态层仍为 `0.6 s`，人员离开后的实时
   清除速度不变。
3. 前向硬停车框半宽由 `0.39 m` 收紧到 `0.36 m`，即物理半宽 `0.333 m` 加
   `2.7 cm` 最终安全余量。2D 激光仍要求至少两个点，未降低厘米级细障碍检测能力。
4. 前向渐进减速框半宽由 `0.50 m` 收紧到 `0.39 m`，门框处于车体实际扫掠区域之外
   时不再把车辆误判为正前方接近障碍。
5. 新增 `approach_min_linear_speed_mps=0.06`。渐进减速同时缩放线速度和角速度并
   保持曲率，但不会再额外把有效速度压到 `0.018 m/s`；真实硬碰撞、scan 断流、
   后退碰撞和原地旋转扫掠碰撞仍直接输出零速。

未修改：

- `cartographer_2d_v9_tightened.lua` 和导航专用
  `cartographer_2d_v9_nav_guarded.lua`；
- 车体 `0.665 m x 0.665 m` footprint、`0.02 m` padding 和 `0.56 m` 膨胀层；
- RPP 导航二档 `0.20 m/s`、目标判定、差速转弯补偿和倒车安全约束；
- 3 cm 局部视觉避障、5 cm 视觉墙和 RTAB-Map / OctoMap 长期 3D 建图。

## 定向复测

```bash
./validate_auto_mapping_jazzy.sh --build
./START_DUAL_2D_3D_NAVIGATION.sh
```

只需重点复测两项：

1. 在全局路径前临时放置障碍。蓝线应保持绕开，不应在一次
   `no valid path found` 后瞬间穿过障碍；障碍移走并重新看到空地后，约
   `1.5-2.5 s` 内应恢复路径。
2. 先出门，再从同一扇门返回。日志中的正常门框点不应长期造成
   `approach_scale=0.30` 和低于 `0.06 m/s` 的最终前进命令；车体真正进入
   `0.36 m` 半宽硬停车走廊的障碍仍必须触发 `front_collision_lock`。

# 2026-07-27 V6.30：全局路径迟滞防横跳与视觉速度 EKF 对照版

## 最新日志结论

`dual_3d_2026-07-27_10-06-15/runtime.log` 中没有
`CARTOGRAPHER_POSE_JUMP`、`NAVI_YAW_RATE_LIMIT`、TF 超时或 costmap 消息丢弃，
因此本次截图中的“出门路线与穿墙角路线来回横跳”不是地图坐标突然旋转，也不是
下位机偏航角异常。

同一目标执行期间，旧行为树约每秒无条件执行一次 `ComputePathToPose`，并持续打印
`Passing new path to controller`。门口动态代价在相邻两次更新间轻微变化时，
SmacPlanner2D 会在两条总代价接近的路线间换边；RPP 随后多次报告
`detected collision ahead`，整段测试累计出现 14 次 `Failed to make progress`。
这与 RViz 中蓝线在门口路线和墙角路线之间切换的现象一致。

## 路径与障碍修正

1. 单点和多点行为树都改为每 `0.5 s` 检查一次当前路径：
   - 新障碍使路径失效时立即重新规划；
   - 目标更新时立即重新规划；
   - 路径仍安全时保持，不再每个周期换成另一条近似等价路线；
   - 最长保持 `4.0 s` 后强制从当前代价地图重新规划，始终短于 TF 缓存周期。
2. SmacPlanner2D 的 `cost_travel_multiplier` 调整为官方平衡值 `2.0`，使路径更重视
   与膨胀区、门框和墙角保持距离。
3. Jazzy 的 SmacPlanner2D 不读取 Hybrid/Lattice 使用的 `smooth_path` 开关，
   因此改用真正生效的 `GridBased.smoother.max_iterations=0` 旁路搜索后平滑。
   5 cm 栅格 A* 已经完成 footprint 碰撞判定，RPP 会在控制阶段生成连续运动；
   再次几何平滑可能把安全折线切过窄门拐角。
4. 全局 RGB-D STVL 的未观测衰减由 `1.5 s` 延长为 `4.5 s`，覆盖完整的路径保持
   窗口和短暂相机漏帧。相机重新看到空地时，清除射线仍可提前移除动态障碍；
   局部实时层继续保持 `0.6 s`，不降低人员离开后的局部响应速度。
5. 规划失败时仍不会先清空真实障碍。受控倒车、后方 2D 雷达、旋转扫掠圆、
   RPP footprint 和 C++ 最终碰撞门全部保留。

## EKF 融合拓扑

新增的融合版不是把视觉位姿、轮式位姿和多个陀螺仪全部强行平均，而是先采用可验证、
无 TF 闭环的保守结构：

```text
STM32 /odom
  - 绝对 yaw
  - 车体前向速度 vx
                    \
                     -> robot_localization EKF -> /odometry/filtered
                    /                            -> odom -> base_link（唯一发布者）
Gemini2 /visual_odom
  - 视觉前向速度 vx
  - 视觉横向速度 vy（用于观察轮胎侧滑）

/odometry/filtered -> Cartographer 运动预测
Cartographer map -> base_link -> /cartographer_pose_odom -> RTAB-Map 3D 图
```

关键约束：

- 融合模式下 `chassis_node` 不发布 `odom -> base_link`，该 TF 只由
  `robot_localization` 发布。
- 不融合视觉绝对 `x/y/yaw`，避免视觉里程计的起始坐标和累计漂移拖动 2D 地图。
- 不重复融合 `/imu_cartographer`。它与 `/odom` 的绝对 yaw/角速度来自同一份
  STM32 NAVI 数据，重复作为独立传感器会制造虚假的高置信度。
- 暂不加入 Gemini2 六轴 IMU。其轴向、安装外参、时钟和协方差尚无实车验证，
  直接加入比暂时不用更容易造成转弯偏航。
- 视觉跟踪丢失时不发布“假零速度”；EKF 自动退回 STM32 输入并继续短时预测。
- EKF 使用 `50 Hz` 当前时刻预测、`0.5 s` 延迟测量历史和异常拒绝门限。
- RTAB-Map 始终使用 Cartographer 修正后的 `/cartographer_pose_odom`，不会因为
  开启 EKF 而脱离 2D 地图坐标。

EKF 能改善轮胎侧滑或轮速噪声造成的短时运动预测，从而有机会减轻局部双墙和墙体
加厚；它不能替代 Cartographer 扫描匹配，也不能修复错误的全局回环。导航版现有
`cartographer_2d_v9_nav_guarded.lua` 仍负责阻止在线错误历史约束旋转整张地图。

## 启动与验收顺序

先验证路径修正，不要一次同时改变两个变量：

```bash
./validate_auto_mapping_jazzy.sh --build
./START_DUAL_2D_3D_NAVIGATION.sh
```

在同一个出门目标上观察：

1. 蓝色全局路径不应再每秒在门口与墙角之间来回切换。
2. 新障碍写入全局代价地图后，旧路径应在约 `0.5 s` 内失效并绕开。
3. 有效路径正常保持约 4 秒后刷新；障碍移走且相机重新看到空地后可提前恢复。
4. 蓝线不能穿过青色致命区、红色膨胀区或 Cartographer 静态墙。

路径验收通过后，再单独测试 EKF 对照版：

```bash
./START_DUAL_2D_3D_NAVIGATION_VISUAL_FUSION.sh
```

启动横幅必须包含：

```text
Visual EKF      : true
odom TF owner   : robot_localization
2D authority    : Cartographer V13 + /odometry/filtered
```

另开终端检查：

```bash
ros2 topic hz /odom
ros2 topic hz /visual_odom
ros2 topic hz /odometry/filtered
ros2 topic hz /cartographer_pose_odom
ros2 run tf2_ros tf2_echo odom base_link
```

验收要求：

- `/odom` 与 `/odometry/filtered` 应接近 50 Hz，`/visual_odom` 应稳定输出且不连续丢失。
- 终端不得出现 `multiple authority`、重复 `odom -> base_link` 或 EKF 诊断错误。
- 用与普通导航版完全相同的直线、90 度转弯、出门返回路线比较墙厚和夹角。
- 若视觉里程计长时间低于约 8 Hz、连续丢失或融合版反而加重双墙，立即回到
  `START_DUAL_2D_3D_NAVIGATION.sh`；普通导航入口没有被 EKF 覆盖。

# 2026-07-27 V6.31：低矮障碍持久记忆与重新观测清除

## 问题根因

此前所谓“长期视觉记忆”实际只覆盖 `/rtabmap_3d/navigation_walls`。该点云要求同一
XY 柱具有足够的垂直跨度，因此会保留墙体，但会主动排除地面和低矮物体。真正负责箱子、
脚、门槛等低矮障碍的 Gemini2 STVL，局部层仅保留 `0.6 s`、全局层仅保留 `4.5 s`。
相机转离障碍后，低矮体素会自动衰减，随后局部控制器和全局规划器都可能把该位置重新当成
可通行区域。这就是“正面能避开，转弯看不见后撞上”的直接原因。

## 修改

1. 局部 3 cm 与全局 5 cm RGB-D STVL 均改为 Jazzy 官方支持的
   `decay_model: 2` 持久模式。未被重新观测的障碍不再按时间自动消失。
2. 保留同一过滤点云作为清除源。相机重新看到该空间为空后，视锥清除约连续确认
   `0.6 s` 才删除旧体素；障碍离开后不会永久形成幽灵占用。
3. 修正 STVL 清除视锥参数含义：`min_z/max_z` 是相机沿光轴的近远裁剪距离，不是
   `base_link` 高度。现设置为 `0.20..3.50 m`，与点云有效距离一致：
   - 小于 20 cm 的相机盲区不会被错误清空。
   - 3.5 m 内已经标记的障碍均能在重新观测为空后被清除。
4. 持久层继续只接收已经经过自车裁剪、标定地面平面、空间滤波、时间滤波和体素邻域
   去噪的 `/local_highres_cloud_v21/sensor`，不把原始深度图或完整 OctoMap 直接灌入
   Nav2，避免把反光地面永久记成墙。
5. RViz 新增默认关闭的
   `Nav2 Persistent Low Obstacle Memory (3 cm)`，订阅
   `/local_costmap/voxel_grid`。它显示的才是当前真正参与 Nav2 的低矮障碍记忆；
   `OctoMap 3D Occupied` 仍只用于检查 RTAB-Map 长期 3D 地图。
6. 一键启动新增 `/local_costmap/voxel_grid` 就绪检查；预检脚本新增持久模型、
   近距离盲区和清除远距的契约检查，防止以后误改回短时衰减。
7. 单目标和多目标行为树移除全部 `ClearEntireCostmap`。旧恢复动作会在倒车或旋转前
   连同真实障碍一起清空 STVL，等价于绕过持久记忆。现在仍按“短倒车、长倒车、左转、
   右转、等待重新观测”的顺序脱困，但所有动作都保留当前 2D/3D 占用并继续接受
   footprint、后向雷达和旋转扫掠圆的碰撞校验。
8. C++ 点云节点将 STVL 标记流与清除流分开：
   - `/local_highres_cloud_v21/sensor` 始终负责标记检测到的障碍。
   - `/local_highres_cloud_v21/clear_sensor` 仅在有效深度采样比例不低于 5% 时发布。
   - 黑帧、大片反光无效深度或相机异常不会再把空点云误当成自由空间；正常地面被完整
     过滤后的空点云仍会发布到清除流，使已经移走的障碍能够消失。

## 未修改

- Cartographer V13 建图与导航专用闭环防跳参数。
- SmacPlanner2D、RPP、行为树、目标判定和受控倒车脱困。
- STM32、AA55 协议、底盘绝对 yaw 与速度二档。
- RTAB-Map 数据库、视觉回环及 OctoMap 3D 长期地图。

## 实车验收

运行：

```bash
./START_DUAL_2D_3D_NAVIGATION.sh
```

RViz 勾选 `Nav2 Persistent Low Obstacle Memory (3 cm)` 后按顺序测试：

1. 在车前方放置低矮箱子，确认橙色 3 cm 体素出现。
2. 原地转动，使相机完全看不到箱子；橙色体素必须继续留在原世界坐标，规划路径不得穿过。
3. 转回并移走箱子，让相机重新看见原位置；体素应在约 `0.6-1.0 s` 内消失。
4. 把障碍贴近相机 20 cm 盲区后转向；记忆不能因为深度失效而被提前清除，最终仍由
   360° 雷达、车体 footprint 与 C++ 碰撞门保护。

日志启动横幅必须包含：

```text
Obstacle memory : persistent 3 cm local / 5 cm global; re-observation clears
```

# 2026-07-27 V6.32：持久障碍三帧确认与 RViz 中途停止

## 日志结论

`dual_3d_2026-07-27_11-52-33/runtime.log` 与
`dual_3d_2026-07-27_12-47-22/runtime.log` 中，Nav2 已经收到目标，串口与最终安全融合也没有
持续锁死运动；真正阻止车辆启动的是 planner 反复报告 `no valid path found`。截图中几乎整张
地图变成蓝紫色并非单纯配色：持久 STVL 将每一帧剩余的少量地面/深度散点永久累计，再经
footprint 膨胀后覆盖了大面积可通行区域。

## 修改

1. C++ 深度点云节点新增独立的持久标记时间确认器：
   - 同一 3 cm 体素或相邻一格必须连续 3 个处理帧都观测到，才有资格进入
     `/local_highres_cloud_v21/sensor`。
   - 计数严格使用上一帧状态；同一帧内相邻的多个点不能伪装成“连续三帧”。
   - 中断一帧即重新累计，孤立反光点、飞点和偶发地面凸点不会写成永久障碍。
2. 实时安全没有等待三帧。`/local_highres_cloud_v21` 仍以单帧约 15 Hz 直接输入 C++
   碰撞门；检测到近障碍时依旧立即减速/停车。
3. 标记流和清除流继续分离：
   - `/sensor` 是三帧确认后的持久标记。
   - `/clear_sensor` 是当前有效的完整过滤点云，用于在障碍移走且重新看见空地后清除旧体素。
   - 黑帧或有效深度不足 5% 时仍禁止清除历史记忆。
4. 局部 3 cm、全局 5 cm STVL 继续使用 `decay_model: 2`，所以已经确认的真实墙体或低障碍
   在相机转走后仍保留；本次只阻止噪点进入记忆，没有恢复短时自动衰减。
5. RViz 增加 `3-Frame Confirmed Persistent Marks` 调试层，可与实时点云和最终
   `Nav2 Persistent Low Obstacle Memory (3 cm)` 对照。
6. RViz 加入 Jazzy 官方 `Navigation 2` 面板，并把目标工具切换为 action-aware
   `nav2_rviz_plugins/GoalTool`。导航途中点击面板的 `Cancel` 会取消当前目标并停车；
   这是安全的中途停止，不会关闭 Nav2 lifecycle，也不会清空地图或障碍记忆。
7. 一键启动横幅与预检脚本增加三帧确认、分流 topic 和 RViz 控制面板契约，防止后续误改。

## 未修改

- Cartographer V13、扫描匹配、闭环门槛和地图 TF。
- SmacPlanner2D、RPP、目标容差、速度二档与脱困行为树。
- 2D 雷达、RTAB-Map 视觉墙、OctoMap 长期 3D 地图和 STM32。

## 实车验收

```bash
./START_DUAL_2D_3D_NAVIGATION.sh
```

1. 勾选 `Live Filtered 3D Points` 与 `3-Frame Confirmed Persistent Marks`：快速闪烁的地面点
   只能出现在前者，不能进入后者。
2. 静止放置真实低障碍约 `0.2 s`，确认它进入三帧标记和 3 cm 持久体素层。
3. 转开相机，确认该真实障碍仍留在世界坐标并继续阻止路径穿过。
4. 转回、移走障碍并重新观察原位置，确认旧体素被有效清除。
5. 标一个可达目标，蓝紫色区域不应再覆盖整张自由空间，planner 不应持续报告
   `no valid path found`。
6. 导航中点击 RViz `Navigation 2 -> Cancel`，车辆应立即停车；重新标点后应能再次导航。

# 2026-07-27 V6.33：近期/永久障碍分级与稳定地面伪影隔离

## 新日志结论

分析 `dual_3d_2026-07-27_14-34-02/runtime.log` 和
`dual_3d_2026-07-27_14-38-08/runtime.log` 后确认：

1. V6.32 的三帧确认已经实际运行，并非旧二进制。两次测试中，三帧确认点通常仍占实时
   点云的约 88%–89%，峰值接近 100%。地砖深度波纹是连续稳定的系统误差，单纯把确认
   帧数从 3 增加到 5 或 10 仍会把它判成真实障碍。
2. 导航 `abort` 不是串口断流或手柄抢控制。日志先出现 `no valid path found`、
   `Failed to make progress` 和倒车碰撞拒绝，BT 恢复耗尽后 Nav2 正常结束 Action，
   `safety_fusion_node` 才按现有规则把空闲控制权交回 PS2。
3. 最终失败时 C++ 近碰撞门已经报告前后与旋转区域均为空，但全局规划器仍无路可走，
   证明堵塞来自累计的历史代价地图，而不是眼前真实障碍。

## 修改

1. C++ 深度点云链改为三级安全输出：
   - `/local_highres_cloud_v21`：单帧实时点云，继续直接驱动 C++ 硬碰撞停车。
   - `/local_highres_cloud_v21/sensor`：三帧确认后，再要求点高于拟合地面至少
     `5 cm`，或附近存在至少 `3 cm` 的竖直结构；用于近期障碍层。
   - `/local_highres_cloud_v21/persistent_sensor`：三帧确认后，再要求点高于地面
     至少 `8 cm`，或附近存在至少 `6 cm` 的竖直结构；用于永久障碍层。
2. 新增每帧 XY 体素柱高度统计。宽阔、平坦且重复出现的瓷砖误差即使连续存在，也没有
   真实竖直边缘，因此不能再进入永久障碍；墙、箱体、门框和障碍边缘仍可通过。
3. 原深度 STVL 改为近期记忆：
   - 局部 3 cm 层线性衰减 `4 s`。
   - 全局 5 cm 层线性衰减 `8 s`。
   - 它能覆盖短暂转头，但残余假点不会无限累计。
4. 新增严格永久 STVL：
   - 局部 3 cm、全局 5 cm，使用 `decay_model: 2`。
   - 只订阅 `/persistent_sensor`。
   - 相机重新看见空地后仍由独立 `/clear_sensor` 清除。
5. `/local_costmap/voxel_grid` 只由严格永久层发布，避免多个 STVL 抢同一 RViz topic。
   RViz 新增 `3-Frame Recent Geometry Marks (4s Local)`、
   `Geometry-Qualified Permanent Marks` 和
   `Nav2 Robust Persistent Obstacle Memory (3 cm)` 三个分层检查项。
6. 一键启动增加永久流就绪检查和分级记忆横幅；Jazzy 预检同步验证两层 topic、衰减模型、
   几何阈值及唯一 voxel-grid 发布者。
7. `ReadMe.md` 已同步更新三条避障链及各 RViz 显示项。

## 安全边界

- 2–5 cm 的极低平面目标无法仅凭当前 3 cm 深度体素可靠地区分于 2–4 cm 地砖深度误差，
  因此不会无条件永久写入全局地图。
- 它仍保留在约 15 Hz 的单帧 C++ 碰撞链中，车辆靠近时会停车；具有可见竖直边缘时也会
  进入近期或永久规划层。
- Cartographer 2D 地图、过滤雷达、RTAB-Map 视觉墙、OctoMap、RPP、行为树、速度和
  STM32 均未修改。

## 实车验收

只需重新运行：

```bash
./START_DUAL_2D_3D_NAVIGATION.sh
```

1. 先不要标点，静止观察 20 秒；局部代价地图的空地不应继续向外累计成整片蓝紫色。
2. 依次勾选 `Live Filtered 3D Points`、`3-Frame Recent Geometry Marks` 和
   `Geometry-Qualified Permanent Marks`。地面波纹可以在实时层短暂出现，但不应大面积
   进入后两层。
3. 放置高度大于 8 cm 的箱体并保持约 1 秒，确认永久标记出现；转开相机后仍应保留。
4. 移走箱体并重新完整观察原位置，确认永久体素被清除。
5. 标记同一可达目标；规划器不应再因地面累计污染持续报告 `no valid path found`。
6. 若 Action 正常结束或明确失败后显示 `Nav2 idle: control returned to PS2`，这是既定交接；
   若仍失败，保留本次完整 `runtime.log`，重点回传新的 `live / temporal / recent /
   persistent` 四组点数。

# 2026-07-27 V6.34：零运动反馈熔断、组件版本门禁与稳定代价层回退

## 本次飞图的确定原因

对比以下三份实车日志：

- 稳定参考：`dual_3d_2026-07-27_14-38-08/runtime.log`
- 故障一：`dual_3d_2026-07-27_15-28-56/runtime.log`
- 故障二：`dual_3d_2026-07-27_15-33-12/runtime.log`

得到的证据如下：

1. 三次均使用 `cartographer_2d_v9_nav_guarded.lua`、`/odom`、
   `/scan_timed_v2_filtered`，且 `Visual EKF=false`。因此本次不是 EKF 接管、雷达方向
   或 Cartographer 参数被切换。
2. 稳定日志在 PS2 行驶期间持续得到有效 `yaw/vx/wz`；两份故障日志中，STM32
   `mcu_tick` 正常每 20 ms 增长，但全部 0x07 帧的 `yaw/vx/wz` 连续数千帧严格为零。
3. 车实际转动而 `/odom` 与 `/imu_cartographer` 同时声称车完全静止，Cartographer
   只能依靠变化的扫描自行解释运动，结果就是墙体围绕车旋转、旧房间与新房间叠加。
4. 故障日志还同时加载了 V6.33 的新 YAML 和 V6.32 的旧 C++ 点云程序。旧程序启动
   横幅仍为 `persistent_mark=confirmed/3 frames`，没有新源码应有的 `recent_guard`、
   `persistent_guard`。这是 Git 文件时间早于缓存目标文件时，普通增量编译没有重建
   C++ 导致的混合版本。

## 修改

1. C++ 点云节点加入编译期版本 `v6.34`：
   - 一键脚本每次启动都只清理并重编小型 `local_depth_cloud_cpp` 包；
   - 启动前直接检查 ELF 中是否包含 `v6.34`；
   - 节点启动后再次读取 `/depth_image_to_local_cloud_v21` 的
     `pipeline_version`；
   - 任一检查不一致就结束启动，不再允许“新版 YAML + 旧版二进制”的半启动状态。
2. 撤回 V6.33 尚未经过实车验证的第二套永久 STVL：
   - 局部代价地图只保留一个三帧确认、3 cm、4 秒衰减的 RGB-D 层；
   - 全局代价地图只保留一个三帧确认、5 cm、8 秒衰减的 RGB-D 层；
   - Cartographer 静态墙、2D 雷达和 RTAB-Map 过滤后的垂直墙仍完整保留；
   - `/local_costmap/voxel_grid` 改由唯一的局部 RGB-D 层发布，避免两套 STVL
     重复膨胀和地面伪影永久累计。
   - `/persistent_sensor` 暂时只作为严格几何候选调试流，不再直接写入 Nav2。
3. 底盘节点新增跨传感器 NAVI 运动反馈看门狗：
   - 正常静止时不因 yaw 恰好为 0 而误报；
   - 分别监控线速度反馈和角速度/绝对 yaw 反馈，单通道冻结也能识别；
   - 若 35 字节编码器帧显示车在动，而 0x07 对应通道保持静止，连续 0.25 秒即触发；
   - 若 Cartographer 在 0.75 秒内检测到至少 8 cm 平移或 3 度旋转，而 0x07
     对应通道仍保持静止，同样触发。
4. 触发后输出唯一关键字 `NAVI_MOTION_FEEDBACK_FAULT`，并执行锁存保护：
   - 软件急停状态立即置位；
   - 通过 0x01 零速 MOVE 从 PS2 接管并每 0.1 秒持续保持零速；
   - Nav2、网页和自动 PS2 归还都不能解除；
   - 必须结束本次进程并修复/恢复 0x07 后重新启动，防止坏里程继续污染地图。
5. 当 `yaw/vx/wz` 连续 100 帧严格为零但 tick 正常增长时，提前输出
   `NAVI_FEEDBACK_UNVERIFIED`。这只是明确警告，不会把合法的零度静止误判为故障。
6. `PUBLISH_LATENCY` 现在附带 `navi_watchdog`、编码器帧数及编码器观测速度，下一份
   日志可以直接判断当前固件是否仍同时发送 35 字节编码器帧。

## 未修改

- Cartographer V13 及 `cartographer_2d_v9_nav_guarded.lua` 的全部参数；
- 2D 雷达方向、外参和滤波输入；
- RPP、SmacPlanner2D、行为树、目标容差与速度设置；
- RTAB-Map 数据库、视觉回环和 OctoMap 长期 3D 地图；
- STM32 程序与通讯协议。

## 下一次实车验证

只运行：

```bash
./START_DUAL_2D_3D_NAVIGATION.sh
```

1. 必须先看到：

```text
[build] Cleared cached local_depth_cloud_cpp for version v6.34
[ready] /depth_image_to_local_cloud_v21 pipeline_version=v6.34
[ready] All three layers are running
```

   未出现最后一行前不要动车。
2. 第一次只做约 0.3 米低速直行，再做约 15 度低速转动；终端中的 `[NAVI]`
   必须出现非零 `vx`、`yaw` 或 `vz`。
3. 若 0x07 仍然全零，系统应在短窗运动确认后打印
   `NAVI_MOTION_FEEDBACK_FAULT` 并接管停车。此时不要继续建图，保留日志并重启/检查
   下位机的 0x07 数据源。
4. 若 0x07 正常，继续完成一次房间内 90 度转弯；地图不得再出现围绕车体旋转或瞬间
   复制房间。

# 2026-07-27 V6.35：启动期运动硬门禁

V6.34 已阻止运行中的 0x07 冻结继续毁图，但 RViz 会先于完整感知链打开；操作者仍可能在
最终就绪检查完成前用 PS2 移动车辆。现在导航版增加独立的启动期运动门禁：

1. `chassis_node` 启动后先接管 MOVE，并每 0.1 秒持续发送四轮零速。
2. 启动期间 PS2 归还、网页运动命令和 `/cmd_vel_safe` 均不能穿透门禁。
3. 一键脚本必须依次通过 C++ `v6.34` 版本、雷达、相机、点云、RTAB-Map、OctoMap、
   局部/全局代价地图、Nav2 lifecycle、导航 Action 和 Frontier 服务检查。
4. 全部检查通过后，启动器才向 `/robot/system_ready` 发布 `true`；底盘解除零速门禁并按既有
   逻辑自动归还 PS2。
5. 任一启动检查失败时不会发布就绪信号，底盘保持零速直到 ROS 进程退出。
6. `NAVI_FEEDBACK_UNVERIFIED` 现在同时打印完整 20 字节 `raw_hex`，下次可直接判断是
   上位机解析偏移错误，还是 STM32 原始 0x07 运动字段本身为零。
7. `validate_auto_mapping_jazzy.sh` 已同步为 V6.34 的单套有界 STVL、RViz 新图层名、
   C++ 版本门禁和 `/robot/system_ready` 契约；旧的永久 STVL 或旧版二进制再次混入时，
   预检会直接失败，不能进入实车运行。

导航版允许移动前必须看到：

```text
[ready] Startup motion interlock released; PS2/Nav2 motion is enabled.
[ready] All three layers are running. Move slowly for the first 10 seconds.
```

纯建图入口不启用该导航启动门禁，保持原有控制方式不变。

# 2026-07-28 V6.36：修复启动门禁永久锁住导航与 PS2

## 日志结论

对比 `dual_3d_2026-07-28_10-50-30`、`10-52-35` 和 `11-00-43`：

1. 三次 0x07 均已有正常非零绝对 yaw，MCU tick、雷达、Cartographer 和新版
   `pipeline=v6.34` 正常，没有触发 `NAVI_MOTION_FEEDBACK_FAULT`。
2. 第三次 Nav2 已接受目标并持续产生约 `0.20 m/s` 的安全速度，说明规划器、RPP 和
   `safety_fusion_node` 没有把速度清零。
3. 启动器只打印到 `/scan_timed_v2_filtered` 就绪，之后再也没有打印最终门禁释放。
   根因是 `ros2 topic info /map` 没有进程级超时；ROS2 CLI 查询卡住后，启动器永远无法
   发布 `/robot/system_ready=true`。底盘因此持续发送 MOVE 零速，Nav2 与 PS2 同时失效。

## 修改

1. `/robot/system_ready` 的一次性 topic 改为可确认的
   `/robot/set_system_ready` (`std_srvs/SetBool`) 服务握手。启动器只有收到
   `success=True` 才报告门禁已释放。
2. `topic info`、Action/Service 图查询和参数查询全部增加独立进程硬超时，并使用真实截止
   时间计时；单次 ros2cli/DDS 发现阻塞不能再冻结整个启动器。
3. 启动检查重新分级：
   - 0x07 `/odom`、`/imu_cartographer`、Cartographer 位姿和 `/map` 是运动/建图硬条件；
   - Gemini2 实时点云、Nav2 代价地图、C++ 碰撞门、lifecycle 和 Action 是导航硬条件；
   - RTAB-Map 诊断、OctoMap 显示、视觉墙和 Frontier 检查放到解锁之后，失败只给出明确警告。
4. 正常退出时底盘先发送零速 MOVE，再发送 PS2 归还；急停或 NAVI 反馈故障状态下仍保持
   锁停，防止错误反馈下恢复运动。
5. Jazzy 预检同步检查服务握手、CLI 硬超时和退出归还契约。

## 验收

运行：

```bash
./START_DUAL_2D_3D_NAVIGATION.sh
```

必须依次看到：

```text
[ready] Confirmed /robot/set_system_ready=true
[ready] Startup motion interlock released; PS2/Nav2 motion is enabled.
[ready] All three layers are running. Move slowly for the first 10 seconds.
```

先只测试 PS2 前后各约 10 cm，再发送一个 0.5 m 内的 Nav2 目标。PS2 与 Nav2 均应能正常
接管，日志不得出现 `NAVI_MOTION_FEEDBACK_FAULT`。正常 Ctrl+C 后 PS2 应立即恢复。

# 2026-07-28 V6.37：修复启动门禁与 Nav2 行为树互相打架

## 两次新日志结论

对比 `dual_3d_2026-07-28_11-56-47`、`12-18-42` 与近期效果较好的
`2026-07-26_22-11-00`：

1. 两次新日志的 STM32 绝对 yaw、轮速、MCU tick、雷达、Cartographer 和 RGB-D 点云均正常，
   没有 `NAVI_MOTION_FEEDBACK_FAULT`，也没有 Cartographer 约束爆发。
2. 全局规划器没有一次 `no valid path found`，因此不是 SmacPlanner2D 突然失去规划能力。
3. 第一轮在底盘放行前约 31 秒就收到目标，第二轮提前约 12 秒收到目标。Nav2 已持续输出速度，
   但 `SYSTEM_STARTUP_NOT_READY` 将实车保持为零速；8 秒后进度检查器必然报
   `Failed to make progress`，行为树提前进入 BackUp/Spin。
4. 第二轮刚放行时执行的已经是 `-0.06 m/s` 倒车恢复，而不是用户下发目标的前进命令。
   第一轮也先经历两次进度失败、五次倒车和五次旋转，之后才有一次正常到点。这就是相较旧版
   明显迟钝、动作不符合预期的直接原因。

## 修改

1. `dual_resolution_3d_slam.launch.py` 新增 `nav_autostart` 入口。直接手工调用 launch 仍默认
   自动启动，保持兼容；一键导航脚本则显式传入 `false`。
2. 一键导航启动时先让 Nav2 lifecycle 保持未激活。RViz 仍然第一时间打开，但提前标点不会让
   控制器在底盘零速门禁里计时和触发恢复。
3. 雷达、里程计、Cartographer 地图、Gemini2、过滤点云和 C++ 碰撞门均有数据后，启动器调用
   `/lifecycle_manager_navigation/manage_nodes` 的 `STARTUP` 命令，一次性配置并激活所有
   Nav2 节点。
4. 生命周期管理服务返回 `success=True`（表示全部托管节点已完成激活）后，立即通过
   `/robot/set_system_ready` 放行底盘。`controller_server`/`bt_navigator` 状态复核、
   Action、代价地图显示、体素显示、RTAB-Map、OctoMap 和 Frontier 检查移到放行后，
   作为非阻塞诊断，避免 CLI 串行探测再制造几十秒死区。
5. 未修改 Cartographer V13、SmacPlanner2D、RPP、目标容差、速度、行为树、C++ 碰撞算法，
   也未修改当前局部/全局 RGB-D 障碍记忆的 4 秒/8 秒设置。

## 复测

只运行：

```bash
./START_DUAL_2D_3D_NAVIGATION.sh
```

等待出现：

```text
[ready] Nav2 lifecycle manager completed startup
[ready] Confirmed /robot/set_system_ready=true
[ready] Startup motion interlock released; PS2/Nav2 motion is enabled.
```

再分别测试 PS2 前后 10 cm 和一个前方 0.5-1.0 m 的 Nav2 目标。若在放行前点过目标，放行后
重新点一次。新日志中第一个 `Begin navigating` 前不得再存在
`SYSTEM_STARTUP_NOT_READY blocked cmd_vel`，正常短距离目标也不得先执行 `Running backup`。

# 2026-07-28 V6.38：启动即归还 PS2，并抑制临时低障碍触发整套脱困

## 日志结论

分析 `dual_3d_2026-07-28_13-31-58/runtime.log`：

1. 导航启动后约 30 秒才出现 `SYSTEM_READY`。这段时间 `chassis_node` 不但拦截
   上位机 Twist，还持续发送 `MOVE + 四轮零速`，因此 STM32 的 PS2 控制权被反复抢走。
   放行后日志虽然打印 `Nav2 idle: control returned to PS2`，约 2 秒后又收到导航目标并接管，
   留给 PS2 的可操作窗口极短。
2. 第二个导航目标开始时，实时碰撞门只在最初约 2 秒锁停，随后持续报告
   `collision=false`，说明“绕过障碍后突然倒车”不是相机硬碰撞误报。
3. 全局 RGB-D STVL 会把确认障碍保留 8 秒，但原行为树的新路径失败后只等待一次
   0.5 秒。13:35:50 左右障碍实时点已经消失，路径记忆仍短暂有效；行为树随即取消
   FollowPath，执行 0.30 m 倒车、左右旋转和多轮恢复，最终在 13:36:36 `Goal failed`。

## 修改

1. 导航版初始化时直接发送 `STARTUP_PS2_RELEASE`。`system_ready` 现在只门禁上位机
   `/cmd_vel_safe`，未就绪回调不再发送会重新接管下位机的 MOVE 零速帧。
2. Nav2 真正收到活动目标后仍自动发送 MOVE 接管；成功、取消或失败后仍按既有延时归还 PS2。
   急停、NAVI 反馈冻结和进程退出时的零速保护保持不变。
3. 单目标和多目标行为树的临时规划重试从 `0.5 秒 × 1 次` 改为
   `0.5 秒 × 最多 20 次`。动态障碍短暂封路时先安全等待并持续重新规划约 10 秒，
   覆盖全局视觉记忆 8 秒的衰减窗口；只有持续阻塞才进入脱困。
4. 外层完整脱困轮数从 8 轮限制为 4 轮，避免一次临时障碍演变成长时间反复
   倒车和左右旋转，同时保留持续死路所需的受控后退与小角度转向。
5. RPP 进度检查窗口从 8 秒调整为 12 秒，使其长于 8 秒全局视觉记忆和约 10 秒
   规划等待，避免控制器抢先把有意停车误判为底盘卡死。
6. Jazzy 验证脚本新增 PS2 启动模式、禁止启动 MOVE 零速抢权、10 秒规划等待窗口
   和有限脱困轮数契约。

## 未修改

- Cartographer V13 与导航专用建图参数；
- SmacPlanner2D、RPP 的速度/转弯参数、二档速度、车体 footprint；
- C++ 近距离硬碰撞停车；
- RGB-D 局部 4 秒、全局 8 秒记忆和三帧几何确认；
- STM32 程序与 AA55 协议。

# 2026-07-28 V6.39：静态墙体可信化与门口软膨胀收窄

## 日志结论

分析 `dual_3d_2026-07-28_14-35-14/runtime.log` 和
`截图 2026-07-28 14-40-59.png`：

1. 动态避障链工作正常，局部 RGB-D、2D 雷达、C++ 近碰撞门和局部/全局 STVL 均已启动。
2. 截图中的蓝红紫色来自 `/local_costmap/costmap`，它只是车周围 `8 x 8 m`
   的滚动局部代价地图。远处没有彩色光圈不等于全局规划器没有地图。
3. Nav2 静态层此前沿用 `lethal_cost_threshold=100`。Cartographer `/map` 发布的是
   `0..100` 概率值，浅灰但真实的墙体小于 100 时会被三值静态层解释成自由空间。
   这会让 Smac 路径穿过当前雷达看不到的旧墙角。
4. RTAB 垂直墙完整点云在本次日志中约每 `5-6 s` 更新一次，但视觉墙 STVL 仅保留
   `3 s`，因此它会在相邻两次更新之间周期性消失。
5. 车体 footprint 已是实测 `66.5 x 66.5 cm`，原来又增加每侧 `2 cm` padding，
   并使用 `0.56 m / 8.0` 的宽软膨胀，门洞中的彩色代价带明显偏宽。

## 修改

1. 局部和全局 costmap 均显式设置：
   - `lethal_cost_threshold: 65`
   - `trinary_costmap: true`
   - `unknown_cost_value: 255`
   - `use_maximum: true`
2. 两个静态层显式绑定 `/map`，订阅完整地图更新，并禁止 footprint 擦除静态墙。
   后续 2D 雷达或视觉清空射线不能覆盖 Cartographer 已确认的墙体。
3. RTAB 过滤垂直墙的局部/全局 STVL 保留时间由 `3 s` 调整为 `15 s`。这能覆盖
   至少两个正常发布周期；RTAB 停止后仍会有界衰减，不会形成永久脏层。
4. 不修改实测车体 footprint。局部和全局 `footprint_padding` 从每侧 `2 cm`
   调回每侧 `1 cm`。
5. 局部和全局软膨胀改为 `inflation_radius=0.52 m`、
   `cost_scaling_factor=10.0`。硬碰撞边界仍由完整矩形 footprint 决定，只收窄门口
   可见的低代价偏好带，不以缩小车体换通行。
6. RViz 将原显示项明确命名为 `Nav2 Local Costmap (8x8 m near robot)`，并新增默认
   关闭的 `Nav2 Global Planner Costmap (Path Audit)`。后者订阅
   `/global_costmap/costmap`，用于直接核对蓝色全局路径是否穿过规划器眼中的致命墙。
7. Jazzy 预检新增静态概率阈值、最大融合、15 秒视觉墙记忆、车体边距、膨胀参数和
   RViz 全局审计层契约。
8. 一键脚本在 Nav2 lifecycle 激活后、放行底盘前，直接读取正在运行的局部/全局
   costmap 参数。任一节点没有实际加载 `lethal_cost_threshold=65` 或
   `inflation_radius=0.52` 都会停止启动并保持运动门禁，不再允许旧参数静默运行。

## 未修改

- Cartographer V13 建图、闭环和 TF 参数；
- RPP 速度、差速转弯、目标容差和行为树；
- RGB-D 动态低障碍 `4 s / 8 s` 记忆；
- C++ 单帧近碰撞急停、STM32 与 AA55 协议；
- RTAB-Map 数据库和 OctoMap 3D 显示。

## 实车复测

```bash
./START_DUAL_2D_3D_NAVIGATION.sh
```

1. 先勾选 `Nav2 Global Planner Costmap (Path Audit)`，暂时取消勾选局部代价地图。
2. 确认 Cartographer 灰色墙在全局 costmap 中具有连续的致命边界和较窄膨胀带。
3. 在同一个出门目标上重复标点三次，蓝色 `/plan` 必须稳定经过门洞，不能在门洞和
   未扫描墙角之间横跳。
4. 门洞若仍不可通过，测量最窄净宽并回传。当前有效车宽为
   `66.5 + 1 + 1 = 68.5 cm`，小于该宽度的门不能安全规划通过。
5. 保留完整 `runtime.log`，并在路径异常时同时截取 Cartographer 地图、全局 costmap
   和蓝色路径，不能只截局部蓝紫层。

# 2026-07-28 V6.40：修复 Nav2/PS2 同时不动与伪急停

## 日志结论

对比 `dual_3d_2026-07-28_15-44-55` 与
`dual_3d_2026-07-28_15-50-27`：

1. 两次 Nav2 生命周期都成功进入 `Managed nodes are active`。RViz 中显示
   `inactive` 不是这两次不能运动的根因。
2. 第一次运行中，`/cartographer_pose_odom` 在车体静止时出现恰好
   `3.00 deg` 的地图修正，被底盘节点误判为真实车体运动并触发
   `NAVI_MOTION_FEEDBACK_FAULT`。该故障会锁存软件急停，因此 Nav2 和
   PS2 都被故意锁为零速。
3. `/cartographer_pose_odom` 是 `map -> base_link` 的全局修正结果，不是
   独立物理里程计。闭环或扫描匹配修正能够改变该坐标，不能用来触发
   需要重启才能解除的底盘硬锁。
4. 第二次运行中，Nav2 于 `15:51:37` 激活，但四项运行参数检查直到
   `15:51:46` 才完成。目标在 `15:51:39` 被接受，控制器在底盘放行前
   已因当前脚印碰撞进入恢复流程。活动目标随后一直占有 MOVE，PS2
   因而无法接管。
5. 当时 RPP 的碰撞异常容忍只有 `0.50 s`，局部静态层也不清除当前
   车体脚印内的 Cartographer 杂点；二者共同放大了启动瞬间的误碰撞。

## 修改

1. 新增 `navi_motion_watchdog_pose_enabled`，导航版默认 `false`：
   - 保留 NAVI/编码器的物理反馈看门狗；
   - 不再用 Cartographer 全局地图修正触发锁存急停；
   - `/cartographer_pose_odom` 仍照常提供给 RTAB-Map 和诊断，不影响
     2D/3D 建图。
2. 启动顺序改为：感知输入就绪 -> 四项 costmap 参数预检 ->
   Nav2 lifecycle 激活 -> 立即调用 `/robot/set_system_ready=true`。
   Action Server 在检查完成前尚未激活，因此提前点击目标不会再让
   进度计时器和恢复树先运行。
3. 仅将局部 `StaticLayer.footprint_clearing_enabled` 改为 `true`，
   清除当前实测车体脚印内的静态杂点。全局静态层仍为 `false`，
   Cartographer 已确认的历史墙体不会被清除。
4. `controller_server.failure_tolerance` 从 `0.50 s` 调为 `2.0 s`，
   给滚动局部 costmap 完成首轮更新和脚印清理的时间。
5. 新增 `nav_zero_command_cancel_sec=25.0`。活动目标连续 25 秒没有任何
   非零的最终安全速度时，底盘节点通过 NavigateToPose 的 CancelGoal
   服务取消该目标；Action 进入空闲后，原有交接逻辑自动归还 PS2。
   这只处理“恢复动作全部被挡住并长期占权”的死锁，不改变正常导航。
6. Jazzy 预检同步验证地图位姿硬锁关闭、25 秒停滞取消、启动顺序、
   局部/全局静态脚印策略和 2 秒控制器容忍。
7. Jazzy 官方 Navigation 2 面板只在构造时读取一次 lifecycle 状态，
   而本项目为尽早显示相机/雷达会先打开 RViz。启动器现在仍立即打开
   RViz，但会在 Nav2 激活并解除运动联锁后自动刷新一次窗口，使面板
   重新读取真实的 `active` 状态；不再需要手动点击 `Startup`。

## 未修改

- Cartographer V13 Lua 建图、闭环和 TF 参数；
- SmacPlanner2D 路径权重、RPP 速度、转弯参数与目标容差；
- RGB-D 低障碍 `4 s / 8 s` 记忆、RTAB 视觉墙与 OctoMap；
- C++ 近距离碰撞门、车体 footprint、STM32 与 AA55 协议。

## 实车复测

只运行：

```bash
./START_DUAL_2D_3D_NAVIGATION.sh
```

必须等终端依次出现：

```text
[startup] Verifying the static-wall and doorway contract before Nav2 activation...
[ready] Nav2 lifecycle manager completed startup
[ready] Confirmed /robot/set_system_ready=true
[ready] Startup motion interlock released; PS2/Nav2 motion is enabled.
```

随后先用 PS2 前后各移动约 10 cm，再发送一个前方 `0.5-1.0 m` 的短目标。
不要点击 RViz Navigation 2 面板里的 `Startup`，生命周期由一键脚本唯一管理。
新日志不得出现 `cartographer_reports_motion_while_navi_static`；若目标确实被
持续障碍封死，应在 25 秒后出现 `NAV_STALL_CANCEL`，随后出现
`Nav2 idle: control returned to PS2`。

# 2026-07-28 V6.41：修复生命周期前误查参数导致整套进程自动退出

## 日志结论

检查以下三次运行：

- `dual_3d_2026-07-28_17-21-43`
- `dual_3d_2026-07-28_17-23-42`
- `dual_3d_2026-07-28_18-36-13`

三次都不是点击导航点后 Nav2、Cartographer 或相机节点崩溃。真实时间线完全一致：

1. 2D 雷达、NAVI、Cartographer、Gemini2、RTAB-Map、局部点云和 C++ 碰撞门均正常启动。
2. Nav2 lifecycle 尚未执行 `startup`，`/robot/set_system_ready` 也始终没有释放。
3. 启动器提前查询
   `/local_costmap/local_costmap lethal_cost_threshold`，连续 10 秒只得到
   `unavailable`。
4. 启动器将该结果当成配置错误，主动执行
   `startup_or_runtime_check_failed`，向整套 ROS 进程发送 SIGINT。
5. 因此日志中大量 `signal_handler(SIGINT/SIGTERM)` 是启动器主动收尾，不是节点自己崩溃。
   目标点击时间恰好与等待窗口重叠，造成了“点一下就卡死并退出”的表象。

根因是 V6.40 对 Jazzy lifecycle 的假设错误：`controller_server` 和
`planner_server` 尚未完成 configure 时，嵌套的局部/全局 costmap 参数服务还不可用，
不能在 Nav2 激活前用 `ros2 param get` 查询。

## 修改

1. 新增启动前 YAML 源配置校验，直接解析实际传给 Nav2 的 costmap 与 RPP override：
   - 局部/全局 `lethal_cost_threshold=65`；
   - 局部/全局 `inflation_radius=0.52`；
   - STVL 导航版必须包含 Cartographer `static_layer`。
   配置不正确时在 RViz 和 ROS 进程启动前直接报出具体文件与字段。
2. 生命周期顺序改为：
   - 校验 YAML 源配置；
   - 等待 2D/3D 安全输入；
   - 激活全部 Nav2 lifecycle 节点；
   - 立即调用 `/robot/set_system_ready=true`；
   - 刷新一次 RViz Navigation 2 面板；
   - 最后读取运行态 costmap 参数做二次审计。
3. 运行态 `ros2 param get` 改为非致命诊断。CLI 发现延迟或暂时查询不到时只输出
   `WARNING`，不会再关闭 Cartographer、RTAB-Map、相机与 Nav2。
4. 保留运行态四项核对；查询成功时仍会明确打印局部/全局墙体阈值和膨胀半径，
   便于确认安装空间实际加载了当前源码。
5. Jazzy 验证脚本同步检查新顺序，禁止以后再次把未 configure 的 costmap 参数服务
   当作生命周期前置条件。

## 未修改

- Cartographer V13 与 `cartographer_2d_v9_nav_guarded.lua`；
- SmacPlanner2D、RPP、行为树、速度和目标容差；
- 局部/全局 STVL、RGB-D 障碍记忆、视觉墙与 OctoMap；
- C++ 碰撞门、NAVI/编码器看门狗、STM32 与 AA55 协议。

## 实车复测

只运行：

```bash
./START_DUAL_2D_3D_NAVIGATION.sh
```

必须看到：

```text
[ready] Navigation source contract: static walls=65, inflation=0.52m
[ready] Nav2 lifecycle manager completed startup
[ready] Confirmed /robot/set_system_ready=true
[ready] Startup motion interlock released; PS2/Nav2 motion is enabled.
```

V6.42 起 RViz 保持同一个进程，不再自动关闭并重开。先验证 PS2 能前后移动，再发送一个前方
`0.5-1.0 m` 的短目标。新日志不得再出现：

```text
lethal_cost_threshold expected=65 actual=unavailable
startup_or_runtime_check_failed: Local costmap did not load
```

# 2026-07-28 V6.42：修复 RViz 自动重启、原地转向永久误锁与相机断流占权

## 三次日志结论

检查以下实车日志：

- `dual_3d_2026-07-28_19-17-47`
- `dual_3d_2026-07-28_19-20-49`
- `dual_3d_2026-07-28_19-45-41`

三次均已完成 Nav2 lifecycle 激活和 `/robot/set_system_ready=true` 握手，RViz
目标也确实被 `NavigateToPose` 接收。因此本次不是“Nav2 inactive”或目标工具失效。

1. 三次日志中的 RViz 关闭并重开都紧跟在启动器主动发送 `SIGUSR1` 之后。伴随的
   `std::system_error` 是强制拆除旧 RViz 进程产生的退出信息，不是点击地图导致
   RViz 自身崩溃。
2. 第一、第三次中 RPP 已输出约 `0.15 rad/s` 的原地转向命令，但 C++ 旋转安全圈在
   车辆静止时持续统计到约 `70-108` 个点，并将最终安全速度清零。前向碰撞区同时为
   空，说明旧算法把车体内部的雷达自反射也计入了旋转扫掠圆。
3. 第二次另有独立的 Gemini2 设备断开：日志明确出现
   `onDeviceDisconnected`、`Device is deactivated/disconnected`，随后深度点云持续
   断流。原安全层正确停止了车辆，但旧逻辑要等待 25 秒才取消目标，表现为长时间占用
   MOVE 又完全不动。

## 修改

1. 删除启动器对 RViz 的一次性 `SIGUSR1` 刷新和对应重启分支：
   - RViz 仍在启动初期自动打开；
   - Nav2 激活后不再关闭并重开；
   - 仅在 RViz 真正异常退出时，原监督器才最多重启三次。
2. C++ 2D 雷达碰撞门新增实测车体自反射过滤：
   - 车体尺寸仍为 `0.665 x 0.665 m`；
   - 只忽略 `|x| <= 0.33 m 且 |y| <= 0.33 m` 的物理车体内部点；
   - 车体外、`0.52 m` 旋转扫掠圆内的墙角或障碍物仍会阻止原地转向；
   - 前进区、后退区、扫掠圆和 3D 低矮障碍硬停车均保留；
   - 状态日志新增 `self_filtered` / `self_filtered_points`，用于下次直接验证误锁是否消失。
3. 安全融合节点新增 `/robot/navigation_sensor_healthy`：
   - 只有所需 RGB-D 点云和 2D 碰撞扫描同时新鲜时才为 `true`；
   - 任一所需输入断流时仍输出零速，不允许无视觉低障碍保护的盲开。
4. 底盘控制交接新增传感器故障快速收尾：
   - 健康状态消息超过 `1.0 s` 未更新也视为断流；
   - 活动导航目标持续断流 `3.0 s` 后自动取消；
   - Action 空闲后按原逻辑归还 PS2；
   - 原有 25 秒“最终安全速度长期为零”兜底仍保留。
5. Jazzy 预检同步检查：
   - 禁止重新加入 RViz `SIGUSR1` 自刷新；
   - 检查 0.33 m 车体自过滤参数；
   - 检查安全传感器健康发布、超时取消和 PS2 归还契约。

## 未修改

- Cartographer V13 的 Lua 参数、雷达滤波、`/odom`、IMU 和 TF 链；
- SmacPlanner2D、RPP、行为树、速度、目标容差与 costmap 参数；
- RGB-D/STVL 障碍记忆、RTAB-Map、OctoMap；
- STM32 程序和 AA55 协议。

冻结链哈希仍为：

```text
cartographer_2d_v9_tightened.lua
  00dfd1c721f0fe8c61ac6f2b417001920694e4fc77e895fb4a1f194330c910d9
cartographer_scan_v2_launch.py
  0571d9810aa44b32ecb7e283fcf035f83089de824ce2ec2a6530a6cdcbb26c4f
laser_filter.yaml
  8583a2ca7e99a29b13f2fc339df468e621562d61f0adfa1e7e1828254705b306
```

## 实车复测

只运行：

```bash
./START_DUAL_2D_3D_NAVIGATION.sh
```

1. 等待终端出现四行 `[ready]` 后再标点。RViz 从打开到测试结束不应再自动关闭重开。
2. 先在周围至少留出 `0.6 m` 的位置发送一个需要原地转向的 `0.5-1.0 m` 短目标。
3. 正常空旷位置应看到：

```text
COLLISION_GATE ... rotation=false ... self_filtered=...
FUSION_STATUS source=nav2 ... safe=(非零,非零或转向) ... nav_sensor_healthy=True
```

4. 将真实墙角放到车体外但旋转角扫掠范围内时，`rotation=true` 仍应阻止转动。
5. 若 Gemini2 再次掉线，车辆应立即零速，并在约 3 秒后出现
   `NAV_STALL_CANCEL: required 2D/3D navigation sensor input has been stale`，
   随后出现 `Nav2 idle: control returned to PS2`。

# 2026-07-29 V6.43：窄门对正、受控脱困与点云实时性隔离

## 日志结论

分析 `dual_3d_2026-07-29_06-56-38/runtime.log`：

1. 前两个 NavigateToPose 目标正常成功，说明 Nav2 生命周期、控制权、传感器和速度链均正常。
2. 第三个和第四个目标在同一门口失败。RPP 到达门框前仍输出带线速度转弯，
   随后门框进入车体前缘硬保护区并触发 `front_collision_lock`。
3. 第一次 `0.22 m` 受控倒车能够完成，但后续恢复树会在仍靠近门框时尝试原地旋转；
   旋转扫掠圆正确识别到门框并阻止动作，因此表现为门口长期不动。
4. 相机没有掉线，RGB 和深度仍约为 `12-13 Hz`。唯一一次约 `3 s` 的局部点云停顿，
   与 RTAB-Map 单次约 `5.51 s` 的视觉邻近回环和图优化同时发生，属于 CPU 调度争抢，
   不是 USB 断流。

## 修改

1. RPP 门口跟踪改为更短的动态前视：
   - `lookahead_dist=0.40 m`
   - `min/max_lookahead_dist=0.30/0.58 m`
   - `lookahead_time=1.50 s`
   - `curvature_lookahead_dist=0.35 m`
2. `rotate_to_heading_min_angle` 从 `1.35 rad` 调整为 `1.05 rad`。路径方向与车头相差约
   `60 deg` 以上时先受控对正，再向门洞前进，普通弯道仍连续走弧线。
3. 开启 RPP 代价调速，窄门附近最低有效速度保持 `0.07 m/s`；不会恢复旧版无效龟速。
4. 局部和全局软膨胀从 `0.52 m / 10.0` 调整为 `0.50 m / 14.0`：
   - `0.50 m` 仍大于实测车体加 padding 后约 `0.485 m` 的外接圆；
   - 只收窄蓝紫色软代价带，不修改 `66.5 x 66.5 cm` 真实 footprint；
   - C++ 前后、旋转和 RGB-D 单帧硬停车均未放宽。
5. 速度平滑器提高过零减速度。Nav2 从前进切换到 BackUp 时不再继续向门框缓慢前蹭约
   一秒，仍保留连续加减速。
6. 两棵行为树均改为“先退出门框，再小角度转向”：
   - 保留首次 `0.22 m` 和外层 `0.30 m` 碰撞检查倒车；
   - 左右 `0.25 rad` 脱困转向前各自必须先成功倒车 `0.22 m`；
   - 后方不安全时不会跳过倒车直接在门口原地旋转。
7. RTAB-Map 保留数据库、彩色持久地图、OctoMap 和视觉回环，只给其进程设置
   `nice -n 8` 并限制内部并行线程。实时局部碰撞点云进程因此可优先获得 CPU。
8. 启动器和 Jazzy 自检已同步锁定新的膨胀、RPP、恢复树和 RTAB-Map 资源隔离契约。

## 未修改

- Cartographer V13 的 Lua、雷达滤波、里程计、IMU 和 TF 链；
- 2D/3D STVL 障碍记忆、Cartographer 静态墙和 RTAB 视觉墙输入；
- 车体 footprint、C++ 厘米级硬碰撞边界与 STM32 协议；
- RTAB-Map 数据库、视觉特征、回环阈值和 OctoMap 分辨率。

## 实车复测

只运行：

```bash
./START_DUAL_2D_3D_NAVIGATION.sh
```

1. 先完成一个空旷区域短目标，确认普通弧线导航没有退化。
2. 从距离门口约 `1.0-1.5 m`、车头有明显偏角的位置发送门外目标。车辆应在门前完成
   对正，再沿门洞中心通过，蓝色路径不应在门口左右横跳。
3. 在门口临时挡住车辆，确认它先停车；障碍持续存在时应先倒出门框再尝试小角度转向，
   不应贴着门框持续原地转。
4. 观察终端不应再出现约 `3 s` 的 `Local cloud output stalled`。视觉大回环仍可能让
   RTAB-Map 自身更新短暂变慢，但实时碰撞点云应继续刷新。
5. 实测门洞最窄净宽。当前有效车宽为 `68.5 cm`；若门洞净宽不大于该值，任何参数都
   不能让实体车辆安全通过，不能继续缩小 footprint 冒险。

# 2026-07-29 V6.44：门外目标无路径与错误倒车恢复修复

## 最新日志结论

分析 `dual_3d_2026-07-29_07-43-24/runtime.log`：

1. 本次共接收 9 个导航目标，其中 3 个门内目标成功；说明 Nav2 生命周期、RPP 控制器、
   控制权交接、底盘速度输出和实时碰撞链路能够正常工作。
2. 门外目标在 FollowPath 启动前连续出现 61 次
   `GridBased ... no valid path found`。旧配置为 `allow_unknown=false`，而在线
   Cartographer 地图的门槛和门洞中容易暂时保留 1 个未知栅格缝隙，因此 Smac 把门内、
   门外判断为两个不连通区域。
3. 旧行为树没有区分“全局规划无路”和“控制器拿到路径后被困”。规划重试耗尽后仍进入
   `SafeEscapeActions`，本次错误触发 6 次 BackUp 和 3 次 Spin。这就是车辆不前进、
   反而持续向后乱动的直接原因，并非 RPP 主动选择倒车。
4. 日志没有持续前向碰撞锁；门外目标的首要失败发生在全局规划阶段，而不是相机、2D 雷达
   或 C++ 硬碰撞门将正常前进速度清零。

## 修改

1. `SmacPlanner2D.allow_unknown` 改为 `true`，仅允许在线地图中“尚未观测”的门洞缝隙
   参与搜索。以下硬约束全部保留：
   - Cartographer `/map` 中概率不低于 65 的静态墙；
   - 2D 雷达障碍层；
   - 全局近期 RGB-D STVL；
   - 全局及本地 RTAB 视觉墙记忆；
   - `66.5 cm` 车体、`1 cm` padding 和完整 footprint 碰撞检查；
   - 本地 3D 代价层与 C++ 2D/3D 最终停车门。
2. 两棵 Jazzy 行为树的即时 BackUp 前增加
   `WouldAControllerRecoveryHelp`。取消、失效路径或规划错误不能再触发倒车。
3. 外层脱困动作增加双重门闩：
   - FollowPath 必须返回 Nav2 认定可恢复的控制器错误；
   - 当前路径仍必须通过 `IsPathValid`。
4. 新增 `PlannerFailureHold`：只有规划失败时，车辆保持零速等待 `0.8 s` 后重试。
   若地图持续没有合法路径，目标最终失败并归还 PS2，不会通过盲目倒车“碰运气”。
5. 启动器与 Jazzy 校验脚本同步锁定：
   - 在线未知门洞缝隙必须允许；
   - 两层控制器恢复门闩必须存在；
   - 规划失败零速等待必须存在。

## 未修改

- Cartographer V13 Lua、2D 雷达滤波、里程计、IMU 与 TF；
- RPP 速度、转弯、前视距离、目标容差；
- 车体 footprint、膨胀半径和 C++ 厘米级硬碰撞范围；
- RGB-D/STVL/RTAB-Map/OctoMap 数据生成和显示；
- STM32 程序及 AA55 通讯协议。

## 实车复测

只运行：

```bash
./START_DUAL_2D_3D_NAVIGATION.sh
```

1. 启动时必须看到：

```text
Navigation source contract: static walls=65, online unknown seams=allowed, inflation=0.50m/14.0
Configured plugin GridBased ... allowing unknown traversal
```

2. 先发送一个门内短目标，再从门内发送一个门外目标。门外目标应生成路径并进入：

```text
Received a goal, begin computing control effort
```

3. 若门外仍确实没有合法路径，终端可以出现 `no valid path found` 和 `Running wait`，
   但在 FollowPath 尚未启动时不得出现 `Running backup` 或 `Running spin`。
4. 若仍无路径，请保留完整新日志，并在 RViz 同时勾选
   `Nav2 Global Planner Costmap (Path Audit)` 截图。此时要进一步判断是静态地图墙、
   当前 2D 雷达障碍、近期 RGB-D 障碍还是视觉墙记忆具体封住了门洞，不能继续猜参数。

# 2026-07-29 V6.45：撤销未知区危险通行并重构导航等待/恢复树

## 对 V6.44 的明确纠正

分析 `dual_3d_2026-07-29_08-35-00/runtime.log` 和
`截图 2026-07-29 08-41-35.png` 后，确认 V6.44 将
`SmacPlanner2D.allow_unknown` 设为 `true` 是错误修改。截图中的青色全局路径已经沿
在线地图外边界绕行，说明规划器把尚未建图的区域当成了可通行捷径。这不是安全的门洞
修复方式，V6.44 中关于“允许未知门缝”的结论由本节完全废止。

现已恢复：

```yaml
allow_unknown: false
```

启动时必须显示：

```text
Navigation source contract: static walls=65, unknown space=blocked, global RTAB wall duplication=off, inflation=0.50m/14.0
```

不得再出现 `allowing unknown traversal`，青色全局路径也不得进入 Cartographer 地图外的
深灰未知区。

## 本次停止不动的真实原因

本次日志不是相机、2D 雷达或 C++ 安全门持续锁死：

- 共接收 5 个导航目标，仅 1 个成功；
- 行为服务器启动约 286 次 `Wait`；
- RViz 的 `Recoveries` 增长到 74；
- 最后 25 秒内安全节点持续显示传感器健康、无前向碰撞，但行为树没有再给出有效控制
  命令，最终由 `NAV_STALL_CANCEL` 取消目标；
- 取消时出现 `Failed to get result for wait in node halt!`，进一步证明旧树在反复抢占
  Wait action。

旧树把每次规划检查都包装成 `20` 次、每次 `0.5 s` 的 RecoveryNode/Wait。它一边让
FollowPath 停止，一边把普通等待记为脱困，最终既不前进又不断增加 recovery 计数。

## 新行为树

`navigate_to_pose_jazzy.xml` 和 `navigate_through_poses_jazzy.xml` 已改为：

1. 使用 Nav2 官方 PipelineSequence 思路，以 `1 Hz` 稳定检查/重规划；
2. 有效路径保留 `3 s`，避免门口两条近似路径每半秒左右横跳；
3. 删除全部 `Wait`、`PlannerFailureHold` 和外层 `NavigateRecovery` 循环；
4. 规划失败只在原地立即重试两次，不清空 costmap、不倒车、不旋转；
5. 刷新规划临时失败时，只能继续使用通过 `IsPathValid` 的旧路径；
6. 只有 FollowPath 返回 Nav2 明确认定可恢复的控制器错误，并且当前路径仍有效时，
   才能进入物理脱困；
7. 物理脱困最多两次：先碰撞检查倒车 `0.16 m`；再次失败才倒车 `0.12 m` 后小转
   `0.20 rad`。后方或扫掠区不安全时动作自身会失败，不允许盲目乱倒。

因此“规划器没有路”会安全结束目标并归还 PS2，而不会再变成几十次等待、倒车和旋转。

## 门洞代价层分工

门口不能再靠开放未知区解决。本次将重复的长期墙来源拆开：

- **全局规划保留**：Cartographer `/map` 静态墙、当前 2D 雷达、经过多帧和地面几何
  确认的近期 RGB-D STVL、真实 footprint 和膨胀层；
- **全局规划移除**：`visual_wall_global_stvl_layer`。RTAB-Map 完整长期墙与
  Cartographer 静态墙叠加时会重复加厚墙体，并可能在窄门中桥接成整块代价；
- **局部控制保留**：`visual_wall_stvl_layer`。相机转开后，已经扫描过的视觉墙仍会
  参与 RPP 局部碰撞预测；
- **最终硬保护保留**：实时 2D 雷达前/后/旋转扫掠区、实时 RGB-D 低矮障碍和 C++
  碰撞门。

RTAB-Map 数据库、彩色持久点云、视觉回环和 OctoMap 生成/显示没有关闭；这里只取消
RTAB 长期墙对全局规划器的重复投影。

## 未修改与校验

未修改 RPP 速度、车体 `0.665 x 0.665 m` footprint、`1 cm` padding、`0.50 m`
膨胀半径、C++ 碰撞范围、STM32 和 AA55 协议。

冻结链哈希仍为：

```text
cartographer_2d_v9_tightened.lua
  00dfd1c721f0fe8c61ac6f2b417001920694e4fc77e895fb4a1f194330c910d9
cartographer_scan_v2_launch.py
  0571d9810aa44b32ecb7e283fcf035f83089de824ce2ec2a6530a6cdcbb26c4f
laser_filter.yaml
  8583a2ca7e99a29b13f2fc339df468e621562d61f0adfa1e7e1828254705b306
```

本地完成：11 份 YAML 解析、两棵 XML 行为树解析、两个 Bash 脚本语法、
`git diff --check` 和 18 个纯逻辑测试。

## 唯一实车复测

只运行：

```bash
./START_DUAL_2D_3D_NAVIGATION.sh
```

1. 等待新的 `Navigation source contract` 行出现后，先发一个门内 `0.5-1.0 m` 短目标；
2. 再从门前 `1.0-1.5 m` 处发送门外目标；
3. RViz 同时打开 `Nav2 Global Planner Costmap (Path Audit)`；
4. 青色路径必须始终位于已建图区域内，不能沿地图外圈绕行；
5. 终端不应再出现连续 `Running wait`，RViz 的 `Recoveries` 也不应每秒增长；
6. 若门仍被判定无路，车辆应原地结束目标并归还 PS2，不应连续倒车。此时保留日志和
   全局代价地图截图，下一步只定位门洞具体由静态墙、2D 雷达还是近期 RGB-D 哪一层
   封住，不再放开未知区或缩小真实车体。

# 2026-07-29 V6.46：缩小门洞软膨胀半径

按实车门洞通过需求，将 Nav2 局部和全局代价地图的障碍物膨胀半径统一从
`0.50 m` 缩小为 `0.49 m`，`cost_scaling_factor` 保持 `14.0`。

```yaml
inflation_radius: 0.49
cost_scaling_factor: 14.0
```

同时更新了：

- 双分辨率导航 RPP 覆盖配置；
- STVL 关闭时使用的基础 Nav2 回退配置，避免最后加载的 YAML 将参数覆盖回
  `0.52 m / 6.0`；
- 一键启动器的源文件契约、运行时参数读取和终端摘要；
- Jazzy 预检脚本。

`0.49 m` 是仍高于当前 `0.665 x 0.665 m` 车体加 `1 cm` padding 后约
`0.485 m` 外接圆的最小整厘米安全值。真实 footprint、padding、2D/3D C++ 硬碰撞门、
前后停车区和旋转扫掠区均未缩小。

启动时必须看到：

```text
Navigation source contract: static walls=65, unknown space=blocked, global RTAB wall duplication=off, inflation=0.49m/14.0
Costmap clearance: measured 66.5cm body + 1cm padding, inflation 0.49m/14.0
```

# 2026-08-03 V6.47: Ubuntu 22.04 / ROS 2 Humble 全面适配

- 正式运行环境从 Ubuntu 24.04 + ROS 2 Jazzy 切换为 Ubuntu 22.04 + ROS 2 Humble。
- 新增 `nav2_auto_mapping_humble.yaml`，显式注册 Humble 的 BT 插件库，并使用 Humble 的单数
  `progress_checker_plugin` 和 `SimpleProgressChecker` 接口。
- 新增 `nav2_dual_3d_rpp_humble_override.yaml`：保留二档 `0.20 m/s`、实车 footprint、
  `0.49 m / 14.0` 膨胀、SmacPlanner2D 和 RPP 数值，删除 Humble 不支持的后期参数。
- 新增 BT.CPP 3 格式的单目标/多目标行为树，去掉 Jazzy BT.CPP 4 的格式标记、错误码端口和
  `WouldA*RecoveryHelp` 节点，保留 1 Hz 验路、3 秒路径稳定窗口和有界脱困。
- 新增 `cartographer_auto_mapping_humble_launch.py`，双分辨率建图导航和 `open_all.sh` 均改为
  加载 Humble 参数、行为树和 Frontier 配置。
- 一键启动、标定、OctoMap 工具和视觉测试脚本统一 source `/opt/ros/humble/setup.bash`；构建缓存改为
  `~/.cache/huichuan_agv_humble_ws`，Python 用户库路径按系统 Python 3.10 动态计算。
- 双分辨率主链路与相机标定使用独立的 `~/.cache/huichuan_agv_dual_3d_humble_ws`，视觉单测使用
  `~/.cache/huichuan_agv_visual_slam_humble_ws`，避免误加载 Jazzy 生成的 CMake、Python 与 pluginlib 缓存。
- 重写依赖安装脚本并新增 `validate_auto_mapping_humble.sh`；GitHub Actions 改用 Ubuntu 22.04/Humble。
- 再次按 Humble pluginlib 导出名逐项审核：SmacPlanner2D 与 Behavior 插件使用 `/`，Costmap、RPP、
  Progress/Goal Checker 和 Waypoint 插件保留 `::`；同时将 Behavior Server 改回 Humble 的单一
  `costmap_topic` / `footprint_topic` 接口。
- Humble GitHub 预检除 YAML/XML/Bash/Python 语法和 `colcon` 编译外，还会依次将 Controller、Planner、
  Behavior Server 与 BT Navigator 转换到 lifecycle `configure`，直接捕获错误的 pluginlib 类名。
- 保留 Cartographer V13 Lua、雷达滤波、NAVI 里程计、IMU 和 STM32 通讯链，不修改稳定建图基线。
# 2026-08-03 V6.48: Humble 构建缓存与行为树编译修复

- 确认 `short_goal_bt` 是旧版自定义 BehaviorTree.CPP 插件，当前 Humble 导航树没有引用其中任何节点。
- 该旧包依赖名和 C++ API 来自另一版 BehaviorTree.CPP；执行全工作区 `colcon build` 时会被自动发现并造成行为树编译失败。现通过 `COLCON_IGNORE` 明确退出构建，不删除源码，便于追溯。
- `open_all.sh` 与 `visual_laser_slam/run_dual_resolution_3d_slam.sh` 新增构建环境指纹，记录 Ubuntu、ROS 发行版、系统 Python、源码绝对路径和 Git 提交。
- 指纹变化时仅清理各自 `~/.cache/huichuan_*_humble_ws` 下的 `build/install/log`，不会删除项目、RTAB-Map 数据库、地图或标定文件。
- `validate_auto_mapping_humble.sh` 新增 colcon 包发现检查，保证旧行为树插件不会混入 Humble 全工作区；Nav2 生命周期测试显式加载项目实际使用的两个 Humble XML 行为树路径。
- GitHub Actions 继续在 Ubuntu 22.04 + ROS 2 Humble 上构建和验证。此改动不修改 Cartographer 稳定建图参数、STM32、车体尺寸、导航速度、代价地图或避障参数。
# 2026-08-04 V6.49: 修复全局规划起点被判为致命障碍

- 分析 `dual_3d_2026-08-04_09-52-43/runtime.log`，确认 RViz 目标、BT Navigator 和 Planner 均正常；全部目标失败于 SmacPlanner2D 的 `Starting point in lethal space`。
- 根因是局部 Cartographer 静态层启用了车体 footprint 清理，而全局静态层错误地关闭了该功能。车底或紧贴车体边缘的单个历史占用像素会把全局规划起点永久判死，因此没有蓝色路径，也不会产生速度命令。
- 全局 `static_layer.footprint_clearing_enabled` 恢复为 `true`。只清除真实 `0.665 x 0.665 m` 车体当前占据的区域，不清除 footprint 外的墙体、2D 雷达障碍或 RGB-D 障碍。
- 一键启动源文件契约和 GitHub Humble 预检现同时要求局部、全局静态层都启用起点 footprint 清理，避免后续 YAML 覆盖再次引入该故障。
- 日志后半段 `/dev/ttyACM0`、2D 扫描和 Gemini2 点云同时断流属于独立的 USB/设备掉线；现有安全看门狗正确保持零速，本次不通过放宽安全条件掩盖掉线。
# 2026-08-04 V6.50：Humble 导航启用安全 DWB 双控制器与定制行为树

## 修改原因

对照 `all.beifen` 的实车导航实现后，确认它表现较好的核心并不是整套工程，而是
`RotationShim + DWB / NoShim DWB` 动态切换和 7 个 C++ 行为树节点。当前项目此前虽然保留
`short_goal_bt` 源码，但被 `COLCON_IGNORE` 排除，Humble 实际运行的是标准行为树和 RPP。

## 本次修改

1. `short_goal_bt` 依赖由 Jazzy 的 `behaviortree_cpp` 改为 Humble 的
   `behaviortree_cpp_v3`，删除 `COLCON_IGNORE`，并由 `lidar_py` 声明运行依赖。
2. `bt_navigator.plugin_lib_names` 加载 `short_goal_behind_bt_node`。其中包含
   `ShortGoalBehind`、`InitialPathPreRotate`、`SpinSafetyCheck`、`SelectController`、
   `ReverseEscapeMonitor`、`ControllerSelected` 和 `ReverseEscapeCompleted`。
3. 新增 `nav2_dual_3d_dwb_humble_override.yaml`：保留 SmacPlanner2D、二档
   `0.20 m/s`、完整 `0.665 m` 方形 footprint、现有 2D/3D costmap 和速度平滑；新增
   `FollowPath`（RotationShim 包装 DWB）及可有限倒车的 `FollowPathNoShim`。
4. 单点导航树保持 1 Hz 重规划与 3 秒路径有效期。新目标首次计算路径后，根据目标/路径方位
   决定是否预旋转；只有局部和全局 costmap 的 `0.51 m` 完整扫掠圆都安全才允许 Spin，
   否则切换 NoShim DWB，以差速弧线或小幅倒车腾出空间。
5. 恢复倒车继续使用 Nav2 `BackUp` 的 footprint 碰撞预测，并通过
   `ReverseEscapeMonitor` 同时检查 `/cmd_vel_nav` 和 `odom` 实际位移；6 秒未产生真实位移即
   判定失败，不把“只发了倒车命令”误认为脱困成功。
6. 一键启动默认切换至 DWB 配置，RPP 文件保留为回退，不删除。没有接入会直接发送
   `/cmd_vel`、自动导航到 `(0,0)` 的 Python 三级脱困节点。
7. Humble GitHub 验证改为实际编译 `short_goal_bt`，并让 controller、planner、behavior 和
   `bt_navigator` lifecycle configure，确保 DWB、RotationShim 和定制 BT `.so` 真正可加载。

## 未修改

- Cartographer V13 参数及其基线哈希；
- 2D 雷达、3D STVL、视觉墙、长期 RTAB-Map 和 C++ 最终碰撞门；
- 车体尺寸、导航二档速度、膨胀半径 `0.49 m` 与未知区域禁止规划；
- STM32 协议和下位机代码。

## Humble CI 补充修正

- 将定制行为树头文件统一切换为 Humble 的 `behaviortree_cpp_v3`，移除仅 BT.CPP 4
  存在的 JSON 注册接口；7 个节点已在 Ubuntu 22.04 CI 中完成编译。
- 增加 `dwb_plugins` 运行时依赖及一键启动前检查。`dwb_core` 只提供控制器接口，
  `dwb_plugins` 才包含 DWB 实际使用的 `StandardTrajectoryGenerator`。
- 增加 `dwb_critics` 运行时依赖；它提供 `RotateToGoalCritic`、`PathDistCritic` 等
  DWB 轨迹评分器，缺失时 controller server 会在 lifecycle configure 阶段拒绝启动。

# 2026-08-05 V6.51：Jetson Gemini2 点云断流治理

- 汇总 8 月 5 日全部有效日志：本地点云最长停顿约 `10.2 s`，但点云单帧计算最坏约
  `127 ms`；停顿期间 RGB 接近 `15 Hz`、Depth 接近 `0 Hz`，确认不是 C++ 点云滤波排队。
- Gemini2 图像和 CameraInfo 改为 `SENSOR_DATA` QoS，防止低速 RTAB-Map/RViz 的可靠传输
  队列反压相机深度发布线程。
- 自动识别 Jetson 平台并默认将 RTAB-Map 调整为 `1 Hz`、单 OMP 线程；PC 默认值保持不变，
  3 cm/15 Hz 本地碰撞点云保持不变。
- 启用 Orbbec 官方帧丢失统计，CSV 随每次运行写入对应 `SLAM_Log` 会话目录。
- 修正 `ros2 --no-daemon param get` 参数位置，并在启动前加载缓存构建 overlay。
- `.gitignore` 新增根目录 `Log/` 与 `SLAM_Log.zip`，GitHub Actions 同时验证 `jetson` 分支。
- 日志还显示同一次运行中 2D 雷达最多重连 97 次；这部分需在 Jetson 实机检查 USB 拓扑、
  供电和是否存在第二个串口读取进程，软件安全看门狗继续在断流时停车。
- GitHub Humble 预检跳过仅由外部 Orbbec/Cartographer 源码引入的 `libgoogle-glog-dev`；
  避免 hosted runner 的 `libunwind` 冲突在项目编译前误报失败，不改变 Jetson 安装脚本。
