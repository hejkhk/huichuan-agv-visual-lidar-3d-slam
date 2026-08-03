# 双分辨率 3D SLAM 与 Nav2 交接文档

## 1. 项目目标

本分支面向 Ubuntu 22.04、ROS 2 Humble、RTX 3060 和 Gemini2 深度相机，实现：

1. Cartographer 2D SLAM 提供全局平面定位、2D 地图和回环。
2. RTAB-Map 低频保存真实 RGB-D 彩色关键帧，不发布 TF。
3. OctoMap 保存低分辨率 3D 占用地图。
4. Gemini2 过滤点云以约 15 Hz 进入 Nav2 的 3 cm 局部和 5 cm 全局 STVL。
5. SmacPlanner2D 负责带代价的全局路径，Regulated Pure Pursuit 以 20 Hz
   负责稳定跟踪、转弯限速和碰撞前视。
6. URDF 显示车体与传感器，Nav2 footprint 才是实际二维碰撞边界。

严禁把 RTAB-Map、EKF 或视觉里程计改成 `map->odom` 或
`odom->base_link` 的发布者。当前唯一平面位姿权威是 Cartographer +
已验证的底盘里程计链。

## 2. 权威数据链

```text
STM32 NAVI + 2D LiDAR
        |
        v
Cartographer 2D -----> map / TF / corrected planar pose
        |                         |
        |                         +-----> Nav2 global costmap + planner
        |                         +-----> RTAB-Map odom input (publish_tf=false)
        |
Gemini2 native depth ---> full filter ---> /local_highres_cloud_v21/sensor
                                              |                 |
                                              |                 +--> global 5 cm STVL
                                              +--> local 3 cm STVL
                                                                 |
Cartographer map + filtered 2D LiDAR + RTAB visual walls ---------+
                                                                 |
                                              SmacPlanner2D --> RPP 20 Hz
                                                                 |
                                              velocity_smoother
                                                                 |
                                              safety_fusion + C++ collision gate
                                                                 |
                                              /cmd_vel_safe --> STM32

Gemini2 native RGB + depth_image_proc registered depth
        |
        +--> RTAB-Map low-rate colored keyframes

filtered depth cloud --> OctoMap --> persistent low-resolution 3D occupancy
```

## 3. 两个启动版本

### 3.1 只建图

```bash
./START_DUAL_2D_3D_MAPPING.sh
```

启动 Cartographer、Gemini2、过滤点云、RTAB-Map、OctoMap、URDF 和 RViz，
不启动 Nav2 控制器。

### 3.2 建图加导航

```bash
./START_DUAL_2D_3D_NAVIGATION.sh
```

在上述模块基础上启动 SmacPlanner2D + Regulated Pure Pursuit、局部/全局
STVL、视觉长期墙体层、速度平滑和最终碰撞安全链。

兼容脚本 `START_DUAL_2D_3D_SLAM.sh` 等同于只建图版本。

## 4. 自动外参标定

标定结果现在会自动写入：

```text
visual_laser_slam/dual_resolution_3d.env
```

每次真正改写前都会在同目录生成：

```text
dual_resolution_3d.env.bak.YYYYMMDD_HHMMSS_microseconds
```

写入采用临时文件加原子替换。质量检查失败时不会修改配置。

### 4.1 地面标定：roll、pitch、camera_z

1. 启动只建图版本。
2. 车停在平整地面，保持完全静止。
3. 清空相机画面下半部分，避免脚、箱子和台阶。
4. 执行：

```bash
./CALIBRATE_CAMERA_EXTRINSIC.sh
```

只有地面内点比例不低于 45%，且平面 RMSE 不高于 18 mm 时才自动写入。
写入后 `CAMERA_EXTRINSIC_CALIBRATED=false`，因为 yaw 尚未完成。

完成后必须结束并重新启动只建图版本，让新 roll、pitch、z 生效。

### 4.2 墙面标定：yaw

1. 让车和相机保持静止。
2. 车前方 1 到 3 m 放置或选择一面同时被 Gemini2 和 2D LiDAR
   看见的大平墙。
3. 墙无需与车完全垂直，但前方不要有大量桌腿和人员。
4. 执行：

```bash
./CALIBRATE_CAMERA_YAW.sh
```

脚本比较深度点云和 LiDAR 的主墙方向。两边内点、墙面跨度以及修正幅度
全部通过安全限制后，自动写入 `CAMERA_YAW_DEG`，并设置：

```text
CAMERA_EXTRINSIC_CALIBRATED=true
```

再次重启主程序后生效。

### 4.3 标定失败处理

出现 `AUTO-WRITE REJECTED` 时，不要手工把本次结果填进去。先检查：

- 车是否在移动或晃动。
- 地面是否反光、台阶过多或被障碍遮挡。
- 墙是否同时位于相机与 LiDAR 的有效视野。
- `/local_highres_cloud_v21`、`/scan_timed_v2` 是否持续更新。
- 相机和雷达 TF 是否存在。

## 5. 2D/3D 避障与重规划的真实职责

| 层 | 输入 | 频率/范围 | 作用 |
|---|---|---|---|
| Cartographer 2D | 2D LiDAR、底盘平面里程 | 持续 | 全局定位、2D 地图、回环 |
| StaticLayer | Cartographer 地图 | 地图更新 | 已建静态墙体 |
| 2D ObstacleLayer | 2D LiDAR | 实时 | 雷达高度上的障碍标记与清除 |
| Local 3D STVL | 过滤深度点云 | 3 cm、costmap 10 Hz | 标记和清除低矮、立体、动态障碍，参与 RPP 碰撞前视 |
| Global 3D STVL | 同一过滤深度点云 | 5 cm、costmap 5 Hz | 使被持续障碍堵住的路径失效并触发全局改道 |
| Visual Wall STVL | RTAB-Map 过滤后长期点云 | 低频长期层 | 补充相机曾扫描到、当前进入盲区的立体墙体 |
| SmacPlanner2D | 全局 costmap | 路径失效立即重算；有效路径每 3 秒刷新 | 生成带代价并经内部平滑的全局路线 |
| Regulated Pure Pursuit | 全局路径、局部 costmap、footprint | 20 Hz | 稳定跟踪、曲率限速、目标对准和碰撞前视 |
| velocity_smoother | RPP 速度 | 30 Hz | 限制加减速，减少底盘突跳 |
| safety_fusion + C++ collision gate | 2D LiDAR、RGB-D 和 Nav2 速度 | 实时 | 前进、倒车、旋转方向的最终门控，输出 `/cmd_vel_safe` |

动态障碍进入点云后，局部 STVL 立即标记，RPP 每 50 ms 检查跟踪弧和
footprint 碰撞。障碍离开并被深度射线重新观测为空闲后，STVL 执行
clearing。若障碍持续挡住路线，全局 STVL 使路径失效，行为树立即重算；
路径仍有效时每 3 秒刷新一次，避免控制器被高频新路径打断。

这套设计能够进行动态和静态避障，但“碰撞级别”不等于安全认证。实车仍需
验证相机盲区、制动距离、串口延迟、地面反光和最小障碍尺寸，并保留物理急停。

## 6. URDF 与碰撞边界

URDF 文件：

```text
lidar/chapt1_ws/src/lidar_py/urdf/agv_box.urdf.xacro
```

车体外接尺寸为 `0.665 x 0.665 x 0.321 m`。URDF 负责：

- RViz 模型显示。
- 相机、雷达安装位置表达。
- TF/模型一致性检查。

Nav2 不会自动读取 URDF collision 当作 2D costmap footprint。真正使用的
footprint 位于：

```text
lidar/chapt1_ws/src/lidar_py/config/nav2_auto_mapping_humble.yaml
```

其半长半宽为 `0.333 m`，导航覆盖文件额外设置
`footprint_padding=0.02 m`。RPP 开启 `use_collision_detection`，
局部代价地图和最终 C++ 碰撞门都按完整车体扫掠范围判断，不把车辆当作质点。

## 7. 关键文件

| 文件 | 作用 |
|---|---|
| `visual_laser_slam/dual_resolution_3d.env` | 双分辨率运行、外参、滤波与地图配置 |
| `visual_laser_slam/run_dual_resolution_3d_slam.sh` | 统一运行器、设备检查、构建、日志、退出保存 |
| `lidar_py/launch/dual_resolution_3d_slam.launch.py` | 2D/3D/导航总拓扑 |
| `lidar_py/config/cartographer_2d_v9_tightened.lua` | 已验证 Cartographer 参数，禁止随意改动 |
| `lidar_py/config/nav2_auto_mapping_humble.yaml` | Nav2 主参数和真实车体 footprint |
| `lidar_py/config/nav2_dual_3d_rpp_override.yaml` | SmacPlanner2D、RPP、二档速度、目标容差和速度平滑 |
| `lidar_py/config/nav2_dual_3d_stvl_override.yaml` | 3 cm 局部、5 cm 全局 STVL 和 RTAB-Map 长期视觉墙体层 |
| `lidar_py/behavior_trees/navigate_to_pose_humble.xml` | 路径失效/周期重算和受控倒车、双向旋转脱困 |
| `local_depth_cloud_cpp/.../depth_image_to_local_cloud_v21_node.cpp` | 深度点云完整滤波 |
| `lidar_py/lidar_py/camera_ground_calibrator.py` | 地面外参标定 |
| `lidar_py/lidar_py/camera_lidar_yaw_calibrator.py` | 相机-LiDAR yaw 标定 |
| `lidar_py/lidar_py/calibration_env.py` | 标定配置备份与原子写入 |
| `lidar_py/urdf/agv_box.urdf.xacro` | 车体与传感器模型 |

## 8. 实车验收步骤

### 8.1 先验证感知，不发运动

```bash
./START_DUAL_2D_3D_MAPPING.sh
ros2 topic hz /local_highres_cloud_v21/sensor
ros2 topic hz /scan_timed_v2
ros2 topic hz /map
```

确认点云方向、地面高度、2D 雷达方向均正确，再进行自动外参标定。

### 8.2 验证 Nav2 生命周期与 costmap

```bash
./START_DUAL_2D_3D_NAVIGATION.sh
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 topic hz /local_costmap/costmap
ros2 topic hz /global_costmap/costmap
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel_safe
```

两台 server 应为 `active`。把箱子放到车前，RViz 局部和全局 costmap
都应出现障碍；移走箱子并让相机重新看到原区域后，障碍应被清除。

### 8.3 低速动态避障

1. 先在开阔区域设置 3 到 5 m 目标。
2. 保持物理急停可触达。
3. 人从路线侧方缓慢进入，不要第一次就从近距离突然冲入。
4. 检查 RPP 是否先减速/停车，持续堵塞时全局路径是否在 3 秒内重算。
5. 检查人离开后 costmap 是否清除，车辆是否平滑恢复。

### 8.4 必须记录

- `runtime.log`。
- `/cmd_vel_nav` 与 `/cmd_vel_safe` 是否同时更新。
- 局部/全局 costmap 截图。
- 障碍出现到车速下降的时间。
- 最近障碍距离、实测停车距离。
- 是否发生点云卡顿或 TF extrapolation。

## 9. 禁止事项

1. 不修改已验证的 `cartographer_2d_v9_tightened.lua` 来解决 3D 问题。
2. 不让 RTAB-Map、视觉里程计或 EKF 发布平面主 TF。
3. 不加载旧黑色/螺旋 RTAB-Map 数据库验证新算法。
4. 不把 OctoMap 当作 20 Hz 实时避障源；实时避障源是过滤点云 STVL。
5. 不仅凭 RViz 模型判断碰撞安全，必须检查 costmap footprint。
6. 不在外参未标定时进行高速导航。

## 10. 当前边界与后续建议

- 本次在 Windows 完成静态检查，未进行 ROS 2 Humble 编译和实车验证。
- 视觉回环当前不接管机器人全局位姿，避免再次出现螺旋地图拖动导航。
- 默认 `RTABMAP_ON_DEMAND_PAUSE=false`，RTAB-Map 不会因 RViz 取消勾选
  MapCloud 而暂停；视觉关键帧和回环持续运行，取消勾选只降低 RViz 渲染
  开销。只有显式改为 `true` 才启用需求管理器。
- RTAB-Map 后台彩色地图计算与 RViz 显示已经解耦，并保持
  `publish_tf=false`，不会发布或抢占主 TF。
- 实车确认 STVL 标记/清除、RPP 跟踪误差和制动距离后，才可逐步提高导航速度。
