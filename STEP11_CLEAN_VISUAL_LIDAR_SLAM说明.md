# STEP11 双分辨率 2D/3D SLAM

## 1. 这次修复了什么

旧 STEP11 的数据链是：

```text
视觉里程计 + 轮速 + IMU -> EKF -> odom->base_link -> Cartographer
```

视觉里程计或 EKF 在原地旋转时一旦产生错误位姿，Cartographer 的整张 2D 地图就会被带着旋转，RViz 中会出现放射状轨迹、双墙和地图乱飞。

新 STEP11 不再启动视觉里程计和 EKF。稳定版 Cartographer 与底盘里程计继续独占平面定位 TF，3D 建图只读取其结果，不允许反向修改控制 TF。

## 2. 新结构

```text
STM32 0x07 -> /odom --------------------------+
                                                |
2D LiDAR -> /scan_timed_v2 -> Cartographer ----+-> map/odom/base_link
                   |                            |   (唯一平面定位权威)
                   +----------------------------+-> RTAB-Map 低分辨率3D图
Gemini2 RGB-D ----------------------------------+
       |
       +-> STEP10V2.1 -> /local_highres_cloud_v21
                         (近场高分辨率、实时避障输入)
```

| 层 | 输出 | 分辨率/频率 | 用途 | 是否影响小车定位 |
|---|---|---:|---|---|
| 2D 全局层 | `/map` | 0.05 m | 建图、全局规划、稳定定位 | 是，唯一权威 |
| 3D 记忆层 | `maps/rtabmap_3d/rtabmap.db`、`/rtabmap_3d/mapData` | 0.08 m、2 Hz | 可关闭后续建的全局低分辨率 3D 图 | 否 |
| 3D 实时层 | `/local_highres_cloud_v21` | 0.03 m、15 Hz | 近场三通道视野、精细避障 | 否 |

RTAB-Map 和 Cartographer 直接使用相同的 `map` 显示坐标。RTAB-Map 设置了 `publish_tf=false`，因此不会发布或覆盖 `map -> odom`。

## 3. 启动

先确认没有运行 `open_all.sh`、旧 STEP 或其他占用相机/串口的程序：

```bash
cd ~/huichuan-agv-ros2-foxy-visual-lidar-slam-step/huichuan-agv-ros2-foxy-main
chmod +x STEP11_CLEAN_VISUAL_LIDAR_SLAM_TEST.sh \
  visual_laser_slam/run_dual_resolution_3d_slam.sh \
  RESET_STEP11_3D_MAP.sh
./STEP11_CLEAN_VISUAL_LIDAR_SLAM_TEST.sh
```

启动器会自动检测 STM32 和雷达串口、开放权限、隔离编译并检查五个关键话题。首次编译时间较长。

退出时只按一次 `Ctrl+C`，等待终端显示数据库保存完成。RTAB-Map 会把图持续保存在：

```text
maps/rtabmap_3d/rtabmap.db
```

下次启动自动读取并继续建图，不需要额外导入。

## 4. 首轮实车验证

1. 启动后静止 10 秒，确认 2D 地图、RGB 和局部点云都更新。
2. 先直行 1 m，再低速原地旋转 90 度；2D 地图不得出现放射状乱飞。
3. 在相机前方的左、中、右三条路径分别放置障碍，确认 `/local_highres_cloud_v21` 都能看到。
4. 绕一个短闭环后按 `Ctrl+C`，再次启动，确认旧 3D 地图仍存在并继续增加节点。
5. 终端执行下面命令，确认不存在旧 EKF 控制链：

```bash
ros2 node list | grep -E 'ekf|rgbd_odometry'
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /local_highres_cloud_v21/stats --once
ros2 topic hz /rtabmap_3d/mapData
```

第一条命令在本 STEP 中应无输出。`map -> base_link` 应连续，不能在车静止时跳变。

## 5. 新建一张 3D 图

不要直接删除数据库。先停止 STEP11，再执行：

```bash
./RESET_STEP11_3D_MAP.sh
```

脚本会把旧数据库改名归档。也可在 `visual_laser_slam/dual_resolution_3d.env` 中临时设置：

```bash
RESET_GLOBAL_3D_MAP=true
```

只运行一次后必须改回 `false`，否则每次启动都会新建地图。

## 6. 参数位置

统一配置位于：

```text
visual_laser_slam/dual_resolution_3d.env
```

正常测试不要修改 `cartographer_2d_v9_tightened.lua`。相机外参变化时，只修改：

```bash
CAMERA_X=0.30
CAMERA_Y=0.0
CAMERA_Z=0.40
CAMERA_ROLL_DEG=0.0
CAMERA_PITCH_DEG=0.0
CAMERA_YAW_DEG=-45.0
CAMERA_EXTRINSIC_CALIBRATED=false
```

角度单位为度。相机向下俯视时 `CAMERA_PITCH_DEG` 填正值。完成标定后将最后一项改为 `true`。

修改任何相机外参后，都要执行一次 `./RESET_STEP11_3D_MAP.sh`，因为数据库中的旧点无法自动改用新外参。

当前启动器还会比较 `rtabmap.db.config`：相机外参、分辨率或 RGB-D 对齐模式变化时，会自动备份旧数据库并新建地图。当前采用 `SW + DEPTH`，把 RGB 对齐到原生深度视场，避免硬件 D2C 裁掉 STEP10V2.1 的左右视野。

当前 URDF 车体为 `0.665 x 0.665 x 0.321 m` 外接长方体，`base_link` 位于地面车体中心。雷达默认位置为 `(0.20, 0, 0.4235) m`，相机默认位置为 `(0.30, 0, 0.371) m`。相机根据静止加速度暂定向下 `25.04 deg`，yaw 仍需通过正对墙面实测。

## 7. Nav2 三维避障接口

已提供 `lidar/chapt1_ws/src/lidar_py/config/nav2_local_3d_voxel_layer.yaml`。它让 Nav2 **局部** costmap 的 VoxelLayer 消费 `/local_highres_cloud_v21`；全局 costmap 仍只使用 2D 地图和激光雷达。

当前 STEP11 是建图/感知验收脚本，不发送电机指令。局部点云稳定验收后，再把该块合入正式 Nav2 参数，并加入 URDF footprint/collision geometry。这样不会在点云链尚未验证时直接控制实车。

## 8. 故障判断

| 现象 | 优先检查 |
|---|---|
| 2D 地图一转就飞 | 是否同时运行了旧 STEP/open_all；是否出现第二个 `odom->base_link` 发布者 |
| 只有 3D 图错位，2D 正常 | 相机外参、RGB-D 同步、RTAB-Map 回环；不要改 Cartographer |
| 局部点云缺左/右通道 | 检查 `LOCAL_Y_MIN/MAX` 是否仍为 `-2.5/2.5` |
| 局部点云延迟 | 查看 `/local_highres_cloud_v21/stats`，重点看 `age_p95_ms` 和 `output_gap_max` |
| 重启后 3D 图消失 | 检查数据库路径、退出是否等待完成、`RESET_GLOBAL_3D_MAP` 是否误设为 true |
