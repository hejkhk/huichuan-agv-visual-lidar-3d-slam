# 汇川四轮差速 AGV：RGB-D 视觉里程计融合 2D 激光 Cartographer 测试说明

本功能是**独立测试链**，不会覆盖或改动原来的 `open_all.sh` 导航启动方式。

最终测试结构：

```text
Gemini2 RGB + Depth
        ↓
RTAB-Map RGB-D Odometry → /visual_odom

STM32 轮速里程计 → /wheel/odom
STM32 IMU z轴角速度 → /imu_cartographer
        ↓
robot_localization EKF → /odometry/filtered + odom→base_link
        ↓
2D LiDAR + Cartographer → map→odom + /map
```

TF 只有两级动态发布者：

```text
Cartographer       发布 map → odom
robot_localization 发布 odom → base_link
```

RTAB-Map 在融合模式下只发布 `/visual_odom`，**不发布 TF**，避免抢占 `odom → base_link`。

---

## 0. 首次安装依赖

```bash
cd ~/huichuan-agv-ros2-foxy
chmod +x STEP*.sh
./STEP0_INSTALL_VISUAL_SLAM_DEPS.sh
```

主要依赖：

- `rtabmap_odom`
- `robot_localization`
- `cartographer_ros`
- `laser_filters`
- `rmw_cyclonedds_cpp`
- 已经能工作的 `orbbec_camera`

如果 Orbbec 驱动在独立工作空间，修改：

```text
visual_laser_slam/visual_laser_slam.env
```

例如：

```bash
ORBBEC_SETUP=/home/peter/OrbbecSDK_ROS2/install/setup.bash
```

---

## STEP1：只测试 Gemini2 RGB-D

```bash
./STEP1_RGBD_CAMERA_TEST.sh
```

应出现：

```text
/camera/color/image_raw
/camera/depth/image_raw
/camera/color/camera_info
/camera/depth/camera_info
```

验收：

1. 静止运行至少 30 秒不掉流；
2. RGB 和 Depth 都能持续刷新；
3. 深度已经对齐到彩色坐标；
4. 树莓派 CPU 没有持续满载。

---

## STEP2：只测试 RGB-D 视觉里程计

```bash
./STEP2_RGBD_VISUAL_ODOM_TEST.sh
```

输出：

```text
/visual_odom
odom → camera_link
```

此步骤不连接 STM32，也不融合轮速。

测试动作：

1. 手推小车向前约 1 m；
2. 再向后回到原位置；
3. 左转约 90°；
4. 走一个小圆弧。

验收：

- `/visual_odom` 连续发布；
- 向前、后退和左右转方向正确；
- 静止时不持续乱跑；
- 不能出现瞬间跳动数米；
- 短时间特征不足后能恢复。

查看数值：

```bash
ros2 topic echo /visual_odom
ros2 topic hz /visual_odom
```

---

## STEP3：只测试轮速 + IMU EKF

```bash
./STEP3_WHEEL_IMU_EKF_TEST.sh
```

输出：

```text
/wheel/odom
/imu_cartographer
/odometry/filtered
odom → base_link
```

为了不和原导航抢命令，本测试专用速度话题是：

```text
/cmd_vel_visual_slam_test
```

低速直线测试：

```bash
ros2 topic pub /cmd_vel_visual_slam_test geometry_msgs/msg/Twist \
"{linear: {x: 0.08}, angular: {z: 0.0}}" -r 20
```

低速原地转向：

```bash
ros2 topic pub /cmd_vel_visual_slam_test geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: 0.12}}" -r 20
```

停止：

```bash
ros2 topic pub --once /cmd_vel_visual_slam_test geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: 0.0}}"
```

验收：

- 固定速度命令下小车不抽搐；
- `/wheel/odom` 和 `/odometry/filtered` 连续；
- 转向时 yaw 连续变化，不突然跳角度；
- 静止时 EKF 不明显漂移。

---

## STEP4：视觉 + 轮速 + IMU 融合里程计

### 先填写相机外参

修改：

```text
visual_laser_slam/visual_laser_slam.env
```

ROS 车体坐标：

```text
x：车头方向
 y：车体左侧
 z：车体上方
```

填写 Gemini2 的 `camera_link` 相对 `base_link` 的位置和角度：

```bash
CAMERA_X=0.30
CAMERA_Y=0.00
CAMERA_Z=0.45
CAMERA_ROLL=0.0
CAMERA_PITCH=0.0
CAMERA_YAW=0.0
CAMERA_TF_CONFIRMED=true
```

上面的数值只是格式演示，必须用实车测量值替换。

启动：

```bash
./STEP4_VISUAL_WHEEL_IMU_FUSION_TEST.sh
```

输出：

```text
/visual_odom
/wheel/odom
/imu_cartographer
/odometry/filtered
odom → base_link
```

RViz 中同时显示三组里程计：

- Visual Odometry
- Wheel Odometry
- EKF Fused Odometry

验收：

- EKF 比单独视觉里程计更连续；
- 相机短暂丢失时 EKF 仍继续运动；
- 轮胎轻微打滑时视觉能提供额外约束；
- `/odometry/filtered` 不随视觉单帧噪声突然跳变。

---

## STEP5：视觉融合 2D 激光 Cartographer SLAM

```bash
./STEP5_VISUAL_LIDAR_SLAM_TEST.sh
```

完整输入：

```text
/scan_timed_v2_filtered
/odometry/filtered
/imu_cartographer
```

输出：

```text
/map
map → odom
odom → base_link
```

第一轮测试顺序：

1. 原地缓慢旋转一圈；
2. 直线行驶 2~3 m；
3. 走一个矩形回路；
4. 通过长走廊；
5. 回到起点检查闭环。

保存地图：

```bash
ros2 run nav2_map_server map_saver_cli \
  -f "$HOME/visual_lidar_map" \
  --ros-args -p save_map_timeout:=10.0
```

验收：

- 地图不重影、不撕裂；
- 原地旋转时墙面不明显弯曲；
- 长走廊方向稳定；
- 视觉暂时丢失时系统可依靠轮速、IMU和激光继续；
- Cartographer 正常发布 `map → odom`；
- EKF 是唯一的 `odom → base_link` 发布者。

---

## 配置和源码位置

```text
根目录脚本：
STEP0_INSTALL_VISUAL_SLAM_DEPS.sh
STEP1_RGBD_CAMERA_TEST.sh
STEP2_RGBD_VISUAL_ODOM_TEST.sh
STEP3_WHEEL_IMU_EKF_TEST.sh
STEP4_VISUAL_WHEEL_IMU_FUSION_TEST.sh
STEP5_VISUAL_LIDAR_SLAM_TEST.sh

统一参数：
visual_laser_slam/visual_laser_slam.env

统一启动器：
visual_laser_slam/run_visual_slam_step.sh

ROS 2 launch：
lidar/chapt1_ws/src/lidar_py/launch/visual_laser_slam.launch.py

EKF：
lidar/chapt1_ws/src/lidar_py/config/ekf_wheel_imu.yaml
lidar/chapt1_ws/src/lidar_py/config/ekf_visual_wheel_imu.yaml

Cartographer：
lidar/chapt1_ws/src/lidar_py/config/cartographer_2d_visual_fusion.lua
```

---

## 重要注意事项

1. 运行 STEP 脚本前，必须先关闭原来的 `open_all.sh`，否则会抢串口和 TF。
2. STEP4、STEP5 前必须尽量准确测量 `base_link → camera_link`。
3. 第一版没有启动 Nav2、视觉避障、网页和自动探索，目的是只验证 SLAM 底层。
4. 原来的 `open_all.sh`、Nav2 配置和视觉避障代码均未覆盖，可随时回到原系统。
5. 每个步骤按 `Ctrl+C` 会关闭该步骤全部进程，运行日志保存在 `SLAM_Log/visual_laser_*`。
