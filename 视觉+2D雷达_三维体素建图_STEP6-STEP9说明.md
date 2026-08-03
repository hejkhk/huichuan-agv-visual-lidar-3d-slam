# 汇川四轮差速 AGV：视觉 + 2D 雷达 + 三维体素地图测试说明

## 1. 这次新增了什么

原来的 STEP1～STEP5 已经完成：

```text
Gemini2 RGB-D 视觉里程计
        +
轮速里程计 + IMU
        ↓
robot_localization EKF
        ↓
odom → base_link
        +
2D 激光雷达 + Cartographer
        ↓
map → odom + 二维地图 /map
```

本次在这套已经验证成功的定位系统旁边增加三维建图支路：

```text
Gemini2 深度点云
        ↓
限频 + 距离裁剪 + 车体坐标裁剪 + 体素降采样
        ↓
OctoMap 三维占据体素地图
```

最终 STEP9 同时生成：

- `/map`：Cartographer 的二维导航地图；
- `/occupied_cells_vis_array`：OctoMap 三维占据体素；
- `/octomap_binary`：可保存的压缩三维地图；
- `/camera/depth/points_filtered`：实时过滤后的深度点云。

> 这不是把 2D 雷达本体变成 3D 雷达。2D 雷达继续负责稳定的平面定位和闭环，Gemini2 提供上下高度信息，二者共同得到类似 3D 雷达建图的三维环境模型。

---

## 2. 新增文件

根目录新增：

```text
STEP6_DEPTH_POINTCLOUD_TEST.sh
STEP7_FILTERED_POINTCLOUD_TEST.sh
STEP8_EKF_OCTOMAP_3D_TEST.sh
STEP9_2D_3D_DUAL_MAP_TEST.sh
SAVE_STEP9_OCTOMAP.sh
RESET_STEP9_OCTOMAP.sh
视觉+2D雷达_三维体素建图_STEP6-STEP9说明.md
```

ROS 2 包中新增：

```text
lidar_py/point_cloud_filter_node.py
```

新增 RViz 配置：

```text
depth_pointcloud_test.rviz
filtered_pointcloud_test.rviz
octomap_odom_debug.rviz
dual_2d_3d_mapping.rviz
visual_laser_slam_map.rviz
```

另外完成了两个小修正：

1. STEP5 的 RViz 默认固定坐标系改为 `map`，避免误把 `odom` 漂移当成 SLAM 闭环失败；
2. 压缩包内的视觉 EKF 配置采用已经验证过的保守方案：轮速 `vy=0` 约束、视觉只融合相对 `x/y`、不融合视觉 yaw、不对视觉位置求差分。

---

## 3. 第一次使用前必须做什么

### 3.1 解压、赋予执行权限

```bash
unzip huichuan-agv-ros2-foxy-visual-lidar-slam-3d-step.zip
cd huichuan-agv-ros2-foxy-main
chmod +x STEP*.sh SAVE_STEP9_OCTOMAP.sh RESET_STEP9_OCTOMAP.sh
```

### 3.2 重新执行 STEP0

虽然以前执行过 STEP0，但这次新增了 OctoMap 软件包，所以需要再执行一次：

```bash
./STEP0_INSTALL_VISUAL_SLAM_DEPS.sh
```

它会额外安装：

```text
ros-humble-octomap-server
ros-humble-octomap-msgs
```

### 3.3 把已验证成功的相机外参填回来

编辑：

```bash
nano visual_laser_slam/visual_laser_slam.env
```

把你 STEP4、STEP5 已经测试成功的参数填入：

```bash
CAMERA_X=实际值
CAMERA_Y=实际值
CAMERA_Z=实际值
CAMERA_ROLL=实际值
CAMERA_PITCH=实际值
CAMERA_YAW=实际值
CAMERA_TF_CONFIRMED=true
```

单位：

- 距离：米；
- 角度：弧度；
- `x` 向车头为正；
- `y` 向车体左侧为正；
- `z` 向上为正。

STEP7～STEP9 检测到 `CAMERA_TF_CONFIRMED=false` 会拒绝启动，避免用错误外参把三维地图拼歪。

### 3.4 每次只运行一个 STEP

运行新脚本前必须关闭：

- `open_all.sh`；
- 其他 STEP 脚本；
- 单独启动的相机、Cartographer 或 OctoMap。

统一用 `Ctrl+C` 停止。

---

# 4. STEP6：原始深度点云测试

运行：

```bash
./STEP6_DEPTH_POINTCLOUD_TEST.sh
```

## 4.1 它启动什么

```text
Gemini2 RGB + Depth
        ↓
/camera/depth/points
        ↓
RViz
```

不启动：

- STM32；
- 2D 雷达；
- EKF；
- Cartographer；
- OctoMap；
- Nav2。

## 4.2 你需要做什么

1. 小车保持不动；
2. 在相机前方放置纸箱、椅子、桌腿等物体；
3. 在 RViz 中旋转视角，确认看到立体点云；
4. 观察透明物体、黑色物体、反光地面附近是否有空洞或飞点；
5. 至少运行 30 秒，确认没有 USB 断流。

另开终端检查：

```bash
ros2 topic hz /camera/depth/points
ros2 topic info /camera/depth/points
```

## 4.3 通过标准

- `/camera/depth/points` 持续发布；
- 点云方向与真实环境一致；
- 物体距离大致正确；
- 没有整片点云反向、倒置或不断闪烁；
- 树莓派没有卡死。

## 4.4 常见问题

### 没有 `/camera/depth/points`

先执行：

```bash
ros2 topic list | grep points
```

如果驱动实际话题不是 `/camera/depth/points`，修改：

```bash
POINT_CLOUD_TOPIC=实际话题
```

### 点云很密，树莓派负载高

STEP6 用于查看原始数据，负载较高是正常的。STEP7 会限频和降采样。

---

# 5. STEP7：过滤和降采样点云测试

运行：

```bash
./STEP7_FILTERED_POINTCLOUD_TEST.sh
```

## 5.1 它启动什么

```text
/camera/depth/points
        ↓
point_cloud_filter_node
        ├─ 最多处理 5 Hz
        ├─ 每隔 4 个点取 1 个
        ├─ 距离裁剪 0.30～4.0 m
        ├─ 按 base_link 坐标范围裁剪
        └─ 6 cm 体素降采样
        ↓
/camera/depth/points_filtered
```

点云输出仍保留原相机坐标帧，使 OctoMap 后续进行射线清空时仍从真实相机位置出发；节点只在内部临时转换到 `base_link` 做裁剪。

## 5.2 你需要做什么

1. 确认相机外参已经填写；
2. 小车先保持不动；
3. 比较 RViz 中过滤点云和现实物体；
4. 在前方不同距离放置物体；
5. 确认 0.3 m 以内和 4 m 以外的点被裁掉；
6. 确认左右、上下没有误裁掉真正需要的障碍。

查看点数和处理时间：

```bash
ros2 topic echo /depth_point_cloud_filter/stats
```

统计内容类似：

```json
{
  "input_points": 256000,
  "output_points": 8000,
  "process_ms": 20.5
}
```

实际数量会随场景变化。

## 5.3 主要调节参数

编辑：

```bash
nano visual_laser_slam/visual_laser_slam.env
```

### 树莓派太卡

依次这样减负载：

```bash
CLOUD_MAX_RATE_HZ=3.0
CLOUD_SAMPLE_STRIDE=6
CLOUD_VOXEL_SIZE=0.08
CLOUD_MAX_RANGE=3.0
```

### 地图太稀疏

逐步提高质量：

```bash
CLOUD_SAMPLE_STRIDE=3
CLOUD_VOXEL_SIZE=0.05
CLOUD_MAX_RATE_HZ=6.0
```

不要一次同时改很多参数。

### 裁剪上下高度

```bash
CLOUD_BASE_Z_MIN=-1.00
CLOUD_BASE_Z_MAX=2.50
```

它们是相对 `base_link` 的高度，不是相机光学坐标。

### 去除车体自身点云

确认车体盒子参数正确后才开启：

```bash
CLOUD_REMOVE_SELF=true
```

错误的车体盒子会把靠近小车的真实障碍一起删掉，因此第一版默认关闭。

## 5.4 通过标准

- 原始点云和过滤点云方向一致；
- 过滤后点数明显减少；
- 前方主要障碍仍然完整；
- 处理频率稳定；
- 没有持续 TF 报错；
- 树莓派 CPU 和画面明显比 STEP6 更轻。

---

# 6. STEP8：EKF 位姿 + OctoMap 三维建图隔离测试

运行：

```bash
./STEP8_EKF_OCTOMAP_3D_TEST.sh
```

## 6.1 它启动什么

```text
视觉里程计 + 轮速 + IMU
        ↓
EKF：odom → base_link

Gemini2 过滤点云
        ↓
OctoMap（世界坐标系 = odom）
        ↓
三维体素地图
```

本步骤**不启动 2D 雷达和 Cartographer**。

它的目的不是得到最终精确地图，而是单独确认：

- 点云能否随小车运动正确拼接；
- 相机外参是否正确；
- OctoMap 是否正常插入和清空体素；
- 计算量是否能被树莓派接受。

## 6.2 你需要做什么

1. 启动后先静止 10 秒；
2. 低速直行约 1 m；
3. 缓慢转弯；
4. 走一个小范围 L 形或矩形；
5. 观察同一面墙是否被拼成连续平面；
6. 回到起点后允许出现一定漂移，因为世界坐标使用的是 `odom`。

低速控制示例：

```bash
ros2 topic pub /cmd_vel_visual_slam_test geometry_msgs/msg/Twist \
"{linear: {x: 0.06}, angular: {z: 0.0}}" -r 20
```

停止后发送零速度：

```bash
ros2 topic pub --once /cmd_vel_visual_slam_test geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: 0.0}}"
```

## 6.3 通过标准

- `/occupied_cells_vis_array` 持续更新；
- 车动时三维地图跟随扩展；
- 静止时地图不持续大幅漂移；
- 墙面、箱子、桌子大致形成正确三维结构；
- 没有严重多层重影；
- CPU 不持续满载导致节点失联。

## 6.4 如何判断外参是否错误

下列现象通常优先检查相机外参：

- 小车原地转向时，静止墙面画出很大的圆环；
- 直行时墙面逐渐倾斜；
- 每转一次弯就多出一层旋转后的房间；
- 点云位置相对车体明显前后、左右或高度错位。

STEP8 使用 `odom`，少量长期漂移正常；几秒内立刻分层通常不是 odom 漂移，而是 TF 或时间延迟问题。

---

# 7. STEP9：视觉 + 2D 雷达 + 二维/三维双地图

运行：

```bash
./STEP9_2D_3D_DUAL_MAP_TEST.sh
```

## 7.1 最终数据链

```text
视觉里程计 + 轮速 + IMU
            ↓
           EKF
            ↓
     odom → base_link

2D 激光雷达 + 融合里程计
            ↓
       Cartographer
            ↓
 map → odom + 二维 /map

Gemini2 深度点云
            ↓
    点云过滤和降采样
            ↓
 OctoMap（世界坐标系 = map）
            ↓
       三维体素地图
```

2D 雷达和 Cartographer 负责长期位置校正，因此三维点云会按照 `map → camera` 位姿拼到全局地图中。

## 7.2 你需要怎么测试

### 第一轮：小范围房间

1. 静止 10 秒；
2. 原地或小圆弧慢慢转一圈，让相机扫到四周；
3. 沿墙低速走一圈；
4. 回到起点和原朝向；
5. 停下等待 5～10 秒，让 Cartographer 优化；
6. 同时观察二维地图和三维体素是否回到一致位置。

### 第二轮：开出房间再回来

沿用你 STEP5 已验证成功的路线：

```text
房间起点 → 开出房间 → 走廊 → 返回房间 → 现实原点
```

检查：

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom
```

预期：

- `odom → base_link` 可以有累计漂移；
- `map → odom` 会补偿漂移；
- `map → base_link` 回到起点附近；
- 三维墙面也应随全局修正回到接近原位置。

## 7.3 RViz 中显示什么

默认显示：

- Cartographer 2D Map；
- 2D LiDAR；
- Gemini2 Filtered Live Cloud；
- OctoMap 3D Occupied Voxels；
- EKF Odom；
- RGB 图像。

RViz 固定坐标系已经设为：

```text
map
```

`OctoMap Projected 2D` 默认关闭，只用于调试，不要拿它替代经过验证的 Cartographer `/map`。

## 7.4 通过标准

第一版建议达到：

- 二维地图仍保持 STEP5 的闭环质量；
- 三维地图没有明显持续撕裂；
- 回到旧区域时三维墙面误差明显小于纯 odom STEP8；
- 低矮障碍、桌面、柜体等能形成高度结构；
- 停止运动后地图不会继续大幅漂移；
- 连续运行 10～20 分钟不崩溃。

---

# 8. 保存、清空三维地图

## 8.1 保存 OctoMap

STEP9 运行中另开终端：

```bash
cd 项目根目录
./SAVE_STEP9_OCTOMAP.sh
```

默认保存到：

```text
maps_3d/octomap_年月日_时分秒.bt
```

也可以指定文件：

```bash
./SAVE_STEP9_OCTOMAP.sh "$HOME/my_room_3d.bt"
```

`.bt` 是压缩二进制占据树；`.ot` 保存完整 OctoMap 树结构。

## 8.2 清空 OctoMap 重新测试

STEP8 或 STEP9 运行时：

```bash
./RESET_STEP9_OCTOMAP.sh
```

它只清空三维 OctoMap，不会清空 Cartographer 轨迹。

---

# 9. 最重要的参数说明

配置文件：

```text
visual_laser_slam/visual_laser_slam.env
```

## 9.1 体素分辨率

```bash
OCTOMAP_RESOLUTION=0.08
```

含义：每个体素边长 8 cm。

- 0.10 m：最省算力，结构较粗；
- 0.08 m：树莓派5推荐起点；
- 0.05 m：更细，但 CPU 和内存压力明显上升；
- 0.03 m：不建议树莓派5第一版使用。

## 9.2 点云频率

```bash
CLOUD_MAX_RATE_HZ=5.0
```

三维地图不需要把相机每一帧都插入。小车低速时 3～5 Hz 通常足以测试。

## 9.3 最大距离

```bash
CLOUD_MAX_RANGE=4.0
OCTOMAP_MAX_RANGE=4.0
```

两处建议保持一致。距离太远的深度噪声容易形成“空中浮点”。

## 9.4 高度范围

```bash
CLOUD_BASE_Z_MIN=-1.00
CLOUD_BASE_Z_MAX=2.50
```

先用 RViz 确认 `base_link` 高度基准，再根据真实地面和天花板位置收紧。

## 9.5 地面过滤

```bash
OCTOMAP_FILTER_GROUND=false
```

第一版关闭，原因：

- RANSAC 地面分割额外消耗算力；
- 依赖 `base_link` 高度定义准确；
- 深度相机点云稀疏或地面反光时可能找不到平面。

如果三维图中的地面层影响很大，优先收紧 `CLOUD_BASE_Z_MIN`；确认 TF 和高度基准后，再测试：

```bash
OCTOMAP_FILTER_GROUND=true
```

---

# 10. 常见故障排查

## 10.1 STEP7 一直提示等待 TF

检查：

```bash
ros2 run tf2_ros tf2_echo base_link camera_link
ros2 run tf2_ros tf2_echo base_link camera_depth_optical_frame
```

如果第二条不存在，检查 Orbbec 是否发布 TF、相机驱动的 frame 名是否改变。

## 10.2 STEP8/STEP9 有点云但没有体素

检查：

```bash
ros2 topic hz /camera/depth/points_filtered
ros2 topic echo /depth_point_cloud_filter/stats
ros2 topic info /occupied_cells_vis_array
```

再看日志中是否出现：

```text
Transform ... does not exist
Lookup would require extrapolation
```

这通常是 TF 或时间戳问题，而不是 OctoMap 分辨率问题。

## 10.3 三维地图出现很多重影

按顺序排查：

1. STEP5 的 `map → base_link` 是否仍然稳定；
2. 相机外参是否准确；
3. 小车是否跑得太快；
4. 点云延迟是否过大；
5. 视觉里程计是否频繁丢失；
6. 体素是否设得过小。

先将速度降到约：

```text
0.04～0.08 m/s
```

再测试。

## 10.4 树莓派 CPU 太高

按此顺序调整：

```bash
USE_RVIZ=false
CLOUD_MAX_RATE_HZ=3.0
CLOUD_SAMPLE_STRIDE=6
CLOUD_VOXEL_SIZE=0.10
CLOUD_MAX_RANGE=3.0
OCTOMAP_RESOLUTION=0.10
```

`USE_RVIZ=false` 只关闭本机可视化，不会停止建图。

## 10.5 同一位置出现两套房间

- STEP8：可能是 odom 漂移，进入 STEP9 再看；
- STEP9：若二维 `/map` 正常但三维仍分层，重点检查相机外参和点云时间延迟；
- 如果二维和三维同时分层，先修 Cartographer/融合定位，不要调 OctoMap。

## 10.6 地面或天花板占据太多

先在 RViz 查看地面相对 `base_link` 的实际 z，再修改：

```bash
CLOUD_BASE_Z_MIN=合适值
CLOUD_BASE_Z_MAX=合适值
```

不要凭感觉直接填 0，因为你们的 `base_link` 不一定正好位于地面。

---

# 11. 推荐测试顺序

你已经通过 STEP1～STEP5，接下来严格执行：

```text
重新运行 STEP0
      ↓
把成功的相机外参填回 env
      ↓
STEP6：确认原始点云
      ↓
STEP7：确认过滤点云和性能
      ↓
STEP8：验证 OctoMap 与外参，不追求闭环
      ↓
STEP9：最终 2D/3D 双地图和全局闭环
      ↓
保存 .bt 三维地图
```

任何一步失败，都先停在当前 STEP 解决，不要直接跳到 STEP9。

---

# 12. 当前版本边界

当前 STEP9 实现的是：

> Cartographer 提供全局二维 SLAM 位姿，Gemini2 深度点云按照该位姿构建三维占据地图。

它可以用于：

- 三维环境展示；
- 低矮、悬空障碍物记录；
- 后续局部三维避障；
- 地形通过性分析的基础数据。

它暂时不是：

- 360° 真三维激光雷达；
- Cartographer 3D；
- RTAB-Map 完整三维回环 SLAM；
- 直接给 Nav2 使用的三维全局规划器。

后续导航仍建议使用 Cartographer 的二维 `/map`；三维 OctoMap 先用于环境理解和局部安全感知。

---

# 13. 参考的官方接口

- Orbbec ROS 2 点云：`enable_point_cloud=true`，深度点云话题 `/camera/depth/points`；
- OctoMap ROS 2：`octomap_server_node` 接收 `PointCloud2` 的 `cloud_in`，发布三维占据体素、二进制地图和二维投影；
- OctoMap 保存：`octomap_saver_node` 保存 `.bt` 或 `.ot`。
