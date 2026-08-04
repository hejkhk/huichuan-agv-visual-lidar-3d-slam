# Orbbec Gemini2 相机 SDK 与 ROS2 Wrapper 替换记录

**日期**: 2026-08-05
**工作空间**: `/home/jetson/Documents/huichuan-agv-visual-lidar-3d-slam/lidar/chapt1_ws/`

---

## 背景

之前的 Orbbec Gemini2 相机在 ROS2 驱动下报 `uvc_stream_start failed! status:115` 错误，无法正常启动视频流。尝试了多种方法（替换 SDK、手动 unbind uvcvideo、Python SDK 测试等）均未解决。

## 操作步骤

### 1. 安装 Orbbec SDK deb 包

```bash
sudo dpkg -i /home/jetson/Downloads/OrbbecSDK_v2.9.3_arm64.deb
```

- 安装路径: `/opt/OrbbecSDK_v2.9.3/`
- 包含完整的 `libob_frame_processor.so` 扩展（之前缺失的关键文件）
- 自动安装 udev 规则: `/etc/udev/rules.d/99-obsensor-libusb.rules`

### 2. 从 Gitee 克隆新的 ROS2 Wrapper

```bash
git clone https://gitee.com/orbbecdeveloper/OrbbecSDK_ROS2.git /tmp/OrbbecSDK_ROS2_new
```

### 3. 替换工作空间中的旧 Wrapper

```bash
# 备份并删除旧 wrapper
mv src/OrbbecSDK_ROS2 src/OrbbecSDK_ROS2.bak
rm -rf src/OrbbecSDK_ROS2.bak  # 因为 colcon 不允许重复包名

# 复制新 wrapper
cp -r /tmp/OrbbecSDK_ROS2_new src/OrbbecSDK_ROS2
```

### 4. 编译

```bash
# 编译到工作空间 install 目录
colcon build --packages-select orbbec_camera_msgs orbbec_camera \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --parallel-workers 1

# 编译到缓存 install 目录（供导航脚本使用）
colcon build --packages-select orbbec_camera_msgs orbbec_camera \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --install-base ~/.cache/huichuan_agv_dual_3d_humble_ws/install \
  --build-base ~/.cache/huichuan_agv_dual_3d_humble_ws/build \
  --parallel-workers 1
```

## 结果

### 相机驱动成功启动

```
Device Orbbec Gemini 2 connected
Serial number: AY3Z33100AS
Firmware version: 1.4.98
SDK version: 2.9.3
USB connect type: USB3.0
Color: 1280x720@30fps MJPG
Depth: 1280x720@30fps Y16
IR: 1280x800@30fps Y8
```

- `uvc_stream_start failed! status:115` 错误已解决
- 固件版本从 1.4.72 升级到 1.4.98
- frameprocessor 扩展完整可用

### 非致命错误

```
onNewFrameSetCallback error: Unsupported image format or invalid encoding detected. status:100
```

此错误不影响相机基本功能，流数据正常发布。

## 遗留问题

### 导航脚本 pipeline_version 检查失败

导航脚本 `START_DUAL_2D_3D_NAVIGATION.sh` 中的 `wait_parameter_value` 函数使用 `ros2 param get` 检查 `depth_image_to_local_cloud_v21` 节点的 `pipeline_version` 参数，但因 ROS2 daemon 通信问题一直返回 `unavailable`。

**已尝试的修复**:
- 在脚本的 `ros2 param get` 命令中加了 `--no-daemon` 参数
- 重启 ROS2 daemon

**待解决**: daemon 通信问题仍需进一步排查。

## 关键文件路径

| 项目 | 路径 |
|------|------|
| 系统 SDK | `/opt/OrbbecSDK_v2.9.3/` |
| 工作空间 | `/home/jetson/Documents/huichuan-agv-visual-lidar-3d-slam/lidar/chapt1_ws/` |
| ROS2 Wrapper | `src/OrbbecSDK_ROS2/orbbec_camera/` |
| 缓存 install | `~/.cache/huichuan_agv_dual_3d_humble_ws/install/` |
| 导航脚本 | `START_DUAL_2D_3D_NAVIGATION.sh` |
| 点云节点源码 | `src/local_depth_cloud_cpp/src/depth_image_to_local_cloud_v21_node.cpp` |

## 2026-08-05 点云卡顿复查与 Jetson 修正

### 日志结论

- 6 次有效测试均出现本地点云断流，最长连续停顿约 `10.2 s`。
- 点云节点单帧处理通常为 `30-80 ms`，最坏约 `127 ms`，并不足以解释数秒停顿。
- 停顿期间 RGB 仍接近 `15 Hz`，Depth 会降至接近 `0 Hz`；因此断点位于点云算法上游。
- RTAB-Map 在 Jetson 上单次处理最高约 `2.02 s`，延迟最高约 `2.45 s`，超过原来的
  `2 Hz` 周期预算，会加剧 CPU 和内存带宽争用。
- 同一轮日志中 2D 雷达串口发生 `97` 次读失败和重连，说明还存在 USB 拓扑、供电或设备
  并发访问问题，不能只用软件降频掩盖。
- SDK 日志出现 Depth 数据长度不符合预期并丢帧。后续运行会保存官方帧丢失 CSV，用于区分
  SDK 接收丢帧与 ROS 发布丢帧。

### 已实施修正

1. Gemini2 的 Color、Depth 和 CameraInfo 改用 `SENSOR_DATA` QoS，慢速 RTAB-Map 或 RViz
   不再通过 Reliable 队列反压实时深度流。
2. Jetson 自动使用 RTAB-Map `1 Hz`、`OMP_NUM_THREADS=1`；PC 保持 `2 Hz`、2 线程。
   可用 `DUAL_3D_RTABMAP_RATE` 和 `DUAL_3D_RTABMAP_THREADS` 显式覆盖。
3. 保留本地避障点云 `640x400 @ 15 Hz`、步长 2、`3 cm` voxel，不降低碰撞避障分辨率。
4. 每次运行保存 `orbbec_frame_timestamp.csv` 到对应 `SLAM_Log` 会话目录。
5. ROS CLI 检查改为正确的 `ros2 --no-daemon param get ...` 形式，并在包检查前加载缓存 overlay。

### Jetson 实机复测

复测时运行原一键脚本即可。若仍有卡顿，请同时保存本次 `SLAM_Log`，并在 Jetson 执行：

```bash
lsusb -t
sudo tegrastats
```

Gemini2 应单独连接 USB3 端口；2D 雷达和 STM32 建议连接独立的有源 USB2 Hub。若新日志中
Color 正常而 Depth 与官方 CSV 同时断流，优先排查 Gemini2 USB 链路、供电和 SDK；若官方
接收正常但 ROS 发布断流，再继续检查 DDS 和 Wrapper 发布线程。
