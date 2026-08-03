# RGB-D时间戳同步：STEP2B诊断与STEP2C视觉里程计验证

## 1. 原问题

原视觉链常用：

```text
Color 15Hz
Depth 10Hz
```

两个周期分别约66.7ms和100ms，RTAB-Map只能近似配对，容易出现约30ms甚至更大的时间差警告。

本补丁不直接覆盖已经实车验证的STEP1～STEP9，而是增加两条独立测试链。

## 2. STEP2B：只测RGB-D时间戳

```bash
./STEP2B_RGBD_TIMESTAMP_SYNC_TEST.sh
```

配置：

```text
Color：1280×720@15Hz
Depth：1280×800@15Hz
Depth registration：开启
Align：HW，目标COLOR
Frame sync：开启
Host time sync：开启
time_domain：device
SensorData QoS
诊断配对窗口：40ms（小于15Hz的一帧周期，避免漏帧后错配到下一帧）
```

统计：

```bash
ros2 topic echo /rgbd_timestamp_sync/stats
```

重点字段：

```text
abs_diff_avg_ms
abs_diff_p95_ms
abs_diff_max_ms
color_hz
depth_hz
color_duplicate / depth_duplicate
color_backward / depth_backward
color_unmatched / depth_unmatched
```

建议验收：

```text
color_hz ≈ depth_hz ≈ 15Hz
abs_diff_p95_ms < 25ms
无重复时间戳
无倒退时间戳
```

同时生成Orbbec官方时间戳CSV：

```text
/tmp/orbbec_rgbd_sync_timestamp.csv
```

## 3. STEP2C：同步版视觉里程计

STEP2B通过后运行：

```bash
./STEP2C_RGBD_VISUAL_ODOM_SYNC_TEST.sh
```

它在STEP2基础上做了：

```text
Color/Depth统一15Hz
frame_sync=true
enable_sync_host_time=true
RTAB-Map SensorData QoS
RTAB-Map topic queue=5
RTAB-Map sync queue=10
approx_sync_max_interval=0.025s
wait_for_transform=0.10s
```

观察：

```bash
ros2 topic hz /visual_odom
ros2 topic echo /rgbd_timestamp_sync/stats
```

手推小车前进、后退和转弯，确认：

- `/visual_odom`连续；
- 不再频繁提示RGB/Depth相差约30ms以上；
- 时间戳无重复或倒退；
- 视觉里程计延迟没有不断积累。

## 4. 为什么暂时保留ApproximateTime

即使开启硬件帧同步，RGB与Depth的ROS时间戳也不一定做到纳秒级完全相同。当前先使用：

```text
approx_sync=true
max interval=25ms
```

只有STEP2B证明大量帧的时间戳完全一致，才考虑ExactTime。否则ExactTime可能导致视觉里程计完全收不到配对数据。

## 5. 测试结束后的处理

时间戳验证完成后，生产运行可关闭CSV：

```bash
RGBD_SYNC_ENABLE_FRAME_TIMESTAMP_CSV=false
```

避免长期写磁盘。
