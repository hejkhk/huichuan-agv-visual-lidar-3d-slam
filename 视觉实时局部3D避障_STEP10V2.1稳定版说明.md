# STEP10V2.1：稳定低延迟局部三维点云

## 1. 为什么需要V2.1

STEP10V2已经把链路从“大型PointCloud2 + Python过滤”改成“1280×800深度图 + C++直接投影”，典型处理时间约10～38ms、总延迟约51～96ms。

但实测仍可能出现“画面突然卡一下，然后恢复”。平均值正常并不能证明没有偶发尖峰，因此V2.1专门处理：

- 订阅回调被计算堵住；
- 某一帧内存扩容或哈希表rehash；
- 每帧等待TF；
- 旧帧排队后继续处理；
- 只有当前值，没有P95/最大值，抓不到尖峰；
- 相机时间戳重复、倒退或帧间隔突然变大。

## 2. V2.1核心改动

### 2.1 最新帧邮箱

深度订阅回调只保存最新的Image共享指针，然后立即返回。

```text
深度回调 → 只覆盖“最新帧邮箱” → 立即返回
                      ↓
                独立工作线程
                      ↓
              生成局部PointCloud2
```

处理过程中又来了新帧，只保留最新帧，不积压历史画面。

### 2.2 静态TF只查询一次

Gemini2固定在车上，`base_link <- camera_depth_optical_frame`属于静态外参。V2.1启动后缓存一次，正常运行时不再逐帧等待TF。

### 2.3 预计算投影射线

1280×800、stride=2时，采样像素位置和相机投影射线只计算一次。后续每帧只做：

```text
depth × 预计算ray → 相机XYZ → base_link XYZ
```

### 2.4 固定体素代数表

V2使用`unordered_set`逐帧去重，可能扩容和rehash。V2.1改成启动时分配固定体素数组，每帧只增加generation编号，不逐帧清空整张表。

### 2.5 点缓存预分配

输出点数组一次性预留完整采样容量，避免障碍点突然变多时vector扩容。

### 2.6 过旧帧直接丢弃

默认：

```bash
STEP10V21_MAX_INPUT_AGE_MS=150.0
```

邮箱中的图像已经超过150ms时，不再生成避障点云。

## 3. 启动

```bash
cd /home/hejkhk/下载/huichuan-agv-ros2-foxy-visual-lidar-slam-step/huichuan-agv-ros2-foxy-main
chmod +x STEP10V21_STABLE_LOCAL_CLOUD_TEST.sh
./STEP10V21_STABLE_LOCAL_CLOUD_TEST.sh
```

输出：

```text
/local_highres_cloud_v21
/local_highres_cloud_v21/stats
/local_highres_cloud_v21/crop_markers
```

## 4. 重点统计字段

```bash
ros2 topic echo /local_highres_cloud_v21/stats
```

主要观察：

```text
process_avg_ms / process_p95_ms / process_max_ms
age_avg_ms / age_p95_ms / age_max_ms
input_gap_p95_ms / input_gap_max_ms
output_gap_p95_ms / output_gap_max_ms
mailbox_replaced_frames
stale_dropped_frames
duplicate_timestamp_frames
backward_timestamp_frames
input_stall_events
output_stall_events
tf_cache_refreshes
ray_table_rebuilds
```

正常特征：

```text
tf_cache_refreshes ≈ 1
ray_table_rebuilds ≈ 1
backward_timestamp_frames = 0
duplicate_timestamp_frames = 0
process_p95_ms < 25ms
age_p95_ms < 100ms
output_gap_max_ms < 120ms
```

`mailbox_replaced_frames`少量增长不是故障，代表计算繁忙时主动跳过旧帧，确保输出的是新画面。

## 5. 如何判断卡顿来自哪里

### 相机输入卡住

```text
input_gap_max_ms很大
arrival_gap_max_ms也很大
input_stall_events增加
```

检查USB3、线材、曝光和Orbbec驱动。

### C++处理尖峰

```text
输入间隔正常
process_max_ms突然很大
output_gap_max_ms同步变大
```

优先尝试：

```bash
STEP10V21_PIXEL_STRIDE=3
```

### RViz渲染卡顿

```text
话题统计稳定
process/age/output gap都稳定
只有RViz肉眼卡
```

关闭Depth Image显示，只保留局部点云；点大小使用1 pixel，Decay Time设为0。

## 6. Orbbec时间戳CSV调试

正常运行默认关闭CSV，避免磁盘写入影响实时性。排查输入卡顿时修改：

```bash
STEP10V21_ENABLE_FRAME_TIMESTAMP_CSV=true
```

CSV路径：

```text
/tmp/orbbec_step10v21_timestamp.csv
```

复现卡顿1～2分钟后停止，检查Depth相邻帧间隔是否突然从约33ms变成70、100或200ms。

## 7. 验收动作

1. 不开电机，快速左右摆动纸板30秒。
2. 放置桌腿、细杆、低矮木板和悬空纸板。
3. 连续运行5分钟。
4. 记录三次stats。
5. 确认肉眼没有明显停顿，P95和最大值满足要求。

STEP10V2.1通过后再进入STEP11局部体素地图。
