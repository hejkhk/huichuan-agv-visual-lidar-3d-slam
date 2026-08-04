# 源码清理与测试说明

## 这版做了什么

1. 用仓库 `f003b7a` 提交完整恢复 STEP1～STEP9 的核心文件：
   - `visual_laser_slam.launch.py`
   - `gemini2_rgbd_640.launch.py`
   - `run_visual_slam_step.sh`
   - `visual_laser_slam.env`
   - 原 RViz 配置
2. 删除另一个 Agent 反复用 `sed` 塞进主启动文件的 `visual_odom_sync*`、`visual_odom_baseline` 等实验 profile。
3. STEP10V2.1 改为完全独立的 launch 和运行脚本，不再碰 STEP1～STEP9。
4. 新增独立、低负载的 STEP2C4 视觉里程计测试：RGB/Depth 都是 15Hz、SensorData QoS、单父节点 TF，默认不开 RGB/Depth RViz 显示和时间监控。
5. 新增 RTAB-Map 日志统计脚本，避免只截取一两帧就宣布通过。

## 运行 STEP10V2.1

```bash
./STEP10V21_STABLE_LOCAL_CLOUD_TEST.sh
```

它使用 Depth 1280×800@30Hz、关闭 RGB、关闭相机完整 PointCloud2，只输出：

- `/local_highres_cloud_v21`
- `/local_highres_cloud_v21/stats`

## 运行干净视觉里程计

```bash
./STEP2C4_CLEAN_VISUAL_ODOM_TEST.sh
```

默认：

- Color 640×480@15Hz
- Depth 640×400@15Hz
- 硬件对齐到 Color
- frame sync + host time sync
- Best Effort/SensorData QoS
- `odom → base_link → camera_link`
- RViz关闭
- 不启动大图时间监控节点

停止后统计完整日志：

```bash
./REPORT_STEP2C4_VISUAL_ODOM.sh
```

手动测图像频率时，必须使用匹配相机的 Best Effort QoS：

```bash
ros2 topic hz /camera/color/image_raw --qos-reliability best_effort
ros2 topic hz /camera/depth/image_raw --qos-reliability best_effort
ros2 topic hz /visual_odom_clean
```

## 重要原则

- 原 STEP9 继续使用原脚本 `STEP9_2D_3D_DUAL_MAP_TEST.sh`，逻辑已恢复。
- STEP10V2.1 高分辨率避障与 STEP9/视觉里程计分开运行测试。
- STEP2C4通过后，再决定是否把它替换进生产融合链；本版不会未经实测直接改 STEP9。
