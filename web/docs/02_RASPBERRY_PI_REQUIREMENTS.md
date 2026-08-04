# 树莓派 5B 端要求

树莓派与 Windows 必须在同一可互访的局域网，并提供：

1. MJPEG 视频服务，例如 `http://IP:8080/video_feed`；
2. rosbridge WebSocket，例如 `ws://IP:9090`；
3. `/map`，需要网页地图时发布；
4. `/robot_pose`，需要显示位置和朝向时发布；
5. `/robot/status`，需要显示真实机器人状态时发布；
6. ROS2 控制节点订阅 `/robot/web_control`；
7. ROS2 控制节点或下位机链路处理 `speed_cnt_per_sec`。

## rosbridge

安装对应 ROS2 发行版的 `rosbridge_server` 后启动：

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

默认监听 `9090`。确认防火墙、AP 隔离与路由没有阻止 Windows 访问该端口。

## 视频服务

返回类型应为浏览器 `<img>` 可直接打开的 MJPEG：

```text
Content-Type: multipart/x-mixed-replace; boundary=frame
```

先在 Windows 浏览器直接打开视频 URL。若直接打开也无画面，应先排查树莓派相机和视频服务，而不是网页。

## 控制链路

建议树莓派控制节点订阅 `/robot/web_control`，解析 JSON 后再转换为 CAN、UART、RS485 或 USB 串口协议。`speed_cnt_per_sec` 是下位机有效速度基准；`/cmd_vel` 只是兼容 RViz2、Nav2 或调试的第一版映射。

网页急停是软件消息，绝不能替代物理急停回路。
