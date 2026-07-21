# 项目概览

Robot Web Console 是运行在 Windows Chrome/Edge 中的本地机器人控制台。它通过 HTTP `<img>` 显示树莓派 MJPEG 相机视频，通过 rosbridge WebSocket 连接树莓派 ROS2；能够显示地图与状态、切换四挡、发布移动和软件急停命令。

## 架构

```text
Windows Browser
├─ MJPEG Video Viewer ───────── HTTP ───────┐
├─ Web Control Panel ─────┐                 │
├─ ROSBridge Client ──────┼─ WebSocket ──┐ │  WiFi LAN
└─ Map Canvas Renderer ───┘              │ │
                                         ▼ ▼
Raspberry Pi 5B
├─ Camera MJPEG Server :8080
├─ rosbridge_server :9090
├─ ROS2 /map
├─ ROS2 /robot_pose（可选）
├─ ROS2 /robot/status（可选）
└─ ROS2 robot control node
```

视频和 ROS2 是两条独立连接：视频服务提供连续图像，rosbridge 将浏览器 JSON/WebSocket 消息与 ROS2 消息转换。网页与 RViz2 都是 ROS2 topic 的同级显示端。

## 模块边界

- `config/gearConfig.js`：挡位、LED、cnt/s 与调试速度映射；
- `ros/`：连接、全部 topic、发布器与订阅器；
- `control/`：挡位循环、10Hz 长按移动、软件急停；
- `video/`：MJPEG 生命周期；
- `map/`：OccupancyGrid、缩放平移、位姿叠加；
- `main.js`：装配模块与同步页面状态。
