# Robot Web Console

一个浏览器端 ROS2 机器人 Web 控制台前端，支持 MJPEG 视频流、ROSBridge 通信、四挡速度控制、软件急停、机器人状态以及 ROS2 `/map` 地图显示。

## 仓库范围

本仓库只包含前端网页控制台，不包含：

- 树莓派或 Ubuntu 端 ROS2 后端；
- ROS2 仿真工作区；
- 机器人控制节点；
- 视频文件、密码、Token 或真实机器人网络配置。

机器人端需要自行启动 `rosbridge_server`，并提供浏览器可访问的 MJPEG 视频流。可选发布 `/map`、`/robot_pose` 和 `/robot/status`，以显示地图、机器人位姿和真实状态。

默认连接格式：

```text
Robot IP:       192.168.1.100
Video URL:      http://192.168.1.100:8080/video_feed
ROSBridge URL:  ws://192.168.1.100:9090
```

实际使用时请替换为机器人在当前局域网中的地址。

> **安全提示：网页 Emergency Stop 只是软件急停，不能替代独立、常闭、失效安全的物理急停回路。**

## 功能

- Camera Stream：连接、断开并显示 MJPEG；
- Motion Control：四挡、10Hz 长按移动、停止、软件急停；
- Robot Status：显示本地状态及可选 `/robot/status`；
- Map：订阅 `/map`，Canvas 绘制、缩放、平移及可选位姿箭头；
- 使用 `localStorage` 保存 Robot IP、Video URL 和 ROSBridge URL。

## 本地运行

需要 Node.js 18+、Chrome 或 Edge。

```powershell
npm install
npm run dev
```

浏览器打开：

```text
http://localhost:5173
```

## 生产构建

```powershell
npm run build
```

构建结果位于本地 `dist/`，该目录不会提交到 GitHub。

## 可选：本地视频模拟

先设置一个只存在于本机的视频绝对路径：

```powershell
$env:ROBOT_MOCK_VIDEO = "C:\path\to\video.mp4"
npm run dev:mock
```

模拟服务仅用于前端视频显示测试，不模拟 ROSBridge。视频文件受 `.gitignore` 保护，不应提交到仓库。

## 机器人端要求

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

机器人端至少需要：

- `ws://<robot-ip>:9090`：ROSBridge WebSocket；
- `http://<robot-ip>:8080/video_feed`：MJPEG 视频流；
- 订阅 `/robot/web_control` 或 `/cmd_vel` 的控制节点；
- 可选 `/map`、`/robot_pose`、`/robot/status`。

## 文档

- [项目概览](docs/00_PROJECT_OVERVIEW.md)
- [Windows 快速开始](docs/01_QUICK_START_WINDOWS.md)
- [树莓派要求](docs/02_RASPBERRY_PI_REQUIREMENTS.md)
- [ROS2 接口](docs/03_ROS2_INTERFACE_SPEC.md)
- [控制协议](docs/04_CONTROL_PROTOCOL.md)
- [挡位与 LED](docs/05_GEAR_AND_LED_SPEC.md)
- [地图显示](docs/06_MAP_DISPLAY_SPEC.md)
- [团队接入指南](docs/07_INTEGRATION_GUIDE.md)
- [测试清单](docs/08_TEST_CHECKLIST.md)
- [FAQ](docs/09_FAQ.md)

## 源码结构

```text
src/config/       四挡与速度映射
src/ros/          ROSBridge、Topic、发布器、订阅器
src/control/      挡位、移动、急停
src/map/          OccupancyGrid 与位姿绘制
src/video/        MJPEG 视频
src/utils/        时间与校验
docs/             接口与交付文档
examples/         JSON 与 ROS2 测试命令
```

所有 ROS topic 名称集中在 `src/ros/topics.js`，所有挡位参数集中在 `src/config/gearConfig.js`。
