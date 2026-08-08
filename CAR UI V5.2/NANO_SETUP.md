# Jetson Orin Nano 接手入口

本工程的最新、唯一部署步骤位于：

- [`DEPLOY_UBUNTU_24_04_JETSON.md`](DEPLOY_UBUNTU_24_04_JETSON.md)
- [`ARCHITECTURE.md`](ARCHITECTURE.md)
- [`INTERFACE_CONTRACT.md`](INTERFACE_CONTRACT.md)

接手 Agent 应先确认 Jetson 型号、`aarch64`、Ubuntu、Python、ROS 2 和桌面会话版本，再安装依赖。不要复制其他机器的 `.venv`、旧 `.deb`、`dist/` 或 `__pycache__/`。

语音、视觉、导航和底盘模块只接 `robot_api` 层，不要在 QML 中直接调用 ROS、SDK、shell 或设备进程。所有方法返回 `ApiResult`，所有状态通过 `RobotSnapshot` 提供。
