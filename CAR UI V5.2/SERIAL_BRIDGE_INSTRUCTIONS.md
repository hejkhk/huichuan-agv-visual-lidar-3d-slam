# 串口通信规则

旧 PTY 串口中继已经停用。`chassis_node` 是 STM32 串口唯一拥有者，UI 不得打开真实串口或创建 PTY。

UI 通过 ROS 2 获取底盘、电池、里程计和 IMU 状态；控制请求通过 `/robot/web_control` 交给 `chassis_node`。例如归还 PS2：

```json
{"command":"serial_command","action":"ps2","source":"car_ui"}
```

请使用 `./run.sh` 启动 UI。历史入口 `./start_ui_with_relay.sh` 仅作为兼容包装器，也会转到 `run.sh`，不会再占用串口。
