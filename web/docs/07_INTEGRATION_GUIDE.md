# 团队接入指南

## 前端同学

- 页面结构在 `index.html`，视觉样式在 `src/style.css`；
- 四挡参数只改 `src/config/gearConfig.js`；
- Topic 名称与类型只改 `src/ros/topics.js`；
- 不要把控制、地图或连接逻辑塞回 `main.js`；
- `/robot/status` 新字段可在 `main.js` 的 `onStatus` 中呈现；
- 修改后执行 `npm run build`，再在 Chrome/Edge 测试桌面与窄屏布局。

## ROS2 同学

树莓派至少启动 rosbridge 和控制节点；地图、位姿、状态按需求发布。快速验证：

```bash
ros2 topic echo /robot/control/gear
ros2 topic echo /robot/web_control
ros2 topic echo /robot/emergency_stop
ros2 topic echo /cmd_vel
ros2 topic echo /map --once
ros2 topic echo /robot_pose
ros2 topic echo /robot/status
```

切挡时应同时看到 gear 与 `gear_change`；长按移动时 web_control 和 cmd_vel 约为 10Hz；松开必须出现 stop 与零 Twist。

## 下位机同学

下位机不直接连接网页。推荐链路：

```text
Web → rosbridge → Raspberry Pi ROS2 control node
    → JSON 解析/安全检查 → CAN / UART / RS485 / USB 串口 → 下位机
```

必须接入 `/robot/web_control` 中的：

- `command`；
- `gear`；
- `led_color`；
- `speed_cnt_per_sec`；
- `multiplier`；
- `timestamp_ms`（可用于超时/陈旧命令判断）。

控制节点应实现命令看门狗：连续运动命令超时即停止。急停还应监听 `/robot/emergency_stop`，并让停止优先级高于移动。`/cmd_vel` 只是调试兼容，不能代替 cnt/s 控制协议。

软件急停之外，机器人必须具备独立物理急停和硬件安全措施。
