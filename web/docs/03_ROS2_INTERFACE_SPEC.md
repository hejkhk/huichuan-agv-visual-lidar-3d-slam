# ROS2 接口规范

所有名称在 `src/ros/topics.js` 集中定义。

| Topic | 类型 | 方向 | 用途 |
|---|---|---|---|
| `/robot/control/gear` | `std_msgs/UInt8` | Web → ROS2 | 当前挡位 1～4 |
| `/robot/web_control` | `std_msgs/String` | Web → ROS2 | JSON 控制命令 |
| `/robot/emergency_stop` | `std_msgs/Bool` | Web → ROS2 | 软件急停状态 |
| `/cmd_vel` | `geometry_msgs/Twist` | Web → ROS2 | ROS 常规速度兼容/调试 |
| `/map` | `nav_msgs/OccupancyGrid` | ROS2 → Web | Canvas 地图 |
| `/robot_pose` | `geometry_msgs/PoseStamped` | ROS2 → Web | 地图机器人位置与朝向 |
| `/robot/status` | `std_msgs/String` | ROS2 → Web | JSON 真实状态 |

## 发布行为

- 切挡：发布 `/robot/control/gear` 和 `/robot/web_control` 的 `gear_change`；
- 按住移动：以 10Hz 发布 `/robot/web_control` 与 `/cmd_vel`；
- 松开移动/点击停止：发布 `stop` 与零 `/cmd_vel`；
- 急停：发布 `emergency_stop`、`/robot/emergency_stop=true`、零 `/cmd_vel`；
- 复位：发布 `reset_estop`、`/robot/emergency_stop=false`。

## `/robot/status`

`std_msgs/String.data` 建议为：

```json
{
  "mode": "manual",
  "state": "moving",
  "gear": 2,
  "led_color": "blue",
  "speed_cnt_per_sec": 30000000,
  "battery_voltage": 24.5,
  "estop": false,
  "message": "Moving forward"
}
```

若未发布 `/robot/status`，页面显示自身已知状态；若未发布 `/robot_pose`，地图仍正常显示但没有机器人箭头。

## `/cmd_vel` 第一版映射

| 挡位 | `linear.x` 绝对值 | `angular.z` 绝对值 |
|---|---:|---:|
| 1 | 0.08 | 0.20 |
| 2 | 0.16 | 0.40 |
| 3 | 0.40 | 0.80 |
| 4 | 0.80 | 1.50 |

前进为正 `linear.x`，后退为负；左旋为正 `angular.z`，右旋为负。真实下位机速度应以 `/robot/web_control.speed_cnt_per_sec` 为准。
