# 四挡与 RGB LED 规范

| 挡位 | 英文名 | RGB LED | 倍率 | 有效速度 |
|---|---|---|---:|---:|
| 1挡 | Slow | Green / 绿色 | x1 | 15M cnt/s |
| 2挡 | Normal | Blue / 蓝色 | x2 | 30M cnt/s |
| 3挡 | Fast | Yellow / 黄色 | x5 | 75M cnt/s |
| 4挡 | Turbo | Red / 红色 | x30 | 450M cnt/s |

页面默认 1挡绿色。点击 `△ Gear` 按 `1 → 2 → 3 → 4 → 1` 循环。每次切挡同步更新：

1. 当前挡位；
2. RGB LED 圆灯颜色；
3. 倍率和速度；
4. `/robot/control/gear`；
5. `/robot/web_control` 的 `gear_change`；
6. Recent events 与 Last Command。

配置唯一来源是 `src/config/gearConfig.js`。需要调整参数时只改该文件，不要在界面或发布器中重复写数值。

前端只是发布挡位、颜色和速度状态；真正驱动 RGB LED 硬件的动作由树莓派 ROS2 节点或下位机完成。
