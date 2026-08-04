# ROS2 联调命令

## 观察网页输出

```bash
ros2 topic echo /robot/control/gear
ros2 topic echo /robot/web_control
ros2 topic echo /robot/emergency_stop
ros2 topic echo /cmd_vel
ros2 topic echo /map --once
ros2 topic echo /robot_pose
ros2 topic echo /robot/status
```

## 手动发布

```bash
ros2 topic pub /robot/control/gear std_msgs/msg/UInt8 "{data: 2}"
ros2 topic pub /robot/emergency_stop std_msgs/msg/Bool "{data: true}"
```

发布状态示例：

```bash
ros2 topic pub --once /robot/status std_msgs/msg/String \
  "{data: '{\"mode\":\"manual\",\"state\":\"idle\",\"gear\":1,\"led_color\":\"green\",\"speed_cnt_per_sec\":15000000,\"battery_voltage\":24.5,\"estop\":false,\"message\":\"Robot is idle\"}'}"
```

发布位姿示例：

```bash
ros2 topic pub --once /robot_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```
