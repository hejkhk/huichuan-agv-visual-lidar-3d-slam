# 交付测试清单

## Windows 页面

- [ ] `npm install` 成功
- [ ] `npm run dev` 成功
- [ ] `http://localhost:5173` 能打开
- [ ] 四宫格完整显示
- [ ] Robot IP 自动生成两个 URL
- [ ] Video URL 刷新后恢复
- [ ] ROSBridge URL 刷新后恢复
- [ ] Connect Video 状态变化正确
- [ ] Connect ROS 状态变化正确
- [ ] Disconnect 同时断开两条连接

## 挡位

- [ ] 默认 1挡绿色、15M cnt/s
- [ ] 点击三角键变 2挡蓝色、30M cnt/s
- [ ] 再点变 3挡黄色、75M cnt/s
- [ ] 再点变 4挡红色、450M cnt/s
- [ ] 再点回 1挡绿色、15M cnt/s
- [ ] 每次发布 `/robot/control/gear`
- [ ] 每次发布 `/robot/web_control gear_change`

## 控制

- [ ] 按住前进以 10Hz 发布 forward
- [ ] 松开前进发布 stop
- [ ] 按住后退以 10Hz 发布 backward
- [ ] 松开后退发布 stop
- [ ] 按住左旋以 10Hz 发布 turn_left
- [ ] 松开左旋发布 stop
- [ ] 按住右旋以 10Hz 发布 turn_right
- [ ] 松开右旋发布 stop
- [ ] Stop 发布速度 0 与零 Twist
- [ ] 急停发布 emergency_stop、Bool true、零 Twist
- [ ] 急停后移动按钮锁定
- [ ] Reset 发布 reset_estop 与 Bool false
- [ ] Reset 后移动按钮恢复

## 地图与状态

- [ ] `/map` 后 Canvas 有地图
- [ ] -1、0、1～99、100 灰度正确
- [ ] 地图 Y 轴方向正确
- [ ] 滚轮缩放与拖拽平移可用
- [ ] `/robot_pose` 后显示朝向箭头
- [ ] 无 `/robot_pose` 时地图仍显示
- [ ] `/robot/status` 合法 JSON 能显示
- [ ] 非法状态 JSON 不使页面崩溃

## ROS2 命令

```bash
ros2 topic echo /robot/control/gear
ros2 topic echo /robot/web_control
ros2 topic echo /robot/emergency_stop
ros2 topic echo /cmd_vel
ros2 topic echo /map --once
```
