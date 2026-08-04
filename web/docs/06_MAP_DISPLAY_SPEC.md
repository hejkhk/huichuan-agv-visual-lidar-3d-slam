# 地图显示规范

地图不是 RViz2 发给网页的。RViz2 与网页是同级 ROS2 显示端；二者都读取 ROS2 `/map`。网页经 rosbridge 订阅 `nav_msgs/OccupancyGrid`，再用 Canvas 自己绘制。

## OccupancyGrid 显示

| 值 | 含义 | 显示 |
|---:|---|---|
| -1 | unknown | 灰色 |
| 0 | free | 浅灰色 |
| 100 | occupied | 黑色 |
| 1～99 | 占用概率 | 从浅到深的灰度 |

ROS 数据从地图原点向上增长，Canvas Y 轴向下，因此图像写入 Canvas 时上下翻转。

## 交互

- ResizeObserver 自动适配容器；
- 保持宽高比并居中；
- 滚轮围绕指针缩放；
- 按住鼠标左键拖拽平移；
- Fit/Reset 恢复自适应；
- 显示分辨率、地图尺寸、更新时间和缩放比例；
- 未收到地图时显示 `Waiting for /map`。

## 位姿

收到 `/robot_pose` 后将 quaternion 转为 yaw，再绘制青色三角箭头。坐标变换：

```text
pixelX = (pose.position.x - map.info.origin.position.x) / map.info.resolution
pixelY = map.info.height - (pose.position.y - map.info.origin.position.y) / map.info.resolution
```

未收到 `/robot_pose` 不影响地图显示。
