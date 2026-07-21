# FAQ

## 1. 为什么地图不是 RViz2 发过来的？

RViz2 不负责向网页转发地图。RViz2 与网页都是显示端，二者共同的数据源是 ROS2 `/map`。

## 2. 为什么网页需要 rosbridge？

浏览器不能直接加入 DDS/ROS2 网络。rosbridge 用 WebSocket 和 JSON 在浏览器与 ROS2 消息之间转换。

## 3. 为什么视频流和 ROS2 是两个连接？

相机是 HTTP MJPEG 连续媒体；控制和状态是 rosbridge WebSocket 消息。分离后任一连接失败不会强制拖垮另一条。

## 4. 为什么 cnt/s 不直接等于 m/s？

cnt/s 是电机编码器或控制器计数速度，换算 m/s 还需要轮径、编码器分辨率、减速比和机械参数。下位机以 `speed_cnt_per_sec` 为准。

## 5. 为什么网页急停不是物理急停？

网页依赖浏览器、WiFi、rosbridge、ROS2 和软件节点，任一环节故障都可能让消息无法到达。物理急停必须独立、失效安全地切断危险能量。

## 6. 没有 `/map` 时为什么右下角没地图？

网页不自造地图；没有 OccupancyGrid 数据时只显示 `Waiting for /map`。

## 7. 没有 `/robot_pose` 时为什么没有机器人箭头？

地图本身不包含机器人实时位姿。只有 `/robot_pose` 提供位置和 quaternion 后才能画箭头。

## 8. ROSBridge 连接不上怎么办？

确认 `rosbridge_server` 已启动、IP/端口正确、两台设备同网段、AP 未启用客户端隔离、防火墙允许 9090，并用浏览器开发者工具查看 WebSocket 错误。

## 9. 视频流能直接打开但网页不显示怎么办？

确认 URL 完整、协议与页面安全策略兼容。HTTPS 页面通常会阻止 HTTP 混合内容，本项目应通过本地 `http://localhost:5173` 打开。还需检查 MJPEG 响应类型和是否限制跨域/Referer。
