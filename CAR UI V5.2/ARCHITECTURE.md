# 架构与扩展说明

## 1. 分层

```text
Qt Quick / QML
  ├─ Theme.qml / AppMetrics.qml
  ├─ pages/
  ├─ components/
  └─ dialogs/
          │ Qt Property / Signal / Slot
          ▼
UiBackend(QObject)
  ├─ 页面选择状态
  ├─ 异步任务与 Toast
  ├─ 750 ms 快照轮询
  └─ QML 数据模型
          │ RobotApiBase
          ▼
Robot API
  ├─ MockRobotApi：可离线运行的完整演示
  └─ TeamRobotApi：ROS 2、Nav2、串口和团队实现入口
```

依赖方向只能向下。QML 不允许导入或调用 `rclpy`、设备 SDK、文件系统、shell 或 subprocess。

## 2. 运行时数据流

### 状态

```text
ROS/设备回调
  → TeamRobotApi 的线程安全缓存
  → get_robot_snapshot()
  → UiBackend._poll_snapshot()
  → snapshotChanged
  → QML 绑定刷新
```

ROS 回调不直接操作 QML。`RobotSnapshot` 是状态字段的单一事实来源。

### 操作

```text
QML 点击/拖动
  → UiBackend @Slot
  → ApiWorker / QThreadPool
  → RobotApiBase 方法
  → ApiResult
  → 刷新数据或显示 Toast
```

失败通过 `ApiResult.fail(message, error_code)` 返回。不可让设备异常穿过 Qt 主线程。

## 3. 页面结构

```text
Main.qml
├─ StartupSplash（上次主题 + 联合品牌 Logo + 1–3 秒加载）
├─ HomePage（地图与车辆状态 + 等高的导航、语音、手柄、视觉四入口）
│  ├─ 左：地图、导航状态与车辆状态
│  └─ 右：控制模式总卡片；语音/视觉详情按需替换右半屏
├─ SettingsPage
├─ PointManagerPage
├─ RobotStatusPage
├─ FollowPage
├─ VoiceControlPage
├─ VoiceprintManagerPage
├─ RvizFullscreenPage
├─ MappingFullscreenPage
├─ GamepadTutorialPage（纯 QML 演示，不接车辆控制接口）
└─ MapSelectorPage
   └─ MapCarousel（只实例化当前及相邻的最多 3 个 PGM 预览）
```

`Main.qml` 持有页面栈和统一底栏。返回键优先关闭当前弹窗，再弹出页面；主页键清空二级页面。

## 4. 设计系统

- `Theme.qml`：亮/暗主题颜色，默认亮色。
- `AppMetrics.qml`：有限范围缩放、字体、间距、圆角和触控高度。
- `AppCard`、`PageHeader`、`SectionHeader`：页面层级。
- `PrimaryButton`、`SecondaryButton`、`DangerButton`：操作语义。
- `DataRow`、`StatusDot`、`StatusBadge`：状态数据。
- `AppSwitch`、`AppSlider`、`SegmentedControl`：输入控件。
- `AppDialog`、`ConfirmDialog`、`EmptyState`：反馈和空状态。
- `StatusBar`：系统状态和应用导航。

页面应引用 Theme/Metrics，避免新增散落硬编码颜色和绝对坐标。

### 性能策略与生命周期

- `Performance.qml` 是三档性能策略的唯一 QML 真值；页面不得自行判断设备型号或复制档位数值。
- 低性能/普通/流畅分别控制动画时长、地图 Canvas 重绘间隔、状态轮询间隔、图片 `sourceSize` 与缓存策略。
- `StackView.push(url)` 创建的二级页面在返回或回主页时销毁；页面图片在不可见时清空 `source`，地图只保留当前及相邻最多三张预览。
- Qt Virtual Keyboard 必须在窗口启动时创建并保持单一实例，但默认隐藏，只有显式键盘按钮请求时才显示。Qt 6 延迟创建或销毁重建会触发内部样式空引用；这是唯一有意常驻的重资源例外。
- 后端地图同步与机器人安全状态采集是全局服务，不能因离开页面而销毁；低性能模式只降低允许降频的 UI 快照刷新频率。

## 5. 地图文件管理

```text
外部工程写入 map/
        │ 低频后台稳定性扫描
        ▼
MapSyncManager
        │ 临时文件 + fsync + os.replace
        ▼
map_cache/
        │ 完整性/YAML/PGM 校验
        ▼
MapManager → UiBackend → MapSelectorPage
```

- `map/` 是主目录和唯一文件事实来源。
- `map_cache/` 是 UI 只读预览副本。
- QML 不扫描、重命名或删除文件。
- `MapSyncManager` 的后台线程约每 2 秒扫描一次，文件稳定后才同步。
- `MapManager` 负责事务、回滚、机器人状态保护和当前地图标记。
- PGM 由 Qt 原生图片插件直接读取，不经过 Base64、JPG 或 PNG 转换。
- 真正切图调用 `RobotApiBase.load_map(main_yaml_path)`，不会把缓存路径传给导航系统。

## 6. ROS 地图显示

`RvizPlaceholder.qml` 不是 RViz 进程，而是轻量 QML 地图组件：

- 读取 `RobotSnapshot.map_image`（OccupancyGrid 转 PNG Data URL）。
- 使用地图 `width`、`height`、`resolution`、`origin` 做像素/世界坐标转换。
- 叠加机器人 Pose、LaserScan、Path 和临时目标。
- 地图点选产生内存目标，不写点位库。
- 全屏页可通过鼠标/单指平移、滚轮/双指缩放；默认缩放为完整地图。
- “车头朝上并自动居中”使用现有 `current_pose.x/y/yaw` 变换画面，不增加 ROS 字段或控制调用。

导航接口：

```text
已保存点 → start_single_navigation(point_id)
临时点   → start_pose_navigation(x, y, yaw)
路线     → start_route_navigation(point_ids, ordered)  ← 当前 TeamRobotApi 返回 NOT_IMPLEMENTED
```

## 6.1 多点导航目标架构

多点导航采用 UI → Adapter → waypoint_manager → Nav2 的分层架构：

```text
QML (PointManagerPage)
  │ backend.startRoute / addRoutePoint / removeRoutePoint
  ▼
UiBackend
  │ MultiNavAdapter.startRoute(routeSnapshot)
  ▼
MultiNavAdapter
  ├── MockMultiNavAdapter (离线模拟)
  └── RosMultiNavAdapter (ROS 2 通信)
        │ ExecuteRoute Action Client
        ▼
      waypoint_manager 节点
        │ FollowWaypoints Action Client
        ▼
      Nav2
```

职责约定：

1. UI 是目标点、路线顺序和显示角度的唯一数据源。
2. 用户点击"开始"后，UI 一次性发送完整路线快照。
3. waypoint_manager 接收路线后调用 Nav2 FollowWaypoints。
4. waypoint_manager 负责反馈进度、暂停、恢复、取消和最终结果。
5. QML 不直接创建 ROS Node 或调用 Action/Topic/Service。
6. 单点导航和多点导航互斥，不能同时向 Nav2 发送 Goal。

详细体检报告见 [`MULTI_NAV_COMPLIANCE_AUDIT.md`](MULTI_NAV_COMPLIANCE_AUDIT.md)。

## 7. 数据模型

点位：

```json
{
  "id": "stable-id",
  "name": "会议室",
  "x": 1.2,
  "y": 3.4,
  "yaw": 0.0,
  "is_charging_point": false
}
```

声纹：

```json
{
  "id": "stable-id",
  "name": "声纹名称",
  "priority": 1
}
```

设置：

```json
{
  "volume": 68,
  "language": "zh",
  "parameters": {
    "max_speed": 0.5,
    "follow_distance": 1.0
  }
}
```

`data/` 是源码运行的种子数据。安装包通过 `ROBOT_UI_DATA_DIR` 使用用户目录，避免升级覆盖现场配置。

## 8. 接入原则

1. 保持 `RobotApiBase` 签名和 `ApiResult` 格式。
2. 未实现接口明确返回 `NOT_IMPLEMENTED` 或 `NOT_SUPPORTED`。
3. Topic 名通过 `ROBOT_UI_*_TOPIC` 环境变量覆盖，默认值保持兼容。
4. 共享快照在 `TeamRobotApi` 内加锁更新。
5. QML 仅消费快照和公开 Qt 接口。
6. 新增状态字段时同步修改：
   - `RobotSnapshot`
   - `UiBackend._poll_snapshot`
   - Mock 默认值
   - `INTERFACE_CONTRACT.md`
   - 对应合同测试

详细接口表见 [`INTERFACE_CONTRACT.md`](INTERFACE_CONTRACT.md)。
