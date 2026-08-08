# 多点导航通信合规性体检报告

**项目**: CAR UI V5.0
**检查日期**: 2026-08-04
**分支**: master @ 53a5232
**检查范围**: UI 与多点导航节点之间的通信、数据、状态和安全逻辑

---

## 执行摘要

| 项目 | 结果 |
|------|------|
| 是否适合直接联调 | **不适合** |
| 合规总分 | **61 / 100** |
| P0 问题 | 3 个 |
| P1 问题 | 8 个 |
| P2 问题 | 6 个 |
| P3 问题 | 4 个 |
| 是否影响单点导航 | 本次检查未修改代码，不影响 |

### 最严重的 5 个问题

1. **P0** — `TeamRobotApi.start_route_navigation()` 返回 `NOT_IMPLEMENTED`，多点导航完全无法工作
2. **P0** — 路线编辑（加入/移除/清空）在导航期间无任何锁定
3. **P0** — 单点导航和多点导航无互斥机制，可能同时向 Nav2 发送 Goal
4. **P1** — 目标点缺少 `map_id` 字段，无法校验路线中的点是否属于当前地图
5. **P1** — 无 `session_id` 机制，旧 Action Feedback 可能污染新任务

---

## 当前架构

### 数据流

```
QML (PointManagerPage / HomePage)
  │  backend.addRoutePoint / startRoute / startSelectedNavigation
  ▼
UiBackend (_route_ids / _map_goal / selected_point_id)
  │  self.api.start_route_navigation / start_single_navigation
  ▼
RobotApiBase
  ├── MockRobotApi (本地模拟，可用)
  └── TeamRobotApi (真实设备)
        │  _send_navigation_goal(point)  ← 仅单点
        ▼
      Ros2Client → ActionClient('/navigate_to_pose', NavigateToPose) → Nav2
```

### 多点导航链路现状

```
QML → UiBackend.startRoute()
  → TeamRobotApi.start_route_navigation()
    → 返回 NOT_IMPLEMENTED ❌
```

### 目标多点导航链路

```
QML → UiBackend → MultiNavAdapter → ExecuteRoute Action Client
  → waypoint_manager 节点 → Nav2 FollowWaypoints
```

---

## 合规评分

| 维度 | 权重 | 得分 | 说明 |
|------|------|------|------|
| UI 与后端职责边界 | 12 | 10 | QML 不直接访问 ROS，但无 MultiNavAdapter 抽象 |
| 目标点和路线数据模型 | 15 | 8 | 有 id/x/y/yaw，缺 map_id，路线仅内存暂存 |
| 坐标与角度处理 | 15 | 12 | yaw 弧度完整，四元数转换正确，缺 yaw_deg 字段 |
| ROS 2 通信接口完整性 | 20 | 5 | 单点完整，多点：零通信实现 |
| 导航状态机和按钮状态 | 15 | 10 | 单点状态机完整，多点无中间态 |
| 控制模式与安全互斥 | 10 | 6 | 地图操作有门控，路线编辑无锁 |
| 线程安全和 UI 性能 | 8 | 7 | QThreadPool + RLock 正确 |
| 错误处理和测试 | 5 | 3 | 多点零测试覆盖 |

---

## 详细检查结果

### 一、目标点数据模型

| 检查项 | 状态 | 说明 |
|--------|------|------|
| 目标点有稳定 id | ✅ PASS | `uuid.uuid4().hex[:10]`，重命名后不变 |
| 有 name | ✅ PASS | |
| 有 x, y（米） | ✅ PASS | |
| 有 yaw（弧度） | ✅ PASS | 存储弧度，显示时转度 |
| 有 is_charging_point | ✅ PASS | |
| 有 created_at | ✅ PASS | ISO 格式 |
| 有 map_id | ❌ NOT_IMPLEMENTED | 无法区分目标点属于哪张地图 |
| 有 updated_at | ❌ NOT_IMPLEMENTED | 无更新时间 |
| 路线无悬空引用 | ⚠️ PARTIAL | 删除点后路线静默过滤，无提示 |
| 不同地图目标点不混用 | ❌ NOT_IMPLEMENTED | 无 map_id 校验 |
| 导航中路线不可修改 | ❌ FAIL | 路线编辑按钮始终可用，P0 风险 |
| 路线有持久化 | ❌ NOT_IMPLEMENTED | `_route_ids` 仅内存，重启丢失 |

### 二、坐标和角度

| 检查项 | 状态 | 说明 |
|--------|------|------|
| 保存角度 | ✅ PASS | `yaw: float`（弧度） |
| 无 degree/radian 混用 | ✅ PASS | 全链路弧度，QML 显示时转度 |
| 发送完整四元数 | ⚠️ PARTIAL | 只设 z/w，x/y 依赖 ROS 默认值 0 |
| 四元数归一化 | ✅ PASS | `sin²+cos²=1` |
| 有限数值校验 | ❌ NOT_IMPLEMENTED | 无 NaN/Inf 检查 |
| frame_id = map | ✅ PASS | |
| PoseStamped.stamp 填当前时间 | ✅ PASS | |
| 四元数反算显示 | ✅ PASS | `quaternion_to_yaw()` 函数存在 |
| 角度边界测试 | ⚠️ PARTIAL | 有基础测试，缺 0°/90°/180°/-90°/359° |

### 三、ROS 2 通信接口

| 接口 | 状态 | 说明 |
|------|------|------|
| RouteWaypoint.msg | ❌ NOT_IMPLEMENTED | 无 amr_interfaces 包 |
| ExecuteRoute.action | ❌ NOT_IMPLEMENTED | 无 Action Client |
| MultiNavStatus.msg | ❌ NOT_IMPLEMENTED | 无 Topic 订阅 |
| SetPaused.srv | ❌ NOT_IMPLEMENTED | 无 Service Client |
| MultiNavAdapter 抽象层 | ❌ NOT_IMPLEMENTED | UiBackend 直接调用 RobotApiBase |
| MockMultiNavAdapter | ❌ NOT_IMPLEMENTED | |
| RosMultiNavAdapter | ❌ NOT_IMPLEMENTED | |

### 四、导航状态机

**当前状态机（仅单点）：**
```
IDLE → TARGET_SELECTED → STARTING → NAVIGATING → ARRIVED/CANCELLED/FAILED → IDLE
```

**目标状态机（多点）：**
```
IDLE → STARTING → NAVIGATING → PAUSING → PAUSED → RESUMING → NAVIGATING
     → SUCCEEDED/CANCELLED/FAILED → IDLE
```

| 检查项 | 状态 | 说明 |
|--------|------|------|
| 开始后不重复发送 | ✅ PASS | 按钮状态控制 |
| Goal 被拒后恢复 | ✅ PASS | 回到 IDLE |
| 导航中冻结路线编辑 | ❌ FAIL | 无锁，P0 |
| 取消期间禁用重复取消 | ⚠️ PARTIAL | 无 CANCELLING 中间态 |
| 旧 session 反馈过滤 | ❌ NOT_IMPLEMENTED | 无 session_id |
| 开始按钮条件完整 | ⚠️ PARTIAL | 缺少接口在线和 Nav2 空闲检查 |

### 五、安全互斥

| 检查项 | 状态 | 说明 |
|--------|------|------|
| 单点和多点互斥 | ❌ FAIL | 共用 navigation_state，无互斥，P0 |
| 导航中禁止换地图 | ✅ PASS | map_manager 有安全门控 |
| 导航中禁止手柄/语音/跟随 | ❌ NOT_IMPLEMENTED | 无检查 |
| 取消后等待停车确认 | ⚠️ PARTIAL | 发送取消但 UI 不等待确认 |
| 取消超时机制 | ❌ NOT_IMPLEMENTED | |

### 六、QML 合规

| 检查项 | 状态 | 说明 |
|--------|------|------|
| QML 不直接创建 ROS Node | ✅ PASS | |
| QML 不直接调用 Action/Topic/Service | ✅ PASS | |
| QML 不通过 subprocess 控制导航 | ✅ PASS | subprocess 仅用于 WiFi 和音频 |
| ROS 回调通过 Signal 切回主线程 | ✅ PASS | RLock + QTimer 轮播 |
| 页面销毁后回调安全 | ✅ PASS | ApiWorker 生命周期管理 |

---

## 目标通信协议（待实现）

### amr_interfaces 包

**msg/RouteWaypoint.msg:**
```
string id
string name
geometry_msgs/PoseStamped pose
```

**action/ExecuteRoute.action:**
```
uint16 protocol_version
string session_id
string route_name
string map_id
uint8 mode
amr_interfaces/RouteWaypoint[] waypoints
---
uint8 result_code
string message
uint32 reached_count
int32 final_index
---
uint8 state
int32 current_index
uint32 reached_count
uint32 total_count
float32 progress
string current_waypoint_name
string message
```

**msg/MultiNavStatus.msg:**
```
bool available
uint8 state
string active_session_id
int32 current_index
uint32 reached_count
uint32 total_count
float32 progress
string message
```

**srv/SetPaused.srv:**
```
string session_id
bool paused
---
bool accepted
uint8 resulting_state
string message
```

### MultiNavAdapter 目标接口

**只读属性：** available, state, activeSessionId, currentIndex, reachedCount, totalCount, progress, canStart, canPause, canResume, canCancel, routeEditingLocked

**方法：** startRoute(routeSnapshot), setPaused(paused), cancelRoute()

**信号：** stateChanged, progressChanged, operationSucceeded, operationFailed, goalAccepted, goalRejected

---

## 修改计划

### 阶段 0：确认协议（无代码修改）
- 确认 amr_interfaces 包定义
- 确认 FollowWaypoints 可用性
- 确认 waypoint_manager 节点包名和命名空间

### 阶段 1：整理数据模型
- 目标点新增 map_id
- 统一角度约定（yaw 弧度内部，yaw_deg 显示）
- 新增 updated_at

### 阶段 2：建立 MultiNavAdapter
- 新增 `robot_api/multi_nav_adapter.py`
- 新增 `robot_api/mock_multi_nav_adapter.py`
- 新增 `robot_api/ros_multi_nav_adapter.py`
- UiBackend 新增多点导航属性和信号

### 阶段 3-6：接入 ROS 2 接口
- ExecuteRoute Action Client
- MultiNavStatus Topic 订阅
- SetPaused Service Client
- 角度和四元数完整处理

### 阶段 7：安全互斥
- 单点/多点互斥
- 路线编辑导航锁
- 控制模式互斥

### 阶段 8：测试
- 多点导航 Adapter 测试
- 路线数据模型测试
- 角度边界测试
- 互斥测试

---

## 需要确认的问题

1. 第一版只支持 FollowWaypoints，还是同时支持 NavigateThroughPoses？
2. 暂停功能第一版是否必须实现？
3. 多点路线是否至少需要 2 个点？
4. 一个点失败后停止还是跳过？
5. map_id 从哪个模块获取？
6. 多点导航节点的包名、节点名和命名空间？

---

## 检查确认

本次体检未修改任何代码文件。
