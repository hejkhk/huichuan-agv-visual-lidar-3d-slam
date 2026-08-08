# 接口契约

本文定义项目允许外部团队实现和调用的公开边界。方法名、参数、Qt 名称和字段格式均视为稳定契约。

## 1. 返回格式

所有 `RobotApiBase` 方法返回：

```python
ApiResult[T](
    success=True | False,
    message="用户可读说明",
    data=...,
    error_code="稳定机器码",
)
```

规则：

- 成功：`ApiResult.ok(data, message)`。
- 失败：`ApiResult.fail(message, error_code)`。
- 失败不抛到 QML；`UiBackend` 会记录日志并显示通知。
- `error_code` 使用大写下划线格式，例如 `NOT_FOUND`、`ROS_UNAVAILABLE`、`NOT_IMPLEMENTED`。
- 列表成功时返回空列表，不返回 `None`。
- 单位必须固定，不随语言或页面改变。

## 2. RobotApiBase

### 状态

| 方法 | 输入 | 成功数据 | 说明 |
|---|---|---|---|
| `get_robot_snapshot()` | 无 | `RobotSnapshot` | 返回缓存，不应执行长阻塞 I/O |
| `get_current_pose()` | 无 | `{x,y,yaw}` | 米、米、弧度 |
| `get_settings()` | 无 | 设置字典 | 返回可序列化对象 |

### 导航

| 方法 | 输入 | 说明 |
|---|---|---|
| `start_single_navigation(point_id)` | 稳定点位 ID | 导航到保存点 |
| `start_pose_navigation(x,y,yaw=0)` | 米、米、弧度 | 导航到临时地图坐标 |
| `start_route_navigation(point_ids,ordered=True)` | ID 列表、顺序标记 | 通过 `NavigateThroughPoses` 按给定顺序执行 |
| `pause_navigation()` | 无 | 取消 Action 并保留目标 |
| `resume_navigation()` | 无 | 重新发送保留的单点或多点目标 |
| `cancel_navigation()` | 无 | 取消当前 Nav2 Goal |
| `start_slam_navigation()` | 无 | 启动汇川导航一键脚本 |
| `stop_slam_system()` | 无 | 停止登记的 SLAM launcher |
| `start_charging()` | 无 | 导航到唯一充电点 |
| `cancel_charging()` | 无 | 取消回充 |

### 建图

| 方法 | 输入 | 说明 |
|---|---|---|
| `start_mapping()` | 无 | 开始建图 |
| `stop_mapping()` | 无 | 停止建图 |
| `save_map(name)` | 地图名称 | 保存当前地图 |
| `load_map(yaml_path)` | 主目录 YAML 绝对路径 | 切换地图并启动一次自动重定位 |

真实设备的地图主目录为汇川工程 `Loc_MAP/`。用于重定位时必须存在同名
PGM、YAML、PBStream；缓存目录只负责 UI 预览。

### 点位

| 方法 | 数据约束 |
|---|---|
| `list_points()` | `[{id,name,x,y,yaw,is_charging_point}]` |
| `save_point(...)` | 名称非空、ID 唯一、最多一个充电点 |
| `rename_point(id,name)` | 保持 ID 不变 |
| `update_point_yaw(id,yaw)` | 更新到达后的车头朝向；弧度会归一化到 `[0, 2π)` |
| `delete_point(id)` | 删除充电点时允许回充变为不可用 |
| `set_charging_point(id)` | 清除其他点的充电标记 |
| `get_charging_point()` | 不存在时返回明确失败 |

### 视觉跟随

| 方法 | 说明 |
|---|---|
| `set_visual_follow_enabled(enabled)` | 开关模块 |
| `list_detected_actors()` | `[{id,x,y,distance}]`；x/y 为画面归一化坐标 |
| `select_follow_target(actor_id)` | 选择目标 |
| `start_following(actor_id)` | 开始跟随 |
| `stop_following()` | 停止跟随 |
| `set_parameter("follow_distance", value)` | 0.5–10.0 m |

### 语音和声纹

| 方法 | 说明 |
|---|---|
| `set_voice_control_enabled(enabled)` | 语音总开关 |
| `set_unknown_voice_control_allowed(enabled)` | 未录入声纹权限 |
| `list_voiceprints()` | 按 priority 升序，最多 10 个 |
| `begin_voiceprint_recording(name)` | 开始录入 |
| `cancel_voiceprint_recording()` | 取消录入 |
| `save_voiceprint(name)` | 保存当前录音 |
| `rename_voiceprint(id,new_name)` | 改名 |
| `delete_voiceprint(id)` | 删除后压紧 priority |
| `move_voiceprint(id,direction)` | `-1` 上移，`1` 下移 |

### 设置和系统

| 方法 | 约束 |
|---|---|
| `set_volume(value)` | 整数 0–100；默认实现控制 Ubuntu 系统输出音量 |
| `set_parameter(name,value)` | 当前公开 `max_speed`、`follow_distance` |
| `list_wifi_networks()` | `[{ssid,signal,secured,connected}]` |
| `connect_wifi(ssid,password)` | 密码不应写日志 |
| `get_ota_status()` | `{version,status,...}` |
| `start_ota_upgrade()` | 长任务必须异步 |

`rviz_zoom_in/out/reset_view/open_rviz_fullscreen` 是兼容接口；当前 QML 地图可以仅记录这些操作。

系统音量不是语音团队接口。默认实现通过 `backend/system_audio.py`
调用 PipeWire `wpctl`，缺失时回退 ALSA `amixer`；语音模块可直接读取
操作系统音量。保留 `set_volume` 名称仅为了兼容既有 QML/Qt 合同。

## 3. RobotSnapshot

### 系统和底盘

| 字段 | 类型/单位 |
|---|---|
| `timestamp` | Unix 秒 |
| `battery_percent` | 0–100 |
| `remaining_range_km` | km |
| `upload_kbps` / `download_kbps` | kb/s |
| `cpu_percent` / `memory_percent` | 0–100 |
| `cpu_temperature` | °C |
| `battery_voltage` | V |
| `vx` / `vy` | m/s |
| `wz` | rad/s |
| `ax` / `ay` / `az` | g |
| `gx` / `gy` / `gz` | °/s |

状态字符串使用 `NORMAL`、`WARNING`、`ERROR`、`UNKNOWN` 或明确业务状态。

### 地图与导航

- `current_pose`: `Pose(x,y,yaw)`，米/米/弧度。
- `pose_available`, `map_available`, `ros_connected`: bool。
- `map_image`: PNG Data URL。
- `map_width`, `map_height`: 栅格数量。
- `map_resolution`: m/cell。
- `map_origin_x`, `map_origin_y`: 米。
- `mapping_state`: `IDLE`、`MAPPING`、`STOPPED`、`COMPLETED`、`FAILED`。
- `mapping_active`: bool。
- `mapping_message`: 建图状态消息。
- `slam_running`, `slam_mode`, `slam_message`: 主工程运行状态。
- `localization_ready`, `localization_state`, `localization_detail`: 重定位结果。
- `recovery_stage`, `recovery_reason`, `recovery_count`: 三级脱困阶段、原因和次数。
- `laser_points`, `path_points`: `[[x,y], ...]`，世界坐标米。
- `navigation_state`: `IDLE`、`TARGET_SELECTED`、`STARTING`、`NAVIGATING`、`PAUSED`、`ARRIVED`、`FAILED`、`CANCELLED`。

### 跟随和语音

- `detected_actors`: Actor 列表。
- `follow_state`: `IDLE`、`SELECTED`、`FOLLOWING` 或团队扩展状态。
- `follow_target`: Actor ID。
- `voice_state`: `LISTENING`、`SPEAKING`、`READY`。
- `speaker_name`, `speaker_voiceprint`: 显示字符串。

不要重新加入 `speaker_distance`、`speaker_angle`、`voice_command_history`；当前页面和合同测试明确不依赖这些字段。

## 4. QML 可见的 UiBackend

### Signals

`dataChanged`、`snapshotChanged`、`busyChanged`、`notificationChanged`、`currentPoseReady(pose)`、`recordingStateChanged`、`languageChanged`。

### Properties

`points`、`recentPoints`、`voiceprints`、`routePoints`、`hasChargingPoint`、`snapshot`、`settings`、`selectedPointId`、`selectedPoint`、`mapGoal`、`hasMapGoal`、`navigationControls`、`busy`、`notification`、`recordingState`、`language`、`wifiNetworks`、`maps`、`mapErrors`、`currentMap`、`mapOperationState`。

### Slots

导航与地图：

```text
selectPoint(str)
selectMapGoal(float,float)
clearNavigationSelection()
startSelectedNavigation()
togglePauseNavigation()
cancelNavigation()
startMapping()
stopMapping()
saveMap(str)
refreshMaps()
useMap(str)
renameMap(str,str)
deleteMap(str)
mapActionAvailability(str,str)
showMapBlockedReason(str,str)
rvizAction(str)
requestCurrentPose()
```

地图文件数据至少包含：

```text
id name pgm_path yaml_path cache_pgm_path cache_yaml_path
is_current modified_time is_complete error_message
```

地图操作状态使用 `IDLE`、`SYNCING`、`LOADING_MAP`、`RENAMING`、
`DELETING`、`ERROR`。这是 UI 地图管理状态，不修改现有 Nav2
`navigation_state`。

点位与路线：

```text
savePoint(str,float,float,float,bool)
renamePoint(str,str)
deletePoint(str)
addRoutePoint(str)
removeRoutePoint(str)
clearRoute()
startRoute(bool)
startCharging()
```

跟随、语音、设置：

```text
setVoiceEnabled(bool)
setFollowEnabled(bool)
setUnknownVoiceAllowed(bool)
selectActor(str)
startFollowing(str)
stopFollowing()
beginVoiceprint(str)
saveVoiceprint(str)
cancelVoiceprintRecording()
renameVoiceprint(str,str)
deleteVoiceprint(str)
moveVoiceprint(str,int)
setVolume(int)
refreshSystemVolume()
setParameter(str,QVariant)
refreshWifi()
connectWifi(str,str)
startOta()
setLanguage(str)
setPerformanceMode(int)
releaseControlToGamepad()
```

修改这些名称或类型会破坏现有 QML。

`setPerformanceMode(int mode)` 接受 `0`（低性能）、`1`（普通）、`2`（流畅），越界值会被钳制并写入 `data/settings.json`。该 Slot 只调整 Qt 快照轮询间隔和 QML 性能策略，不新增或修改 Robot API、ROS、Nav2、串口、语音或底盘接口，也不得用它暂停机器人安全状态采集。

`releaseControlToGamepad()` 调用 `RobotApiBase.release_control_to_gamepad()`，
只向下位机发送一次“释放上位机控制权”命令，不订阅确认、不轮询控制权，
也不伪造下位机反馈。当前 Mock 仅记录调用；真实串口码由下位机团队在 Team
适配层补入。首页显示的“上位机接管/手柄接管”是本次运行周期内的操作提示，
不是下位机遥测状态。

`refreshSystemVolume()` 只刷新操作系统当前输出音量到 `settings`，无参数、
无返回值，不属于语音团队接口。键盘显示/隐藏完全位于 QML 和 Qt 输入法
层，不新增 Robot API、ROS 或设备接口。

## 5. ROS 2

| 环境变量 | 默认 Topic/Action | 类型 |
|---|---|---|
| `ROBOT_UI_MAP_TOPIC` | `/map` | `nav_msgs/OccupancyGrid` |
| `ROBOT_UI_AMCL_TOPIC` | `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` |
| `ROBOT_UI_LOCALIZATION_TOPIC` | `/localization_pose` | 同上 |
| `ROBOT_UI_SCAN_TOPIC` | `/scan` | `sensor_msgs/LaserScan` |
| `ROBOT_UI_PATH_TOPIC` | `/plan` | `nav_msgs/Path` |
| `ROBOT_UI_PREVIEW_PATH_TOPIC` | `/web/preview_path` | `nav_msgs/Path` |
| `ROBOT_UI_STATUS_TOPIC` | `/robot/status` | `std_msgs/String` JSON |
| `ROBOT_UI_NAVIGATION_STATUS_TOPIC` | `/web/navigation_status` | `std_msgs/String` JSON |
| `ROBOT_UI_MAPPING_STATUS_TOPIC` | `/web/mapping_status` | `std_msgs/String` JSON |
| `ROBOT_UI_NAVIGATION_ACTION` | `/navigate_to_pose` | `nav2_msgs/NavigateToPose` |

地图使用 reliable + transient local + keep last 1；传感器数据使用适合传感器的 QoS。ROS 在独立 Context/Executor 线程运行，不阻塞 Qt。

## 6. 多点导航目标接口

当前 `TeamRobotApi.start_route_navigation()` 直接使用 Nav2
`/navigate_through_poses`，并按 UI 路线列表顺序构造 `PoseStamped[]`。

### 6.1 amr_interfaces 包

**msg/RouteWaypoint.msg:**

```text
string id
string name
geometry_msgs/PoseStamped pose
```

**action/ExecuteRoute.action:**

```text
uint16 PROTOCOL_VERSION_1=1
uint8 MODE_FOLLOW_WAYPOINTS=0
uint8 MODE_NAVIGATE_THROUGH_POSES=1

uint16 protocol_version
string session_id
string route_name
string map_id
uint8 mode
amr_interfaces/RouteWaypoint[] waypoints
---
uint8 RESULT_SUCCESS=0
uint8 RESULT_CANCELLED=1
uint8 RESULT_ABORTED=2
uint8 RESULT_INVALID_ROUTE=3
uint8 RESULT_NAV2_UNAVAILABLE=4
uint8 RESULT_BUSY=5
uint8 RESULT_MAP_MISMATCH=6
uint8 RESULT_INTERNAL_ERROR=7

uint8 result_code
string message
uint32 reached_count
int32 final_index
---
uint8 STATE_STARTING=1
uint8 STATE_NAVIGATING=2
uint8 STATE_PAUSING=3
uint8 STATE_PAUSED=4
uint8 STATE_RESUMING=5
uint8 STATE_CANCELLING=6

uint8 state
int32 current_index
uint32 reached_count
uint32 total_count
float32 progress
string current_waypoint_name
string message
```

Action 名称：`/multi_nav/execute_route`

**msg/MultiNavStatus.msg:**

```text
uint8 STATE_IDLE=0
uint8 STATE_STARTING=1
uint8 STATE_NAVIGATING=2
uint8 STATE_PAUSING=3
uint8 STATE_PAUSED=4
uint8 STATE_RESUMING=5
uint8 STATE_CANCELLING=6
uint8 STATE_SUCCEEDED=7
uint8 STATE_CANCELLED=8
uint8 STATE_FAILED=9

bool available
uint8 state
string active_session_id
int32 current_index
uint32 reached_count
uint32 total_count
float32 progress
string current_waypoint_name
string message
```

Topic 名称：`/multi_nav/status`
QoS：RELIABLE + TRANSIENT_LOCAL + KEEP_LAST 1

**srv/SetPaused.srv:**

```text
string session_id
bool paused
---
bool accepted
uint8 resulting_state
string message
```

Service 名称：`/multi_nav/set_paused`

### 6.2 MultiNavAdapter 接口

QML 不关心具体实现，只绑定统一接口。

**只读属性：**

```text
bool available
int state
string activeSessionId
int currentIndex
int reachedCount
int totalCount
double progress
string currentWaypointName
string message
bool canStart / canPause / canResume / canCancel
bool routeEditingLocked
```

**方法：**

```text
startRoute(routeSnapshot)
setPaused(bool)
cancelRoute()
```

**信号：**

```text
stateChanged()
progressChanged()
operationSucceeded(string message)
operationFailed(int code, string message)
goalAccepted(string sessionId)
goalRejected(int code, string message)
```

详细体检报告见 [`MULTI_NAV_COMPLIANCE_AUDIT.md`](MULTI_NAV_COMPLIANCE_AUDIT.md)。

## 7. 扩展检查清单

新增或修改接口时：

1. 更新 `RobotApiBase` 类型签名。
2. 更新 Mock 和 Team 实现。
3. 更新 `RobotSnapshot` 与 `UiBackend._poll_snapshot`。
4. 保证所有返回值为 `ApiResult`。
5. 为错误定义稳定 `error_code`。
6. 更新本文和对应测试。
7. 运行全部 pytest 与 QML offscreen 测试。
