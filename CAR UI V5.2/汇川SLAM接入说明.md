# 汇川 SLAM / Nav2 接入说明

## 运行原则

- Ubuntu 22.04、ROS 2 Humble，默认 `ROS_DOMAIN_ID=88`。
- `chassis_node` 是 STM32 串口的唯一拥有者。UI 不打开、转发或抢占串口。
- UI 和 SLAM 主工程可以分别启动；两者通过 ROS 2 话题、Action 和运行状态文件自动连接。
- 主工程位置可通过 `HUICHUAN_SLAM_ROOT` 指定。未指定时，UI 会搜索同级目录、桌面和用户目录中的 `huichuan-agv-visual-lidar-3d-slam`。

## 使用方法

### 推荐：UI + 重定位 + Nav2 联合启动

在主工程根目录执行：

```bash
./START_UI_LOCALIZATION_NAVIGATION.sh
```

脚本优先恢复 UI 上次选择的地图；没有历史选择但只有一张完整地图时会自动使用该地图。
首次存在多张地图时，脚本只先打开 UI，进入“地图管理”选择地图并点击“使用地图”后，
UI 会自动启动 `START_DUAL_2D_3D_LOCALIZATION.sh <地图名>`。定位版已经包含 Nav2，
不需要再启动第二个导航脚本。

也可以明确指定地图：

```bash
./START_UI_LOCALIZATION_NAVIGATION.sh floor_1
```

联合脚本以前台方式管理 UI；关闭 UI 或在终端按 Ctrl+C 时，会安全结束当前登记的定位导航栈。
UI 内切换地图时，联合脚本不会退出，旧定位栈结束后会继续监督新地图对应的定位栈。

### 方式一：先启动主工程，再打开 UI

```bash
cd ~/huichuan-agv-visual-lidar-3d-slam
./START_DUAL_2D_3D_LOCALIZATION.sh floor_1

cd ~/CAR\ UI\ V5.2
./run.sh
```

UI 会读取 `~/.cache/huichuan_agv/` 中的运行模式、PID 和当前地图，并自动订阅现有 ROS 2 数据。

### 方式二：只打开 UI

```bash
cd ~/CAR\ UI\ V5.2
HUICHUAN_SLAM_ROOT=~/huichuan-agv-visual-lidar-3d-slam ./run.sh
```

在“车辆详细状态”页面点击“启动 SLAM 导航”。UI 启动的主工程默认不再打开第二个 RViz；如确实需要，启动 UI 前设置 `ROBOT_UI_START_RVIZ=true`。

## 已接入功能

| 功能 | UI 行为 | 主工程接口 |
|---|---|---|
| 开始建图 | 启动建图一键脚本 | `START_DUAL_2D_3D_MAPPING.sh` |
| 停止建图 | 向登记的 launcher PID 发送 SIGINT | `~/.cache/huichuan_agv/launcher.pid` |
| 保存地图 | 保存 PGM、YAML、PBStream | `/map`、`/write_state` |
| 切换地图 | 将地图准备到 `Loc_MAP` 并启动定位版 | `START_DUAL_2D_3D_LOCALIZATION.sh <地图名>` |
| 多点导航 | 按 UI 排列顺序发送目标 | `/navigate_through_poses` |
| 暂停/继续 | 暂停时取消当前 Action，但保留目标；继续时重新发送 | Nav2 Action |
| 重定位结果 | 自动显示状态和完成结果 | `/cartographer_reloc/state`、`/localization_ready` |
| 整套启停 | 启动导航脚本或停止登记进程 | 稳定运行状态目录 |
| 三级脱困 | 显示阶段、原因和次数 | `/navigation/recovery_status` |
| 归还 PS2 | 只发布 ROS JSON，由底盘节点发串口帧 | `/robot/web_control` → `chassis_node` |

## 地图要求

`Loc_MAP` 是唯一主地图目录。每张可用于重定位的地图必须同名包含：

```text
地图名.pgm
地图名.yaml
地图名.pbstream
```

UI 的地图缓存会同步 PBStream。重命名和删除地图时三个文件会一起处理。只有 PGM/YAML 的旧地图仍可预览，但启动重定位会明确提示缺少 PBStream。

## 串口所有权

UI 归还手柄时发布：

```json
{"command":"serial_command","action":"ps2","source":"car_ui"}
```

`chassis_node` 解析后发送现有 AA55 串口帧。不要再启动 `serial_relay.py`、`start_ui_with_relay.sh`，也不要给 UI 配置 STM32 串口。

## 状态说明

- `tracking`：正常路径跟踪。
- `level1 / costmap_or_path_blocked`：清理局部与全局代价地图后重规划。
- `level2 / no_progress_backing_out`：受碰撞检查保护的短距离倒车脱困。
- `level3 / dead_end_direction_search`：死路方向搜索和再次重规划。

UI 只显示 Nav2 行为树真实发布的状态，不自行推测脱困阶段。
