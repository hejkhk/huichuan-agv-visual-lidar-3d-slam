# 机器人车载触控屏

PySide6 + Qt Quick/QML 实现的 AGV/AMR 车载触控界面。当前版本按
1920×1080 固定触控屏验收；汇川集成目标为 Ubuntu 22.04、ROS 2 Humble 和 Jetson ARM64。

汇川 SLAM/Nav2 的独立启动、地图三件套、重定位、三级脱困和串口所有权说明见
[`汇川SLAM接入说明.md`](汇川SLAM接入说明.md)。

下一位开发者或 AI 开始工作前，请依次阅读：

1. [`HANDOFF_UBUNTU_X86_CODEX.md`](HANDOFF_UBUNTU_X86_CODEX.md)：当前状态、冻结项和验收命令。
2. [`ARCHITECTURE.md`](ARCHITECTURE.md)：分层、页面和数据流。
3. [`INTERFACE_CONTRACT.md`](INTERFACE_CONTRACT.md)：QML、Qt、Robot API 与 ROS 接口契约。
4. [`DEPLOY_UBUNTU_24_04_JETSON.md`](DEPLOY_UBUNTU_24_04_JETSON.md)：Ubuntu/Jetson 部署。

## 当前功能

- 首页：左侧为地图与车辆状态，右侧在“控制模式”总卡片中等高展示导航、语音、手柄、视觉控制四种方式。手柄仅提供单向控制权交接；视觉和语音详情均采用半全屏控制，左侧地图和车辆状态保持，退出后页面实例销毁。
- 启动页：按上次主题使用纯白/纯黑背景，依次显示文思汇通 Logo、`&`、洪昕德立 Logo、“AMR 操作系统”和加载动画，随机停留 1–3 秒。
- 操作员文案不显示 ROS、/map、RViz、Actor、VX/VY/WZ 等内部术语；内部接口、字段和日志仍保留原名，方便开发调试。
- 设备状态：CPU、内存、温度、传感器、电池、速度、加速度、陀螺仪。
- 视觉跟随：Actor 选择、开始/停止、0.5–10.0 m 跟随距离。
- 语音与声纹：说话人、识别阶段、未知声纹开关、声纹增删改与优先级。
- 设置：Wi-Fi、系统音量、参数、OTA/语言、四套配色、亮暗主题、三级字体、
  三级边框、三档性能模式、“我的小车”和只读开发者模式。
- 输入：Qt 官方触控键盘，支持简体中文拼音与英文切换；所有输入框均提供显式“显示键盘/隐藏键盘”按钮。
- 导航管理：点位增删改、充电点、路线列表、多点导航预留。
- 建图模式：首页一键进入建图全屏页，支持取消建图和完成建图后输入名称保存。
- 地图管理：后台同步 `map/` 到 `map_cache/`，支持 PGM 三卡片预览、安全切换、重命名和删除。
- 地图卡片显示 PGM 创建时间；Linux 无 birth time 时回退到修改时间。
- 开发者模式当前密码为 `123`，仅本次运行解锁，只读展示 UI 日志与 ROS 2 Topic 摘要。
- 全屏地图：默认完整显示地图，支持点选目标、鼠标/单指拖动、滚轮/双指缩放，以及“车头朝上并自动居中”。
- 手柄教程：7 页介绍手柄、十字键、急停、六档切换、四个普通档和两个建图档；按键原图闪烁并配合车辆动作演示，动画结束 1 秒后自动翻页，手动操作后停用本次自动翻页；全程为纯 QML 演示，不调用车辆接口。建图确认页可进入教程，返回后自动恢复建图提示。
- 统一底栏：电量、续航、蓝牙、ROS、返回、主页、全屏、时间和网络。

首次运行默认亮色；主题和配色通过 QML `Settings` 跨重启保存。语言、性能模式和机器人参数写入 `data/settings.json`。

性能模式位于“设置 → 外观设置”，默认“普通模式”：

- 低性能模式：关闭装饰动画，降低地图重绘、后端快照和时钟刷新频率，并降低非关键图片采样尺寸。
- 普通模式：保持当前认可的交互观感，在资源占用与响应速度之间平衡。
- 流畅模式：增加页面、弹窗、轮播和控件过渡，并提高地图与状态刷新频率；会占用更多算力和内存。

切换档位只改变 QML 动画、渲染质量和已有后台快照定时器的间隔，不停止安全状态、ROS、地图同步或任何机器人任务。

当前产品验收分辨率固定为 1920×1080。`AppMetrics.qml` 统一维护三级字体，
外观设置可选择小（80%）、标准（100%）和大（120%）；卡片与控件边框可选择
细（1 px）、中（2 px）和粗（4 px）。页面不得另写一套局部字号层级。

## 分层边界

```text
QML 页面与组件
        │ Qt Property / Signal / Slot
        ▼
backend.UiBackend
        │ RobotApiBase（稳定公开接口）
        ▼
MockRobotApi / TeamRobotApi
        │
        ├─ ROS 2 / Nav2
        ├─ 串口中继
        └─ 本地 JSON 演示数据
```

QML 不读写文件、不导入 ROS、不启动进程，也不直接访问设备。真实设备接入应实现 `robot_api/`，不要把通信逻辑写进页面。

## Ubuntu 24.04 快速运行

```bash
sudo apt update
sudo apt install -y \
  python3 python3-venv \
  libgl1 libegl1 libdbus-1-3 libfontconfig1 \
  libxkbcommon0 libxkbcommon-x11-0 libxcb1 libxcb-cursor0 \
  libwayland-client0 wireplumber alsa-utils

python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt

ROBOT_API_MODE=mock ./run.sh
```

`run.sh` 在 QML 启动成功后退出启动终端，UI 在后台继续运行，日志写入
`logs/ui.log`。调试或测试需要前台进程时使用：

```bash
ROBOT_UI_FOREGROUND=1 ROBOT_API_MODE=mock ./run.sh
```

真实模式下，从 UI 启动建图、导航或重定位时，完整启动器输出同时写入
`logs/slam_launcher.log`。设置页的“开发者模式”会显示两个日志的末尾，并检查实际发布者数量、
Nav2 Action、地图保存和重定位服务；圆点为 `○` 表示接口名称可能存在，但当前没有发布端或服务端。
启动失败时优先保存以下结果：

```bash
tail -n 300 logs/ui.log
tail -n 500 logs/slam_launcher.log
ros2 node list
ros2 topic list -t
```

每次由 UI 启动 SLAM，`slam_launcher.log` 都会写入时间、模式、Domain ID、RViz 开关、工作目录
和完整启动命令，便于区分“UI 没接上 ROS”和“SLAM 子进程自身启动失败”。UI Python 使用
无缓冲输出，SLAM 子进程通过 `stdbuf` 按行刷新，因此日志会先实时写入本地文件；开发者页面每
1.5 秒读取最新尾部。`ROBOT_UI_FOREGROUND=1` 时终端输出也会通过 `tee` 同步写入 `ui.log`。

真实 ROS/团队模式：

```bash
./run.sh
# 等价于默认 ROBOT_API_MODE=team
```

嵌入汇川主工程后的推荐启动方式是在主工程根目录执行：

```bash
./START_UI_LOCALIZATION_NAVIGATION.sh
```

它会同时管理 UI、所选地图的重定位和 Nav2。地图页面显示 `Loc_MAP/` 中的 PGM
预览；点击“使用地图”会切换完整 `PGM + YAML + PBStream` 地图集并自动启动重定位。
定位成功后，同一个地图区域显示 ROS `/map` 的实时规划地图、机器人和导航路径。

应用默认使用 XCB/XWayland，因为 Qt 6 不支持在普通 Wayland 客户端中直接
驱动内嵌 Virtual Keyboard。Ubuntu 24.04 必须安装 `libxcb-cursor0`。
如需显式指定：

```bash
QT_QPA_PLATFORM=xcb ./run.sh
```

仅在已经由自定义 Wayland compositor 承载输入法时才覆盖为
`QT_QPA_PLATFORM=wayland`；普通 GNOME Wayland 会话下这样运行将无法召唤
应用内键盘。

## 测试

开发环境额外安装：

```bash
python -m pip install -r requirements-dev.txt
```

运行全部测试：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 QT_QPA_PLATFORM=offscreen \
  python -m pytest -q
```

当前基线：`69 passed, 1 skipped`。

## 目录

- `main.py`：应用入口、QML 引擎和上下文注入。
- `qml/`：全部界面、主题、尺寸体系和公共组件。
- `backend/`：Qt 桥接、线程任务、页面状态和 JSON 存储。
- `map/`：外部工程和导航系统使用的主地图目录。
- `map_cache/`：UI 专用地图副本；页面只从这里读取 PGM 预览。
- `robot_api/`：公开抽象接口、Mock、ROS/团队适配和类型。
- `assets/vehicle.png`：首页车辆视觉素材。
- `assets/branding/`：启动页亮/暗 Logo。
- `assets/tutorial/`：手柄教程图片。
- `data/`：首次运行种子数据；安装包运行数据应放到外部数据目录。
- `tests/`：接口、QML、地图、导航、语音和打包契约测试。
- `packaging/`、`build_*.sh`：Debian/ARM64 构建辅助文件。

## 运行环境变量

| 变量 | 默认值 | 用途 |
|---|---|---|
| `ROBOT_API_MODE` | `team` | `team` 或 `mock` |
| `ROBOT_UI_DATA_DIR` | 项目 `data/` | 外部运行数据目录 |
| `ROBOT_UI_PROJECT_ROOT` | `main.py` 所在目录 | `map/` 与 `map_cache/` 的共同父目录；安装包启动器会设置稳定路径 |
| `ROBOT_UI_LOG_LEVEL` | `INFO` | Python/Qt 日志等级 |
| `ROBOT_UI_ROS_SETUP` | 自动探测 | ROS setup.bash 路径 |
| `ROS_DOMAIN_ID` | `88`（可由设置或环境覆盖） | 汇川 ROS Domain |
| `ROBOT_UI_MAP_TOPIC` | `/map` | OccupancyGrid Topic |
| `HUICHUAN_SLAM_ROOT` | 自动搜索 | 汇川 SLAM 主工程目录 |
| `ROBOT_UI_START_RVIZ` | `false` | UI 启动主工程时是否额外打开 RViz |
| `ROBOT_UI_AUTOCLOSE_MS` | 未设置 | 测试自动退出；设置为 `0` 时窗口模式运行 |
| `QT_QPA_PLATFORM` | `xcb` | 内嵌键盘要求 XCB/XWayland；离屏测试使用 `offscreen` |
| `QT_IM_MODULE` | `qtvirtualkeyboard` | 由应用强制设置为 Qt 内置输入法 |

完整 Topic 和方法表见 [`INTERFACE_CONTRACT.md`](INTERFACE_CONTRACT.md)。

## 已知边界

- 多点导航已直接接入 Nav2 `NavigateThroughPoses`；历史审计文档只保留作设计记录。
- 真实视觉识别、真实语音/声纹、Wi-Fi 和 OTA 仍需要团队设备接口。
- 地图组件直接绘制 OccupancyGrid、LaserScan 和 Path，不是完整 RViz。
- 没有 ROS 时 UI 仍可运行，相关状态显示“未知/未连接/警告”；UI 永不直接打开串口。
- Ubuntu 22.04 优先通过 `wpctl` 控制默认输出音量，缺失时回退 `amixer`。
- 地图切换要求 `Loc_MAP` 中存在同名 PGM、YAML 和 PBStream，切换后自动启动重定位。
- 不要把 `.venv/`、`.git/`、`qa/`、`dist/`、缓存或编译产物放入源码交付 ZIP。
