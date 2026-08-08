# Ubuntu 24.04 / Jetson Orin Nano 部署说明

本文面向接手工程的 Ubuntu Agent。开始前先阅读 `HANDOFF_UBUNTU_X86_CODEX.md` 和 `INTERFACE_CONTRACT.md`。工程是 Python + PySide6 + Qt Quick/QML 车载 UI，不在 QML 中直接调用 ROS。

## 1. 目标环境

- Jetson Orin Nano，`uname -m` 应输出 `aarch64`
- Ubuntu 24.04
- Python 3.12
- ROS 2 Jazzy（需要真实地图/导航时）
- 具备 X11 或 XWayland 的桌面会话和触摸屏

先记录实机信息：

```bash
cat /proc/device-tree/model 2>/dev/null; echo
uname -m
cat /etc/os-release
python3 --version
echo "${XDG_SESSION_TYPE:-unknown}"
```

如果不是 Ubuntu 24.04、Python 3.12 或 ROS 2 Jazzy，不要直接安装项目提供的 ARM64 `.deb`，先按实机版本调整依赖。

## 2. 解包与依赖

```bash
unzip 'CAR UI V4.2.zip'
cd 'CAR UI V4.2'

sudo apt update
sudo apt install -y \
  python3 python3-pip python3-venv \
  libgl1 libegl1 libdbus-1-3 libfontconfig1 \
  libxkbcommon0 libxkbcommon-x11-0 libxcb1 libxcb-cursor0 \
  libwayland-client0 wireplumber alsa-utils

python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
chmod +x run.sh start_ui_with_relay.sh
```

若 Jetson 上的 `pip install PySide6` 没有匹配的 ARM64 wheel，优先使用系统或团队已验证的 PySide6 环境；不要在不了解 JetPack 与 Python ABI 的情况下混装不同版本 Qt。

如改用 Ubuntu 系统 Qt/PySide6，还需安装虚拟键盘：

```bash
sudo apt install -y \
  qml6-module-qtquick-virtualkeyboard qt6-virtualkeyboard-plugin
```

PySide6 官方 wheel/项目 ARM64 构建脚本已包含该模块和拼音插件，不需要
另行下载第三方输入法。

本项目的输入弹窗内嵌 Qt Virtual Keyboard。Qt 6 普通 Wayland 客户端不支持
这种 client-side 输入法，因此 `run.sh` 默认设置
`QT_QPA_PLATFORM=xcb`，在 Ubuntu GNOME 下由 XWayland 承载。不要删除
`libxcb-cursor0`，也不要在普通 Wayland 会话中强制覆盖为 `wayland`。

## 3. 首次启动

`run.sh` 会优先读取 `/opt/ros/<distro>/setup.bash`；汇川集成默认使用 `ROS_DOMAIN_ID=88`。
它还会固定 `QT_IM_MODULE=qtvirtualkeyboard`，避免 Ubuntu 的 IBus 环境变量
截获“显示键盘”请求。

```bash
cd car_ui_portable
source .venv/bin/activate
./run.sh
```

调试日志：

```bash
ROBOT_UI_LOG_LEVEL=DEBUG ./run.sh
```

只检查 UI、本地声纹排序和弹窗，不连接 ROS：

```bash
ROBOT_API_MODE=mock ./run.sh
```

如桌面终端看不到窗口，先检查：

```bash
echo "$DISPLAY"
echo "$WAYLAND_DISPLAY"
```

不要在纯 SSH、没有转发显示服务的会话里判断 UI 启动失败。

触控键盘检查：

1. 打开新增或重命名目标点/声纹弹窗。
2. 点击输入框右侧“显示键盘”，确认可输入拼音并选择中文候选词。
3. 点击“隐藏键盘”，确认不会立即重新弹出。
4. 再次点击“显示键盘”，确认键盘可以恢复。

## 4. ROS 2 Jazzy 接入

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=88
./run.sh
```

地图和导航沿用现有 `robot_api/ros2_client.py` 与 `robot_api/team.py`。语音团队只需要实现 `robot_api` 边界，不要修改 QML：

导航接口：

- 地图继续订阅环境变量 `ROBOT_UI_MAP_TOPIC`，默认值必须保留为 `/map`。
- 已保存点导航：`start_single_navigation(point_id)`。
- 首页任意地图坐标导航：`start_pose_navigation(x, y, yaw=0.0)`。
- 首页临时目标不会保存到点位库，也不会进入最近 3 个导航点。
- 临时目标和已保存点目标互斥；地图复位会清除当前选择。
- `TeamRobotApi.start_pose_navigation` 当前复用 `_send_navigation_goal`，最终交给 Nav2；接手 Agent 应保持此调用边界。
- 联调时应确认地图元数据 `width`、`height`、`resolution`、`origin` 正确，否则点击坐标会整体偏移。

语音接口：

- `RobotSnapshot.voice_state`：`LISTENING`、`SPEAKING`、`READY`
- `RobotSnapshot.speaker_name`：当前说话人显示名
- `RobotSnapshot.speaker_voiceprint`：匹配到的声纹名
- `list_voiceprints()`：按 `priority` 升序返回，最多 10 条
- `set_unknown_voice_control_allowed(enabled)`
- `begin_voiceprint_recording(name)` / `cancel_voiceprint_recording()`
- `save_voiceprint(name)` / `rename_voiceprint(...)` / `delete_voiceprint(...)`
- `move_voiceprint(voiceprint_id, direction)`：`-1` 上移，`1` 下移

真实语音模块尚未接入时，`TeamRobotApi` 会继承本地 Mock 行为，页面仍可用于 UI 联调。

## 5. 检查

需要运行测试时额外安装开发依赖：

```bash
source .venv/bin/activate
python -m pip install -r requirements-dev.txt
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q
python -m compileall -q backend robot_api tests
```

地图目标专项检查：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  python -m pytest tests/test_navigation_state.py tests/test_map_goal_contract.py -q

ros2 topic info "${ROBOT_UI_MAP_TOPIC:-/map}"
ros2 topic echo "${ROBOT_UI_MAP_TOPIC:-/map}" --once
```

实机上至少验证：地图重复点击会更新红色临时目标、复位会清除目标、点击最近点会改为已保存点选择、临时目标开始导航后不会写入最近 3 个列表。

地图选择与管理目录：

```text
<工程根目录>/map
<工程根目录>/map_cache
```

外部建图或导航程序只读写 `map/`；UI 只从 `map_cache/` 读取 PGM。
每张地图必须同时存在同名 `.pgm` 和 `.yaml`。真实地图切换需在
`TeamRobotApi.load_map(yaml_path)` 中连接现场已有地图加载服务；未接入时
页面会显示明确失败，不会只改变 UI 标记。

有桌面环境后再做 QML 离屏加载检查：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 QT_QPA_PLATFORM=offscreen \
  python -m pytest tests/test_qml_load.py -q
```

## 6. ARM64 安装包

在联网的 Ubuntu 24.04 ARM64 构建环境执行：

```bash
sudo apt install -y unzip fakeroot dpkg-dev binutils
./build_arm64_deb.sh
```

产物位于：

```text
dist/robot-touch-ui_<version>_arm64.deb
```

安装：

```bash
sudo apt install ./dist/robot-touch-ui_<version>_arm64.deb
```

`dist/`、`.venv/`、`__pycache__/` 和 `.pytest_cache/` 都是可再生成内容，不应提交或塞回源码 ZIP。

`assets/vehicle.png` 是主页右上车辆状态区使用的视觉素材。源码运行、ARM64 `.deb` 和 `pyside6-deploy` 三条路径都必须保留该文件；若启动后车辆图缺失，先检查安装目录 `/opt/robot-touch-ui/app/assets/vehicle.png`，不要用 QML 占位图替代。
