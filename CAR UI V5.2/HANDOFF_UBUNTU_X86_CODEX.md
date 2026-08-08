# Ubuntu x86 / Codex 接手说明

更新日期：2026-07-31。本文是下一位 AI/开发者的首要入口；旧版“主页布局损坏”问题已经修复，不要再按旧截图回退。

## 1. 当前状态

- UI 已完成统一亮/暗主题，默认亮色。
- 首页及全部二级页已统一卡片、按钮、列表、弹窗和底栏。
- 首页改为“4 种控制方式一眼可见”：左侧约 48% 为地图和车辆状态，右侧“控制模式”总卡片内是导航、语音、手柄、视觉控制四张等高入口卡。地图高度约为左列的 63%。
- 导航首页只显示最近使用的 1 个目的地，完整列表从“管理目的地”进入；“地图管理”位于地图标题栏，“创建新地图”位于地图管理页。
- 手柄控制为单向占位接口：UI 调用 `releaseControlToGamepad()` 后仅在本次运行周期显示“手柄接管”，不等待或伪造下位机反馈；真实串口码由下位机团队补入。
- 操作员界面已将 ROS、/map、RViz、Actor 和运动缩写替换为可直接理解的描述，底层接口命名不变。
- 底栏图标使用可随主题着色的 QML Canvas，不再使用固定白色 SVG。
- 视觉“全屏控制”和语音详情均采用半全屏：左侧地图和车辆状态保持，右侧按需加载完整控制；顶部和底栏均可返回，退出后 Loader 销毁页面实例。
- 跟随距离可在首页概览卡和半全屏控制区调整，范围 0.5–10.0 m、步进 0.1 m，两处调用同一现有参数接口。
- 文本输入使用 Qt Virtual Keyboard，自带简体中文拼音与英文切换。
- 每个输入框右侧都有显式“显示键盘/隐藏键盘”按钮；隐藏时先把焦点移到非输入项，避免输入法立即重开。
- Ubuntu GNOME 会话默认通过 XCB/XWayland 运行；Qt 6 普通 Wayland 客户端不能直接驱动内嵌键盘。
- 系统音量优先使用 PipeWire `wpctl`，并以 `amixer` 作为回退。
- 外观包含工业蓝、石墨青、深空紫、钛金橙，每套均有亮/暗模式。
- 主题和配色由 QML `Settings` 跨重启保存；启动页会直接使用上次亮/暗选择。
- 外观页包含低性能、普通、流畅三档；默认普通，设置持久化到 `data/settings.json`。
- 二级页面退出即由 StackView 销毁；地图和车辆图片不可见时释放 source；虚拟键盘保持单一常驻实例但默认隐藏。
- 车辆素材统一为 `assets/vehicle.png`。
- 当前产品只按 1920×1080 验收，不再以 1024×600 或 1366×768 作为布局合同。
- 外观页可切换小（80%）、标准（100%）、大（120%）三级全局字体，以及细（1 px）、中（2 px）、粗（4 px）三级全局边框。
- 全量自动化基线：`69 passed, 1 skipped`。
- 首页新增「建图模式」按钮，进入全屏建图页（复用 RvizPlaceholder），底部提供「取消建图」和「完成建图并保存」。保存弹出输入框输入地图名称。RobotApiBase 新增 `start_mapping` / `stop_mapping` / `save_map`，TeamRobotApi 暂返回 NOT_IMPLEMENTED。
- 首页新增「地图管理」，主目录 `map/` 由后台同步到 UI 专用 `map_cache/`。地图页直接读取 PGM，支持三卡片轮播、事务式重命名/删除和机器人任务状态保护。
- `RobotApiBase.load_map(yaml_path)` 是唯一新增切图边界；Mock 可用，Team 暂返回 `NOT_IMPLEMENTED`，真实团队需接入已有地图加载服务。
- `design-qa.md` 的最终状态为 `passed`。
- 启动页随机显示 1–3 秒：文思汇通 Logo、`&`、洪昕德立 Logo、“AMR 操作系统”和加载指示。亮暗素材在 `assets/branding/`。
- 首页“控制模式”标题右侧显示两家公司 Logo 和 `&`；底栏不再重复公司文字。
- 车辆状态已去掉装饰小车图，改为上位机、下位机、运行与连接三列；“我的小车”页集中展示车辆图片、Orin Nano、磁盘、系统、地图与联合制造信息。
- 开发者模式当前密码为 `123`，只在本次运行解锁；页面只读展示 `logs/ui.log` 和选定 ROS 2 公共 Topic 摘要，不提供发布或控制功能。
- 地图卡片显示源 PGM 的创建时间；Linux 无 birth time 时按已确认规则回退到 PGM 修改时间。
- `run.sh` 默认后台启动并在 UI 成功后退出终端，日志写入 `logs/ui.log`；测试或调试用 `ROBOT_UI_FOREGROUND=1`。
- 全屏地图默认完整显示，并支持点选、鼠标/单指拖动、滚轮/双指缩放，以及“车头朝上并自动居中”；回到全图会清除平移、缩放和朝向模式。
- 手柄教程整合为 7 页自动演示：开场后按“认识手柄、十字键、急停、档位切换、普通档位、建图档位、完成”依次播放；按键以原图透明高亮层闪烁，车辆动作仅由 QML 演示。动画结束 1 秒后自动翻页，用户手动翻页后本次教程停用自动翻页。建图提示可进入教程，教程返回后重新打开建图确认。
- 多点导航通信合规性体检已完成（2026-08-04），合规总分 61/100。UI 层路线编辑已就绪，但 ROS 2 通信层未实现（TeamRobotApi.start_route_navigation() 返回 NOT_IMPLEMENTED）。存在 3 个 P0 问题：路线编辑无导航锁、单点/多点无互斥、多点导航完全未实现。详细报告见 MULTI_NAV_COMPLIANCE_AUDIT.md。

## 2. 修改范围与冻结项

若任务是纯视觉调整：

- 只修改 `qml/`、`assets/` 和设计文档。
- 不改 `RobotApiBase` 方法名、参数、返回类型。
- 不改 `UiBackend` 的 Signal、Property、Slot 名称或 QML 调用方式。
- 不在 QML 中调用 ROS、文件、shell、subprocess 或系统服务。
- 不编造不存在的 ROS Topic/Service/Action。

若任务是设备接入：

- 先阅读 `INTERFACE_CONTRACT.md`。
- 优先实现 `robot_api/team.py` 或独立适配模块。
- 返回统一 `ApiResult`，不要把异常抛入 Qt 主线程。
- 状态通过 `RobotSnapshot` 缓存提供，不要从 QML 直接轮询设备。

## 3. 视觉真值

- 用户认可的风格参考：`/home/cyn/Desktop/深色参考图.png`（本机路径，交付 ZIP 外）。
- 最新 QA 截图在开发工作区 `qa/`，最小运行 ZIP 不包含这些截图。
- 唯一目标与验收分辨率：1920×1080。

设计规则：

- 地图优先，状态集中，操作层级明确。
- 不增加粗黑框、多层套卡、巨大空按钮或霓虹装饰。
- 普通操作用蓝色，正常/回充用绿色，警告用橙色，危险操作用红色，语音用紫色。
- 最小触控区域约 48×48 px。

## 4. 关键文件

```text
qml/Theme.qml                      亮暗主题令牌
qml/Performance.qml                三档动画、刷新和图片质量策略
qml/AppMetrics.qml                 尺寸、间距、字体、触控高度
qml/pages/HomePage.qml             首页布局和全部摘要卡
qml/components/RvizPlaceholder.qml 地图渲染与坐标换算
qml/components/StartupSplash.qml    1–3 秒联合品牌启动页
qml/pages/GamepadTutorialPage.qml   手柄教程页面与步骤编排
qml/components/GamepadFocusView.qml 手柄图片旋转、遮罩和重点高亮
qml/components/VehicleDemo.qml      不调用后端的车辆动作演示
qml/components/StatusBar.qml       统一底栏
qml/components/AppSlider.qml       主题滑块
qml/components/AppInputDialog.qml  不遮挡键盘的窗口内输入弹窗
qml/components/KeyboardToggleButton.qml 显式键盘开关
qml/components/VirtualKeyboard.qml 中文/英文内嵌键盘与主题绑定
qml/pages/FollowPage.qml           视觉跟随详情
qml/pages/SettingsPage.qml         设置、主题与性能档位切换
qml/pages/MappingFullscreenPage.qml 建图全屏页
qml/pages/MapSelectorPage.qml       地图选择与管理页
qml/components/MapCarousel.qml      最多三个 PGM 预览与分档轻量过渡
backend/map_sync_manager.py         主目录到缓存的后台原子同步
backend/map_manager.py              地图事务、校验和安全保护
qml/dialogs/MappingConfirmDialog.qml 建图确认弹窗
qml/dialogs/SaveMapDialog.qml      保存地图输入弹窗
backend/ui_backend.py              QML 唯一后端入口
robot_api/base.py                  稳定公开接口
robot_api/team.py                  真实 ROS/设备适配入口
robot_api/ros2_client.py           ROS 订阅与 Nav2 Action
robot_api/types.py                 ApiResult / RobotSnapshot
```

## 5. 运行

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
ROBOT_API_MODE=mock ./run.sh
```

`run.sh` 默认设置 `QT_QPA_PLATFORM=xcb` 和
`QT_IM_MODULE=qtvirtualkeyboard`。系统必须安装 `libxcb-cursor0`；不要在普通
GNOME Wayland 客户端中强制改回 `wayland`，否则键盘按钮无法唤起输入法。

使用真实 ROS/串口：

```bash
./run.sh
```

窗口模式便于截图：

```bash
ROBOT_API_MODE=mock ROBOT_UI_AUTOCLOSE_MS=0 python main.py
```

## 6. 必须保留的接口行为

- 地图 Topic 默认 `/map`。
- 地图点选调用 `UiBackend.selectMapGoal(x, y)`。
- 临时地图目标不写点位库、不写最近历史。
- 已保存点调用 `start_single_navigation(point_id)`。
- 临时坐标调用 `start_pose_navigation(x, y, yaw)`。
- 多点导航接口可以返回 `NOT_IMPLEMENTED`，不可伪造成功。
- 地图切换只调用 `load_map(map_yaml_path)`；真实接口未接入时必须保留失败结果。
- 跟随距离使用现有 `setParameter("follow_distance", value)`。
- 声纹最多 10 个，每页 5 个，优先级连续。
- 语音 UI 不依赖 `speaker_distance`、`speaker_angle` 或 `voice_command_history`。

完整字段、错误和线程规则见 `INTERFACE_CONTRACT.md`。

## 7. 验收命令

```bash
source .venv/bin/activate
python -m pip install -r requirements-dev.txt

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 QT_QPA_PLATFORM=offscreen \
  python -m pytest -q

python -m compileall -q backend robot_api tests
git diff --check
```

QML 手工检查：

1. 亮色底栏返回、主页、全屏图标可见。
2. 主题切换后所有页面文字、状态灯和弹窗可读。
3. 1920×1080 地图区域最大且可点选。
4. 三级字体和三级边框切换后首页、设置、语音及弹窗不裁切。
5. 拖动跟随距离时数值实时变化，松手后写入参数。
6. 页面返回/主页/全屏以及右上角关闭按钮正常。
7. 控制台没有非退出阶段 QML warning/error。
8. 新增/重命名目标点、声纹和 Wi-Fi 密码输入框可显式显示、隐藏并再次显示键盘。
9. `map/` 新增、更新、删除或重命名完整地图后，`map_cache/` 最终一致。
10. 地图页只加载当前、上一张和下一张 PGM；快速点击不会越界，并显示创建时间。
11. 导航、建图、回充、跟随及地图加载期间，切换/重命名/删除均被阻止。
12. 启动页在上次亮/暗主题下保持纯白/纯黑，两个 Logo 之间只显示 `&`。
13. 全屏地图拖动、滚轮/双指缩放后仍能正确点选目标；车头朝上模式持续自动居中。
14. 手柄教程离开后销毁；从建图提示进入教程时，返回应重新出现建图提示。
15. 开发者模式只读显示 UI 日志和 ROS 2 摘要，不能发布 Topic 或控制车辆。

## 8. 性能基准（2026-07-28）

同一 Ubuntu x86_64 虚拟机、1920×1080、XCB、Mock 首页空闲，使用 `pidstat` 连续采样：

| 版本/档位 | 平均 CPU | 平均 RSS |
|---|---:|---:|
| 修改前普通界面 | 1.70% | 321928 KiB |
| 新版低性能 | 1.00% | 259125 KiB |
| 新版普通 | 1.99% | 260370 KiB |
| 新版流畅 | 2.67% | 265464 KiB |

新版相对修改前稳态 RSS 分别下降约 19.5%、19.1%、17.5%。数据只用于同机同场景回归，不代表 Jetson/树莓派绝对值。完整 QML 页面遍历峰值测试中，修改前为 267436 KiB，低性能为 210940 KiB、普通为 240812 KiB、流畅为 256364 KiB，分别下降约 21.1%、10.0%、4.1%。

## 9. 交付规则

源码 ZIP 应包含源码、文档、测试和小型资源，但排除：

```text
.git/ .venv/ __pycache__/ .pytest_cache/ qa/ dist/ deployment/
*.pyc *.pyo *.o *.so *.a *.deb *.zip
```

不要复制其他机器的虚拟环境或编译库。ZIP 解压后应能通过 `requirements.txt` 新建环境并运行。
