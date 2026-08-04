# road_v5_5_640_modular

这是把 `road_v5_5_3_single_line_minmax_640x480_old_calib.py` 拆分后的模块化版本。

## 运行

```bash
cd road_v5_5_640_modular
python main.py
```

## 文件分工

- `main.py`：主循环，尽量只保留“每一帧做什么”。
- `config_switches.py`：串口、相机、显示开关、HSV/滤波/状态机参数。
- `calibration_640.py`：640x480 鸟瞰图点、扫描线、三道路 ROI 标定点。
- `camera_orbbec.py`：Gemini2 相机启动、取帧、关闭。
- `line_vision.py`：HSV、鸟瞰图、蓝色胶带巡线、error1~error5。
- `obstacle_vision.py`：深度滤波、ROI 防抖、障碍统计、ROI 合并。
- `navigation.py`：mode 决策、绕障状态机、error 偏置。
- `serial_comm.py`：打开串口、生成命令、发送给 STM32。
- `display_debug.py`：RGB 调试显示、鼠标深度、FPS、状态文字。
- `profile_tools.py`：性能 profile 打印。
- `utils.py`：通用小工具函数。

## 改配置优先顺序

1. 相机分辨率 / 串口 / 显示窗口：改 `config_switches.py`。
2. 标定点 / ROI / 扫描线：改 `calibration_640.py`。
3. 巡线算法：改 `line_vision.py`。
4. 避障算法：改 `obstacle_vision.py`。
5. 绕障状态机：改 `navigation.py`。

原来的函数注释和大部分行内注释都按原样保留，只是把不同职责的代码搬进了不同文件。

## 640×480 半自动标定工具

运行：

```bash
python semi_auto_calibrate_640.py
```

推荐流程：

1. 把车放在赛道上，相机支架固定好。
2. 按 `f` 冻结当前画面。
3. 按提示依次点击 16 个点：
   - NEAR：左外 / 中左 / 中右 / 右外
   - 45cm：左外 / 中左 / 中右 / 右外
   - 55cm：左外 / 中左 / 中右 / 右外
   - 95cm：左外 / 中左 / 中右 / 右外
4. 点错了按 `z` 撤销上一个点。
5. 全部点完后按 `s` 保存。

保存后会生成：

```text
calibration_override_640.py
calibration_override_640.json
```

`calibration_640.py` 会自动加载 `calibration_override_640.py`。如果想回到旧 640 临时标定，删除这个 override 文件即可。


## V5 avoid-logic-clean 说明

本版重点整理避障逻辑：

1. 新增 `ENABLE_LINE_FOLLOW`，可以关闭蓝线寻线，只保留 Gemini2 深度近地面避障。
2. 深度时间滤波、ROI 防抖、避障状态机固定启用，删除旧的 True/False 分支。
3. baseline 无效补偿固定启用，不再保留 `ENABLE_HOLE_RECOVERY` 和 `HOLE_RECOVERY_EXTRA_MARGIN_MM`。
4. 新避障策略：只有中间红/黄警戒区触发紧急停车；左右/三车道都有障碍不会直接停车。
5. 紧急停车等待 `BLOCKED_WAIT_BEFORE_SPIN_SEC` 秒后，如果警戒区仍有障碍，进入原地转向 `MODE_SPIN_LEFT/RIGHT`，直到警戒区清空。
