# road_v5_5_640_modular_v7_unified_io

这一版主要试两个事情：

1. **统一相机入口**：`camera_input.py`
   - `CAMERA_BACKEND="ros2"`：用 Orbbec ROS2 Wrapper 发布的 `/camera/color/image_raw` 和 `/camera/depth/image_raw`。
   - `CAMERA_BACKEND="sdk"`：回退到旧的 `pyorbbecsdk Pipeline()` 直连。
   - `main.py` 现在只认 `UnifiedCameraManager`，以后不再到处改相机初始化。

2. **统一下位机串口协议**：`serial_comm.py`
   - 默认 `SERIAL_PROTOCOL="binary_aa55"`。
   - 下行帧：`AA 55 cmd spd0 spd1 spd2 spd3 checksum`。
   - 命令字：`0x01 MOVE`，`0x02 STOP`，`0x03 ESTOP`，`0x04 PS2`。
   - 这个协议和你发的激光雷达底盘测试 `uart_control.py` 保持一致。

## 先跑 ROS2 相机

```bash
source /opt/ros/humble/setup.bash
ros2 launch orbbec_camera gemini2_rgbd_1280_full_test.launch.py
```

另开终端：

```bash
source /opt/ros/humble/setup.bash
cd road_v5_5_640_modular_v7_unified_io
python3 main.py
```

## 配置开关

在 `config_switches.py` 里：

```python
CAMERA_BACKEND = "ros2"          # ros2 / sdk
SERIAL_PROTOCOL = "binary_aa55"  # binary_aa55 / text_debug
ENABLE_SERIAL_SEND = False       # 先 False，确认速度方向后再 True
```

## 注意

当前 `binary_aa55` 里的速度生成还是“旧视觉 error -> 四轮速度”的过渡方案。
后面接激光雷达导航时，建议新增 `chassis_serial_node.py`：

```text
/cmd_vel_safe -> v/w -> 四轮速度 -> AA55 -> STM32
```

也就是说，未来不要让视觉、雷达、深度相机各自直接碰串口。串口只能有一个出口。
