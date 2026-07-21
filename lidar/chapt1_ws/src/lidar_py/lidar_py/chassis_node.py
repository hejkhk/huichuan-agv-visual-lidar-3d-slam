#!/usr/bin/env python3
"""
底盘节点 - 从 STM32 读取编码器 + IMU 数据，发布 /odom 和 TF(odom→base_link)

二进制帧协议（与 STM32 上下位机通信一致）:

下行帧 (树莓派 → STM32, 20字节):
  AA 55 cmd spd[0..3](各4字节 LE int32) checksum
  cmd: 0x01=MOVE  0x02=STOP  0x03=ESTOP  0x04=PS2
       0x05=ECHO_ON  0x06=ECHO_OFF
  0x08=MAPPING is parsed only for legacy diagnostics; this node never sends it.

上行帧:
  编码器帧 (35字节): AA 55 pos[0..3](各4字节 LE int32) spd[0..3] cksum
  IMU 帧    (23字节): AA 56 accel[3] gyro[3] temp roll pitch yaw cksum

速度帧顺序与符号约定（与 STM32 一致）:
  RF, LF, RR, LR
  右侧 RF/RR: 负值=前进；左侧 LF/LR: 正值=前进

TF树:
  odom → base_link (由本节点发布)
  base_link → laser_frame (由 lidar_node 发布静态TF)
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String, Bool
from tf2_ros import TransformBroadcaster
import serial
import struct
import math
import time
import json
from collections import deque


# ========== 二进制帧协议定义 ==========

# 下行帧 (TX, 20字节)
TX_FRAME_LEN   = 20
TX_HEADER1     = 0xAA
TX_HEADER2     = 0x55
TX_OFFSET_CMD  = 2
TX_OFFSET_SPD  = 3
TX_OFFSET_CKSUM = 19

CTRL_CMD_MOVE  = 0x01
CTRL_CMD_STOP  = 0x02
CTRL_CMD_ESTOP = 0x03
CTRL_CMD_PS2   = 0x04
CTRL_CMD_ECHO_ON  = 0x05
CTRL_CMD_ECHO_OFF = 0x06
CTRL_CMD_NAVI     = 0x07
CTRL_CMD_MAPPING  = 0x08

CTRL_FRAME_LEN = 20
CTRL_OFFSET_CMD = 2
CTRL_OFFSET_SPD = 3
CTRL_OFFSET_CKSUM = 19

# 编码器上行帧 (RX, 35字节)
ENC_FRAME_LEN    = 35
ENC_HDR2         = 0x55
ENC_OFFSET_POS   = 2
ENC_OFFSET_SPD   = 18
ENC_OFFSET_CKSUM = 34

# IMU 上行帧 (23字节，含 yaw)
IMU_FRAME_LEN    = 23
IMU_HDR2         = 0x56
IMU_OFFSET_ACCEL = 2    # accel[3]: 3×int16
IMU_OFFSET_GYRO  = 8    # gyro[3]:  3×int16
IMU_OFFSET_TEMP  = 14   # temp:     int16
IMU_OFFSET_ROLL  = 16   # roll:     int16 (0.01°/unit)
IMU_OFFSET_PITCH = 18   # pitch:    int16 (0.01°/unit)
IMU_OFFSET_YAW   = 20   # yaw:      int16 (0.01°/unit)
IMU_OFFSET_CKSUM = 22

FRAME_HDR1 = 0xAA

# 电机符号表
MOTOR_SIGN = [-1, 1, -1, 1]

BASE_SPD   = 75_000_000
TURN_SPD   = 50_000_000


class ChassisNode(Node):
    def __init__(self):
        super().__init__('chassis_node')

        # ===== 声明参数 =====
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('pulse_per_rev', 8388608.0)
        self.declare_parameter('gear_ratio', 25.0)
        self.declare_parameter('wheel_radius', 0.0755)
        self.declare_parameter('wheel_base_h', 0.2145)
        self.declare_parameter('wheel_track_w', 0.2825)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('odom_publish_mode', 'navi')  # timer | navi
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_imu', False)          # 兼容 /imu；SLAM 使用独立的 /imu_cartographer
        self.declare_parameter('publish_cartographer_planar_imu', False)
        self.declare_parameter('use_imu_rp', True)            # 是否用 IMU roll/pitch 修正 odom 姿态
        self.declare_parameter('use_navi_odom', True)         # 使用 STM32 0x07 NAVI 帧更新 odom
        self.declare_parameter('serial_echo_on_start', False) # 启动后是否发送 0x05 开启控制帧回传
        self.declare_parameter('navi_yaw_sign', 1.0)
        self.declare_parameter('navi_vx_sign', 1.0)
        self.declare_parameter('navi_vz_sign', 1.0)
        self.declare_parameter('navi_yaw_offset_deg', 0.0)
        self.declare_parameter('navi_odom_yaw_source', 'absolute')  # gyro | absolute
        self.declare_parameter('navi_vx_scale', 1.0)
        self.declare_parameter('navi_vx_deadband_mps', 0.003)
        self.declare_parameter('navi_turn_vx_scale', 0.75)
        self.declare_parameter('navi_turn_wz_threshold_rad_s', 0.25)
        self.declare_parameter('navi_vz_deadband_deg_s', 0.15)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_safe')
        self.declare_parameter('wheel_speed_topic', '/wheel_speed_sent')
        self.declare_parameter('show_serial_window', True)
        self.declare_parameter('serial_defaults_on_start', True)
        self.declare_parameter('mapping_mode_on_start', True)
        self.declare_parameter('publish_latency_stats', True)
        self.declare_parameter('publish_stall_warn_ms', 20.0)
        self.declare_parameter('publish_latency_report_sec', 1.0)
        self.declare_parameter('serial_debug_stream_hz', 10.0)

        serial_port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value
        gear_ratio = self.get_parameter('gear_ratio').value
        pulse_per_rev = self.get_parameter('pulse_per_rev').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base_h = self.get_parameter('wheel_base_h').value
        self.wheel_track_w = self.get_parameter('wheel_track_w').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_topic = self.get_parameter('odom_topic').value
        publish_rate = self.get_parameter('publish_rate').value
        self.odom_publish_mode = str(
            self.get_parameter('odom_publish_mode').value or 'timer').lower()
        if self.odom_publish_mode not in ('timer', 'navi'):
            self.get_logger().warn(
                f"Unknown odom_publish_mode={self.odom_publish_mode}, fallback to timer")
            self.odom_publish_mode = 'timer'
        self.publish_tf = self.get_parameter('publish_tf').value
        self.publish_imu = self.get_parameter('publish_imu').value
        self.publish_cartographer_planar_imu = bool(
            self.get_parameter('publish_cartographer_planar_imu').value)
        self.use_imu_rp = self.get_parameter('use_imu_rp').value
        self.use_navi_odom = self.get_parameter('use_navi_odom').value
        self.serial_echo_on_start = self.get_parameter('serial_echo_on_start').value
        self.navi_yaw_sign = float(self.get_parameter('navi_yaw_sign').value)
        self.navi_vx_sign = float(self.get_parameter('navi_vx_sign').value)
        self.navi_vz_sign = float(self.get_parameter('navi_vz_sign').value)
        self.navi_yaw_offset_deg = float(self.get_parameter('navi_yaw_offset_deg').value)
        self.navi_odom_yaw_source = str(self.get_parameter('navi_odom_yaw_source').value or 'gyro').lower()
        if self.navi_odom_yaw_source not in ('gyro', 'absolute'):
            self.get_logger().warn(
                f"Unknown navi_odom_yaw_source={self.navi_odom_yaw_source}, fallback to gyro")
            self.navi_odom_yaw_source = 'gyro'
        self.navi_vx_scale = float(self.get_parameter('navi_vx_scale').value)
        self.navi_vx_deadband_mps = abs(float(self.get_parameter('navi_vx_deadband_mps').value))
        self.navi_turn_vx_scale = float(self.get_parameter('navi_turn_vx_scale').value)
        self.navi_turn_wz_threshold_rad_s = abs(
            float(self.get_parameter('navi_turn_wz_threshold_rad_s').value))
        self.navi_vz_deadband_rad_s = math.radians(
            abs(float(self.get_parameter('navi_vz_deadband_deg_s').value)))
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.wheel_speed_topic = self.get_parameter('wheel_speed_topic').value
        self.show_serial_window = self.get_parameter('show_serial_window').value
        self.serial_defaults_on_start = bool(
            self.get_parameter('serial_defaults_on_start').value)
        self.mapping_mode = bool(self.get_parameter('mapping_mode_on_start').value)
        self.publish_latency_stats = bool(
            self.get_parameter('publish_latency_stats').value)
        self.publish_stall_warn_ms = max(
            0.1, float(self.get_parameter('publish_stall_warn_ms').value))
        self.publish_latency_report_sec = max(
            0.2, float(self.get_parameter('publish_latency_report_sec').value))
        self.serial_debug_stream_hz = max(
            1.0, float(self.get_parameter('serial_debug_stream_hz').value))
        self.serial_debug_stream_interval_ns = int(
            1_000_000_000 / self.serial_debug_stream_hz)

        # ===== 运动学常量 =====
        self.effective_ppr = pulse_per_rev * gear_ratio
        self.pulse_to_ms_factor = (2.0 * math.pi * self.wheel_radius) / self.effective_ppr
        # Two wheels are summed on each side below, so the yaw denominator is
        # twice the physical left/right wheel-center track.
        self.track_width = 4.0 * self.wheel_track_w

        self.get_logger().info(
            f"减速比={gear_ratio}, 等效脉冲/轮子圈={self.effective_ppr:.0f}")

        # ===== 串口连接 =====
        self.ser = None
        self.serial_port = serial_port
        self.baudrate = baudrate
        self._connect_serial()

        # ===== 里程计累计位姿 =====
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0

        # 当前速度
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vz = 0.0
        self.navi_turn_sign_samples = 0
        self.navi_turn_sign_mismatches = 0
        self.navi_turn_sign_reported = False
        self.motion_serial_enabled = True
        self.control_mode = "move"
        self.echo_enabled = False
        self.software_estop = False
        self.baseline_ready = False
        self.web_profile = "mapping" if self.mapping_mode else "normal"
        self.web_gear = 1
        self.show_obstacle_fill = False
        self.show_roi_polygons = False
        self.show_rgb_debug_text = False
        self.slam_log_enabled = False
        self.slam_log_interval_sec = 3.0

        # 上一帧时间戳
        self.last_frame_time = self.get_clock().now()
        self.first_frame = True

        # ===== IMU 数据（最新值） =====
        self.imu_roll  = 0.0    # 度
        self.imu_pitch = 0.0    # 度
        self.imu_yaw   = 0.0    # 度
        self.imu_gyro_z = 0.0   # °/s
        self.imu_available = False
        self.last_navi_ros_time = None  # ROS time of last NAVI frame, for forward prediction
        self.navi_frame_count = 0
        self.latest_navi_tick_ms = 0
        self.navi_unwrapped_yaw_rad = None
        self.navi_tick_last_raw = None
        self.navi_tick_unwrapped_ms = None
        self.navi_tick_offset_ns = None
        self.navi_tick_last_sample_ms = None
        self.latest_measurement_stamp = None
        self.latest_measurement_seq = 0
        self.published_measurement_seq = -1

        self._publish_latency = {
            name: {'count': 0, 'total_ms': 0.0, 'max_ms': 0.0, 'over': 0}
            for name in ('serial_debug', 'imu', 'cartographer_imu', 'tf', 'odom')
        }
        self._publish_latency_window_ns = time.monotonic_ns()
        self._publish_stall_last_warn_ns = {name: 0 for name in self._publish_latency}

        # ===== 发布器 =====
        odom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, odom_qos)
        self.wheel_speed_pub = self.create_publisher(
            Int32MultiArray, self.wheel_speed_topic, odom_qos)
        self.serial_debug_pub = self.create_publisher(String, '/robot/serial_debug', 50)
        self.control_state_pub = self.create_publisher(String, '/robot/control_state', 10)
        self.software_estop_pub = self.create_publisher(
            Bool, '/robot/emergency_stop_state', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        if self.publish_imu:
            self.imu_pub = self.create_publisher(Imu, '/imu', odom_qos)
        if self.publish_cartographer_planar_imu:
            sensor_qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=5
            )
            self.cartographer_imu_pub = self.create_publisher(
                Imu, '/imu_cartographer', sensor_qos)

        # ===== 订阅器: fused /cmd_vel_safe =====
        cmd_vel_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist, self.cmd_vel_topic, self.cmd_vel_callback, cmd_vel_qos)
        self.web_control_sub = self.create_subscription(
            String, '/robot/web_control', self.web_control_callback, 10)
        self.baseline_sub = self.create_subscription(
            Bool, '/depth/baseline_ready', self._on_baseline_ready, 10)
        self.software_estop_sub = self.create_subscription(
            Bool, '/robot/emergency_stop', self._on_software_estop, 10)

        # ===== 定时器 =====
        self.create_timer(0.005, self.read_serial)
        if self.odom_publish_mode == 'timer' or not self.use_navi_odom:
            self.create_timer(1.0 / publish_rate, self.publish_odometry)
        self.create_timer(0.5, self._publish_control_state)
        if self.publish_latency_stats:
            self.create_timer(
                self.publish_latency_report_sec, self._report_publish_latency)

        # ===== 统计 =====
        self.frame_count = 0
        self.imu_frame_count = 0
        self.error_count = 0
        self.rx_lines = deque(maxlen=24)
        self.rx_line_count = 0
        self.last_tx_line = "TX_FRAME waiting for /cmd_vel_safe"
        self.last_tx_payload = None
        self.serial_debug_seq = 0
        self._serial_debug_last_stream_ns = {'navi': 0, 'tx_motion': 0}
        if self.show_serial_window:
            self.rx_lines.append("Waiting for STM32 AA55/AA56 serial frames...")

        self.get_logger().info(
            f"底盘节点启动: 串口={serial_port}, 波特率={baudrate}\n"
            f"  脉冲/圈(电机)={pulse_per_rev}, 减速比={gear_ratio}, 等效脉冲/圈(轮子)={self.effective_ppr:.0f}\n"
            f"  轮半径={self.wheel_radius}m\n"
            f"  half_wheelbase={self.wheel_base_h}m, half_track={self.wheel_track_w}m, "
            f"wheel_track={2.0 * self.wheel_track_w:.3f}m\n"
            f"  发布频率={publish_rate}Hz\n"
            f"  速度输入={self.cmd_vel_topic}, 左右轮输出={self.wheel_speed_topic}\n"
            f"  NAVI odom yaw source={self.navi_odom_yaw_source}, "
            f"publish_mode={self.odom_publish_mode}, "
            f"vx_scale={self.navi_vx_scale:.3f}, turn_vx_scale={self.navi_turn_vx_scale:.3f}, "
            f"vz_deadband={math.degrees(self.navi_vz_deadband_rad_s):.2f}deg/s\n"
            f"  协议: NAVI帧(20B AA55 cmd=0x07) + ECHO帧(20B AA55) + 编码器帧(35B AA55) + IMU帧(23B AA56)"
        )

        if self.show_serial_window:
            self.create_timer(0.05, self.update_serial_window)

        self.startup_default_queue = []
        if self.serial_defaults_on_start:
            self.startup_default_queue = [
                (CTRL_CMD_ECHO_OFF, [0, 0, 0, 0], "STARTUP_ECHO_OFF"),
                (CTRL_CMD_MOVE, [0, 0, 0, 0], "STARTUP_MOVE_TAKEOVER"),
            ]
        self.startup_defaults_timer = self.create_timer(0.20, self._startup_defaults_tick)

    def _timed_publish(self, channel, publisher, message):
        if not self.publish_latency_stats:
            publisher.publish(message)
            return

        start_ns = time.monotonic_ns()
        try:
            publisher.publish(message)
        finally:
            self._record_publish_latency(channel, start_ns)

    def _timed_send_transform(self, transform):
        if not self.publish_latency_stats:
            self.tf_broadcaster.sendTransform(transform)
            return

        start_ns = time.monotonic_ns()
        try:
            self.tf_broadcaster.sendTransform(transform)
        finally:
            self._record_publish_latency('tf', start_ns)

    def _record_publish_latency(self, channel, start_ns):
        elapsed_ms = (time.monotonic_ns() - start_ns) / 1_000_000.0
        stats = self._publish_latency[channel]
        stats['count'] += 1
        stats['total_ms'] += elapsed_ms
        stats['max_ms'] = max(stats['max_ms'], elapsed_ms)
        if elapsed_ms < self.publish_stall_warn_ms:
            return

        stats['over'] += 1
        now_ns = time.monotonic_ns()
        if now_ns - self._publish_stall_last_warn_ns[channel] >= 1_000_000_000:
            self._publish_stall_last_warn_ns[channel] = now_ns
            ros_sec = self.get_clock().now().nanoseconds / 1_000_000_000.0
            self.get_logger().warn(
                f"PUBLISH_STALL channel={channel} elapsed_ms={elapsed_ms:.3f} "
                f"threshold_ms={self.publish_stall_warn_ms:.1f} "
                f"navi_count={self.navi_frame_count} mcu_tick_ms={self.latest_navi_tick_ms} "
                f"ros_sec={ros_sec:.6f}")

    def _report_publish_latency(self):
        now_ns = time.monotonic_ns()
        window_sec = (now_ns - self._publish_latency_window_ns) / 1_000_000_000.0
        parts = []
        for channel, stats in self._publish_latency.items():
            count = stats['count']
            avg_ms = stats['total_ms'] / count if count else 0.0
            parts.append(
                f"{channel}[n={count},avg={avg_ms:.3f},max={stats['max_ms']:.3f},"
                f"over={stats['over']}]")
            stats.update(count=0, total_ms=0.0, max_ms=0.0, over=0)
        self._publish_latency_window_ns = now_ns
        ros_sec = self.get_clock().now().nanoseconds / 1_000_000_000.0
        self.get_logger().info(
            f"PUBLISH_LATENCY window_sec={window_sec:.3f} ros_sec={ros_sec:.6f} "
            + ' '.join(parts))

    def _connect_serial(self):
        for attempt in range(10):
            try:
                self.ser = serial.Serial(
                    self.serial_port, self.baudrate, timeout=0.05)
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                self.get_logger().info(f"串口 {self.serial_port} 打开成功")
                if getattr(self, 'serial_echo_on_start', False):
                    frame = self._make_ctrl_frame(CTRL_CMD_ECHO_ON, [0, 0, 0, 0])
                    self.ser.write(frame)
                    self.last_tx_line = self._format_tx_frame(frame, [0, 0, 0, 0])
                return
            except serial.SerialException as e:
                self.get_logger().warn(
                    f"串口打开失败 ({attempt+1}/10): {e}")
                time.sleep(1.0)
        self.get_logger().error(
            f"无法打开串口 {self.serial_port}，节点将以无数据模式运行")
        self.ser = serial.Serial.__new__(serial.Serial)
        self.ser.is_open = False

    # ==================== 上行帧解析 ====================

    def read_serial(self):
        if not hasattr(self, 'ser') or not self.ser.is_open:
            return

        in_waiting = self.ser.in_waiting
        if in_waiting <= 0:
            return

        data = self.ser.read(in_waiting)

        if not hasattr(self, '_buffer'):
            self._buffer = bytearray()
        self._buffer.extend(data)

        self._parse_buffer()

    def _parse_buffer(self):
        """
        从缓冲区中解析上行帧：
          AA 55 = 编码器帧 (35字节)
          AA 56 = IMU 帧   (23字节)
        """
        while True:
            if len(self._buffer) < 2:
                return

            if self._buffer[0] != FRAME_HDR1:
                del self._buffer[:1]
                continue

            hdr2 = self._buffer[1]

            if hdr2 == ENC_HDR2:
                if len(self._buffer) < CTRL_FRAME_LEN:
                    return

                frame20 = bytes(self._buffer[:CTRL_FRAME_LEN])
                checksum20 = sum(frame20[:CTRL_OFFSET_CKSUM]) & 0xFF
                cmd = frame20[CTRL_OFFSET_CMD]
                known_cmds = (
                    CTRL_CMD_MOVE,
                    CTRL_CMD_STOP,
                    CTRL_CMD_ESTOP,
                    CTRL_CMD_PS2,
                    CTRL_CMD_ECHO_ON,
                    CTRL_CMD_ECHO_OFF,
                    CTRL_CMD_NAVI,
                    CTRL_CMD_MAPPING,
                )
                if checksum20 == frame20[CTRL_OFFSET_CKSUM] and cmd in known_cmds:
                    self._buffer = self._buffer[CTRL_FRAME_LEN:]
                    if cmd == CTRL_CMD_NAVI:
                        line = self._format_navi_frame(frame20, True, checksum20)
                        self._append_serial_line(line)
                        self._publish_navi_debug(frame20, True, checksum20, line)
                        self._process_navi_frame(frame20)
                    else:
                        line = self._format_echo_frame(frame20, True, checksum20)
                        self._append_serial_line(line)
                        self._publish_echo_debug(frame20, True, checksum20, line)
                        self._apply_echo_state(frame20)
                    continue

                if len(self._buffer) < ENC_FRAME_LEN:
                    return

                frame = bytes(self._buffer[:ENC_FRAME_LEN])
                checksum = sum(frame[:ENC_OFFSET_CKSUM]) & 0xFF
                if checksum == frame[ENC_OFFSET_CKSUM]:
                    self._buffer = self._buffer[ENC_FRAME_LEN:]
                    self._append_serial_line(self._format_rx_frame(frame, "ENC_AA55_35B", True, checksum))
                    if not self.use_navi_odom:
                        self._process_encoder_frame(frame)
                else:
                    bad_frame = bytes(self._buffer[:CTRL_FRAME_LEN])
                    line = self._format_rx_frame(bad_frame, "AA55_20B", False, checksum20)
                    self._append_serial_line(line)
                    self._publish_rx_debug(bad_frame, "AA55_20B", False, checksum20, line)
                    del self._buffer[:1]

            elif hdr2 == IMU_HDR2:
                # ── IMU 帧 ──
                if len(self._buffer) < IMU_FRAME_LEN:
                    return
                frame = bytes(self._buffer[:IMU_FRAME_LEN])
                checksum = sum(frame[:IMU_OFFSET_CKSUM]) & 0xFF
                if checksum == frame[IMU_OFFSET_CKSUM]:
                    self._buffer = self._buffer[IMU_FRAME_LEN:]
                    line = self._format_rx_frame(frame, "IMU_AA56_23B", True, checksum)
                    self._append_serial_line(line)
                    self._publish_rx_debug(frame, "IMU_AA56_23B", True, checksum, line)
                    self._process_imu_frame(frame)
                else:
                    bad_frame = bytes(self._buffer[:IMU_FRAME_LEN])
                    line = self._format_rx_frame(bad_frame, "IMU_AA56_23B", False, checksum)
                    self._append_serial_line(line)
                    self._publish_rx_debug(bad_frame, "IMU_AA56_23B", False, checksum, line)
                    del self._buffer[:1]

            else:
                del self._buffer[:1]

    def _process_encoder_frame(self, frame):
        """处理编码器帧，更新里程计。"""
        pos = list(struct.unpack_from('<4i', frame, ENC_OFFSET_POS))
        spd = list(struct.unpack_from('<4i', frame, ENC_OFFSET_SPD))

        self.frame_count += 1

        now = self.get_clock().now()
        if self.first_frame:
            self.first_frame = False
            self.last_pos = pos
            self.last_frame_time = now
            return

        dt = (now - self.last_frame_time).nanoseconds / 1e9
        if dt <= 0 or dt > 1.0:
            self.last_pos = pos
            self.last_frame_time = now
            return

        self.last_frame_time = now

        # 位置差 (脉冲) → 位移 (米)
        dp = [pos[i] - self.last_pos[i] for i in range(4)]
        self.last_pos = pos

        factor = (math.pi * 2.0 * self.wheel_radius) / self.effective_ppr
        d_m = [dp[i] * factor * MOTOR_SIGN[i] for i in range(4)]

        # 正解算
        dx = (d_m[0] + d_m[1] + d_m[2] + d_m[3]) / 4.0
        # ROS yaw is CCW-positive: a left turn has the right wheels travelling
        # farther than the left wheels.
        dtheta = (d_m[0] + d_m[2] - d_m[1] - d_m[3]) / self.track_width

        if dt > 0:
            self.current_vx = dx / dt
            self.current_vz = dtheta / dt
        self.current_vy = 0.0

        # 里程计积分 (圆弧模型)
        delta_x = dx * math.cos(self.odom_theta + dtheta / 2.0)
        delta_y = dx * math.sin(self.odom_theta + dtheta / 2.0)

        self.odom_x += delta_x
        self.odom_y += delta_y
        self.odom_theta += dtheta
        self.odom_theta = math.atan2(
            math.sin(self.odom_theta), math.cos(self.odom_theta))

        # 调试日志
        if self.frame_count <= 50 or self.frame_count % 100 == 0:
            spd_ms = [s * self.pulse_to_ms_factor for s in spd]
            self.get_logger().info(
                f"[帧{self.frame_count}] "
                f"Vx={self.current_vx:+.4f}m/s  Vz={self.current_vz:+.4f}rad/s  "
                f"pose=({self.odom_x:.3f},{self.odom_y:.3f},{self.odom_theta:.3f})  "
                f"dt={dt*1000:.1f}ms  "
                f"轮速: {spd_ms[0]:+.3f} {spd_ms[1]:+.3f} {spd_ms[2]:+.3f} {spd_ms[3]:+.3f} m/s"
            )

    @staticmethod
    def _wrap_pi(angle_rad: float) -> float:
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    def _unwrap_navi_yaw(self, yaw_rad: float) -> float:
        yaw_rad = self._wrap_pi(yaw_rad)
        if self.navi_unwrapped_yaw_rad is None:
            self.navi_unwrapped_yaw_rad = yaw_rad
            return self.navi_unwrapped_yaw_rad

        prev_wrapped = self._wrap_pi(self.navi_unwrapped_yaw_rad)
        delta = self._wrap_pi(yaw_rad - prev_wrapped)
        self.navi_unwrapped_yaw_rad += delta
        return self.navi_unwrapped_yaw_rad

    def _navi_sample_time(self, tick_raw: int, receipt_time):
        """Map STM32 HAL_GetTick milliseconds into the ROS clock domain."""
        tick_raw &= 0xFFFFFFFF
        if tick_raw == 0:
            return receipt_time, None, False

        reset = self.navi_tick_last_raw is None
        if not reset:
            delta_ms = (tick_raw - self.navi_tick_last_raw) & 0xFFFFFFFF
            # Normal wrap produces a small positive delta. A huge jump means
            # the STM32 rebooted or an out-of-order/corrupt sample arrived.
            if delta_ms == 0 or delta_ms > 5000:
                reset = True

        if reset:
            self.navi_tick_unwrapped_ms = int(tick_raw)
            self.navi_tick_offset_ns = None
            dt = None
        else:
            self.navi_tick_unwrapped_ms += int(delta_ms)
            dt = delta_ms * 0.001

        self.navi_tick_last_raw = tick_raw
        self.navi_tick_last_sample_ms = self.navi_tick_unwrapped_ms

        # A 20-byte 8N1 frame occupies about 1.74 ms at 115200 baud. Removing
        # that fixed wire time aligns the MCU sampling instant more closely
        # with host-stamped LiDAR data. Keep the minimum observed offset so OS
        # scheduling delays cannot move measurements forward and backward.
        wire_time_ns = int((20.0 * 10.0 / self.baudrate) * 1e9)
        candidate_offset = (
            receipt_time.nanoseconds
            - self.navi_tick_unwrapped_ms * 1_000_000
            - wire_time_ns
        )
        if self.navi_tick_offset_ns is None:
            self.navi_tick_offset_ns = candidate_offset
        elif candidate_offset < self.navi_tick_offset_ns:
            self.navi_tick_offset_ns = candidate_offset

        mapped_ns = (
            self.navi_tick_offset_ns
            + self.navi_tick_unwrapped_ms * 1_000_000
        )
        if (self.latest_measurement_stamp is not None
                and mapped_ns <= self.latest_measurement_stamp.nanoseconds):
            mapped_ns = self.latest_measurement_stamp.nanoseconds + 1
        return Time(nanoseconds=int(mapped_ns), clock_type=receipt_time.clock_type), dt, True

    def _process_navi_frame(self, frame):
        """处理 STM32 新协议 NAVI 帧：spd0=yaw*100, spd1=vx*1000, spd2=vz_deg_s*100。"""
        yaw_raw, vx_raw, vz_raw = struct.unpack_from('<3i', frame, CTRL_OFFSET_SPD)
        tick_raw = struct.unpack_from('<I', frame, CTRL_OFFSET_SPD + 12)[0]
        yaw_deg = self.navi_yaw_sign * (yaw_raw / 100.0) + self.navi_yaw_offset_deg
        raw_vx = self.navi_vx_sign * (vx_raw / 1000.0)
        vz_deg_s = self.navi_vz_sign * (vz_raw / 100.0)
        vz_rad = math.radians(vz_deg_s)
        vx = 0.0 if abs(raw_vx) < self.navi_vx_deadband_mps else raw_vx * self.navi_vx_scale
        if abs(vz_rad) >= self.navi_turn_wz_threshold_rad_s:
            vx *= self.navi_turn_vx_scale
        yaw_rad_measured = self._unwrap_navi_yaw(math.radians(yaw_deg))

        self.navi_frame_count += 1
        receipt_time = self.get_clock().now()
        self.last_navi_ros_time = receipt_time
        sample_time, tick_dt, has_mcu_tick = self._navi_sample_time(tick_raw, receipt_time)
        self.latest_measurement_stamp = sample_time
        self.latest_measurement_seq += 1

        if self.first_frame:
            self.first_frame = False
            self.last_frame_time = sample_time
            if self.navi_odom_yaw_source == 'absolute':
                self.odom_theta = self._wrap_pi(yaw_rad_measured)
            self.current_vx = vx
            self.current_vy = 0.0
            self.current_vz = vz_rad
            self.imu_yaw = yaw_deg
            self.imu_gyro_z = vz_deg_s
            self.imu_available = True
            if self.publish_imu or self.publish_cartographer_planar_imu:
                self._publish_navi_imu(yaw_rad_measured, vz_rad, sample_time)
            self._publish_navi_odometry(sample_time)
            return

        dt = tick_dt if has_mcu_tick and tick_dt is not None else (
            (sample_time - self.last_frame_time).nanoseconds / 1e9)
        self.last_frame_time = sample_time
        if dt <= 0.0 or dt > 0.5:
            if self.navi_odom_yaw_source == 'absolute':
                self.odom_theta = self._wrap_pi(yaw_rad_measured)
            self.current_vx = vx
            self.current_vz = vz_rad
            if self.publish_imu or self.publish_cartographer_planar_imu:
                self._publish_navi_imu(yaw_rad_measured, vz_rad, sample_time)
            self._publish_navi_odometry(sample_time)
            return

        old_theta = self.odom_theta
        if self.navi_odom_yaw_source == 'absolute':
            self.odom_theta = self._wrap_pi(yaw_rad_measured)
            dtheta = self._wrap_pi(self.odom_theta - old_theta)
        else:
            used_vz = 0.0 if abs(vz_rad) < self.navi_vz_deadband_rad_s else vz_rad
            dtheta = used_vz * dt
            self.odom_theta = self._wrap_pi(old_theta + dtheta)

        # In absolute-yaw mode the measured yaw derivative and gyro z must
        # have the same ROS sign. A persistent disagreement makes Cartographer
        # rotate its IMU extrapolation opposite to the odometry pose.
        if self.navi_odom_yaw_source == 'absolute':
            measured_wz = dtheta / dt
            if abs(measured_wz) >= 0.03 and abs(vz_rad) >= 0.03:
                self.navi_turn_sign_samples += 1
                if measured_wz * vz_rad < 0.0:
                    self.navi_turn_sign_mismatches += 1
                if (not self.navi_turn_sign_reported
                        and self.navi_turn_sign_samples >= 25):
                    mismatch_ratio = (
                        self.navi_turn_sign_mismatches
                        / self.navi_turn_sign_samples
                    )
                    if mismatch_ratio >= 0.8:
                        self.get_logger().error(
                            "NAVI yaw/vz sign mismatch: "
                            f"{self.navi_turn_sign_mismatches}/"
                            f"{self.navi_turn_sign_samples} turn samples disagree; "
                            f"check navi_yaw_sign={self.navi_yaw_sign:+.1f} and "
                            f"navi_vz_sign={self.navi_vz_sign:+.1f}")
                        self.navi_turn_sign_reported = True
        mid_theta = old_theta + dtheta * 0.5

        ds = vx * dt
        self.odom_x += ds * math.cos(mid_theta)
        self.odom_y += ds * math.sin(mid_theta)
        self.current_vx = vx
        self.current_vy = 0.0
        self.current_vz = vz_rad
        self.imu_yaw = yaw_deg
        self.imu_gyro_z = vz_deg_s
        self.imu_available = True

        if self.publish_imu or self.publish_cartographer_planar_imu:
            self._publish_navi_imu(yaw_rad_measured, vz_rad, sample_time)
        self._publish_navi_odometry(sample_time)

        if self.navi_frame_count <= 20 or self.navi_frame_count % 100 == 0:
            self.get_logger().info(
                f"[NAVI {self.navi_frame_count}] yaw={yaw_deg:+.2f}deg "
                f"vx={vx:+.3f}m/s raw_vx={raw_vx:+.3f}m/s vz={vz_rad:+.3f}rad/s "
                f"tick_ms={tick_raw} time={'mcu' if has_mcu_tick else 'host'} "
                f"odom_yaw_source={self.navi_odom_yaw_source} "
                f"pose=({self.odom_x:.3f},{self.odom_y:.3f},{self.odom_theta:.3f})"
            )

    def _process_imu_frame(self, frame):
        """处理 IMU 帧，存储 IMU 数据。"""
        accel = list(struct.unpack_from('<3h', frame, IMU_OFFSET_ACCEL))
        gyro  = list(struct.unpack_from('<3h', frame, IMU_OFFSET_GYRO))
        temp  = struct.unpack_from('<h', frame, IMU_OFFSET_TEMP)[0]
        roll  = struct.unpack_from('<h', frame, IMU_OFFSET_ROLL)[0] / 100.0
        pitch = struct.unpack_from('<h', frame, IMU_OFFSET_PITCH)[0] / 100.0
        yaw   = struct.unpack_from('<h', frame, IMU_OFFSET_YAW)[0] / 100.0

        self.imu_roll  = roll
        self.imu_pitch = pitch
        self.imu_yaw   = yaw
        self.imu_gyro_z = gyro[2] / 65.5  # ±500°/s → 65.5 LSB/(°/s)
        self.imu_available = True
        self.imu_frame_count += 1

        # 发布 /imu 消息
        if self.publish_imu:
            self._publish_imu(accel, gyro, temp, roll, pitch)

    def _publish_navi_imu(self, yaw_rad, wz_rad_s, sample_time):
        msg = Imu()
        msg.header.stamp = sample_time.to_msg()
        msg.header.frame_id = self.base_frame
        msg.angular_velocity.z = wz_rad_s

        cy = math.cos(yaw_rad * 0.5)
        sy = math.sin(yaw_rad * 0.5)
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = sy
        msg.orientation.w = cy

        for i in range(9):
            msg.linear_acceleration_covariance[i] = 0.0
            msg.angular_velocity_covariance[i] = 0.0
            msg.orientation_covariance[i] = 0.0
        msg.angular_velocity_covariance[8] = 0.05
        msg.orientation_covariance[8] = 0.05
        if self.publish_imu:
            self._timed_publish('imu', self.imu_pub, msg)

        if self.publish_cartographer_planar_imu:
            planar = Imu()
            planar.header = msg.header
            planar.angular_velocity.z = wz_rad_s
            # Cartographer consumes angular velocity and linear acceleration,
            # not the orientation quaternion. The chassis is constrained to a
            # level 2D plane, so publish a stable gravity observation together
            # with the real NAVI gyro z rate.
            planar.linear_acceleration.z = 9.80665
            planar.orientation_covariance[0] = -1.0
            planar.angular_velocity_covariance[0] = 1e3
            planar.angular_velocity_covariance[4] = 1e3
            planar.angular_velocity_covariance[8] = 0.05
            planar.linear_acceleration_covariance[0] = 1e3
            planar.linear_acceleration_covariance[4] = 1e3
            planar.linear_acceleration_covariance[8] = 0.05
            self._timed_publish(
                'cartographer_imu', self.cartographer_imu_pub, planar)

    def _publish_imu(self, accel_raw, gyro_raw, temp_raw, roll_deg, pitch_deg):
        """发布 sensor_msgs/Imu 消息。"""
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame

        # 加速度 (m/s²): raw / 8192 LSB/g * 9.80665
        msg.linear_acceleration.x = accel_raw[0] / 8192.0 * 9.80665
        msg.linear_acceleration.y = accel_raw[1] / 8192.0 * 9.80665
        msg.linear_acceleration.z = accel_raw[2] / 8192.0 * 9.80665

        # 角速度 (rad/s): raw / 65.5 LSB/(°/s) * π/180
        msg.angular_velocity.x = gyro_raw[0] / 65.5 * math.pi / 180.0
        msg.angular_velocity.y = gyro_raw[1] / 65.5 * math.pi / 180.0
        msg.angular_velocity.z = gyro_raw[2] / 65.5 * math.pi / 180.0

        # 姿态 (四元数): roll/pitch 来自加速度计，yaw 来自里程计
        # 如果启用 IMU 融合，用 IMU roll/pitch + odometry yaw
        if self.use_imu_rp and self.imu_available:
            roll_r  = math.radians(roll_deg)
            pitch_r = math.radians(pitch_deg)
            yaw_r   = self.odom_theta

            # 欧拉角 → 四元数 (ZYX)
            cr = math.cos(roll_r * 0.5)
            cp = math.cos(pitch_r * 0.5)
            cy = math.cos(yaw_r * 0.5)
            sr = math.sin(roll_r * 0.5)
            sp = math.sin(pitch_r * 0.5)
            sy = math.sin(yaw_r * 0.5)

            msg.orientation.x = sr * cp * cy - cr * sp * sy
            msg.orientation.y = cr * sp * cy + sr * cp * sy
            msg.orientation.z = cr * cp * sy - sr * sp * cy
            msg.orientation.w = cr * cp * cy + sr * sp * sy
        else:
            msg.orientation.x = 0.0
            msg.orientation.y = 0.0
            msg.orientation.z = 0.0
            msg.orientation.w = 1.0

        # 协方差 (使用默认值)
        for i in range(9):
            msg.linear_acceleration_covariance[i] = 0.0
            msg.angular_velocity_covariance[i] = 0.0
            msg.orientation_covariance[i] = 0.0
        msg.linear_acceleration_covariance[0] = 0.01
        msg.linear_acceleration_covariance[4] = 0.01
        msg.linear_acceleration_covariance[8] = 0.01
        msg.angular_velocity_covariance[0] = 0.01
        msg.angular_velocity_covariance[4] = 0.01
        msg.angular_velocity_covariance[8] = 0.01
        msg.orientation_covariance[0] = 0.01
        msg.orientation_covariance[4] = 0.01
        msg.orientation_covariance[8] = 0.01

        self._timed_publish('imu', self.imu_pub, msg)

    # ==================== 下行帧打包 ====================

    def _make_ctrl_frame(self, cmd: int, spd: list) -> bytes:
        body = struct.pack('<BBB4i', TX_HEADER1, TX_HEADER2, cmd, *spd)
        checksum = sum(body) & 0xFF
        return body + bytes([checksum])

    @staticmethod
    def _hex(data: bytes) -> str:
        return " ".join(f"{b:02X}" for b in data)

    def _append_serial_line(self, line: str):
        if not self.show_serial_window:
            return
        self.rx_lines.append(line)

    def _cmd_name(self, cmd: int) -> str:
        return {
            CTRL_CMD_MOVE: "MOVE",
            CTRL_CMD_STOP: "STOP",
            CTRL_CMD_ESTOP: "ESTOP",
            CTRL_CMD_PS2: "PS2",
            CTRL_CMD_ECHO_ON: "ECHO_ON",
            CTRL_CMD_ECHO_OFF: "ECHO_OFF",
            CTRL_CMD_NAVI: "NAVI",
            CTRL_CMD_MAPPING: "MAPPING",
        }.get(cmd, f"CMD_0x{cmd:02X}")

    def _publish_serial_debug(self, payload: dict):
        self.serial_debug_seq += 1
        payload.setdefault("seq", self.serial_debug_seq)
        payload.setdefault("stamp_sec", self.get_clock().now().nanoseconds / 1e9)
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._timed_publish('serial_debug', self.serial_debug_pub, msg)

    def _serial_debug_stream_due(self, stream: str) -> bool:
        """Rate-limit high-rate telemetry without delaying the control path."""
        now_ns = time.monotonic_ns()
        last_ns = self._serial_debug_last_stream_ns.get(stream, 0)
        if now_ns - last_ns < self.serial_debug_stream_interval_ns:
            return False
        self._serial_debug_last_stream_ns[stream] = now_ns
        return True

    def _frame_payload(self, frame: bytes, ok: bool = True, calc: int = None) -> dict:
        recv = frame[-1] if frame else 0
        if calc is None and len(frame) >= 1:
            calc = sum(frame[:-1]) & 0xFF
        return {
            "ok": bool(ok),
            "len": len(frame),
            "hdr": f"{frame[0]:02X} {frame[1]:02X}" if len(frame) >= 2 else "",
            "cmd": frame[CTRL_OFFSET_CMD] if len(frame) > CTRL_OFFSET_CMD else None,
            "cmd_hex": f"0x{frame[CTRL_OFFSET_CMD]:02X}" if len(frame) > CTRL_OFFSET_CMD else "",
            "cmd_name": self._cmd_name(frame[CTRL_OFFSET_CMD]) if len(frame) > CTRL_OFFSET_CMD else "",
            "checksum": recv,
            "checksum_hex": f"0x{recv:02X}",
            "calc": calc,
            "calc_hex": f"0x{calc:02X}" if calc is not None else "",
            "hex": self._hex(frame),
        }

    def _publish_tx_debug(self, frame: bytes, spd: list, source: str):
        payload = self._frame_payload(frame, True)
        payload.update({
            "kind": "tx",
            "source": source,
            "order": "RF,LF,RR,LR",
            "speeds": list(spd),
            "line": self._format_tx_frame(frame, spd),
        })
        self.last_tx_payload = payload
        if source in ('CMD_VEL_SAFE', 'WEB_RUNTIME_ZERO_HOLD'):
            if not self._serial_debug_stream_due('tx_motion'):
                return
        self._publish_serial_debug(payload)

    def _publish_echo_debug(self, frame: bytes, ok: bool, calc: int, line: str):
        payload = self._frame_payload(frame, ok, calc)
        speeds = list(struct.unpack_from('<4i', frame, CTRL_OFFSET_SPD)) if len(frame) >= CTRL_FRAME_LEN else []
        payload.update({
            "kind": "echo",
            "speeds": speeds,
            "line": line,
        })
        self._publish_serial_debug(payload)

    def _publish_navi_debug(self, frame: bytes, ok: bool, calc: int, line: str):
        yaw_raw, vx_raw, vz_raw = struct.unpack_from('<3i', frame, CTRL_OFFSET_SPD)
        tick_ms = struct.unpack_from('<I', frame, CTRL_OFFSET_SPD + 12)[0]
        self.latest_navi_tick_ms = tick_ms
        if not self._serial_debug_stream_due('navi'):
            return
        yaw_deg_ros = self.navi_yaw_sign * (yaw_raw / 100.0) + self.navi_yaw_offset_deg
        vx_mps_ros = self.navi_vx_sign * (vx_raw / 1000.0)
        vz_deg_s_ros = self.navi_vz_sign * (vz_raw / 100.0)
        yaw_rad_ros = self._wrap_pi(math.radians(yaw_deg_ros))
        if self.navi_unwrapped_yaw_rad is None:
            yaw_deg_unwrapped_ros = math.degrees(yaw_rad_ros)
        else:
            prev_wrapped = self._wrap_pi(self.navi_unwrapped_yaw_rad)
            delta = self._wrap_pi(yaw_rad_ros - prev_wrapped)
            yaw_deg_unwrapped_ros = math.degrees(self.navi_unwrapped_yaw_rad + delta)
        payload = self._frame_payload(frame, ok, calc)
        payload.update({
            "kind": "navi",
            "yaw_deg": yaw_raw / 100.0,
            "vx_mps": vx_raw / 1000.0,
            "vz_deg_s": vz_raw / 100.0,
            "vz_rad_s": math.radians(vz_raw / 100.0),
            "yaw_deg_ros": yaw_deg_ros,
            "yaw_deg_unwrapped_ros": yaw_deg_unwrapped_ros,
            "vx_mps_ros": vx_mps_ros,
            "vz_deg_s_ros": vz_deg_s_ros,
            "vz_rad_s_ros": math.radians(vz_deg_s_ros),
            "mcu_tick_ms": tick_ms,
            "time_source": "mcu" if tick_ms else "host_fallback",
            "raw": [yaw_raw, vx_raw, vz_raw, tick_ms],
            "line": line,
        })
        self._publish_serial_debug(payload)

    def _publish_rx_debug(self, frame: bytes, name: str, ok: bool, calc: int, line: str):
        payload = self._frame_payload(frame, ok, calc)
        payload.update({
            "kind": "rx",
            "name": name,
            "line": line,
        })
        self._publish_serial_debug(payload)

    def _format_rx_frame(self, frame: bytes, name: str, ok: bool, calc: int) -> str:
        self.rx_line_count += 1
        recv = frame[-1] if frame else 0
        state = "OK" if ok else "BAD"
        return (
            f"RX#{self.rx_line_count:05d} {name} {state} len={len(frame)} "
            f"hdr={frame[0]:02X} {frame[1]:02X} checksum=0x{recv:02X} "
            f"calc=0x{calc:02X} hex={self._hex(frame)}"
        )

    def _format_echo_frame(self, frame: bytes, ok: bool, calc: int) -> str:
        cmd = frame[CTRL_OFFSET_CMD] if len(frame) > CTRL_OFFSET_CMD else 0
        name = {
            CTRL_CMD_MOVE: "ECHO_MOVE",
            CTRL_CMD_STOP: "ECHO_STOP",
            CTRL_CMD_ESTOP: "ECHO_ESTOP",
            CTRL_CMD_PS2: "ECHO_PS2",
            CTRL_CMD_ECHO_ON: "ECHO_ON_ACK",
            CTRL_CMD_ECHO_OFF: "ECHO_OFF_ACK",
            CTRL_CMD_MAPPING: "ECHO_MAPPING",
        }.get(cmd, f"ECHO_CMD_0x{cmd:02X}")
        spd = list(struct.unpack_from('<4i', frame, CTRL_OFFSET_SPD)) if len(frame) >= CTRL_FRAME_LEN else []
        return self._format_rx_frame(frame, f"{name} spd={spd}", ok, calc)

    def _format_navi_frame(self, frame: bytes, ok: bool, calc: int) -> str:
        yaw_raw, vx_raw, vz_raw = struct.unpack_from('<3i', frame, CTRL_OFFSET_SPD)
        tick_ms = struct.unpack_from('<I', frame, CTRL_OFFSET_SPD + 12)[0]
        yaw = yaw_raw / 100.0
        vx = vx_raw / 1000.0
        vz = vz_raw / 100.0
        self.rx_line_count += 1
        recv = frame[-1] if frame else 0
        state = "OK" if ok else "BAD"
        return (
            f"RX#{self.rx_line_count:05d} NAVI_AA55_20B {state} len={len(frame)} "
            f"yaw={yaw:+.2f}deg vx={vx:+.3f}m/s wz={vz:+.2f}deg/s tick={tick_ms}ms "
            f"checksum=0x{recv:02X} calc=0x{calc:02X} hex={self._hex(frame)}"
        )

    def _format_tx_frame(self, frame: bytes, spd: list) -> str:
        checksum = frame[-1] if frame else 0
        return (
            f"TX_FRAME len={len(frame)} hdr=AA 55 cmd=0x{frame[2]:02X} "
            f"order=RF,LF,RR,LR speeds={spd} checksum=0x{checksum:02X} "
            f"hex={self._hex(frame)}"
        )

    def update_serial_window(self):
        if not self.show_serial_window:
            return
        try:
            import cv2 as cv
            import numpy as np
        except Exception:
            return

        width = 1500
        height = 760
        img = np.zeros((height, width, 3), dtype=np.uint8)
        cv.putText(
            img,
            "STM32 Serial TX/RX Frames",
            (14, 38),
            cv.FONT_HERSHEY_SIMPLEX,
            0.82,
            (0, 220, 255),
            2,
            cv.LINE_AA,
        )
        cv.putText(
            img,
            self.last_tx_line[:170],
            (14, 76),
            cv.FONT_HERSHEY_SIMPLEX,
            0.48,
            (80, 255, 120),
            1,
            cv.LINE_AA,
        )

        y = 116
        for line in list(self.rx_lines)[-22:]:
            color = (180, 180, 180)
            if " OK " in line:
                color = (80, 255, 80)
            elif " BAD " in line:
                color = (60, 120, 255)
            cv.putText(
                img,
                line[:190],
                (14, y),
                cv.FONT_HERSHEY_SIMPLEX,
                0.48,
                color,
                1,
                cv.LINE_AA,
            )
            y += 29
        cv.imshow("STM32 Serial TX RX Frames", img)
        cv.waitKey(1)

    def _cmd_vel_to_spd(self, vx: float, vz: float) -> list:
        """
        逆运动学解算：Vx/Vz (m/s, rad/s) → 四轮脉冲速度 (脉冲/s)
        """
        left_cnt, right_cnt = self._cmd_vel_to_lr_counts(vx, vz)
        return [
            -right_cnt,  # 右前
            left_cnt,    # 左前
            -right_cnt,  # 右后
            left_cnt,    # 左后
        ]

    def _cmd_vel_to_lr_counts(self, vx: float, vz: float):
        """
        差速四轮抽象：Vx/Vz → 左右两侧物理轮速脉冲。
        正数表示该侧车轮物理前进。
        """
        r = self.wheel_track_w
        left_ms = vx - vz * r
        right_ms = vx + vz * r

        factor = self.effective_ppr / (2.0 * math.pi * self.wheel_radius)
        return int(round(left_ms * factor)), int(round(right_ms * factor))

    def web_control_callback(self, msg: String):
        try:
            data = json.loads(msg.data or "{}")
        except json.JSONDecodeError:
            return

        command = data.get("command")
        if command == "serial_command":
            self._handle_serial_command(str(data.get("action", "")))
            return

        if command == "gear_change":
            try:
                gear = int(data.get("gear", self.web_gear))
            except (TypeError, ValueError):
                gear = self.web_gear
            profile = str(data.get("profile", self.web_profile)).lower()
            max_gear = 2 if profile == "mapping" else 4
            if profile in ("mapping", "normal") and 1 <= gear <= max_gear:
                self.web_profile = profile
                self.web_gear = gear
                self.mapping_mode = profile == "mapping"
                self._publish_control_state()
            return

        if command in ("slam_log_enable", "slam_log_disable", "slam_log_config"):
            if "interval_sec" in data:
                try:
                    interval = float(data.get("interval_sec"))
                except (TypeError, ValueError):
                    interval = self.slam_log_interval_sec
                if math.isfinite(interval):
                    self.slam_log_interval_sec = round(
                        min(60.0, max(0.5, interval)) * 2.0) / 2.0
            if command != "slam_log_config":
                self.slam_log_enabled = command == "slam_log_enable"
            self._publish_control_state()
            return

        if command != "runtime_options":
            return

        previous_motion_enabled = self.motion_serial_enabled
        if "motion_serial_enabled" in data:
            self.motion_serial_enabled = bool(data.get("motion_serial_enabled"))
        if "show_obstacle_fill" in data:
            self.show_obstacle_fill = bool(data.get("show_obstacle_fill"))
        if "show_roi_polygons" in data:
            self.show_roi_polygons = bool(data.get("show_roi_polygons"))
        if "show_rgb_debug_text" in data:
            self.show_rgb_debug_text = bool(data.get("show_rgb_debug_text"))

        if self.motion_serial_enabled and not previous_motion_enabled:
            self.get_logger().warn("Web runtime option: chassis serial MOVE output enabled")
        elif previous_motion_enabled and not self.motion_serial_enabled:
            self.get_logger().warn("Web runtime option: chassis serial MOVE output disabled")
            if self.control_mode == "move":
                self._send_ctrl_frame(
                    CTRL_CMD_MOVE, [0, 0, 0, 0], "WEB_RUNTIME_ZERO_HOLD")
        self._publish_control_state()

    def _on_baseline_ready(self, msg: Bool):
        self.baseline_ready = bool(msg.data)
        self._publish_control_state()

    def _on_software_estop(self, msg: Bool):
        self.software_estop = bool(msg.data)
        self._publish_control_state()

    def _startup_defaults_tick(self):
        if not self.startup_default_queue:
            self.startup_defaults_timer.cancel()
            self._publish_control_state()
            return
        cmd, speeds, source = self.startup_default_queue.pop(0)
        if self._send_ctrl_frame(cmd, speeds, source):
            self._apply_control_state(cmd, speeds)

    def _control_state_payload(self) -> dict:
        return {
            "source": "chassis_node",
            "control_mode": self.control_mode,
            "echo_enabled": self.echo_enabled,
            "software_estop": self.software_estop,
            "mapping_mode": self.mapping_mode,
            "motion_serial_enabled": self.motion_serial_enabled,
            "baseline_ready": self.baseline_ready,
            "web_profile": self.web_profile,
            "web_gear": self.web_gear,
            "show_obstacle_fill": self.show_obstacle_fill,
            "show_roi_polygons": self.show_roi_polygons,
            "show_rgb_debug_text": self.show_rgb_debug_text,
            "slam_log_enabled": self.slam_log_enabled,
            "slam_log_interval_sec": self.slam_log_interval_sec,
            "initialized": not bool(self.startup_default_queue),
            "stamp_sec": self.get_clock().now().nanoseconds / 1e9,
        }

    def _publish_control_state(self):
        if not hasattr(self, 'control_state_pub'):
            return
        msg = String()
        msg.data = json.dumps(self._control_state_payload(), ensure_ascii=False)
        self.control_state_pub.publish(msg)
        self.software_estop_pub.publish(Bool(data=self.software_estop))

    def _apply_control_state(self, cmd: int, speeds: list):
        if cmd == CTRL_CMD_MOVE:
            self.control_mode = "move"
        elif cmd == CTRL_CMD_PS2:
            self.control_mode = "ps2"
            self.motion_serial_enabled = False
        elif cmd == CTRL_CMD_ECHO_ON:
            self.echo_enabled = True
        elif cmd == CTRL_CMD_ECHO_OFF:
            self.echo_enabled = False
        self._publish_control_state()

    def _apply_echo_state(self, frame: bytes):
        if len(frame) < CTRL_FRAME_LEN:
            return
        speeds = list(struct.unpack_from('<4i', frame, CTRL_OFFSET_SPD))
        self._apply_control_state(frame[CTRL_OFFSET_CMD], speeds)

    def _send_ps2_release(self):
        self._send_ctrl_frame(CTRL_CMD_PS2, [0, 0, 0, 0], "WEB_RUNTIME_PS2_RELEASE")

    def _handle_serial_command(self, action: str):
        action = action.strip().lower()
        if action in ("mapping_on", "mapping_off"):
            # Backward compatibility for cached web clients: 0x08 is now a
            # web-only speed profile and must never be written to the STM32.
            self.mapping_mode = action == "mapping_on"
            self.web_profile = "mapping" if self.mapping_mode else "normal"
            self.web_gear = 1
            self._publish_serial_debug({
                "kind": "control",
                "action": action,
                "ok": True,
                "line": f"WEB_PROFILE_ONLY {self.web_profile}; serial 0x08 suppressed",
            })
            self._publish_control_state()
            return

        if action in ("enable_move", "move"):
            if self._send_ctrl_frame(
                    CTRL_CMD_MOVE, [0, 0, 0, 0], "WEB_MOVE_TAKEOVER"):
                self.motion_serial_enabled = True
                self.control_mode = "move"
                self.get_logger().warn(
                    "Web serial command: MOVE output enabled")
            else:
                self.get_logger().error(
                    "Web serial command: MOVE takeover was not sent; "
                    "autonomous output remains disabled")
            self._publish_control_state()
            return

        if action == "ps2" and not self.baseline_ready:
            self._publish_serial_debug({
                "kind": "control",
                "action": action,
                "ok": False,
                "line": "PS2 release blocked: depth baseline is not ready",
            })
            self._publish_control_state()
            return

        commands = {
            "zero_move": (CTRL_CMD_MOVE, [0, 0, 0, 0], "WEB_ZERO_MOVE"),
            "stop": (CTRL_CMD_STOP, [0, 0, 0, 0], "WEB_STOP"),
            "estop": (CTRL_CMD_ESTOP, [0, 0, 0, 0], "WEB_ESTOP"),
            "ps2": (CTRL_CMD_PS2, [0, 0, 0, 0], "WEB_PS2"),
            "echo_on": (CTRL_CMD_ECHO_ON, [0, 0, 0, 0], "WEB_ECHO_ON"),
            "echo_off": (CTRL_CMD_ECHO_OFF, [0, 0, 0, 0], "WEB_ECHO_OFF"),
        }
        item = commands.get(action)
        if item is None:
            self.get_logger().warn(f"Unknown web serial command: {action}")
            return

        cmd, speeds, source = item
        if action == "ps2":
            self.motion_serial_enabled = False
        if self._send_ctrl_frame(cmd, speeds, source):
            self._apply_control_state(cmd, speeds)

    def _send_ctrl_frame(self, cmd: int, speeds: list, source: str) -> bool:
        frame = self._make_ctrl_frame(cmd, speeds)
        self.last_tx_line = self._format_tx_frame(frame, speeds)
        self._publish_tx_debug(frame, speeds, source)

        if not hasattr(self, 'ser') or not self.ser.is_open:
            self.get_logger().warn(f"{source}: serial is not open")
            return False

        try:
            self.ser.write(frame)
            return True
        except serial.SerialException as e:
            self.get_logger().warn(f"{source}: serial write failed: {e}")
            return False

    # ==================== /cmd_vel 处理 ====================

    def cmd_vel_callback(self, msg):
        self._send_cmd_vel_to_stm32(msg)

    def _send_cmd_vel_to_stm32(self, msg):
        vx = msg.linear.x
        vz = msg.angular.z
        left_cnt, right_cnt = self._cmd_vel_to_lr_counts(vx, vz)

        wheel_msg = Int32MultiArray()
        wheel_msg.data = [left_cnt, right_cnt]
        self.wheel_speed_pub.publish(wheel_msg)

        if not hasattr(self, 'ser') or not self.ser.is_open:
            return

        spd = self._cmd_vel_to_spd(vx, vz)
        frame = self._make_ctrl_frame(CTRL_CMD_MOVE, spd)
        self.last_tx_line = self._format_tx_frame(frame, spd)

        if not self.motion_serial_enabled:
            self.last_tx_line = "TX_FRAME blocked by web runtime option: motion_serial_enabled=false"
            self._publish_serial_debug({
                "kind": "tx_blocked",
                "ok": True,
                "line": self.last_tx_line,
                "candidate_hex": self._hex(frame),
                "speeds": spd,
            })
            return

        try:
            self.ser.write(frame)
            self._publish_tx_debug(frame, spd, "CMD_VEL_SAFE")
        except serial.SerialException as e:
            self.get_logger().warn(f"发送 {self.cmd_vel_topic} 到串口失败: {e}")

    # ==================== 发布 Odometry ====================

    def _publish_navi_odometry(self, sample_time):
        if self.use_navi_odom and self.odom_publish_mode == 'navi':
            self.published_measurement_seq = self.latest_measurement_seq
            self._publish_odometry_state(
                sample_time.to_msg(), self.odom_x, self.odom_y,
                self.odom_theta, self.current_vx, self.current_vy,
                self.current_vz)

    def publish_odometry(self):
        stamp = self.get_clock().now().to_msg()
        if self.use_navi_odom and self.latest_measurement_stamp is not None:
            self.published_measurement_seq = self.latest_measurement_seq

        # ==== 前向外推：在两个 NAVI 帧之间用最后已知速度预测位姿 ====
        pred_x = self.odom_x
        pred_y = self.odom_y
        pred_theta = self.odom_theta

        if (self.use_navi_odom
                and hasattr(self, 'last_navi_ros_time')
                and self.last_navi_ros_time is not None):
            dt = (self.get_clock().now() - self.last_navi_ros_time).nanoseconds / 1e9
            if 0.0 < dt < 0.5:
                vz_used = 0.0 if abs(self.current_vz) < 0.005 else self.current_vz
                mid_theta = pred_theta + vz_used * dt * 0.5
                pred_theta += vz_used * dt
                pred_x += self.current_vx * math.cos(mid_theta) * dt
                pred_y += self.current_vx * math.sin(mid_theta) * dt

        self._publish_odometry_state(
            stamp, pred_x, pred_y, pred_theta,
            self.current_vx, self.current_vy, self.current_vz)

    def _publish_odometry_state(self, stamp, x, y, yaw, vx, vy, wz):
        """Publish odom and TF from one coherent, timestamped state sample."""

        # 从 IMU 获取 roll/pitch（如可用），yaw 来自里程计
        if self.use_imu_rp and self.imu_available:
            roll_r  = math.radians(self.imu_roll)
            pitch_r = math.radians(self.imu_pitch)
        else:
            roll_r  = 0.0
            pitch_r = 0.0
        yaw_r = yaw

        cr = math.cos(roll_r * 0.5)
        cp = math.cos(pitch_r * 0.5)
        cy = math.cos(yaw_r * 0.5)
        sr = math.sin(roll_r * 0.5)
        sp = math.sin(pitch_r * 0.5)
        sy = math.sin(yaw_r * 0.5)
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy

        # ===== TF: odom → base_link =====
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self._timed_send_transform(t)

        # ===== /odom =====
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        odom_msg.pose.covariance = [
            0.01, 0.0,  0.0,  0.0, 0.0, 0.0,
            0.0,  0.01, 0.0,  0.0, 0.0, 0.0,
            0.0,  0.0,  1e-6, 0.0, 0.0, 0.0,
            0.0,  0.0,  0.0,  1e-6,0.0, 0.0,
            0.0,  0.0,  0.0,  0.0, 1e-6,0.0,
            0.0,  0.0,  0.0,  0.0, 0.0, 0.20
        ]

        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = wz

        odom_msg.twist.covariance = [
            0.01, 0.0,  0.0,  0.0, 0.0, 0.0,
            0.0,  0.01, 0.0,  0.0, 0.0, 0.0,
            0.0,  0.0,  1e-6, 0.0, 0.0, 0.0,
            0.0,  0.0,  0.0,  1e-6,0.0, 0.0,
            0.0,  0.0,  0.0,  0.0, 1e-6,0.0,
            0.0,  0.0,  0.0,  0.0, 0.0, 0.10
        ]

        self._timed_publish('odom', self.odom_pub, odom_msg)

    # ==================== 资源清理 ====================

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("串口已关闭")
        if getattr(self, 'show_serial_window', False):
            try:
                import cv2 as cv
                cv.destroyWindow("STM32 Serial TX RX Frames")
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ChassisNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
