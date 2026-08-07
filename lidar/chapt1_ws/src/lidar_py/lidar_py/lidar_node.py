import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

from lidar_py.fixed_scan_grid import FixedScanGridBuilder
from lidar_py.lidar_timing import (
    MonotonicMinimumDelayMapper,
    WrappingMillisecondClock,
)

import serial
import math
import time

# =============================================================================
# CRC8 lookup table (polynomial 0x4d - LD14P standard)
# =============================================================================
CRC_TABLE = [
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae,
    0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
    0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
    0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
    0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1,
    0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18,
    0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea,
    0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
    0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62,
    0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39,
    0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f,
    0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d,
    0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
    0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
    0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2,
    0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b,
    0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
    0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
    0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
    0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64,
    0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec,
    0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e,
    0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
    0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
    0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3,
    0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a,
    0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
]


def cal_crc8(data):
    """Calculate CRC8 checksum using LD14P polynomial (0x4d)."""
    crc = 0
    for b in data:
        crc = CRC_TABLE[(crc ^ b) & 0xff]
    return crc


# =============================================================================
# LD14P Protocol Constants
# Frame: header(1) + ver_len(1) + speed(2) + start_angle(2) + 12*point(3) + end_angle(2) + timestamp(2) + crc(1) = 47 bytes
# =============================================================================
FRAME_HEADER = 0x54
FRAME_VER_LEN = 0x2C  # high 3 bits=frame_type(1), low 5 bits=points_per_frame(12)
FRAME_SIZE = 47
POINTS_PER_FRAME = 12
POINT_SIZE = 3  # 2 bytes distance (mm) + 1 byte intensity

# Byte offsets within a frame
OFFSET_HEADER = 0
OFFSET_VER_LEN = 1
OFFSET_SPEED = 2       # 2 bytes, LSB first, unit: °/s
OFFSET_START_ANGLE = 4  # 2 bytes, LSB first, unit: 0.01°
OFFSET_POINTS = 6       # 12 * 3 = 36 bytes
OFFSET_END_ANGLE = 42   # 2 bytes, LSB first, unit: 0.01°
OFFSET_TIMESTAMP = 44   # 2 bytes, LSB first, unit: ms
OFFSET_CRC = 46         # 1 byte

# Data validation thresholds
# LD14P actual effective range: 0.05m ~ 8m (from datasheet)
# Values outside this range are noise
MAX_VALID_DISTANCE_MM = 8000   # 8 meters max (LD14P effective range)
MIN_VALID_DISTANCE_MM = 50     # 5 cm min (LD14P effective range)
MAX_INTENSITY = 255
# Minimum intensity threshold - very low intensity = noise
MIN_VALID_INTENSITY = 5


class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')

        # ===== Declare parameters =====
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 230400)
        self.declare_parameter('frame_id', 'laser_frame')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('scan_interval', 0.1)  # 10Hz publish rate
        # LiDAR rotation center is measured 0.20 m ahead of the chassis center.
        self.declare_parameter('laser_x', 0.20)
        self.declare_parameter('laser_y', 0.0)
        self.declare_parameter('laser_z', 0.0)
        self.declare_parameter('laser_yaw_deg', 0.0)
        self.declare_parameter('scan_angle_sign', -1.0)
        self.declare_parameter('scan_angle_offset_deg', 0.0)
        self.declare_parameter('publish_timed_scan', False)
        self.declare_parameter('timed_scan_topic', '/scan_timed')
        self.declare_parameter('publish_fixed_timed_scan', False)
        self.declare_parameter('fixed_timed_scan_topic', '/scan_timed_v2')
        self.declare_parameter('fixed_scan_bins', 360)
        # The real LD14P has been observed from about 8.1 Hz / 288 rays to
        # about 3.7 Hz / 630 rays. Angular wrap and scan duration still prove
        # that this is a complete revolution; 300 incorrectly rejects the
        # faster, otherwise healthy operating point.
        self.declare_parameter('fixed_scan_min_raw_points', 180)
        self.declare_parameter('fixed_scan_max_raw_points', 720)
        self.declare_parameter('fixed_scan_min_valid_points', 0)
        self.declare_parameter('fixed_scan_min_time_sec', 0.10)
        self.declare_parameter('fixed_scan_max_time_sec', 0.35)
        self.declare_parameter('clock_max_adjustment_ns', 100000)
        self.declare_parameter('serial_stall_timeout_sec', 1.0)

        serial_port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value
        scan_topic = str(self.get_parameter('scan_topic').value)
        scan_interval = self.get_parameter('scan_interval').value
        laser_x = float(self.get_parameter('laser_x').value)
        laser_y = float(self.get_parameter('laser_y').value)
        laser_z = float(self.get_parameter('laser_z').value)
        laser_yaw = math.radians(float(self.get_parameter('laser_yaw_deg').value))
        self.scan_angle_sign = float(self.get_parameter('scan_angle_sign').value)
        self.scan_angle_offset_deg = float(self.get_parameter('scan_angle_offset_deg').value)
        self.publish_timed_scan = bool(self.get_parameter('publish_timed_scan').value)
        timed_scan_topic = str(self.get_parameter('timed_scan_topic').value)
        self.publish_fixed_timed_scan = bool(
            self.get_parameter('publish_fixed_timed_scan').value)
        fixed_timed_scan_topic = str(
            self.get_parameter('fixed_timed_scan_topic').value)
        clock_max_adjustment_ns = int(
            self.get_parameter('clock_max_adjustment_ns').value)
        self.serial_stall_timeout_sec = max(
            0.5, float(self.get_parameter('serial_stall_timeout_sec').value))

        # ===== Open serial port with retry =====
        self.ser = None
        self.serial_port = serial_port
        self.baudrate = baudrate
        self._connect_serial()



        # ===== LaserScan publisher (RELIABLE for Nav2 / AMCL compatibility) =====
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.scan_pub = self.create_publisher(LaserScan, scan_topic, qos)
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.timed_scan_pub = None
        if self.publish_timed_scan:
            self.timed_scan_pub = self.create_publisher(
                LaserScan, timed_scan_topic, sensor_qos)
        self.fixed_timed_scan_pub = None
        self.fixed_scan_builder = None
        if self.publish_fixed_timed_scan:
            self.fixed_timed_scan_pub = self.create_publisher(
                LaserScan, fixed_timed_scan_topic, sensor_qos)
            self.fixed_scan_builder = FixedScanGridBuilder(
                bins=int(self.get_parameter('fixed_scan_bins').value),
                angle_sign=self.scan_angle_sign,
                angle_offset_deg=self.scan_angle_offset_deg,
                min_raw_points=int(
                    self.get_parameter('fixed_scan_min_raw_points').value),
                max_raw_points=int(
                    self.get_parameter('fixed_scan_max_raw_points').value),
                min_valid_points=int(
                    self.get_parameter('fixed_scan_min_valid_points').value),
                min_scan_time=float(
                    self.get_parameter('fixed_scan_min_time_sec').value),
                max_scan_time=float(
                    self.get_parameter('fixed_scan_max_time_sec').value),
            )
        self.fixed_scan_last_drop_count = 0
        self.fixed_scan_last_drop_log_time = 0.0
        self.fixed_scan_last_reported_drop_count = 0

        # ===== Internal state =====
        self.buffer = bytearray()
        self.scan_points = {}  # angle_deg -> (distance_m, intensity)
        # Cartographer input must preserve the physical acquisition order.
        # The legacy dictionary above is still used for the fixed-grid /scan.
        self.timed_scan_points = []  # (ros_angle_deg, distance_m, intensity)
        self.prev_start_angle = None  # for detecting full rotation wrap-around
        self.scan_interval = max(0.01, float(scan_interval))
        self.scan_start_time = None
        self.scan_start_ros_angle_deg = None
        self.scan_speed_samples = []
        self.have_full_scan_start = False
        self.device_clock = WrappingMillisecondClock(
            modulus=30000, max_step_ms=2000)
        self.device_tick_wrap_count = 0
        self.device_tick_reset_count = 0
        self.device_tick_invalid_count = 0
        self.device_tick_unwrapped_ms = None
        self.device_time_mapper = MonotonicMinimumDelayMapper(
            max_adjustment_ns=clock_max_adjustment_ns)
        self.device_clock_lag_ms = 0.0
        self.scan_start_device_ms = None
        self.completed_device_scan_time = None


        # Statistics for debugging
        self.frame_count = 0
        self.crc_error_count = 0
        self.crc_ok_count = 0
        self.invalid_point_count = 0
        self.zero_distance_count = 0
        self.last_valid_frame_monotonic = time.monotonic()
        self.serial_reconnect_count = 0


        # ===== Static TF: base_link -> laser_frame =====
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        t_base_laser = TransformStamped()
        t_base_laser.header.stamp = self.get_clock().now().to_msg()
        t_base_laser.header.frame_id = 'base_link'
        t_base_laser.child_frame_id = self.frame_id
        t_base_laser.transform.translation.x = laser_x
        t_base_laser.transform.translation.y = laser_y
        t_base_laser.transform.translation.z = laser_z
        t_base_laser.transform.rotation.x = 0.0
        t_base_laser.transform.rotation.y = 0.0
        t_base_laser.transform.rotation.z = math.sin(laser_yaw * 0.5)
        t_base_laser.transform.rotation.w = math.cos(laser_yaw * 0.5)

        self.static_tf_broadcaster.sendTransform(t_base_laser)
        self.get_logger().info(
            f"Published static TF: base_link -> {self.frame_id} "
            f"xyz=({laser_x:.3f}, {laser_y:.3f}, {laser_z:.3f}) "
            f"yaw={math.degrees(laser_yaw):.1f}deg"
        )

        # ===== Timer for reading data =====
        self.timer = self.create_timer(0.005, self.read_data)  # 5ms for responsive reading


        self.get_logger().info(f"LiDAR node initialized, publishing to {scan_topic}")
        if self.publish_timed_scan:
            self.get_logger().info(
                f"Motion-aware LaserScan enabled: {timed_scan_topic} "
                "(acquisition order + per-ray timing)"
            )
        if self.publish_fixed_timed_scan:
            self.get_logger().info(
                f"Fixed-grid LaserScan enabled: {fixed_timed_scan_topic} "
                f"bins={self.fixed_scan_builder.bins} "
                "(point-level wrap + measured-angle indexing)"
            )
        self.get_logger().info(
            f"Laser scan angle mapping: ros_angle = "
            f"({self.scan_angle_sign:+.1f} * raw_angle + "
            f"{self.scan_angle_offset_deg:+.1f}) mod 360"
        )

    def _connect_serial(self, attempts=10):
        """Open serial port with retry logic."""
        import time as _time
        for attempt in range(attempts):
            try:
                self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=0.1)
                self.ser.reset_input_buffer()
                if hasattr(self, 'buffer'):
                    self.buffer.clear()
                self.last_valid_frame_monotonic = time.monotonic()
                self.get_logger().info(f"Serial port {self.serial_port} opened at {self.baudrate} baud")
                return
            except serial.SerialException as e:
                self.get_logger().warn(
                    f"Serial port attempt {attempt+1}/{attempts} failed: {e}")
                _time.sleep(1.0)
        self.get_logger().error(
            f"Failed to open serial port {self.serial_port} after {attempts} attempts")
        # Create a dummy serial to avoid crashes - node will run without data
        self.ser = serial.Serial.__new__(serial.Serial)
        self.ser.is_open = False

    def read_data(self):
        """Read data from serial buffer and parse frames."""
        if not self.ser.is_open:
            # Try to reconnect
            self._connect_serial(attempts=1)
            return

        try:
            in_waiting = self.ser.in_waiting
            if in_waiting > 0:
                data = self.ser.read(in_waiting)
                self.buffer.extend(data)
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f"LiDAR serial read failed: {exc}")
            self._restart_serial_stream("read error")
            return

        self._parse_buffer()
        if (time.monotonic() - self.last_valid_frame_monotonic
                > self.serial_stall_timeout_sec):
            self._restart_serial_stream(
                f"no valid packet for {self.serial_stall_timeout_sec:.1f}s")

    def _restart_serial_stream(self, reason):
        """Reopen a stalled USB serial stream and restart device time mapping."""
        self.serial_reconnect_count += 1
        self.get_logger().error(
            f"LiDAR stream stalled ({reason}); reopening {self.serial_port} "
            f"(count={self.serial_reconnect_count})")
        try:
            if self.ser is not None and self.ser.is_open:
                self.ser.close()
        except (serial.SerialException, OSError):
            pass
        self.buffer.clear()
        self.scan_points.clear()
        self.timed_scan_points.clear()
        self.prev_start_angle = None
        self.have_full_scan_start = False
        self.scan_start_time = None
        self.scan_start_device_ms = None
        self.completed_device_scan_time = None
        self.device_clock = WrappingMillisecondClock(
            modulus=30000, max_step_ms=2000)
        self.device_time_mapper.reset()
        self.last_valid_frame_monotonic = time.monotonic()
        self._connect_serial(attempts=1)

    def _parse_buffer(self):
        """Parse complete frames from the buffer using CRC validation."""
        while True:
            if len(self.buffer) < FRAME_SIZE:
                return

            # Find candidate frame header (0x54 followed by 0x2C)
            found = False
            for i in range(len(self.buffer) - FRAME_SIZE + 1):
                if (self.buffer[i] == FRAME_HEADER and 
                    self.buffer[i + 1] == FRAME_VER_LEN):
                    # Candidate found, validate CRC
                    candidate = self.buffer[i:i + FRAME_SIZE]
                    data_part = candidate[:OFFSET_CRC]
                    crc_recv = candidate[OFFSET_CRC]
                    crc_calc = cal_crc8(data_part)
                    
                    if crc_calc == crc_recv:
                        # Valid frame found!
                        frame = candidate
                        self.buffer = self.buffer[i + FRAME_SIZE:]
                        self._process_frame(frame)
                        found = True
                        break
                    # CRC failed, this is a false positive - continue searching
            
            if not found:
                # No valid frame found, clear buffer to avoid infinite loop
                # But keep the last (FRAME_SIZE-1) bytes in case a frame straddles the boundary
                if len(self.buffer) > FRAME_SIZE * 2:
                    self.buffer = self.buffer[-(FRAME_SIZE - 1):]
                return

    def _process_frame(self, frame):
        """Process a single 47-byte frame according to LD14P protocol."""
        self.last_valid_frame_monotonic = time.monotonic()
        self.frame_count += 1
        self.crc_ok_count += 1

        # ===== Parse speed (degrees/second) =====
        speed_raw = frame[OFFSET_SPEED] | (frame[OFFSET_SPEED + 1] << 8)

        frame_receipt_time = self.get_clock().now()

        # The LD14P timestamp resets at 30000 ms (not at uint16 overflow).
        # Preserve its relative timing and map it once into the ROS clock domain instead
        # of timestamping every revolution from USB/Python scheduling time.
        timestamp_raw = frame[OFFSET_TIMESTAMP] | (frame[OFFSET_TIMESTAMP + 1] << 8)
        try:
            tick_update = self.device_clock.update(timestamp_raw)
        except ValueError:
            self.device_tick_invalid_count += 1
            if (self.device_tick_invalid_count <= 3
                    or self.device_tick_invalid_count % 100 == 0):
                self.get_logger().warning(
                    "Discarding LD14P packet with invalid timestamp "
                    f"{timestamp_raw}ms (count={self.device_tick_invalid_count})")
            return
        self.device_tick_unwrapped_ms = tick_update.unwrapped_ms
        tick_reset = tick_update.reset
        if tick_update.wrapped:
            self.device_tick_wrap_count += 1
            if self.device_tick_wrap_count <= 3:
                self.get_logger().info(
                    "LD14P 30000ms timestamp rollover handled "
                    f"without resetting scan timing (count={self.device_tick_wrap_count})")
        if tick_reset:
            self.device_tick_reset_count += 1
            self.device_time_mapper.reset()
            self.scan_start_device_ms = None

        packet_wire_ns = int((47.0 * 10.0 / self.baudrate) * 1e9)
        mapped_device_ns = self.device_time_mapper.map_ms(
            self.device_tick_unwrapped_ms,
            frame_receipt_time.nanoseconds,
            packet_wire_ns,
        )
        self.device_clock_lag_ms = (
            frame_receipt_time.nanoseconds - mapped_device_ns) / 1e6
        device_stamp = Time(
            nanoseconds=mapped_device_ns,
            clock_type=frame_receipt_time.clock_type,
        )

        # ===== Parse angles =====
        start_angle_raw = frame[OFFSET_START_ANGLE] | (frame[OFFSET_START_ANGLE + 1] << 8)
        end_angle_raw = frame[OFFSET_END_ANGLE] | (frame[OFFSET_END_ANGLE + 1] << 8)

        start_angle = start_angle_raw / 100.0
        end_angle = end_angle_raw / 100.0

        # A wrap marks the boundary between two physical revolutions. Detect it
        # before adding the current packet so points from the new revolution are
        # never mixed into the previous LaserScan.
        wrapped = (
            self.prev_start_angle is not None
            and self.prev_start_angle > 300.0
            and start_angle < 60.0
        )
        if wrapped:
            self.completed_device_scan_time = None
            if self.scan_start_device_ms is not None:
                device_scan_time = (
                    self.device_tick_unwrapped_ms - self.scan_start_device_ms) * 0.001
                if 0.05 <= device_scan_time <= 1.0:
                    self.completed_device_scan_time = device_scan_time
            if self.have_full_scan_start and self.scan_points:
                self._publish_scan()
            else:
                # Startup usually begins halfway through a revolution. Drop that
                # partial scan and start publishing only after one complete turn.
                self.scan_points.clear()
                self.timed_scan_points.clear()

            self.have_full_scan_start = True
            self.scan_start_time = device_stamp
            self.scan_start_device_ms = self.device_tick_unwrapped_ms
            self.scan_speed_samples = []
            self.scan_start_ros_angle_deg = (
                self.scan_angle_sign * start_angle + self.scan_angle_offset_deg
            ) % 360.0

        # Handle angle wrap-around (crossing 360°)
        if end_angle < start_angle:
            end_angle += 360.0

        angle_step = (end_angle - start_angle) / (POINTS_PER_FRAME - 1)

        if self.have_full_scan_start and speed_raw > 0:
            self.scan_speed_samples.append(float(speed_raw))

        # ===== Parse 12 measurement points =====
        for i in range(POINTS_PER_FRAME):
            offset = OFFSET_POINTS + i * POINT_SIZE

            distance_mm = frame[offset] | (frame[offset + 1] << 8)
            intensity = frame[offset + 2]

            # Calculate angle (degrees) via linear interpolation
            angle_deg = start_angle + i * angle_step
            raw_angle_deg = angle_deg % 360.0

            # The packet timestamp is located at the end of the LD14P frame.
            # Reconstruct each ray's relative acquisition time from the
            # measured angular speed so a wrap inside the 12-ray packet does
            # not move the scan start by one whole packet.
            point_timestamp_ns = device_stamp.nanoseconds
            if speed_raw > 1.0 and angle_step >= 0.0:
                remaining_angle_deg = (
                    POINTS_PER_FRAME - 1 - i) * angle_step
                point_timestamp_ns -= int(
                    remaining_angle_deg / speed_raw * 1e9)

            # Normalize to [0, 360)
            # Convert the LD14P angle direction to the convention already
            # verified by the existing vehicle setup.
            angle_deg = (
                self.scan_angle_sign * angle_deg + self.scan_angle_offset_deg
            ) % 360.0
            angle_key = round(angle_deg, 1)

            # Keep every ray in acquisition order for /scan_timed, including
            # invalid returns. LaserScan timing is indexed by ray position, so
            # dropping a ray would shift the timestamps of all following rays.
            timed_distance = float('inf')
            timed_intensity = 0.0

            # ===== Data validation =====
            if distance_mm == 0:
                self.zero_distance_count += 1
                if angle_key not in self.scan_points:
                    self.scan_points[angle_key] = (float('inf'), 0)
            elif not self._is_valid_point(distance_mm, intensity):
                self.invalid_point_count += 1
                if angle_key not in self.scan_points:
                    self.scan_points[angle_key] = (float('inf'), 0)
            else:
                distance_m = distance_mm / 1000.0
                timed_distance = distance_m
                timed_intensity = float(intensity)

                # Legacy fixed-grid scan keeps the nearest duplicate return.
                if (angle_key not in self.scan_points
                        or distance_m < self.scan_points[angle_key][0]):
                    self.scan_points[angle_key] = (distance_m, intensity)

            if self.have_full_scan_start:
                self.timed_scan_points.append(
                    (angle_deg, timed_distance, timed_intensity))

            if self.fixed_scan_builder is not None:
                fixed_scan = self.fixed_scan_builder.add_ray(
                    raw_angle_deg=raw_angle_deg,
                    distance_m=timed_distance,
                    intensity=timed_intensity,
                    timestamp_ns=point_timestamp_ns,
                )
                if fixed_scan is not None:
                    self._publish_fixed_timed_scan(fixed_scan, device_stamp)
                elif (self.fixed_scan_builder.dropped_count
                      > self.fixed_scan_last_drop_count):
                    self.fixed_scan_last_drop_count = (
                        self.fixed_scan_builder.dropped_count)
                    now_monotonic = time.monotonic()
                    if now_monotonic - self.fixed_scan_last_drop_log_time >= 2.0:
                        new_drops = (
                            self.fixed_scan_builder.dropped_count -
                            self.fixed_scan_last_reported_drop_count
                        )
                        self.get_logger().warn(
                            "Dropped malformed fixed-grid revolution: "
                            f"{self.fixed_scan_builder.last_drop_reason}; "
                            f"drops_since_report={new_drops}"
                        )
                        self.fixed_scan_last_drop_log_time = now_monotonic
                        self.fixed_scan_last_reported_drop_count = (
                            self.fixed_scan_builder.dropped_count)

        self.prev_start_angle = start_angle


    def _publish_fixed_timed_scan(self, fixed_scan, device_stamp):
        """Publish a validated revolution built from measured ray angles."""
        msg = LaserScan()
        msg.header.stamp = Time(
            nanoseconds=fixed_scan.start_time_ns,
            clock_type=device_stamp.clock_type,
        ).to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = fixed_scan.angle_min
        msg.angle_max = fixed_scan.angle_max
        msg.angle_increment = fixed_scan.angle_increment
        msg.time_increment = fixed_scan.time_increment
        msg.scan_time = fixed_scan.scan_time
        msg.range_min = 0.05
        msg.range_max = 8.0
        msg.ranges = list(fixed_scan.ranges)
        msg.intensities = list(fixed_scan.intensities)
        self.fixed_timed_scan_pub.publish(msg)

        if self.fixed_scan_builder.published_count % 100 == 0:
            self.get_logger().info(
                "Fixed-grid scan: "
                f"raw={fixed_scan.raw_point_count}, "
                f"filled={fixed_scan.filled_bin_count}/"
                f"{self.fixed_scan_builder.bins}, "
                f"valid={fixed_scan.valid_point_count}, "
                f"scan_time={fixed_scan.scan_time:.4f}s, "
                f"device_lag={self.device_clock_lag_ms:.2f}ms, "
                f"dropped={self.fixed_scan_builder.dropped_count}"
            )


    def _is_valid_point(self, distance_mm, intensity):
        """Validate a single measurement point."""
        if distance_mm < MIN_VALID_DISTANCE_MM or distance_mm > MAX_VALID_DISTANCE_MM:
            return False
        if intensity < 0 or intensity > MAX_INTENSITY:
            return False
        return True

    def _publish_scan(self):
        """Publish accumulated scan points as a LaserScan message."""
        if not self.scan_points:
            return

        angles = sorted(self.scan_points.keys())
        if len(angles) < 10:
            return

        # ===== Compute dynamic angle resolution from actual data =====
        # LD14P spec: 0.54° resolution at 6Hz -> ~666 points per rotation
        # But we compute it from the actual angle differences in the data
        # This ensures the LaserScan message accurately represents the real data

        # Fixed angle resolution for consistent scan point count.
        # LD14P at ~4.7Hz produces ~514 points with ~0.7° resolution.
        # Dynamic computation caused varying point counts (514/600/720) which
        # confuses scan matching and causes map jumping.
        ANGLE_INCREMENT_DEG = 0.7

        angle_increment = math.radians(ANGLE_INCREMENT_DEG)

        # Compute number of points: we want angle_min + (N-1)*inc = angle_max
        # For a full 360° scan: angle_min=0, angle_max=360-inc
        # N = 360/inc
        num_points = int(round(360.0 / ANGLE_INCREMENT_DEG))
        if num_points < 10:
            num_points = 360  # fallback

        now = self.get_clock().now()
        measured_scan_time = self.scan_interval
        if self.scan_start_time is not None:
            measured_scan_time = (now - self.scan_start_time).nanoseconds / 1e9

        speed_scan_time = 0.0
        average_speed_raw = 0.0
        if self.scan_speed_samples:
            average_speed_raw = sum(self.scan_speed_samples) / len(self.scan_speed_samples)
            if average_speed_raw > 1.0:
                speed_scan_time = 360.0 / average_speed_raw

        # Prefer the LiDAR's own clock. Host wrap timing is only a fallback for
        # old/invalid packets, and motor speed is the final sanity fallback.
        scan_time = (
            self.completed_device_scan_time
            if self.completed_device_scan_time is not None
            else measured_scan_time
        )
        if not 0.05 <= scan_time <= 1.0:
            scan_time = speed_scan_time if 0.05 <= speed_scan_time <= 1.0 else self.scan_interval
        self.scan_interval = scan_time

        # Legacy positive-angle scan retained for RViz and web compatibility.
        msg = LaserScan()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id

        msg.angle_min = 0.0
        msg.angle_max = math.radians(360.0 - ANGLE_INCREMENT_DEG)
        msg.angle_increment = angle_increment
        msg.time_increment = 0.0
        msg.scan_time = scan_time
        msg.range_min = 0.05  # LD14P effective min range
        msg.range_max = 8.0   # LD14P effective max range

        # Fill ranges and intensities (all inf by default)
        msg.ranges = [float('inf')] * num_points
        msg.intensities = [0.0] * num_points

        valid_count = 0
        for angle_deg, (distance_m, intensity) in self.scan_points.items():
            idx = int(round(angle_deg / ANGLE_INCREMENT_DEG))
            if 0 <= idx < num_points:
                # Final sanity check on distance
                if 0.05 <= distance_m <= 8.0:
                    msg.ranges[idx] = distance_m
                    msg.intensities[idx] = float(intensity)
                    valid_count += 1

        # Publish scan message
        self.scan_pub.publish(msg)

        # Cartographer gets the untouched acquisition order. Do not rebuild this
        # sequence from the angle-keyed legacy dictionary: that loses duplicate
        # rays and changes each point's implied acquisition time.
        if self.timed_scan_pub is not None and len(self.timed_scan_points) >= 10:
            timed_count = len(self.timed_scan_points)
            timed = LaserScan()
            timed.header.stamp = (
                self.scan_start_time.to_msg()
                if self.scan_start_time is not None
                else now.to_msg()
            )
            timed.header.frame_id = self.frame_id
            start_angle_deg = self.timed_scan_points[0][0]
            timed.angle_min = math.radians(start_angle_deg)
            timed.angle_increment = math.copysign(
                2.0 * math.pi / timed_count, self.scan_angle_sign)
            timed.angle_max = (
                timed.angle_min + (timed_count - 1) * timed.angle_increment)
            # Samples occupy [scan_start, next_scan_start). Dividing by N-1
            # incorrectly makes the final ray equal to the next scan timestamp,
            # which Cartographer rejects as non-increasing sensor data.
            timed.time_increment = scan_time / timed_count
            timed.scan_time = scan_time
            timed.range_min = msg.range_min
            timed.range_max = msg.range_max
            timed.ranges = [point[1] for point in self.timed_scan_points]
            timed.intensities = [point[2] for point in self.timed_scan_points]

            self.timed_scan_pub.publish(timed)

        # Log periodically
        if self.frame_count % 500 == 0:
            speed_hz = average_speed_raw / 360.0 if average_speed_raw else 0
            self.get_logger().info(
                f"Published scan: {valid_count}/{len(self.scan_points)} valid points, "
                f"num_points={num_points}, inc={ANGLE_INCREMENT_DEG:.3f}°, "
                f"frames={self.frame_count}, "
                f"CRC ok={self.crc_ok_count}, err={self.crc_error_count}, "
                f"speed={speed_hz:.1f}Hz, scan_time={scan_time:.3f}s, "
                f"timed_points={len(self.timed_scan_points)}, clock=device, "
                f"device_lag={self.device_clock_lag_ms:.2f}ms, "
                f"zero_dist={self.zero_distance_count}, "
                f"invalid={self.invalid_point_count}"
            )

        # Clear for next scan cycle
        self.scan_points.clear()
        self.timed_scan_points.clear()

    def destroy_node(self):
        """Clean up serial port on shutdown."""
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            if rclpy.ok():
                self.get_logger().info("Serial port closed")
        super().destroy_node()



def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Jazzy may invalidate the shared launch context before spin() wakes.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
