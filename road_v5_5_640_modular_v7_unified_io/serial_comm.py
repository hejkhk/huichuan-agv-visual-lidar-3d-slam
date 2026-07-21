"""
serial_comm.py

统一下位机通信层。

V7 改动：
    1. 默认使用 AA55 二进制协议，和激光雷达底盘控制代码 uart_control.py 对齐。
    2. 保留 text_debug 协议，方便旧 STM32 或终端调试。
    3. main.py 仍然调用 build_serial_command(...) / send_command_if_needed(...)，
       这样上层视觉/避障逻辑不用一次性大改。

AA55 下行帧：
    AA 55 cmd spd0 spd1 spd2 spd3 checksum
    cmd: 0x01 MOVE, 0x02 STOP, 0x03 ESTOP, 0x04 PS2
    spd0~spd3: int32 little-endian，单位沿用 STM32/汇川伺服当前 cnt/s 写法。
"""

from __future__ import annotations

import time
import struct
import threading
from collections import deque
from dataclasses import dataclass
from typing import Optional, List

from config_switches import *
from calibration_640 import INVALID_ERROR

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None

FRAME_HDR1 = 0xAA
FRAME_HDR2 = 0x55
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
_rx_monitor_running = False
_rx_monitor_thread = None
_rx_monitor_lock = threading.Lock()
_rx_monitor_lines = deque(maxlen=80)
_rx_monitor_count = 0
_rx_raw_count = 0
_tx_status_lock = threading.Lock()
_latest_tx_status = {
    "has_tx": False,
    "updated_at": 0.0,
    "command": "",
    "frame": "",
    "status": "none",
    "protocol": "",
}


@dataclass
class OutboundCommand:
    """同时保存“给 STM32 的真实 payload”和“给人看的调试文本”。"""
    protocol: str
    payload: bytes
    text: str
    cmd_code: int = 0
    wheel_speeds: Optional[List[int]] = None

    def strip(self) -> str:
        return self.text.strip()

    def __str__(self) -> str:
        return self.text


def _auto_detect_port() -> Optional[str]:
    if serial is None:
        return None
    ports = serial.tools.list_ports.comports()
    candidates = []
    for p in ports:
        desc = (p.description or "").lower()
        dev = p.device
        if any(kw in desc for kw in ["stlink", "stm32", "ch340", "ch341", "cp210", "ftdi", "pl2303", "usb serial", "uart", "serial"]):
            candidates.append(dev)
        if "ttyAMA" in dev or "ttyS" in dev:
            candidates.append(dev)
    if not candidates and ports:
        candidates = [ports[0].device]
    candidates = list(dict.fromkeys(candidates))
    if not candidates:
        return None
    if len(candidates) > 1:
        print("找到多个串口候选：")
        for i, p in enumerate(candidates):
            print(f"  [{i}] {p}")
        print("默认使用第 0 个；如不对，请在 config_switches.py 里写死 SERIAL_PORT。")
    return candidates[0]


def open_serial_port():
    """
    打开树莓派到 STM32 的串口。
    """
    if not ENABLE_SERIAL_SEND:
        print("ℹ️ 当前为本地调试模式：不向 STM32 发送串口数据")
        return None

    if serial is None:
        print("❌ 未安装 pyserial，无法打开串口。可执行：sudo apt install python3-serial")
        return None

    port = SERIAL_PORT
    if SERIAL_AUTO_DETECT or str(SERIAL_PORT).lower() == "auto":
        detected = _auto_detect_port()
        if detected:
            port = detected

    try:
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = BAUD_RATE
        ser.bytesize = serial.EIGHTBITS
        ser.parity = serial.PARITY_NONE
        ser.stopbits = serial.STOPBITS_ONE
        ser.timeout = 0.1
        ser.rtscts = False
        ser.dsrdtr = False
        ser.dtr = False
        ser.rts = False
        ser.open()
        print(f"✅ 成功连接 STM32: {port}, baud={BAUD_RATE}, protocol={SERIAL_PROTOCOL}")
        if SERIAL_CLEAR_BUFFERS_ON_OPEN:
            time.sleep(float(SERIAL_CLEAR_SETTLE_SEC))
            try:
                ser.reset_input_buffer()
                ser.reset_output_buffer()
            except Exception:
                pass
            time.sleep(float(SERIAL_CLEAR_SETTLE_SEC))
            try:
                ser.reset_input_buffer()
                ser.reset_output_buffer()
            except Exception:
                pass
            print("🧹 串口输入/输出缓冲已清空")
        else:
            time.sleep(0.5)
        # 打开串口后先发 STOP，防止上一次残留速度。
        stop_cmd = OutboundCommand(
            "binary_aa55",
            _make_aa55_frame(CTRL_CMD_STOP, [0, 0, 0, 0]),
            "AA55 INIT STOP raw=[0, 0, 0, 0]",
            CTRL_CMD_STOP,
            [0, 0, 0, 0],
        )
        if ENABLE_SERIAL_INIT_STOP:
            print(stop_cmd.strip())
            tx_frame = _format_tx_payload(stop_cmd, actually_sent=True)
            print(tx_frame)
            _update_latest_tx_status(stop_cmd, tx_frame, actually_sent=True)
            ser.write(stop_cmd.payload)
        if bool(globals().get("SERIAL_ECHO_ON_OPEN", False)):
            echo_cmd = OutboundCommand(
                "binary_aa55",
                _make_aa55_frame(CTRL_CMD_ECHO_ON, [0, 0, 0, 0]),
                "AA55 ECHO_ON raw=[0, 0, 0, 0]",
                CTRL_CMD_ECHO_ON,
                [0, 0, 0, 0],
            )
            print(echo_cmd.strip())
            tx_frame = _format_tx_payload(echo_cmd, actually_sent=True)
            print(tx_frame)
            _update_latest_tx_status(echo_cmd, tx_frame, actually_sent=True)
            ser.write(echo_cmd.payload)
        return ser
    except Exception as e:
        print(f"❌ 串口连接失败: {e}")
        return None


# ──────────────────────────────────────────
# AA55 二进制协议
# ──────────────────────────────────────────

def _make_aa55_frame(cmd: int, speeds: List[int]) -> bytes:
    speeds = [int(max(-2_147_483_648, min(2_147_483_647, s))) for s in speeds]
    body = struct.pack("<BBB4i", FRAME_HDR1, FRAME_HDR2, int(cmd) & 0xFF, *speeds)
    checksum = sum(body) & 0xFF
    return body + bytes([checksum])


def _payload_hex(payload: bytes) -> str:
    return " ".join(f"{b:02X}" for b in payload)


def _format_tx_payload(cmd: OutboundCommand, actually_sent: bool) -> str:
    status = "sent" if actually_sent else "dry-run"
    payload = cmd.payload or b""
    if cmd.protocol == "binary_aa55" and len(payload) >= 20:
        checksum = payload[-1]
        return (
            f"TX_FRAME status={status} protocol={cmd.protocol} len={len(payload)} "
            f"hdr={payload[0]:02X} {payload[1]:02X} cmd=0x{payload[2]:02X} "
            f"order=RF,LF,RR,LR speeds={cmd.wheel_speeds} checksum=0x{checksum:02X} "
            f"hex={_payload_hex(payload)}"
        )
    return (
        f"TX_FRAME status={status} protocol={cmd.protocol} len={len(payload)} "
        f"hex={_payload_hex(payload)}"
    )


def _update_latest_tx_status(cmd: OutboundCommand, tx_frame: str, actually_sent: bool):
    with _tx_status_lock:
        _latest_tx_status.update(
            {
                "has_tx": True,
                "updated_at": time.time(),
                "command": cmd.strip(),
                "frame": tx_frame,
                "status": "sent" if actually_sent else "dry-run",
                "protocol": cmd.protocol,
            }
        )


def get_latest_tx_status():
    with _tx_status_lock:
        data = dict(_latest_tx_status)
    now = time.time()
    data["age_sec"] = round(now - float(data.get("updated_at") or now), 3)
    return data


def _checksum_ok(frame: bytes) -> bool:
    return len(frame) >= 2 and ((sum(frame[:-1]) & 0xFF) == frame[-1])


def _format_rx_frame(frame: bytes, frame_type: str = "AA55") -> str:
    global _rx_monitor_count
    _rx_monitor_count += 1
    calc = sum(frame[:-1]) & 0xFF if len(frame) >= 2 else 0
    recv = frame[-1] if frame else 0
    ok = "OK" if calc == recv else "BAD"
    cmd_text = f"cmd=0x{frame[2]:02X}" if len(frame) > 2 else "cmd=NA"
    return (
        f"RX#{_rx_monitor_count:05d} {frame_type} {ok} len={len(frame)} hdr={frame[0]:02X} {frame[1]:02X} "
        f"{cmd_text} checksum=0x{recv:02X} calc=0x{calc:02X} hex={_payload_hex(frame)}"
    )


def _format_rx_ctrl20(frame: bytes) -> str:
    cmd = frame[CTRL_OFFSET_CMD] if len(frame) > CTRL_OFFSET_CMD else 0
    if cmd == CTRL_CMD_NAVI and len(frame) >= CTRL_FRAME_LEN:
        yaw_raw, vx_raw, vz_raw, _ = struct.unpack_from("<4i", frame, CTRL_OFFSET_SPD)
        return _format_rx_frame(
            frame,
            f"NAVI_AA55_20B yaw={yaw_raw / 100.0:+.2f}deg "
            f"vx={vx_raw / 1000.0:+.3f}m/s vz={vz_raw / 100.0:+.2f}deg/s",
        )

    name = {
        CTRL_CMD_MOVE: "ECHO_MOVE_AA55_20B",
        CTRL_CMD_STOP: "ECHO_STOP_AA55_20B",
        CTRL_CMD_ESTOP: "ECHO_ESTOP_AA55_20B",
        CTRL_CMD_PS2: "ECHO_PS2_AA55_20B",
        CTRL_CMD_ECHO_ON: "ECHO_ON_ACK_AA55_20B",
        CTRL_CMD_ECHO_OFF: "ECHO_OFF_ACK_AA55_20B",
        CTRL_CMD_MAPPING: "ECHO_MAPPING_AA55_20B",
    }.get(cmd, f"AA55_20B_CMD_0x{cmd:02X}")
    return _format_rx_frame(frame, name)


def _format_rx_text(data: bytes) -> str:
    global _rx_raw_count
    _rx_raw_count += 1
    text = data.decode("ascii", errors="replace").strip()
    return f"RX_TEXT#{_rx_raw_count:05d} {text}"


def _try_pop_known_rx_frame(buf: bytearray):
    if len(buf) < 2:
        return None

    if buf[0] == 0xAA and buf[1] == 0x56:
        frame_len = 23
        if len(buf) < frame_len:
            return None
        frame = bytes(buf[:frame_len])
        del buf[:frame_len]
        return _format_rx_frame(frame, "IMU_AA56")

    if buf[0] == 0xAA and buf[1] == 0x55:
        candidates = [
            (20, "CTRL_AA55_20B"),
            (35, "ENC_AA55_35B"),
        ]
        for frame_len, name in candidates:
            if len(buf) >= frame_len:
                frame = bytes(buf[:frame_len])
                if _checksum_ok(frame):
                    del buf[:frame_len]
                    if frame_len == CTRL_FRAME_LEN:
                        return _format_rx_ctrl20(frame)
                    return _format_rx_frame(frame, name)

        max_len = max(frame_len for frame_len, _ in candidates)
        if len(buf) < max_len:
            return None

        frame_len = max(4, int(SERIAL_RX_FRAME_LEN))
        if len(buf) < frame_len:
            return None
        frame = bytes(buf[:frame_len])
        del buf[:frame_len]
        return _format_rx_frame(frame, "AA55_UNKNOWN")

    return None


def _format_rx_raw(data: bytes) -> str:
    global _rx_raw_count
    _rx_raw_count += 1
    return f"RX_RAW#{_rx_raw_count:05d} len={len(data)} hex={_payload_hex(data)}"


def _format_rx_chunk(data: bytes) -> str:
    global _rx_raw_count
    _rx_raw_count += 1
    return f"RX_CHUNK#{_rx_raw_count:05d} len={len(data)} hex={_payload_hex(data)}"


def _format_rx_zero_test(data: bytes) -> str:
    global _rx_raw_count
    _rx_raw_count += 1
    return f"RX_ZERO20#{_rx_raw_count:05d} len={len(data)} hex={_payload_hex(data)}"


def _append_rx_line(line: str):
    with _rx_monitor_lock:
        _rx_monitor_lines.append(line)


def _serial_rx_worker(ser):
    global _rx_monitor_running
    buf = bytearray()
    raw_discard = bytearray()
    while _rx_monitor_running:
        try:
            data = ser.read(max(1, getattr(ser, "in_waiting", 0)))
        except Exception as e:
            _append_rx_line(f"RX_ERROR {e}")
            time.sleep(0.05)
            continue

        if data:
            if SERIAL_RX_SHOW_READ_CHUNKS:
                _append_rx_line(_format_rx_chunk(data))
            buf.extend(data)
        else:
            time.sleep(0.005)
            continue

        while len(buf) >= 2:
            parsed_line = _try_pop_known_rx_frame(buf)
            if parsed_line is not None:
                if raw_discard:
                    _append_rx_line(_format_rx_raw(bytes(raw_discard)))
                    raw_discard.clear()
                _append_rx_line(parsed_line)
                continue

            if buf[0] == FRAME_HDR1 and buf[1] in (0x55, 0x56):
                break

            if buf[0] == FRAME_HDR1:
                if SERIAL_RX_SHOW_RAW_BYTES:
                    raw_discard.append(buf[0])
                del buf[:1]
                continue

            if buf[0] not in (FRAME_HDR1, 0x0A, 0x0D):
                if SERIAL_RX_SHOW_RAW_BYTES:
                    raw_discard.append(buf[0])
                    zero_len = max(1, int(SERIAL_RX_ZERO_TEST_FRAME_LEN))
                    while (
                        SERIAL_RX_GROUP_ZERO_TEST_FRAME
                        and len(raw_discard) >= zero_len
                        and all(b == 0x00 for b in raw_discard[:zero_len])
                    ):
                        _append_rx_line(_format_rx_zero_test(bytes(raw_discard[:zero_len])))
                        del raw_discard[:zero_len]
                    if len(raw_discard) >= max(16, zero_len):
                        _append_rx_line(_format_rx_raw(bytes(raw_discard)))
                        raw_discard.clear()
                del buf[:1]
                continue

            if buf[0] in (0x0A, 0x0D):
                if raw_discard:
                    _append_rx_line(_format_rx_text(bytes(raw_discard)))
                    raw_discard.clear()
                del buf[:1]
                continue


def start_serial_rx_monitor(ser):
    global _rx_monitor_running, _rx_monitor_thread, _rx_monitor_count, _rx_raw_count
    if not ENABLE_SERIAL_RX_MONITOR or ser is None:
        if ENABLE_SERIAL_RX_MONITOR and ser is None:
            print("⚠️ 串口回传监视器未启动：ser=None，请确认 ENABLE_SERIAL_SEND=True 且串口已打开")
        return
    if _rx_monitor_thread is not None and _rx_monitor_thread.is_alive():
        return
    _rx_monitor_count = 0
    _rx_raw_count = 0
    with _rx_monitor_lock:
        _rx_monitor_lines.clear()
        _rx_monitor_lines.append("Waiting for STM32 serial echo bytes / frames...")
    _rx_monitor_running = True
    _rx_monitor_thread = threading.Thread(target=_serial_rx_worker, args=(ser,), daemon=True)
    _rx_monitor_thread.start()
    print("✅ 串口回传监视器已启动：AA55 RX window")


def stop_serial_rx_monitor():
    global _rx_monitor_running, _rx_monitor_thread
    _rx_monitor_running = False
    if _rx_monitor_thread is not None:
        _rx_monitor_thread.join(timeout=0.2)
    _rx_monitor_thread = None


def update_serial_rx_window():
    if not ENABLE_SERIAL_RX_MONITOR:
        return
    try:
        import cv2 as cv
        import numpy as np
    except Exception:
        return

    with _rx_monitor_lock:
        lines = list(_rx_monitor_lines)[-max(1, int(SERIAL_RX_WINDOW_LINES)):]

    width = 1280
    line_h = 22
    height = max(180, 34 + line_h * max(1, int(SERIAL_RX_WINDOW_LINES)))
    img = np.zeros((height, width, 3), dtype=np.uint8)
    cv.putText(img, "STM32 Serial RX Frames", (12, 24), cv.FONT_HERSHEY_SIMPLEX, 0.65, (0, 220, 255), 2)
    y = 54
    for line in lines:
        color = (80, 255, 120) if " OK " in f" {line} " else (80, 120, 255) if " BAD " in f" {line} " else (220, 220, 220)
        cv.putText(img, line[:155], (12, y), cv.FONT_HERSHEY_SIMPLEX, 0.48, color, 1)
        y += line_h
    cv.imshow("STM32 AA55 RX", img)


def _limit_speed(x: int) -> int:
    return int(max(-BINARY_MAX_WHEEL_CNT, min(BINARY_MAX_WHEEL_CNT, x)))


def _raw_from_physical(left_cnt: int, right_cnt: int) -> List[int]:
    """
    输入物理左右侧速度：正数=车轮物理前进。
    输出 STM32 raw 四轮速度，符号遵守 MOTOR_SIGN。
    """
    left_cnt = _limit_speed(left_cnt)
    right_cnt = _limit_speed(right_cnt)
    return [
        _limit_speed(-right_cnt),  # 右前
        _limit_speed(left_cnt),    # 左前
        _limit_speed(-right_cnt),  # 右后
        _limit_speed(left_cnt),    # 左后
    ]


def _weighted_error(errors) -> Optional[float]:
    # 近处扫描线权重大，远处权重小。
    weights = [0.36, 0.26, 0.18, 0.12, 0.08]
    total = 0.0
    wsum = 0.0
    for i, e in enumerate(errors):
        if e == INVALID_ERROR:
            continue
        w = weights[i] if i < len(weights) else 0.05
        total += float(e) * w
        wsum += w
    if wsum <= 1e-6:
        return None
    return total / wsum


def _vision_to_binary_command(errors, mode, obs_flag, nearest_dist, avoid_dir) -> OutboundCommand:
    """
    过渡方案：把旧视觉 final_errors/mode 转成 AA55 四轮速度。

    注意：
        这只是为了让旧视觉避障先能驱动现在的二进制底盘协议。
        后面激光雷达规划接入后，更推荐直接由 /cmd_vel_safe -> 四轮速度。
    """
    mode_name = {
        MODE_TRACE: "TRACE",
        MODE_STOP: "STOP",
        MODE_AVOID_LEFT: "AVOID_LEFT",
        MODE_AVOID_RIGHT: "AVOID_RIGHT",
        MODE_LINE_LOST: "LINE_LOST",
        MODE_SPIN_LEFT: "SPIN_LEFT",
        MODE_SPIN_RIGHT: "SPIN_RIGHT",
    }.get(mode, f"MODE_{mode}")

    # 停车/丢线：先用正常 STOP，不直接 ESTOP。
    if mode in (MODE_STOP, MODE_LINE_LOST):
        frame = _make_aa55_frame(CTRL_CMD_STOP, [0, 0, 0, 0])
        return OutboundCommand("binary_aa55", frame, f"AA55 STOP mode={mode_name} obs={obs_flag} dist={nearest_dist}", CTRL_CMD_STOP, [0,0,0,0])

    # 原地找空路：直接左右侧反向转。
    if mode == MODE_SPIN_LEFT:
        speeds = _raw_from_physical(-BINARY_SPIN_CNT, BINARY_SPIN_CNT)
        frame = _make_aa55_frame(CTRL_CMD_MOVE, speeds)
        return OutboundCommand("binary_aa55", frame, f"AA55 MOVE {mode_name} spd={speeds}", CTRL_CMD_MOVE, speeds)

    if mode == MODE_SPIN_RIGHT:
        speeds = _raw_from_physical(BINARY_SPIN_CNT, -BINARY_SPIN_CNT)
        frame = _make_aa55_frame(CTRL_CMD_MOVE, speeds)
        return OutboundCommand("binary_aa55", frame, f"AA55 MOVE {mode_name} spd={speeds}", CTRL_CMD_MOVE, speeds)

    base = BINARY_TRACE_BASE_CNT
    turn = 0

    if mode == MODE_AVOID_LEFT:
        base = BINARY_AVOID_BASE_CNT
        turn = BINARY_AVOID_TURN_CNT       # turn>0：左侧慢、右侧快，车左转
    elif mode == MODE_AVOID_RIGHT:
        base = BINARY_AVOID_BASE_CNT
        turn = -BINARY_AVOID_TURN_CNT      # turn<0：左侧快、右侧慢，车右转
    else:
        err = _weighted_error(errors)
        if err is not None:
            # 旧约定：error=center_x-BIRD_WIDTH/2；正数表示线在右边。
            # 线在右边时车应右转，所以 turn 取负。
            turn = int(-err * BINARY_KP_CNT_PER_PIXEL)

    left = int(base - turn)
    right = int(base + turn)
    speeds = _raw_from_physical(left, right)
    frame = _make_aa55_frame(CTRL_CMD_MOVE, speeds)

    err_text = _weighted_error(errors)
    text = f"AA55 MOVE mode={mode_name} e={err_text if err_text is not None else 'NA'} L={left} R={right} raw(RF,LF,RR,LR)={speeds}"
    if BINARY_PRINT_HEX:
        text += " hex=" + " ".join(f"{b:02X}" for b in frame)
    return OutboundCommand("binary_aa55", frame, text, CTRL_CMD_MOVE, speeds)


# ──────────────────────────────────────────
# 旧文本协议保留
# ──────────────────────────────────────────

def _build_text_command(errors, mode, obs_flag, nearest_dist, avoid_dir) -> OutboundCommand:
    text = (
        f"error1:{errors[0]}, "
        f"error2:{errors[1]}, "
        f"error3:{errors[2]}, "
        f"error4:{errors[3]}, "
        f"error5:{errors[4]}, "
        f"mode:{mode}, "
        f"obs:{obs_flag}, "
        f"dist:{nearest_dist}, "
        f"dir:{avoid_dir}\n"
    )
    return OutboundCommand("text_debug", text.encode("utf-8"), text)


def build_serial_command(errors, mode, obs_flag, nearest_dist, avoid_dir) -> OutboundCommand:
    """
    main.py 调用的统一构造函数。
    根据 SERIAL_PROTOCOL 选择输出 AA55 二进制帧或旧文本行。
    """
    if str(SERIAL_PROTOCOL).lower() in ("binary", "binary_aa55", "aa55"):
        return _vision_to_binary_command(errors, mode, obs_flag, nearest_dist, avoid_dir)
    return _build_text_command(errors, mode, obs_flag, nearest_dist, avoid_dir)


def send_command_if_needed(ser, cmd: OutboundCommand, last_send_time):
    """
    按固定频率向 STM32 发送串口指令。
    ser=None 时只打印，不真正发送。
    """
    now = time.time()
    if now - last_send_time < SEND_INTERVAL_SEC:
        return last_send_time

    print(cmd.strip())

    if ser is not None:
        try:
            tx_frame = _format_tx_payload(cmd, actually_sent=True)
            print(tx_frame)
            _update_latest_tx_status(cmd, tx_frame, actually_sent=True)
            ser.write(cmd.payload)
            return now
        except Exception as e:
            print(f"❌ 串口发送失败: {e}")
            return now

    tx_frame = _format_tx_payload(cmd, actually_sent=False)
    print(tx_frame)
    _update_latest_tx_status(cmd, tx_frame, actually_sent=False)
    return now
