"""
config_switches.py

作用：
    集中放所有“开关 / 阈值 / 相机参数 / 串口参数”。
    以后你想调功能开关、显示窗口、滤波参数、状态机参数，优先改这个文件。
"""

# ==============================
# 2. 全局参数配置区
#    以后你主要改这里
# ==============================

# ---------- 2.1 串口参数 ----------

SERIAL_PORT = "/dev/ttyUSB0"          # 树莓派常用 USB-TTL 串口；Windows 调试时可改成 "COM8"
BAUD_RATE = 115200                    # 串口波特率，必须和 STM32 的 USART1 波特率一致
SEND_INTERVAL_SEC = 1              # 调试时 20Hz 打印/发送；上车可改回 0.02=50Hz
ENABLE_SERIAL_SEND = False            # 融合导航模式下由 chassis_node 统一发底盘串口；手动单独调视觉 main.py 时再改 True
ENABLE_SERIAL_INIT_STOP = False       # True：串口刚打开时先发一帧 STOP；下位机调试回显时建议先关
SERIAL_ECHO_ON_OPEN = False           # True：串口打开后发送 0x05 ECHO_ON；默认关闭，避免连接时额外发控制帧
ENABLE_SERIAL_RX_MONITOR = True       # True：读取并单独窗口显示 STM32 回传的 AA55 串口帧
SERIAL_RX_FRAME_LEN = 20              # STM32 回传 AA55 帧长度；当前按下行 echo 的 20 字节解析
SERIAL_RX_WINDOW_LINES = 22           # 串口回传窗口最多显示多少行历史帧
SERIAL_RX_SHOW_RAW_BYTES = True       # True：不是 AA55 帧头的原始回传字节也显示，方便排查接线/协议问题
SERIAL_RX_SHOW_READ_CHUNKS = False    # True：显示每次 serial.read() 读到的原始数据块；会和解析帧重复显示
SERIAL_RX_GROUP_ZERO_TEST_FRAME = False # True：连续 20 个 0x00 作为 RX_ZERO20 测试帧显示
SERIAL_RX_ZERO_TEST_FRAME_LEN = 20    # 下位机链路测试：连续 20 个 0x00 作为一行显示


# ---------- 2.1.1 下位机统一通信协议 ----------
# 现在建议淘汰旧的 error1:... 文本协议，改成和激光雷达底盘代码一致的 AA55 二进制协议。
#   "binary_aa55"：树莓派 -> STM32：AA 55 cmd spd0 spd1 spd2 spd3 checksum，20字节。
#   "text_debug" ：保留旧文本协议，只用于旧 STM32 / 终端调试。
SERIAL_PROTOCOL = "binary_aa55"

# SERIAL_PORT 可以写死为 /dev/ttyUSB0，也可以设为 "auto" 自动找 STLink/CH340/CP210 等串口。
# 上车前建议先 ENABLE_SERIAL_SEND=False，确认终端打印的 AA55 速度方向没问题后再改 True。
SERIAL_AUTO_DETECT = True
SERIAL_CLEAR_BUFFERS_ON_OPEN = True   # True：串口打开后清空输入/输出缓冲，避免显示/处理旧数据
SERIAL_CLEAR_SETTLE_SEC = 0.2         # 清缓冲后等待时间，给 USB-UART/STM32 刚连接时的残留字节一点排空时间

# AA55 命令字，和现在激光雷达/底盘测试代码 uart_control.py 保持一致。
CTRL_CMD_MOVE  = 0x01
CTRL_CMD_STOP  = 0x02
CTRL_CMD_ESTOP = 0x03
CTRL_CMD_PS2   = 0x04
CTRL_CMD_ECHO_ON = 0x05
CTRL_CMD_ECHO_OFF = 0x06
CTRL_CMD_NAVI = 0x07
CTRL_CMD_MAPPING = 0x08

# 电机符号表：索引0=左前，1=右前，2=左后，3=右后。
# 这组符号来自你给的 uart_control.py：物理前进时 raw=[-s,+s,-s,+s]。
MOTOR_SIGN = [-1, 1, -1, 1]

# 下面这组是“视觉/避障旧算法 -> 二进制四轮速度”的过渡参数。
# 后续接激光雷达路径规划时，建议直接让 chassis_serial_node 接 /cmd_vel_safe。
BINARY_TRACE_BASE_CNT = 75_000_000      # 正常寻线前进基础速度
BINARY_AVOID_BASE_CNT = 45_000_000      # 绕障/恢复时基础速度，低于直行速度，避免贴近障碍时过猛
BINARY_SPIN_CNT = 50_000_000            # 原地转向速度，等价 uart_control.py 里的 TURN_SPD
BINARY_MAX_WHEEL_CNT = 100_000_000      # 单轮速度限幅，需高于基础速度+差速修正
BINARY_KP_CNT_PER_PIXEL = 190_000       # error 像素 -> 左右轮差速修正；调大转向更灵敏
BINARY_AVOID_TURN_CNT = 65_000_000      # AVOID_LEFT/RIGHT 时固定叠加的移动差速转向量
BINARY_PRINT_HEX = False                 # True：终端打印完整十六进制帧；False：打印简短速度



# ---------- 2.2 相机参数 ----------

# 相机输入后端：
#   "sdk" ：当前实车一键启动使用，直接用 pyorbbecsdk Pipeline() 打开 Gemini2，延迟更低。
#   "ros2"：兼容模式。先用 Orbbec ROS2 Wrapper 打开 Gemini2，本程序订阅 RGB + Depth topic。
CAMERA_BACKEND = "sdk"

FRAME_WIDTH = 1280                    # 1280x720 调试视野
FRAME_HEIGHT = 720                    # 1280x720 调试视野
FRAME_FPS = 30                        # 旧 SDK 后端请求 FPS；ROS2 后端实际 FPS 由 launch 文件决定

# ---------- 2.2.1 ROS2 RGB-D 图像订阅参数 ----------
# 使用方法：
#   1. 先另开终端启动 Orbbec ROS2 Wrapper：
#      ros2 launch orbbec_camera gemini2_rgbd_640.launch.py
#      或 ros2 launch orbbec_camera gemini2_rgbd_1280_full_test.launch.py
#   2. 再运行本 main.py。
#
# 注意：如果 ROS2 相机输出 1280x720，而这里 FRAME_WIDTH/FRAME_HEIGHT 还是 640x480，
# ROS2_FORCE_RESIZE_TO_FRAME_SIZE=True 会自动缩放到 640x480，保证旧标定代码能先跑起来。
# 真正做 1280 标定时，把它改 False，同时把 FRAME_WIDTH/FRAME_HEIGHT 和 calibration 改成 1280 版本。
ROS2_COLOR_TOPIC = "/camera/color/image_raw"
ROS2_DEPTH_TOPIC = "/camera/depth/image_raw"
ROS2_COLOR_INFO_TOPIC = "/camera/color/camera_info"
ROS2_DEPTH_INFO_TOPIC = "/camera/depth/camera_info"
ROS2_RESIZE_DEPTH_TO_COLOR = True              # depth 尺寸和 RGB 不一致时，先最近邻 resize 到 RGB 尺寸
ROS2_FORCE_RESIZE_TO_FRAME_SIZE = False        # 1280x720 调试时不强制缩回 640x480
ROS2_WAIT_FOR_NEW_DEPTH_FRAME = False          # 调试时不等新 depth，保持 RGB 窗口刷新
ROS2_FRAME_TIMEOUT_SEC = 1.0                   # 等待首帧 / 新 depth 的最长时间
ROS2_PRINT_INPUT_FPS = False                   # 每隔一段时间打印 ROS2 输入 RGB/Depth 实际 FPS
ROS2_FPS_REPORT_INTERVAL_SEC = 2.0

# [V4.8-新增] 鸟瞰图控制尺寸。
# 这里不是裁剪视野，而是把 1280x720 原图中的巡线梯形区域 warp 成 640x480。
# 好处：原始视野保留，HSV/mask/扫描线计算量明显下降，串口 error 数值也更接近早期 640x480 版本。
BIRD_WIDTH = 640                      # 鸟瞰图宽度：640x480 版本直接保持 640
BIRD_HEIGHT = 480                     # 鸟瞰图高度：640x480 版本直接保持 480

# [V4.7/V4.8-新增] 显示与帧率配置
SHOW_RGB_WINDOW = True                # 是否显示 RGB + Depth + Obstacle ROI 主窗口
SHOW_BIRD_WINDOW = True               # 是否显示 Bird's Eye View 鸟瞰图窗口
SHOW_MASK_WINDOW = True              # 是否显示 MASK 窗口；默认关掉以提高 FPS，需要调 HSV 时再打开
TEXT_EVERY_N_FRAMES = 5               # [V4.8.1-修正] 画面每帧刷新；只有文字/标签每 5 帧绘制一次，减少 putText 开销

# ---------- 2.2.2 Web MJPEG 图像输出 ----------
# 开启后，main.py 会在 0.0.0.0:8080 发布 /video_feed，网页填树莓派 IP 即可查看。
ENABLE_WEB_VIDEO_STREAM = True
WEB_VIDEO_HOST = "0.0.0.0"
WEB_VIDEO_PORT = 8080
WEB_VIDEO_FPS = 12
WEB_VIDEO_JPEG_QUALITY = 75

SHOW_ROI_POLYGONS = True              # 是否绘制红/黄/绿 ROI 边框
SHOW_OBSTACLE_FILL = True             # 调试阈值时显示障碍填充
SHOW_STATUS_TEXT = True               # 是否显示 MODE/OBS/BASELINE/串口命令等状态文字
SHOW_ZONE_DEBUG_TEXT = False           # 调试 ROI 时显示区域统计文字
SHOW_MOUSE_DEPTH = False               # 鼠标 XYZ 依赖 camera_info，调点位时打开
SHOW_FPS = False                       # 是否在 RGB 调试窗口右上角显示当前处理帧率
FPS_EMA_ALPHA = 0.20                  # FPS 平滑系数；越小越稳，越大越跟手；0.2 适合观察整体性能
FPS_TEXT_MARGIN = 16                  # FPS 文字距离窗口右边缘/上边缘的像素距离

# ---------- 2.x 性能诊断配置 ----------
# [V4.9-新增] 性能诊断开关。
# 这个版本不是为了增加功能，而是为了找出 FPS 卡在哪里：
#     camera：相机取帧 + SDK D2C 对齐 + numpy reshape
#     depth_filter：深度时间滤波
#     line：鸟瞰图 + HSV + mask + 5 条扫描线
#     baseline：baseline 采集/中位数生成
#     obstacle：9 个 ROI 的深度障碍统计
#     decision：mode 决策 + 绕障偏置 + 串口命令生成/发送
#     display：画面复制/画框/文字/imshow
#     waitkey：OpenCV waitKey
PROFILE_MODE = True                  # True：每隔 PROFILE_PRINT_EVERY_N_FRAMES 帧打印一次模块耗时
PROFILE_PRINT_EVERY_N_FRAMES = 30    # 每多少帧打印一次平均耗时
CAMERA_ONLY_TEST = False             # True：只测相机取帧和显示，不跑巡线/避障/串口，用来判断相机本身极限 FPS


# ---------- 2.3 HSV 蓝色胶带默认阈值 ----------

HSV_DEFAULT = {
    "H_min": 100,                     # H 色相下限：蓝色通常在 100 左右开始
    "H_max": 124,                     # H 色相上限：蓝色通常到 124 左右
    "S_min": 70,                      # [V5.5.1-改动] 提高饱和度下限，减少灰地面/阴影被当蓝线
    "S_max": 255,                     # S 饱和度上限：最大值
    "V_min": 35,                      # [V5.5.1-改动] 亮度下限略放宽，暗处蓝胶带仍能识别
    "V_max": 255,                     # V 亮度上限：最大值
}

# ---------- 2.3.1 [V5.5.2-新增] 巡线几何抗误识别过滤 ----------
# 这一版先不动避障深度逻辑，只解决“HSV 已经调好了，但反光/杂物仍被当成路线”的问题。
# 重点：这里不是写死“蓝色”。
# HSV 负责决定“你要找什么颜色”，下面这些过滤只看形状和连续性：
#   1. 连通域过滤：删除面积太小、高度太短的颜色碎片。
#   2. 扫描线连续段过滤：一行里先找一段一段连续的 mask，而不是直接 min/max。
#   3. 左右边界成对过滤：只接受间距合理、中心位置合理的一左一右边界。
#   4. 禁止最后退回整行 min/max，避免一个远处杂物把道路宽度拉偏。
#
# 说明：
#   颜色优势过滤默认关闭。
#   这个过滤不是 HSV 的替代品，而是 HSV 后面的“二次保险”：
#       HSV 先选出某种颜色；
#       颜色优势过滤再要求某个 B/G/R 通道明显强于另外两个通道。
#
#   COLOR_DOMINANCE_CHANNEL 的取值：
#       False：不启用颜色优势过滤，只用 HSV + 几何过滤。
#       0    ：B 通道优势，适合蓝色线/蓝胶带。
#       1    ：G 通道优势，适合绿色线。
#       2    ：R 通道优势，适合红色线。
#
#   注意：黄色、橙色、白色、黑色通常不适合用单通道优势过滤。
#   例如黄色是 R 和 G 都强，蓝色弱；黑色是三个通道都低。
#   遇到这类颜色，建议设为 False，只靠 HSV + 几何过滤。
COLOR_DOMINANCE_CHANNEL = False          # False/0/1/2：不启用/B/G/R 通道优势过滤
COLOR_DOMINANCE_MARGIN = 18              # 优势通道至少比另外两个通道大多少；误识别多就调大，暗处漏识别就调小
COLOR_MIN_ABSOLUTE_VALUE = 45            # 优势通道最低亮度，避免特别暗的噪点通过；太暗识别不到可调低

ENABLE_LINE_COMPONENT_FILTER = True      # True：删除小碎片/孤立反光块；这一步与颜色无关
LINE_MIN_COMPONENT_AREA = 160            # 连通域最小面积；误识别多就调到 220/300
LINE_MIN_COMPONENT_WIDTH = 3             # 连通域最小宽度，太细的噪声删除
LINE_MIN_COMPONENT_HEIGHT = 45           # 连通域最小高度；真实道路边界在鸟瞰图里应跨越较长距离

# [单条蓝色胶带巡线-回退] 删除 V5.5.3 的“双边界道路配对”扫描线逻辑。
# 现在扫描线恢复为旧版 min/max：
#     在这一行 mask 里找到所有目标颜色像素；
#     x_left  = 最左目标颜色像素；
#     x_right = 最右目标颜色像素；
#     center  = (x_left + x_right) / 2。
# 这样适合“一条蓝色胶带”的巡线模式。
# 抗小碎片仍然交给上面的 ENABLE_LINE_COMPONENT_FILTER 连通域过滤。



# ---------- 2.8 深度过滤参数 ----------
# 你现在遇到的关键问题：
#     反光地面会导致“空地面的 depth 数据丢失”。
#     如果 baseline 中某些地面像素是 0，传统 baseline 差分就无法比较。
# V4 的解决办法：
#     1. baseline 有效时，优先使用“当前深度比地面近很多”的逻辑。
#     2. baseline 无效但当前深度有效且距离很近时，启用“反光地面补偿候选点”。
#     3. 所有障碍都必须满足数量阈值，避免一个噪点就触发停车。

MIN_VALID_DEPTH_MM = 150                       # 小于这个值通常太近或不稳定，先过滤掉
MAX_VALID_DEPTH_MM = 3000                      # 大于这个值对小车避障意义不大，先过滤掉
BASELINE_NEARER_THAN_MM = 70                   # 当前深度比空地面近超过 70mm，认为是凸起障碍候选
# baseline 无效补偿现在默认启用，不再设置开关。
# 逻辑在 obstacle_vision.calculate_obstacle_stats_fast() 中固定执行：
#     baseline 有效：优先用 baseline 差分判断障碍。
#     baseline 无效：如果当前深度在该 ROI 阈值内，也作为低矮障碍候选。
MIN_OBSTACLE_PIXELS = 90                       # 检测到至少 90 个障碍像素才算有效；调大可降低误判灵敏度
MIN_OBSTACLE_RATIO = 0.012                     # 障碍像素 / 当前有效深度像素 的比例阈值；调大可减少零散噪点误判
MIN_OBSTACLE_AREA_RATIO = 0.0015               # 障碍像素 / ROI 总面积 的比例阈值，防止 valid_count 太少时误判
OBSTACLE_MORPH_KERNEL_SIZE = 3                 # 对障碍候选 mask 做 3x3 形态学开运算，去掉孤立噪点
BASELINE_CAPTURE_FRAME_COUNT = 20              # 按 b 后连续采 20 帧，用中位数生成更稳定的 baseline

# ---------- 2.9 深度抗抖滤波参数 ----------
# [V4.6-新增]
# 你现在遇到的现象是：某些像素的深度会在“有效值”和“void/0无效值”之间横跳。
# 这会导致同一个障碍物这一帧被识别，下一帧又消失，mode 跟着乱跳。
# 所以这里加两层滤波：
#     1. depth 像素级时间滤波：最近几帧刚刚有效过的点，当前变 0 时先用旧值顶一下。
#     2. ROI 决策级防抖：某个区域必须连续几帧检测到障碍，才真正变成“有障碍”。

# 深度时间滤波和 ROI 防抖现在固定启用，不再保留 True/False 旧逻辑开关。
# 原因：近地面避障依赖深度相机，depth 的 void/valid 抖动和 ROI 抖动必须默认处理。
DEPTH_EMA_ALPHA = 0.45                  # 新深度占比；越大越跟手，越小越平滑。0.35~0.60 通常合适
DEPTH_HOLD_MAX_FRAMES = 4               # 当前帧 depth 变 0 后，最多用旧有效深度顶住几帧
ZONE_ON_CONFIRM_FRAMES = 3              # 连续多少帧检测到障碍，才确认“真的有障碍”
ZONE_OFF_CONFIRM_FRAMES = 4             # 连续多少帧检测不到障碍，才确认“障碍消失”



# ---------- 2.10.1 避障 / 寻线策略参数 ----------
# 新逻辑只保留一种避障状态机，不再保留“退回 V5.1 单帧判断”的旧分支。
# 当前避障状态机：TRACE -> AVOID -> RETURN，同时加入 EMERGENCY_STOP -> SPIN_SEARCH。

ENABLE_LINE_FOLLOW = False               # True：启用蓝线寻线/巡线；False：关闭巡线，只保留深度近地面避障和调试显示
AVOID_BIAS_TARGET_PIXELS = 230          # 绕障最大目标偏置；越大绕得越明显
AVOID_BIAS_RAMP_PX_PER_SEC = 760.0      # 进入绕障时，偏置每秒最多增加多少像素
RETURN_BIAS_RAMP_PX_PER_SEC = 260.0     # 回线时，偏置每秒减少多少像素；越小越温柔，越大回线越快
AVOID_HOLD_TIME_SEC = 0.65              # 绿色远处障碍短暂消失后，继续保持绕障偏置的时间，防止车头一偏就误判安全
AVOID_MIN_ACTIVE_TIME_SEC = 0.35        # 避障至少持续这么久，才允许进入 RETURN，防止刚触发就退出
RETURN_FINISH_BIAS_PIXELS = 8           # 当前偏置绝对值小于这个值，认为已经回正
LINE_REACQUIRE_MIN_COUNT = 2            # 回线完成前，至少要有几条扫描线有效，才允许回 TRACE
LINE_CENTER_TOL_PIXELS = 180            # 回线完成前，加权 error 不能太离谱；太大说明线还没稳定回到视野中心
AVOID_LOST_LINE_GRACE_SEC = 0.25        # 绕障/回线时短暂丢线允许保持一点点时间，超过后进入 LINE_LOST 停车保护

BLOCKED_WAIT_BEFORE_SPIN_SEC = 2.0      # 到达红/黄警戒区后先紧急停车；等待这么久障碍仍存在，就原地转向找空路
BLOCKED_SPIN_DEFAULT_DIR = 1            # 原地转向默认方向：-1 左转，1 右转；左右同分或没有有效建议时使用
SPIN_SEARCH_ERROR_PIXELS = 220          # 如果旧 STM32 仍按 error 做控制，原地转向时给一个固定大误差帮助转向


# ---------- 2.10 运动模式编号 ----------
# 这个编号会通过串口发给 STM32。
# 注意：旧版 STM32 只解析 error1~error5，不解析 mode 也没关系。
# 后续你升级 STM32 代码时，可以用 mode 直接做状态机。

MODE_TRACE = 0                         # 正常巡线
MODE_STOP = 1                          # 障碍停车
MODE_AVOID_LEFT = 2                    # 向左绕障
MODE_AVOID_RIGHT = 3                   # 向右绕障
MODE_LINE_LOST = 4                     # 丢线找线
MODE_SPIN_LEFT = 5                    # 原地左转找空路 / 脱困
MODE_SPIN_RIGHT = 6                   # 原地右转找空路 / 脱困
