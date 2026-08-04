"""
navigation.py

作用：
    根据巡线误差和避障统计决定 mode/dir，并用新的避障状态机输出平滑绕障偏置。

本版整理重点：
    1. 删除“是否启用避障状态机”的旧分支，固定使用新状态机。
    2. 删除“所有车道都有障碍就停车”的思想，只有中间红/黄警戒区触发紧急停车。
    3. 紧急停车等待 BLOCKED_WAIT_BEFORE_SPIN_SEC 秒后，如果障碍仍在警戒区，就原地转向找空路。
    4. 增加 ENABLE_LINE_FOLLOW：可以关闭蓝线寻线，只保留近地面深度避障。
"""

from config_switches import *
from obstacle_vision import combine_stats, lane_obstacle_score
from calibration_640 import INVALID_ERROR


def count_valid_lines(errors):
    """
    统计当前 5 条巡线 error 中，有多少条是有效的。

    error = 999 表示这条扫描线没看到蓝线，所以不算有效。
    """
    return sum(1 for e in errors if e != INVALID_ERROR)


def weighted_line_error(errors):
    """
    计算一个简单的加权平均 error，用来判断“线有没有基本回到视野中心”。

    注意：真正 PID 加权在 STM32 端做。
    这里的加权只是 Python 端状态机判断是否可以退出 RETURN。
    """
    weights = [0.40, 0.25, 0.18, 0.11, 0.06]
    total = 0.0
    weight_sum = 0.0

    for e, w in zip(errors, weights):
        if e != INVALID_ERROR:
            total += float(e) * w
            weight_sum += w

    if weight_sum <= 0.0:
        return None

    return total / weight_sum


def ramp_towards(current, target, max_delta):
    """
    让 current 以不超过 max_delta 的速度靠近 target。

    这就是“偏置斜坡”：
        不是 0 -> 160 一帧完成，
        而是 0 -> 20 -> 40 -> ... 慢慢推过去。
    """
    if current < target:
        return min(current + max_delta, target)
    if current > target:
        return max(current - max_delta, target)
    return current


def choose_better_side(zone_stats):
    """
    根据左右三段 ROI 的障碍像素数量，选择更适合转向/绕障的一侧。

    返回：
        -1：左侧更适合。
         1：右侧更适合。

    注意：
        这里不会因为左右都有障碍就直接停车。
        停车只由中间红/黄警戒区触发，左右区域只参与“往哪边转”的选择。
    """
    left_path = combine_stats(
        [zone_stats["L_RED"], zone_stats["L_YELLOW"], zone_stats["L_GREEN"]],
        name="LEFT_PATH"
    )
    right_path = combine_stats(
        [zone_stats["R_RED"], zone_stats["R_YELLOW"], zone_stats["R_GREEN"]],
        name="RIGHT_PATH"
    )

    left_score = lane_obstacle_score([
        zone_stats["L_RED"], zone_stats["L_YELLOW"], zone_stats["L_GREEN"]
    ])
    right_score = lane_obstacle_score([
        zone_stats["R_RED"], zone_stats["R_YELLOW"], zone_stats["R_GREEN"]
    ])

    left_blocked = left_path["is_obstacle"]
    right_blocked = right_path["is_obstacle"]

    if (not left_blocked) and right_blocked:
        return -1
    if left_blocked and (not right_blocked):
        return 1

    if left_score < right_score:
        return -1
    if right_score < left_score:
        return 1

    return -1 if BLOCKED_SPIN_DEFAULT_DIR < 0 else 1


def get_obstacle_summary(zone_stats):
    """
    汇总所有 ROI 的障碍标志和最近障碍距离。

    obs_flag 只是告诉调试界面/下位机：当前视野里是否检测到障碍。
    它不等于停车命令；真正停车由 mode 决定。
    """
    all_stats = list(zone_stats.values())
    obstacle_distances = [
        s["min_depth"] for s in all_stats
        if s["is_obstacle"] and s["min_depth"] != 9999
    ]
    obs_flag = 1 if obstacle_distances else 0
    nearest_dist = min(obstacle_distances) if obstacle_distances else 999
    return obs_flag, nearest_dist


def decide_mode_and_direction(errors, zone_stats):
    """
    根据“红/黄/绿 × 左/中/右”九个区域决定当前瞬时运动建议。

    新规则：
        1. C_RED 或 C_YELLOW 有障碍：说明已经到达中间警戒线，输出 MODE_STOP。
           后续是否等待、是否原地转向，由 AvoidanceStateMachine 统一处理。
        2. C_GREEN 有障碍：还没到警戒线，不停车，选择更空的一侧提前绕。
        3. 左右三条路都有障碍也不直接停车，只有中间红/黄警戒区才有停车优先级。
        4. 如果 ENABLE_LINE_FOLLOW=False，关闭蓝线寻线，不再因为 error 全 999 输出 LINE_LOST。
    """
    valid_line_count = count_valid_lines(errors)
    obs_flag, nearest_dist = get_obstacle_summary(zone_stats)
    better_dir = choose_better_side(zone_stats)

    center_danger = (
        zone_stats["C_RED"]["is_obstacle"]
        or zone_stats["C_YELLOW"]["is_obstacle"]
    )

    center_far = zone_stats["C_GREEN"]["is_obstacle"]

    # ============================================================
    # 第一优先级：中间红/黄警戒区触发紧急停车。
    # ============================================================
    if center_danger:
        return MODE_STOP, better_dir, obs_flag, nearest_dist

    # ============================================================
    # 第二优先级：中间绿色远处观察区触发提前绕障。
    # ============================================================
    if center_far:
        if better_dir < 0:
            return MODE_AVOID_LEFT, -1, obs_flag, nearest_dist
        return MODE_AVOID_RIGHT, 1, obs_flag, nearest_dist

    # ============================================================
    # 第三优先级：寻线开关打开时，蓝线全丢才进入 LINE_LOST。
    # ============================================================
    if ENABLE_LINE_FOLLOW and valid_line_count == 0:
        return MODE_LINE_LOST, 0, obs_flag, nearest_dist

    return MODE_TRACE, 0, obs_flag, nearest_dist


class AvoidanceStateMachine:
    """
    避障记忆状态机。

    新版只保留这一套逻辑，不再保留“退回旧版单帧判断”的分支。

    状态含义：
        TRACE：正常巡线/正常运行。
        AVOID：远处绿色区看到障碍，锁定方向，偏置逐渐增大。
        RETURN：远处障碍消失，偏置逐渐回 0。
        EMERGENCY_STOP：中间红/黄警戒区有障碍，先紧急停车等待。
        SPIN_SEARCH：等待超时后障碍仍在警戒区，原地转向，直到警戒区清空。
    """

    STATE_TRACE = "TRACE"
    STATE_AVOID = "AVOID"
    STATE_RETURN = "RETURN"
    STATE_EMERGENCY_STOP = "EMERGENCY_STOP"
    STATE_SPIN_SEARCH = "SPIN_SEARCH"

    def __init__(self):
        self.state = self.STATE_TRACE              # 当前内部状态
        self.locked_dir = 0                        # 锁定绕障/转向方向：-1 左，1 右，0 无
        self.current_bias = 0.0                    # 当前实际施加到 error 上的偏置，带斜坡
        self.last_update_time = None               # 上一次更新时间，用来算 dt
        self.avoid_start_time = 0.0                # 本次绕障开始时间
        self.stop_start_time = 0.0                 # 本次紧急停车开始时间
        self.last_center_obstacle_time = -999.0    # 最近一次看到中间绿色障碍的时间
        self.last_line_valid_time = 0.0            # 最近一次看到有效蓝线的时间

    def reset(self):
        """清空状态机记忆。按 r 重新标定/清空滤波时调用。"""
        self.state = self.STATE_TRACE
        self.locked_dir = 0
        self.current_bias = 0.0
        self.last_update_time = None
        self.avoid_start_time = 0.0
        self.stop_start_time = 0.0
        self.last_center_obstacle_time = -999.0
        self.last_line_valid_time = 0.0

    def _dt(self, now):
        """计算两次 update 之间的时间差，并做限幅防止偶发卡顿导致 bias 跳太大。"""
        if self.last_update_time is None:
            self.last_update_time = now
            return 0.02

        dt = now - self.last_update_time
        self.last_update_time = now

        if dt < 0.0:
            dt = 0.0
        if dt > 0.10:
            dt = 0.10
        return dt

    def _spin_output(self):
        """根据 locked_dir 输出原地转向 mode。"""
        if self.locked_dir < 0:
            return MODE_SPIN_LEFT, -1, -SPIN_SEARCH_ERROR_PIXELS, self.state
        return MODE_SPIN_RIGHT, 1, SPIN_SEARCH_ERROR_PIXELS, self.state

    def update(self, instant_mode, instant_dir, errors, now):
        """
        根据当前帧的“瞬时视觉判断”更新状态机。

        参数：
            instant_mode：decide_mode_and_direction() 得到的单帧 mode。
            instant_dir ：单帧建议绕障/转向方向，-1 左，1 右。
            errors      ：未加偏置的 error1~error5。
            now         ：当前时间戳，time.perf_counter()。

        返回：
            output_mode：真正发给下位机/调试界面的 mode。
            output_dir ：真正发给下位机/调试界面的 dir。
            bias       ：当前要加到 error 上的偏置。
            state_name ：内部状态名，方便调试。
        """
        dt = self._dt(now)
        valid_count = count_valid_lines(errors)
        line_error = weighted_line_error(errors)

        if valid_count > 0:
            self.last_line_valid_time = now

        # ============================================================
        # 最高优先级：红/黄警戒区触发紧急停车。
        # 先停住，不立刻乱转；等待 BLOCKED_WAIT_BEFORE_SPIN_SEC 后如果还堵，再原地转向。
        # ============================================================
        if instant_mode == MODE_STOP:
            if self.state not in (self.STATE_EMERGENCY_STOP, self.STATE_SPIN_SEARCH):
                self.state = self.STATE_EMERGENCY_STOP
                self.stop_start_time = now
                self.current_bias = 0.0
                self.locked_dir = instant_dir if instant_dir in (-1, 1) else (1 if BLOCKED_SPIN_DEFAULT_DIR >= 0 else -1)

            if self.state == self.STATE_EMERGENCY_STOP:
                if (now - self.stop_start_time) < BLOCKED_WAIT_BEFORE_SPIN_SEC:
                    return MODE_STOP, 0, 0, self.state
                self.state = self.STATE_SPIN_SEARCH

            # SPIN_SEARCH：警戒区仍然有障碍，就持续原地转向。
            return self._spin_output()

        # 如果上一帧正在紧急停车/原地转向，但这一帧中间警戒区已经清空，则退出脱困状态。
        if self.state in (self.STATE_EMERGENCY_STOP, self.STATE_SPIN_SEARCH):
            self.state = self.STATE_TRACE
            self.current_bias = 0.0
            self.locked_dir = 0

        # ============================================================
        # 寻线丢失保护。寻线开关关闭时，不因为 error 全 999 停车。
        # ============================================================
        if ENABLE_LINE_FOLLOW and valid_count == 0:
            if self.state in (self.STATE_AVOID, self.STATE_RETURN):
                if (now - self.last_line_valid_time) <= AVOID_LOST_LINE_GRACE_SEC:
                    pass
                else:
                    self.state = self.STATE_TRACE
                    self.locked_dir = 0
                    self.current_bias = 0.0
                    return MODE_LINE_LOST, 0, 0, self.state
            else:
                self.current_bias = 0.0
                return MODE_LINE_LOST, 0, 0, self.state

        # ============================================================
        # 绿色远处观察区有障碍：提前绕障。
        # ============================================================
        if instant_mode in (MODE_AVOID_LEFT, MODE_AVOID_RIGHT):
            new_dir = -1 if instant_mode == MODE_AVOID_LEFT else 1

            if self.state != self.STATE_AVOID or self.locked_dir != new_dir:
                self.state = self.STATE_AVOID
                self.locked_dir = new_dir
                self.avoid_start_time = now

            self.last_center_obstacle_time = now

            target_bias = float(self.locked_dir * AVOID_BIAS_TARGET_PIXELS)
            max_delta = AVOID_BIAS_RAMP_PX_PER_SEC * dt
            self.current_bias = ramp_towards(self.current_bias, target_bias, max_delta)

            output_mode = MODE_AVOID_LEFT if self.locked_dir < 0 else MODE_AVOID_RIGHT
            return output_mode, self.locked_dir, int(round(self.current_bias)), self.state

        # 走到这里，说明当前帧没有中间红/黄/绿障碍。
        if self.state == self.STATE_AVOID:
            avoid_alive_time = now - self.avoid_start_time
            obstacle_missing_time = now - self.last_center_obstacle_time

            if avoid_alive_time < AVOID_MIN_ACTIVE_TIME_SEC or obstacle_missing_time < AVOID_HOLD_TIME_SEC:
                target_bias = float(self.locked_dir * AVOID_BIAS_TARGET_PIXELS)
                max_delta = AVOID_BIAS_RAMP_PX_PER_SEC * dt
                self.current_bias = ramp_towards(self.current_bias, target_bias, max_delta)
                output_mode = MODE_AVOID_LEFT if self.locked_dir < 0 else MODE_AVOID_RIGHT
                return output_mode, self.locked_dir, int(round(self.current_bias)), self.state

            self.state = self.STATE_RETURN

        if self.state == self.STATE_RETURN:
            max_delta = RETURN_BIAS_RAMP_PX_PER_SEC * dt
            self.current_bias = ramp_towards(self.current_bias, 0.0, max_delta)

            if abs(self.current_bias) > RETURN_FINISH_BIAS_PIXELS:
                output_dir = -1 if self.current_bias < 0 else 1
                output_mode = MODE_AVOID_LEFT if output_dir < 0 else MODE_AVOID_RIGHT
                return output_mode, output_dir, int(round(self.current_bias)), self.state

            line_ok = True
            if ENABLE_LINE_FOLLOW:
                line_ok = (valid_count >= LINE_REACQUIRE_MIN_COUNT)
                if line_error is not None:
                    line_ok = line_ok and (abs(line_error) <= LINE_CENTER_TOL_PIXELS)

            if line_ok:
                self.state = self.STATE_TRACE
                self.locked_dir = 0
                self.current_bias = 0.0
                return MODE_TRACE, 0, 0, self.state

            output_dir = -1 if self.locked_dir < 0 else 1
            output_mode = MODE_AVOID_LEFT if output_dir < 0 else MODE_AVOID_RIGHT
            return output_mode, output_dir, int(round(self.current_bias)), self.state

        self.state = self.STATE_TRACE
        self.locked_dir = 0
        self.current_bias = 0.0
        return MODE_TRACE, 0, 0, self.state


def apply_navigation_bias(errors, bias, mode):
    """
    把状态机给出的 bias 加到 error1~error5 上。

    新增：
        原地转向 MODE_SPIN_LEFT / MODE_SPIN_RIGHT 时，即使当前没有识别到蓝线，
        也生成一组固定的大误差，方便仍按 error PID 的旧下位机做转向测试。
    """
    biased_errors = list(errors)

    if mode == MODE_SPIN_LEFT:
        return [-SPIN_SEARCH_ERROR_PIXELS] * len(biased_errors)
    if mode == MODE_SPIN_RIGHT:
        return [SPIN_SEARCH_ERROR_PIXELS] * len(biased_errors)

    if bias == 0:
        return biased_errors

    for i, error in enumerate(biased_errors):
        if error == INVALID_ERROR:
            continue
        biased_errors[i] = int(error + bias)

    return biased_errors
