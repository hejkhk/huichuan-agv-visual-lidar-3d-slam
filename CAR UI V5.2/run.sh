#!/usr/bin/env bash
set -euo pipefail
UI_ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$UI_ROOT"

# In the production repository the UI lives directly below the Huichuan
# project root. Keep standalone deployments configurable, but make the
# integrated checkout work without an extra environment variable.
if [[ -z "${HUICHUAN_SLAM_ROOT:-}" ]] && \
    [[ -x "$UI_ROOT/../START_DUAL_2D_3D_LOCALIZATION.sh" ]]; then
  export HUICHUAN_SLAM_ROOT="$(cd "$UI_ROOT/.." && pwd)"
fi

# Match the Huichuan stack's DDS process exactly. A ROS node can initialize
# successfully with another RMW implementation yet still discover no map
# publishers, which previously left the UI waiting with no useful diagnosis.
if [[ -n "${HUICHUAN_SLAM_ROOT:-}" ]]; then
  DDS_CONFIG="$HUICHUAN_SLAM_ROOT/visual_laser_slam/cyclonedds_dual_3d.xml"
  if [[ -f "$DDS_CONFIG" ]]; then
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI="${DUAL_3D_CYCLONEDDS_URI:-file://$DDS_CONFIG}"
  fi
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-88}"
export ROBOT_UI_MAP_TOPIC="${ROBOT_UI_MAP_TOPIC:-/map}"
export ROBOT_UI_PROJECT_ROOT="${ROBOT_UI_PROJECT_ROOT:-${HUICHUAN_SLAM_ROOT:-$UI_ROOT}}"

# The UI never owns an STM32 serial port; chassis_node remains the sole serial
# endpoint. main.py applies DDS domain 88 unless settings or the environment
# explicitly select another domain.
export ROBOT_UI_ENABLE_SERIAL_RELAY=0

if [[ -z "${ROBOT_UI_ROS_SETUP:-}" ]]; then
  for _distro in "${ROS_DISTRO:-}" humble jazzy; do
    if [[ -r "/opt/ros/${_distro}/setup.bash" ]]; then
      ROBOT_UI_ROS_SETUP="/opt/ros/${_distro}/setup.bash"
      break
    fi
  done
fi
ROS_SETUP="${ROBOT_UI_ROS_SETUP:-}"
if [[ -n "$ROS_SETUP" && -r "$ROS_SETUP" ]]; then
  # shellcheck disable=SC1090
  set +u
  source "$ROS_SETUP"
  set -u
fi

# This HMI owns its on-screen keyboard. Ubuntu exports QT_IM_MODULE=ibus by
# default, which would route show/hide requests away from the embedded panel.
export QT_IM_MODULE=qtvirtualkeyboard
# Qt Virtual Keyboard is not supported as a client-side input context by the
# Qt Wayland plugin. XCB runs through Ubuntu's XWayland and supports it.
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"

PYTHON_BIN="python3"
if [[ -x ".venv/bin/python" ]]; then
  PYTHON_BIN=".venv/bin/python"
fi
export PYTHONUNBUFFERED=1

LOG_DIR="$PWD/logs"
LOG_FILE="$LOG_DIR/ui.log"
PID_FILE="$LOG_DIR/ui.pid"
mkdir -p "$LOG_DIR"

# Serialize launch attempts, then retire only an earlier UI whose working
# directory and command line both belong to this exact project. Never use a
# broad pkill pattern here: ROS 2 and unrelated Python processes must survive.
if command -v flock >/dev/null 2>&1; then
  exec 9>"$LOG_DIR/ui.launch.lock"
  flock 9
fi

is_project_ui_pid() {
  local candidate="$1" candidate_cwd candidate_cmd
  [[ "$candidate" =~ ^[0-9]+$ ]] || return 1
  [[ "$candidate" != "$$" && -r "/proc/$candidate/cmdline" ]] || return 1
  candidate_cwd="$(readlink -f "/proc/$candidate/cwd" 2>/dev/null || true)"
  [[ "$candidate_cwd" == "$PWD" ]] || return 1
  candidate_cmd="$(tr '\0' ' ' <"/proc/$candidate/cmdline" 2>/dev/null || true)"
  [[ "$candidate_cmd" == *"main.py"* ]]
}

declare -A OLD_UI_PIDS=()
if [[ -r "$PID_FILE" ]]; then
  RUNNING_PID="$(<"$PID_FILE")"
  if is_project_ui_pid "$RUNNING_PID"; then
    OLD_UI_PIDS["$RUNNING_PID"]=1
  fi
fi
for _proc in /proc/[0-9]*; do
  _pid="${_proc##*/}"
  if is_project_ui_pid "$_pid"; then
    OLD_UI_PIDS["$_pid"]=1
  fi
done

if ((${#OLD_UI_PIDS[@]})); then
  echo "正在关闭旧版机器人车载 UI……"
  kill -TERM "${!OLD_UI_PIDS[@]}" 2>/dev/null || true
  for _attempt in $(seq 1 30); do
    _still_running=0
    for _pid in "${!OLD_UI_PIDS[@]}"; do
      if kill -0 "$_pid" 2>/dev/null; then _still_running=1; fi
    done
    ((_still_running == 0)) && break
    sleep 0.1
  done
  for _pid in "${!OLD_UI_PIDS[@]}"; do
    if kill -0 "$_pid" 2>/dev/null; then kill -KILL "$_pid" 2>/dev/null || true; fi
  done
fi
rm -f "$PID_FILE"

if [[ -s "$LOG_FILE" ]]; then
  mv -f "$LOG_FILE" "$LOG_DIR/ui.previous.log"
fi

# Foreground diagnostics still write every line to the same local log while
# tee keeps the terminal readable. Python -u prevents delayed exception and
# connection messages when stdout is no longer attached to a TTY.
if [[ "${ROBOT_UI_FOREGROUND:-0}" == "1" ]]; then
  exec > >(tee -a "$LOG_FILE") 2>&1
  exec "$PYTHON_BIN" -u main.py "$@"
fi

nohup "$PYTHON_BIN" -u main.py "$@" >>"$LOG_FILE" 2>&1 < /dev/null &
UI_PID=$!
printf '%s\n' "$UI_PID" >"$PID_FILE"

for _attempt in $(seq 1 80); do
  if grep -q "机器人车载 UI 已启动" "$LOG_FILE" 2>/dev/null; then
    echo "机器人车载 UI 已启动（PID $UI_PID）"
    exit 0
  fi
  if ! kill -0 "$UI_PID" 2>/dev/null; then
    echo "机器人车载 UI 启动失败，请查看 $LOG_FILE" >&2
    tail -n 30 "$LOG_FILE" >&2 || true
    exit 1
  fi
  sleep 0.1
done

echo "机器人车载 UI 正在启动，日志：$LOG_FILE"
