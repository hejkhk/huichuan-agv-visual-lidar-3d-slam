#!/usr/bin/env bash
set -Eeo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG="${1:-}"
if [ -z "$LOG" ]; then
  LOG="$(find "$ROOT_DIR/SLAM_Log" -maxdepth 2 -path '*/isolated_visual_odom_clean_*/runtime.log' -printf '%T@ %p\n' 2>/dev/null | sort -nr | head -1 | cut -d' ' -f2-)"
fi
[ -n "$LOG" ] && [ -f "$LOG" ] || { echo "未找到 visual_odom_clean runtime.log" >&2; exit 1; }
exec python3 "$ROOT_DIR/tools/rtabmap_odom_log_stats.py" "$LOG"
