#!/usr/bin/env bash
set -Eeo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 只改变RTAB-Map内部图像降采样倍率，不修改相机输出、STEP9或STEP10V2.1。
export VO_ODOM_IMAGE_DECIMATION=2
exec "$ROOT_DIR/visual_laser_slam/run_isolated_perception.sh" visual_odom_clean
