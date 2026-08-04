#!/usr/bin/env bash
set -Eeo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
ENV_FILE="$ROOT_DIR/visual_laser_slam/visual_laser_slam.env"
[ -f "$ENV_FILE" ] && source "$ENV_FILE"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-88}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 service call /octomap_server_3d/reset std_srvs/srv/Empty "{}"
