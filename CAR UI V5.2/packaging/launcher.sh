#!/bin/sh
set -eu

ROBOT_DATA_ROOT="${XDG_DATA_HOME:-${HOME}/.local/share}/robot-touch-ui"
mkdir -p "$ROBOT_DATA_ROOT"
export ROBOT_UI_DATA_DIR="$ROBOT_DATA_ROOT"
export ROBOT_UI_PROJECT_ROOT="${ROBOT_UI_PROJECT_ROOT:-/opt/robot-touch-ui}"

exec /opt/robot-touch-ui/robot-touch-ui.bin "$@"
