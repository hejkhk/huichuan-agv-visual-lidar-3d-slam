#!/usr/bin/env bash
set -euo pipefail

APP_ROOT="/opt/robot-touch-ui"
DATA_ROOT="${XDG_DATA_HOME:-${HOME}/.local/share}/robot-touch-ui"
ROS_SETUP="${ROBOT_UI_ROS_SETUP:-/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash}"

if [[ -r "$ROS_SETUP" ]]; then
    # shellcheck disable=SC1090
    set +u
    source "$ROS_SETUP"
    set -u
fi

mkdir -p "$DATA_ROOT"
export ROBOT_UI_DATA_DIR="$DATA_ROOT"
export ROBOT_UI_PROJECT_ROOT="${ROBOT_UI_PROJECT_ROOT:-$APP_ROOT/app}"
export ROBOT_API_MODE="${ROBOT_API_MODE:-team}"
export PYTHONPATH="$APP_ROOT/vendor${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$APP_ROOT/vendor/PySide6/Qt/lib:$APP_ROOT/vendor/PySide6:$APP_ROOT/vendor/shiboken6${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export QT_PLUGIN_PATH="$APP_ROOT/vendor/PySide6/Qt/plugins${QT_PLUGIN_PATH:+:$QT_PLUGIN_PATH}"
export QML2_IMPORT_PATH="$APP_ROOT/vendor/PySide6/Qt/qml${QML2_IMPORT_PATH:+:$QML2_IMPORT_PATH}"

exec /usr/bin/python3 "$APP_ROOT/app/main.py" "$@"
