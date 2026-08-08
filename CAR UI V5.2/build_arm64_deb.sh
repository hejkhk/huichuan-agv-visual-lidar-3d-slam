#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VERSION="${ROBOT_UI_VERSION:-1.0.0}"
PYSIDE_VERSION="${ROBOT_UI_PYSIDE_VERSION:-6.11.1}"
ARCH="arm64"
DIST_DIR="$PROJECT_ROOT/dist"
PYTHON_BIN="$PROJECT_ROOT/.venv/bin/python"

if [[ ! -x "$PYTHON_BIN" ]]; then
    PYTHON_BIN="$(command -v python3 || true)"
fi
if [[ -z "$PYTHON_BIN" ]]; then
    echo "缺少 Python 3，无法下载 ARM64 运行依赖。" >&2
    exit 1
fi
for command_name in unzip fakeroot dpkg-deb readelf; do
    if ! command -v "$command_name" >/dev/null 2>&1; then
        echo "缺少构建工具：$command_name" >&2
        exit 1
    fi
done

WORK_ROOT="$(mktemp -d -t robot-touch-ui-arm64-build-XXXXXX)"
WHEEL_DIR="$WORK_ROOT/wheels"
PACKAGE_ROOT="$WORK_ROOT/package"
APP_ROOT="$PACKAGE_ROOT/opt/robot-touch-ui"
VENDOR_ROOT="$APP_ROOT/vendor"
cleanup() { rm -rf -- "$WORK_ROOT"; }
trap cleanup EXIT

mkdir -p "$DIST_DIR" "$WHEEL_DIR" "$VENDOR_ROOT" "$APP_ROOT/app"

echo "[1/4] 下载 PySide6 $PYSIDE_VERSION 的 ARM64 官方二进制……"
"$PYTHON_BIN" -m pip download \
    --dest "$WHEEL_DIR" \
    --no-deps \
    --only-binary=:all: \
    --platform manylinux_2_39_aarch64 \
    --implementation cp \
    --python-version 312 \
    --abi abi3 \
    "PySide6_Essentials==$PYSIDE_VERSION" \
    "shiboken6==$PYSIDE_VERSION"

echo "[2/4] 组装 ARM64 应用文件……"
for wheel in "$WHEEL_DIR"/*.whl; do
    unzip -q -o "$wheel" -d "$VENDOR_ROOT"
done
find "$VENDOR_ROOT" -type f \( -name '*.pyi' -o -name '*.pyc' \) -delete
find "$VENDOR_ROOT" -type d -name '__pycache__' -prune -exec rm -rf -- {} +

cp -a \
    "$PROJECT_ROOT/main.py" \
    "$PROJECT_ROOT/backend" \
    "$PROJECT_ROOT/robot_api" \
    "$PROJECT_ROOT/qml" \
    "$PROJECT_ROOT/assets" \
    "$PROJECT_ROOT/map" \
    "$PROJECT_ROOT/map_cache" \
    "$APP_ROOT/app/"

ARM_LIBRARY="$(find "$VENDOR_ROOT" -type f -name 'QtCore.abi3.so' -print -quit)"
if [[ -z "$ARM_LIBRARY" ]] || ! readelf -h "$ARM_LIBRARY" | grep -q 'AArch64'; then
    echo "下载内容不是 AArch64，停止打包。" >&2
    exit 1
fi

install -d \
    "$PACKAGE_ROOT/DEBIAN" \
    "$PACKAGE_ROOT/usr/bin" \
    "$PACKAGE_ROOT/usr/share/applications" \
    "$PACKAGE_ROOT/usr/share/icons/hicolor/scalable/apps"
install -m 0755 "$PROJECT_ROOT/packaging/launcher-arm64.sh" "$PACKAGE_ROOT/usr/bin/robot-touch-ui"
install -m 0644 "$PROJECT_ROOT/packaging/robot-touch-ui.desktop" "$PACKAGE_ROOT/usr/share/applications/robot-touch-ui.desktop"
install -m 0644 "$PROJECT_ROOT/assets/icons/status.svg" "$PACKAGE_ROOT/usr/share/icons/hicolor/scalable/apps/robot-touch-ui.svg"

echo "[3/4] 写入 Debian ARM64 元数据……"
INSTALLED_SIZE="$(du -sk "$PACKAGE_ROOT" | cut -f1)"
cat > "$PACKAGE_ROOT/DEBIAN/control" <<EOF
Package: robot-touch-ui
Version: $VERSION
Section: utils
Priority: optional
Architecture: $ARCH
Installed-Size: $INSTALLED_SIZE
Maintainer: Robot UI Team
Depends: python3 (>= 3.12), python3 (<< 3.13), python3-numpy, python3-yaml, ros-jazzy-rclpy, ros-jazzy-geometry-msgs, ros-jazzy-nav-msgs, ros-jazzy-nav2-msgs, ros-jazzy-sensor-msgs, ros-jazzy-std-msgs, libc6 (>= 2.39), libgl1, libegl1, libdbus-1-3, libfontconfig1, libxkbcommon0, libxkbcommon-x11-0, libxcb1, libxcb-cursor0, libwayland-client0, wireplumber, alsa-utils
Description: Robot vehicle touchscreen control panel for ARM64
 Responsive PySide6/Qt Quick interface with bundled ARM64 Qt libraries for
 navigation, vehicle status, following, voice control, and settings.
EOF

PACKAGE_PATH="$DIST_DIR/robot-touch-ui_${VERSION}_${ARCH}.deb"
echo "[4/4] 生成 $PACKAGE_PATH ……"
fakeroot dpkg-deb --build --root-owner-group "$PACKAGE_ROOT" "$PACKAGE_PATH"

echo
echo "ARM64 打包完成：$PACKAGE_PATH"
echo "安装命令：sudo apt install '$PACKAGE_PATH'"
