#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VERSION="${ROBOT_UI_VERSION:-1.0.0}"
ARCH="amd64"
DIST_DIR="$PROJECT_ROOT/dist"
DEPLOY_TOOL="$PROJECT_ROOT/.venv/bin/pyside6-deploy"
export PATH="$PROJECT_ROOT/.venv/bin:$PATH"

if [[ ! -x "$DEPLOY_TOOL" ]]; then
    echo "缺少 .venv 或 pyside6-deploy，请先按 README 安装依赖。" >&2
    exit 1
fi

if ! "$PROJECT_ROOT/.venv/bin/python" -c "import zstandard" >/dev/null 2>&1; then
    echo "安装 onefile 压缩依赖 zstandard……"
    "$PROJECT_ROOT/.venv/bin/python" -m pip install zstandard
fi

WORK_ROOT="$(mktemp -d -t robot-touch-ui-build-XXXXXX)"
DEPLOY_SOURCE="$WORK_ROOT/source"
PACKAGE_ROOT="$WORK_ROOT/package"
DEPLOY_BIN="$DEPLOY_SOURCE/robot-touch-ui.bin"
cleanup() { rm -rf -- "$WORK_ROOT"; }
trap cleanup EXIT

mkdir -p "$DIST_DIR" "$DEPLOY_SOURCE"
cp -a \
    "$PROJECT_ROOT/main.py" \
    "$PROJECT_ROOT/backend" \
    "$PROJECT_ROOT/robot_api" \
    "$PROJECT_ROOT/qml" \
    "$PROJECT_ROOT/assets" \
    "$PROJECT_ROOT/robot_ui.pyproject" \
    "$DEPLOY_SOURCE/"

echo "[1/3] 生成自包含程序……"
cd "$DEPLOY_SOURCE"
"$DEPLOY_TOOL" main.py \
    --force \
    --name robot-touch-ui \
    --mode onefile \
    --extra-modules=QtCore,QtGui,QtQml,QtQuick,QtSvg

if [[ ! -f "$DEPLOY_BIN" ]]; then
    echo "未找到部署产物：$DEPLOY_BIN" >&2
    exit 1
fi

echo "[2/3] 组装 Debian 安装目录……"
install -d \
    "$PACKAGE_ROOT/DEBIAN" \
    "$PACKAGE_ROOT/opt/robot-touch-ui" \
    "$PACKAGE_ROOT/opt/robot-touch-ui/map" \
    "$PACKAGE_ROOT/opt/robot-touch-ui/map_cache" \
    "$PACKAGE_ROOT/usr/bin" \
    "$PACKAGE_ROOT/usr/share/applications" \
    "$PACKAGE_ROOT/usr/share/icons/hicolor/scalable/apps"

install -m 0755 "$DEPLOY_BIN" "$PACKAGE_ROOT/opt/robot-touch-ui/robot-touch-ui.bin"
install -m 0755 "$PROJECT_ROOT/packaging/launcher.sh" "$PACKAGE_ROOT/usr/bin/robot-touch-ui"
install -m 0644 "$PROJECT_ROOT/packaging/robot-touch-ui.desktop" "$PACKAGE_ROOT/usr/share/applications/robot-touch-ui.desktop"
install -m 0644 "$PROJECT_ROOT/assets/icons/status.svg" "$PACKAGE_ROOT/usr/share/icons/hicolor/scalable/apps/robot-touch-ui.svg"

INSTALLED_SIZE="$(du -sk "$PACKAGE_ROOT" | cut -f1)"
cat > "$PACKAGE_ROOT/DEBIAN/control" <<EOF
Package: robot-touch-ui
Version: $VERSION
Section: utils
Priority: optional
Architecture: $ARCH
Installed-Size: $INSTALLED_SIZE
Maintainer: Robot UI Team
Depends: libc6 (>= 2.39), libgl1, libegl1, libxkbcommon0, libxkbcommon-x11-0, libxcb1, libxcb-cursor0, wireplumber, alsa-utils
Description: Robot vehicle touchscreen control panel
 Responsive PySide6/Qt Quick interface for navigation, vehicle status,
 following, voice control, multilingual settings, and mock robot APIs.
EOF

PACKAGE_PATH="$DIST_DIR/robot-touch-ui_${VERSION}_${ARCH}.deb"
echo "[3/3] 生成 $PACKAGE_PATH ……"
fakeroot dpkg-deb --build --root-owner-group "$PACKAGE_ROOT" "$PACKAGE_PATH"

echo
echo "打包完成：$PACKAGE_PATH"
echo "安装命令：sudo apt install '$PACKAGE_PATH'"
