#!/usr/bin/env bash
set -Eeo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAP_NAME="${1:-${LOCALIZATION_MAP_NAME:-map}}"
MAP_DIR="$ROOT_DIR/Loc_MAP"
MAP_YAML="$MAP_DIR/$MAP_NAME.yaml"
MAP_PBSTREAM="$MAP_DIR/$MAP_NAME.pbstream"
RVIZ_CONFIG="$ROOT_DIR/lidar/chapt1_ws/src/lidar_py/rviz/dual_resolution_3d_localization.rviz"

cd "$ROOT_DIR"

if [ -z "$MAP_NAME" ] || [[ "$MAP_NAME" == */* ]] || \
    [[ "$MAP_NAME" == *\\* ]] || [ "$MAP_NAME" = "." ] || \
    [ "$MAP_NAME" = ".." ]; then
  printf '[ERROR] Invalid localization map basename: %s\n' "$MAP_NAME" >&2
  exit 2
fi

if [ ! -f "$MAP_YAML" ]; then
  printf '[ERROR] Missing localization map: %s\n' "$MAP_YAML" >&2
  printf '        Put map.yaml + its image + map.pbstream in Loc_MAP/.\n' >&2
  exit 2
fi
if [ ! -f "$MAP_PBSTREAM" ]; then
  printf '[ERROR] Missing Cartographer state: %s\n' "$MAP_PBSTREAM" >&2
  exit 2
fi

# Resolve the image declared by the YAML before ROS starts. Older exported
# maps may keep their original image name while users rename the YAML/PGM pair.
# In that case create a hidden runtime YAML pointing at the colocated PGM; the
# source YAML remains untouched.
MAP_RUNTIME_YAML="$(/usr/bin/python3 - "$MAP_YAML" "$MAP_DIR/$MAP_NAME.pgm" <<'PY'
import pathlib
import sys

import yaml

yaml_path = pathlib.Path(sys.argv[1]).resolve()
fallback = pathlib.Path(sys.argv[2]).resolve()
try:
    payload = yaml.safe_load(yaml_path.read_text(encoding="utf-8")) or {}
except Exception as exc:
    raise SystemExit(f"[ERROR] Invalid localization YAML {yaml_path}: {exc}")
image_value = str(payload.get("image", "")).strip()
if not image_value:
    raise SystemExit(f"[ERROR] Localization YAML has no image field: {yaml_path}")
image_path = pathlib.Path(image_value).expanduser()
if not image_path.is_absolute():
    image_path = (yaml_path.parent / image_path).resolve()
if image_path.is_file():
    print(yaml_path)
    raise SystemExit(0)
if not fallback.is_file():
    raise SystemExit(
        f"[ERROR] Map image is missing: YAML requests {image_path}; "
        f"fallback {fallback} is also missing")
payload["image"] = fallback.name
runtime_yaml = yaml_path.parent / f".{yaml_path.stem}.runtime.yaml"
runtime_yaml.write_text(
    yaml.safe_dump(payload, allow_unicode=True, sort_keys=False),
    encoding="utf-8",
)
print(runtime_yaml)
PY
)" || exit $?
if [ ! -f "$RVIZ_CONFIG" ]; then
  printf '[ERROR] Missing localization RViz profile: %s\n' "$RVIZ_CONFIG" >&2
  exit 2
fi

export DUAL_3D_ENABLE_NAVIGATION=true
export DUAL_3D_LOCALIZATION_MODE=true
export DUAL_3D_STACK_MODE=localization
export DUAL_3D_LOCALIZATION_MAP_NAME="$MAP_NAME"
export DUAL_3D_LOCALIZATION_MAP_YAML="$MAP_RUNTIME_YAML"
export DUAL_3D_LOCALIZATION_PBSTREAM="$MAP_PBSTREAM"
export DUAL_3D_RVIZ_CONFIG_FILE="$RVIZ_CONFIG"
export DUAL_3D_CARTOGRAPHER_CONFIG=cartographer_2d_localization.lua
export DUAL_3D_DATABASE="maps/rtabmap_3d/rtabmap_localization_live.db"
export DUAL_3D_RESET_GLOBAL_MAP=true
export DUAL_3D_REQUIRE_DEPTH_BASELINE_FOR_PS2=false
export USE_RVIZ="${USE_RVIZ:-true}"

printf '[localization] Static map : %s\n' "$MAP_RUNTIME_YAML"
printf '[localization] PBSTREAM   : %s\n' "$MAP_PBSTREAM"
printf '[localization] Keep the vehicle stationary until the panel says localized.\n'

exec bash "$ROOT_DIR/visual_laser_slam/run_dual_resolution_3d_slam.sh"
