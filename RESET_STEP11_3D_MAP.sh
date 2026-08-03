#!/usr/bin/env bash
set -Eeo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DB="$ROOT_DIR/maps/rtabmap_3d/rtabmap_v3.db"

if pgrep -f 'dual_resolution_3d_slam.launch.py|rtabmap_3d/rtabmap' >/dev/null 2>&1; then
  echo "STEP11 is running. Stop it before resetting the 3D map." >&2
  exit 1
fi

if [ ! -f "$DB" ]; then
  echo "No STEP11 database exists at $DB"
  exit 0
fi

STAMP="$(date +%Y-%m-%d_%H-%M-%S)"
BACKUP="${DB%.db}_backup_${STAMP}.db"
mv "$DB" "$BACKUP"
echo "The previous 3D map was archived at: $BACKUP"
