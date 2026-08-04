#!/usr/bin/env bash
set -Eeo pipefail

# Full-save profile:
#   - periodic SLAM Log remains controlled by the web page
#   - Ctrl+C always saves final_map.pgm, final_map.yaml and result.pbstream
# Runtime behavior is shared with open_all.sh so startup, health checks and
# cleanup cannot drift between the two entry points.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export SAVE_FINAL_ARTIFACTS=true
export RUN_PROFILE_NAME=open_all_log

printf '%s\n' \
    "[profile] open_all_log: periodic SLAM Log is web-controlled." \
    "[profile] open_all_log: final PGM/YAML/PBStream save is ENABLED."

exec bash "$ROOT_DIR/open_all.sh" "$@"
