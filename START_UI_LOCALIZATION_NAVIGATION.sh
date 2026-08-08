#!/usr/bin/env bash
set -Eeo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UI_DIR="$ROOT_DIR/CAR UI V5.2"
MAP_DIR="$ROOT_DIR/Loc_MAP"
STATE_DIR="$HOME/.cache/huichuan_agv"
SELECTED_MAP_FILE="$STATE_DIR/selected_map"
STACK_PID_FILE="$STATE_DIR/launcher.pid"
STACK_ROOT_FILE="$STATE_DIR/project_root"
INITIAL_STACK_PID=""
SHUTTING_DOWN=false

log() { printf '[ui-nav] %s\n' "$*"; }
die() { printf '[ui-nav] ERROR: %s\n' "$*" >&2; exit 2; }

valid_map_name() {
  local name="$1"
  [ -n "$name" ] && [[ "$name" != */* ]] && \
    [[ "$name" != *\\* ]] && [ "$name" != "." ] && [ "$name" != ".." ]
}

complete_map() {
  local name="$1"
  valid_map_name "$name" && \
    [ -s "$MAP_DIR/$name.pgm" ] && \
    [ -s "$MAP_DIR/$name.yaml" ] && \
    [ -s "$MAP_DIR/$name.pbstream" ]
}

read_selected_map() {
  [ -r "$SELECTED_MAP_FILE" ] || return 0
  head -n 1 "$SELECTED_MAP_FILE" 2>/dev/null || true
}

write_selected_map() {
  local name="$1"
  mkdir -p "$STATE_DIR"
  printf '%s\n' "$name" >"$SELECTED_MAP_FILE.tmp"
  mv -f "$SELECTED_MAP_FILE.tmp" "$SELECTED_MAP_FILE"
}

choose_initial_map() {
  local requested="${1:-}" remembered="" yaml="" stem=""
  local -a complete_maps=()

  if [ -n "$requested" ]; then
    valid_map_name "$requested" || die "Invalid map basename: $requested"
    complete_map "$requested" || \
      die "Map '$requested' needs matching PGM, YAML and PBStream files in Loc_MAP"
    printf '%s' "$requested"
    return 0
  fi

  remembered="$(read_selected_map)"
  if [ -n "$remembered" ] && complete_map "$remembered"; then
    printf '%s' "$remembered"
    return 0
  fi
  if complete_map map; then
    printf '%s' map
    return 0
  fi

  for yaml in "$MAP_DIR"/*.yaml; do
    [ -e "$yaml" ] || continue
    stem="$(basename "$yaml" .yaml)"
    complete_map "$stem" && complete_maps+=("$stem")
  done
  if [ "${#complete_maps[@]}" -eq 1 ]; then
    printf '%s' "${complete_maps[0]}"
  fi
  return 0
}

registered_launcher_pid() {
  local pid="" owner_root="" cmdline=""
  [ -r "$STACK_PID_FILE" ] || return 1
  pid="$(head -n 1 "$STACK_PID_FILE" 2>/dev/null || true)"
  owner_root="$(head -n 1 "$STACK_ROOT_FILE" 2>/dev/null || true)"
  [[ "$pid" =~ ^[0-9]+$ ]] || return 1
  [ "$pid" -gt 1 ] && kill -0 "$pid" 2>/dev/null || return 1
  [ "$owner_root" = "$ROOT_DIR" ] || return 1
  cmdline="$(tr '\0' ' ' <"/proc/$pid/cmdline" 2>/dev/null || true)"
  [[ "$cmdline" == *run_dual_resolution_3d_slam.sh* ]] || return 1
  printf '%s' "$pid"
}

stop_registered_stack() {
  local pid="" cmdline="" _=""
  pid="$(registered_launcher_pid 2>/dev/null || true)"
  if [ -z "$pid" ] && [[ "$INITIAL_STACK_PID" =~ ^[0-9]+$ ]] && \
      kill -0 "$INITIAL_STACK_PID" 2>/dev/null; then
    cmdline="$(tr '\0' ' ' <"/proc/$INITIAL_STACK_PID/cmdline" \
      2>/dev/null || true)"
    if [[ "$cmdline" == *"$ROOT_DIR/START_DUAL_2D_3D_LOCALIZATION.sh"* ]] || \
        [[ "$cmdline" == *run_dual_resolution_3d_slam.sh* ]]; then
      pid="$INITIAL_STACK_PID"
    fi
  fi
  [ -n "$pid" ] || return 0
  log "Stopping localization/navigation launcher pid=$pid"
  kill -INT "$pid" 2>/dev/null || true
  for _ in $(seq 1 200); do
    kill -0 "$pid" 2>/dev/null || return 0
    sleep 0.1
  done
  kill -TERM "$pid" 2>/dev/null || true
  for _ in $(seq 1 50); do
    kill -0 "$pid" 2>/dev/null || return 0
    sleep 0.1
  done
  log "Launcher pid=$pid did not stop after SIGINT and SIGTERM"
}

cleanup() {
  [ "$SHUTTING_DOWN" = "true" ] && return 0
  SHUTTING_DOWN=true
  stop_registered_stack
  if [ -n "$INITIAL_STACK_PID" ]; then
    wait "$INITIAL_STACK_PID" 2>/dev/null || true
  fi
}

trap 'exit 130' INT TERM
trap cleanup EXIT

[ -x "$UI_DIR/run.sh" ] || die "Missing UI launcher: $UI_DIR/run.sh"
[ -x "$ROOT_DIR/START_DUAL_2D_3D_LOCALIZATION.sh" ] || \
  die "Missing localization launcher"
mkdir -p "$MAP_DIR"

REQUESTED_MAP="${1:-${LOCALIZATION_MAP_NAME:-}}"
INITIAL_MAP="$(choose_initial_map "$REQUESTED_MAP")"
if [ -n "$INITIAL_MAP" ]; then
  write_selected_map "$INITIAL_MAP"
  log "Initial map: $INITIAL_MAP"
else
  log "No unambiguous initial map. Open Map Management and press Use Map."
fi

export HUICHUAN_SLAM_ROOT="$ROOT_DIR"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-88}"
export ROBOT_UI_START_RVIZ="${ROBOT_UI_START_RVIZ:-false}"
export ROBOT_UI_FOREGROUND=1

if [ -n "$INITIAL_MAP" ]; then
  USE_RVIZ="$ROBOT_UI_START_RVIZ" \
    bash "$ROOT_DIR/START_DUAL_2D_3D_LOCALIZATION.sh" "$INITIAL_MAP" &
  INITIAL_STACK_PID=$!
fi

log "Starting CAR UI; closing the UI will stop the active localization stack."
bash "$UI_DIR/run.sh"
