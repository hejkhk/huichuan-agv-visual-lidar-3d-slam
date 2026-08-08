#!/usr/bin/env bash
set -euo pipefail

# Compatibility entry point. The old PTY relay caused two owners to compete
# for STM32 bytes. chassis_node is now the only serial owner.
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
printf '[deprecated] Serial relay is disabled; starting the ROS-only UI.\n'
exec "$ROOT_DIR/run.sh" "$@"
