#!/bin/bash
# Rosmaster console (web UI on port 8090). Pauses Explorer's control/watchdog (they also publish /cmd_vel) and restores them.
#   scripts/console.sh --dry-run    UI + telemetry only, nothing is sent to the robot
source /home/vlad/Explorer/bin/env.sh
export FASTRTPS_DEFAULT_PROFILES_FILE="$EXPLORER_ROOT/config/fastdds-rgbd.xml"
cd "$(dirname "$0")/.."
PY=${ROSMASTER_PY:-$HOME/rosmaster-venv/bin/python}
SERVICES="explorer-control.service explorer-watchdog.service"
LIVE=1; for a in "$@"; do [ "$a" = "--dry-run" ] && LIVE=0; done
if [ "$LIVE" = 1 ]; then
  rm -f /tmp/cubebot.stop
  trap 'systemctl --user start $SERVICES' EXIT
  systemctl --user stop $SERVICES; sleep 1
fi
exec_py() { "$PY" -m cubebot.web.server "$@"; }
exec_py "$@"
