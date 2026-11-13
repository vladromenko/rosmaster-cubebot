#!/bin/bash
# Run CubeBot on the robot. For motion runs Explorer's control/watchdog services are stopped
# (they also publish /cmd_vel) and restored on exit, whatever the exit reason.
#   scripts/run.sh --diagnose        no motion, services untouched
#   scripts/run.sh --dry-run         plans only, services untouched
#   scripts/run.sh [--config FILE]   the real mission
source /home/vlad/Explorer/bin/env.sh
export FASTRTPS_DEFAULT_PROFILES_FILE="$EXPLORER_ROOT/config/fastdds-rgbd.xml"
cd "$(dirname "$0")/.."
SERVICES="explorer-control.service explorer-watchdog.service"
MOTION=1
for a in "$@"; do case "$a" in --diagnose|--dry-run|--preflight-only) MOTION=0;; esac; done
if [ "$MOTION" = 1 ]; then
  rm -f /tmp/cubebot.stop
  restore() { systemctl --user start $SERVICES; }
  trap restore EXIT
  systemctl --user stop $SERVICES
  sleep 1
fi
python3 -m cubebot.main "$@"
