#!/bin/bash
# Ask a running mission to stop: it zeroes the base, parks the arm and restores Explorer.
touch /tmp/cubebot.stop
source /home/vlad/Explorer/bin/env.sh
timeout 3 ros2 topic pub --once /cubebot/stop std_msgs/msg/Bool "{data: true}" >/dev/null 2>&1 || true
