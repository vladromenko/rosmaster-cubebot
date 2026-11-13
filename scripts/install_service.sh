#!/bin/bash
# Installs (does not enable) the cubebot user service:  systemctl --user start cubebot
mkdir -p ~/.config/systemd/user
cp "$(dirname "$0")/cubebot.service" "$(dirname "$0")/rosmaster-console.service" "$(dirname "$0")/rosmaster-bridge.service" ~/.config/systemd/user/
systemctl --user daemon-reload
echo "installed; start with: systemctl --user start rosmaster-console  (UI: http://explorer.local:8090)  or  cubebot (mission only)"
