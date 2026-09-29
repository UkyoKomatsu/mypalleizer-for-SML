#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/galactic/setup.bash
set -u
Xvfb :99 -screen 0 1600x900x24 &
sleep 2
fluxbox >/tmp/fluxbox.log 2>&1 &
x11vnc -display :99 -forever -shared -nopw -listen 127.0.0.1 >/tmp/x11vnc.log 2>&1 &
websockify --web /usr/share/novnc 6080 localhost:5900 >/tmp/novnc.log 2>&1 &
gzclient >/tmp/gzclient.log 2>&1 &
exec ros2 launch /opt/sim/launch/sim.launch.py
