#!/bin/bash
# Both cameras through pi_camera.launch.py: the CSI camera on /camera, the C920 on /camera_lane when
# it is plugged in. Arguments go to the launch file: CAMERA_FPS=15 or fps:=15 for a crowded 2.4 GHz
# band (the detectors were tuned at 30), lane:=false.
source "$(dirname "$(readlink -f "$0")")/pi_env.sh"
exec ros2 launch autorace_bringup pi_camera.launch.py fps:=${CAMERA_FPS:-30} "$@"
