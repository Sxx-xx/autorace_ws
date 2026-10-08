#!/bin/bash
# [로봇] 2번 + 차선 제어기 + cmd_vel_mux. 먼저 1번이 떠 있어야 한다.
# 결과: 웹 뷰어는 그대로, 로봇은 4번(주행시작) 전까지 정지 대기.
source "$(dirname "$0")/../_env.sh"
pkill -f "ustreamer|csi_camera.py.*camera_lane" 2>/dev/null
exec ros2 launch autorace_bringup real_lane.launch.py drive:=true
