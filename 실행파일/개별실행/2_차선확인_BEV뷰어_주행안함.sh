#!/bin/bash
# [로봇] 차선 카메라 -> BEV -> 차선 인식 -> 웹 뷰어. 바퀴는 절대 안 움직인다.
# 결과: PC 브라우저에서 http://<로봇IP>:8090 (원본 / BEV+격자 / BEV+인식선 / 차선 지도)
source "$(dirname "$0")/../_env.sh"
pkill -f "ustreamer|csi_camera.py.*camera_lane" 2>/dev/null   # video0 점유 해제
exec ros2 launch autorace_bringup real_lane.launch.py
