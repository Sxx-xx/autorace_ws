#!/bin/bash
# [로봇] 이 폴더의 스크립트가 띄운 것 전부와 기존 bringup·카메라 송출을 끈다.
# 바퀴에 0 속도를 먼저 보낸다. 백그라운드로 남은 프로세스 정리용.
source "$(dirname "$0")/_env.sh"
mkdir -p "$WS/logs"
if pgrep -f "turtlebot3_ros" >/dev/null; then
  pkill -INT -f "lane_controller|cmd_vel_mux" 2>/dev/null
  timeout 4 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1
fi
pkill -INT -f "real_lane.launch.py|turtlebot3_bringup robot.launch.py" 2>/dev/null
sleep 1
pkill -f "autorace_perception/lib|autorace_core/lib|turtlebot3_ros|hlds_laser_publisher|ld_lidar.py|ustreamer|csi_camera.py" 2>/dev/null
sleep 1
[ "$1" = quiet ] || echo "전부 종료함"
