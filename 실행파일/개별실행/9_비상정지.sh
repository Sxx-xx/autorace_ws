#!/bin/bash
# [로봇] 비상 정지: 주행 해제 + 제어 노드 종료 + 0 속도 1초간 반복.
# 그래도 움직이면 1번 창에서 Ctrl+C (OpenCR 정지).
source "$(dirname "$0")/../_env.sh"
ros2 topic pub -t 10 -r 10 /autorace/run_active std_msgs/msg/Bool "{data: false}" &
pkill -INT -f "lane_controller|cmd_vel_mux" 2>/dev/null
timeout 4 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null
wait
echo "정지 명령 보냄"
