#!/bin/bash
# [로봇] 주행 정지 (3번은 계속 떠 있음, 4번으로 다시 출발 가능).
source "$(dirname "$0")/../_env.sh"
ros2 topic pub -t 10 -r 10 /autorace/run_active std_msgs/msg/Bool "{data: false}"
