#!/bin/bash
# [로봇] 차선 추종 주행 시작 (perception_real.yaml 의 lane_controller.max_speed, 기본 0.10 m/s).
source "$(dirname "$0")/../_env.sh"
ros2 topic pub -t 10 -r 10 /autorace/run_active std_msgs/msg/Bool "{data: true}"
