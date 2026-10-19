#!/bin/bash
# Base + laser with /cmd_vel as plain Twist (our stack publishes Twist): param/tb3_burger.yaml.
source "$(dirname "$(readlink -f "$0")")/pi_env.sh"
export LDS_MODEL=LDS-01
exec ros2 launch turtlebot3_bringup robot.launch.py \
  tb3_param_dir:=$(ros2 pkg prefix --share autorace_bringup)/param/tb3_burger.yaml
