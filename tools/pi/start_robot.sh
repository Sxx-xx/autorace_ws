#!/bin/bash
# Base + laser with /cmd_vel as plain Twist (our stack publishes Twist).
source /opt/ros/jazzy/setup.bash
# The middleware and discovery the PC uses too (tools/dds.sh); ROS_DOMAIN_ID 30 by default.
DDS_SH="$(dirname "$(readlink -f "$0")")/../dds.sh"
[ -f "$DDS_SH" ] || DDS_SH=$HOME/dds.sh
if [ -f "$DDS_SH" ]; then source "$DDS_SH"; else export ROS_DOMAIN_ID=30;
  echo "dds.sh not found: Fast DDS; the PC must not use CycloneDDS" >&2; fi
export TURTLEBOT3_MODEL=burger LDS_MODEL=LDS-01
exec ros2 launch turtlebot3_bringup robot.launch.py tb3_param_dir:=$HOME/tb3_burger_twist.yaml
