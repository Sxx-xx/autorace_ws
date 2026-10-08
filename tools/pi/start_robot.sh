#!/bin/bash
# Base + laser with /cmd_vel as plain Twist (our stack publishes Twist).
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger LDS_MODEL=LDS-01 ROS_DOMAIN_ID=30
exec ros2 launch turtlebot3_bringup robot.launch.py tb3_param_dir:=$HOME/tb3_burger_twist.yaml
