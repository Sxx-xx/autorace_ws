#!/bin/bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=30
exec python3 $HOME/ld_lidar.py
