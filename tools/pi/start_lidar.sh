#!/bin/bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=30
# The lidar sits with its zero pointing backwards: what the camera saw ahead (the parking
# sign, the parked robot) the scan put behind, and only with 180 deg do the returns stay
# put in the world as the robot moves (bags of 2026-10-27).
exec python3 $HOME/ld_lidar.py --ros-args -p angle_offset_deg:=180.0
