#!/bin/bash
# Stop the robot. gz-sim's DiffDrive keeps applying the last command it was
# given, so killing the controllers is not enough: the robot keeps driving.
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
