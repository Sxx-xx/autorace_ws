#!/bin/bash
# Stop the real robot now: kill the controllers and hold zero velocity for a
# moment. The OpenCR keeps the last command it was given, so killing the
# nodes alone does not stop the wheels.
pkill -f cmd_vel_mux 2>/dev/null
pkill -f lane_controller 2>/dev/null
pkill -f _mission 2>/dev/null
ros2 topic pub -r 10 -t 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null
echo "stopped"
