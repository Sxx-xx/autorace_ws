#!/bin/bash
# Stop what start_all.sh started. The OpenCR keeps its last command: stop the PC's controllers first
# (tools/stop_robot.sh on the PC sends zero velocity).
source "$(dirname "$(readlink -f "$0")")/pi_env.sh"
stop_pid robot watchdog lidar cameras camera camera_lane
sleep 2
pgrep -af 'turtlebot3_ros|cmd_vel_watchdo[g]|ld_lidar|csi_camera|hlds_laser_publishe[r]' | grep -v pgrep || echo "all stopped"
