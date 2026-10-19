#!/bin/bash
# The cameras through the workspace's pi_camera.launch.py (CSI on /camera, the C920 on /camera_lane
# when plugged in), detached; log in ~/pi_camera.log, pid in ~/pi_camera.pid. Extra arguments go to
# the launch file (fps:=15, lane:=false). Over ssh .bashrc stops early, so the ROS setup is here.
cd $HOME
[ -f pi_camera.pid ] && kill $(cat pi_camera.pid) 2>/dev/null && sleep 2
source /opt/ros/jazzy/setup.bash
source $HOME/autorace_ws/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-30} RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
[ -f $HOME/cyclonedds.xml ] && export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
setsid nohup ros2 launch autorace_bringup pi_camera.launch.py "$@" > pi_camera.log 2>&1 < /dev/null &
echo $! > pi_camera.pid
echo "started pi_camera.launch.py (pid $(cat pi_camera.pid)), log ~/pi_camera.log"
