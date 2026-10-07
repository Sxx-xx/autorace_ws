#!/bin/bash
# lane_view.sh [camera_topic] : the lane pipeline WITHOUT the controller (the robot does not move):
# decode <camera_topic>/compressed, project to BEV, detect the lane. Debug images on
# /camera/image_projected (BEV) and /detect/image_lane. Ctrl-C stops it.
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash; source install/setup.bash
CAM=${1:-/camera_lane/image_raw}; INFO=${CAM%/image_raw}/camera_info
P=$(ros2 pkg prefix autorace_bringup)/share/autorace_bringup/param/perception_real.yaml
ros2 run image_transport republish --ros-args -p in_transport:=compressed -p out_transport:=raw -r in/compressed:=$CAM/compressed -r out:=$CAM/decoded > /tmp/republish.log 2>&1 &
ros2 run autorace_perception bev_projector --ros-args --params-file $P -r /camera/image_input:=$CAM/decoded -r /camera/camera_info:=$INFO -r /camera/image_output:=/camera/image_projected > /tmp/bev.log 2>&1 &
ros2 run autorace_perception detect_lane --ros-args --params-file $P -r /detect/image_input:=/camera/image_projected -r /detect/image_output:=/detect/image_lane > /tmp/detect_lane.log 2>&1 &
echo "lane pipeline up (no controller). stop: kill %1 %2 %3 or pkill -f 'bev_projector|detect_lane|republish'"
wait
