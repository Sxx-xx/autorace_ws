#!/bin/bash
# color_capture.sh SCENE [SECONDS] : record both cameras for a few seconds, for tuning colours later.
#
# Only the two compressed camera streams (and their camera_info) go into
# bags/color/SCENE_HHMMSS, SECONDS long (default 5). Nothing else needs to run
# but the robot's ~/start_all.sh: no launch, the robot does not move. Hold or
# place the robot so the camera sees the scene, then run, e.g.
#   tools/color_capture.sh light_green
#   tools/color_capture.sh lane_curve 8
# Tune afterwards from the recording with tools/color_replay.sh (체크리스트.md 1-0).
cd "$(dirname "$0")/.."
export PATH=/usr/bin:$PATH   # the system python: conda's has no rclpy
source /opt/ros/jazzy/setup.bash; source install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-30}
SCENE=${1:?scene name, e.g. light_green}; SECONDS_=${2:-5}
mkdir -p bags/color
OUT=bags/color/${SCENE}_$(date +%H%M%S)
timeout -s INT $((SECONDS_ + 3)) ros2 bag record -o "$OUT" \
  /camera/image_raw/compressed /camera_lane/image_raw/compressed \
  /camera/camera_info /camera_lane/camera_info > /dev/null 2>&1
sleep 1
if [ -f "$OUT/metadata.yaml" ]; then
  echo "saved $OUT"
  ros2 bag info "$OUT" 2>/dev/null | grep -E "image_raw/compressed" | sed 's/^ */  /'
else
  echo "nothing saved (is the robot up? tools/check_robot.sh)"
fi
