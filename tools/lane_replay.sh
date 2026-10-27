#!/bin/bash
# lane_replay.sh BAG [START_S] [PARAM:=VALUE ...] : the lane pipeline (no control) fed from a bag.
#
# Plays the bag's lane camera (/camera_lane/image_raw/compressed) from START_S and runs
# republish -> bev_projector -> detect_lane on ROS_DOMAIN_ID 77, so a running robot (30) is
# not touched. Extra arguments are detect_lane parameter overrides, e.g.
#   tools/lane_replay.sh bags/construction+parking_190053 60 detect.glare_fill_m:=0.0
# Watch with ROS_DOMAIN_ID=77 ros2 run rqt_image_view rqt_image_view /detect/image_lane
# (or /camera/image_projected). Ctrl-C stops everything.
#
# The bags carry no camera_info: the C920's intrinsics from param/camera_lane.yaml are
# passed to the projector (432x240, fx 306.2, centre 216 x 120).
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash; source install/setup.bash
export ROS_DOMAIN_ID=77
BAG=$1; START=${2:-0}; shift 2 2>/dev/null
OVERRIDES=(); for a in "$@"; do OVERRIDES+=(-p "$a"); done
CAM=/camera_lane/image_raw
P=$(ros2 pkg prefix autorace_bringup)/share/autorace_bringup/param/perception_real.yaml
LIB=$(ros2 pkg prefix autorace_perception)/lib/autorace_perception
/opt/ros/jazzy/lib/image_transport/republish --ros-args -p in_transport:=compressed -p out_transport:=raw \
  -r in/compressed:=$CAM/compressed -r out:=$CAM/decoded > /dev/null 2>&1 &
$LIB/bev_projector --ros-args --params-file $P -p camera.fx:=306.2 -p camera.fy:=306.2 -p camera.cx:=216.0 -p camera.cy:=120.0 \
  -r /camera/image_input:=$CAM/decoded -r /camera/image_output:=/camera/image_projected > /dev/null 2>&1 &
$LIB/detect_lane --ros-args --params-file $P "${OVERRIDES[@]}" \
  -r /detect/image_input:=/camera/image_projected -r /detect/image_output:=/detect/image_lane &
PIDS="$!"
sleep 3
trap 'kill -TERM $(jobs -p) 2>/dev/null; sleep 1; kill -KILL $(jobs -p) 2>/dev/null' EXIT INT TERM
ros2 bag play "$BAG" --start-offset "$START" --topics $CAM/compressed
