#!/bin/bash
# color_replay.sh BAG KIND [OVERRIDES...] : replay a colour capture in a loop and show one detector on it.
#
# Runs on ROS_DOMAIN_ID 77, so a running robot (30) is not touched. KIND:
#   lane   lane detector (white / yellow lines) on the lane camera; window /detect/image_lane
#          overrides: detect.lane.white.lightness_l:=180 ...
#   light  tools/traffic_light_view.py on the forward camera (click: HSV)
#          overrides: --val 180 --sat 80 ...
#   sign   sign detector on the forward camera; window /detect/image_sign
#          overrides: blue.saturation_l:=100 ...
#   bar    tools/level_crossing_view.py on the forward camera (click: HSV)
#          overrides: -p red.value_l:=25 ...
# Ctrl-C (or q in the view window) stops everything. Values found here go into
# src/autorace_bringup/param/*.yaml by hand (체크리스트.md).
cd "$(dirname "$0")/.."
export PATH=/usr/bin:$PATH   # the system python: conda's has no rclpy
source /opt/ros/jazzy/setup.bash; source install/setup.bash
export ROS_DOMAIN_ID=77
BAG=${1:?bag}; KIND=${2:?lane, light, sign or bar}; shift 2
PERC=$(ros2 pkg prefix autorace_bringup)/share/autorace_bringup/param/perception_real.yaml
MISS=$(ros2 pkg prefix autorace_bringup)/share/autorace_bringup/param/mission_real.yaml
LIB=$(ros2 pkg prefix autorace_perception)/lib/autorace_perception
RP=/opt/ros/jazzy/lib/image_transport/republish
trap 'kill -TERM $(jobs -p) 2>/dev/null; sleep 1; kill -KILL $(jobs -p) 2>/dev/null' EXIT INT TERM
P=(); for a in "$@"; do P+=(-p "$a"); done
case $KIND in
  lane)
    CAM=/camera_lane/image_raw
    $RP --ros-args -p in_transport:=compressed -p out_transport:=raw \
      -r in/compressed:=$CAM/compressed -r out:=$CAM/decoded > /dev/null 2>&1 &
    $LIB/bev_projector --ros-args --params-file $PERC -p camera.fx:=306.2 -p camera.fy:=306.2 \
      -p camera.cx:=216.0 -p camera.cy:=120.0 \
      -r /camera/image_input:=$CAM/decoded -r /camera/image_output:=/camera/image_projected > /dev/null 2>&1 &
    $LIB/detect_lane --ros-args --params-file $PERC "${P[@]}" \
      -r /detect/image_input:=/camera/image_projected -r /detect/image_output:=/detect/image_lane > /dev/null 2>&1 &
    ros2 run rqt_image_view rqt_image_view /detect/image_lane > /dev/null 2>&1 & VIEW=$!
    TOPICS=$CAM/compressed ;;
  sign)
    $RP --ros-args -p in_transport:=compressed -p out_transport:=raw \
      -r in/compressed:=/camera/image_raw/compressed -r out:=/camera/image_decoded > /dev/null 2>&1 &
    $LIB/detect_sign --ros-args --params-file $MISS "${P[@]}" \
      -r /detect/image_input:=/camera/image_decoded > /dev/null 2>&1 &
    ros2 run rqt_image_view rqt_image_view /detect/image_sign > /dev/null 2>&1 & VIEW=$!
    TOPICS=/camera/image_raw/compressed ;;
  light)
    /usr/bin/python3 tools/traffic_light_view.py "$@" & VIEW=$!
    TOPICS=/camera/image_raw/compressed ;;
  bar)
    /usr/bin/python3 tools/level_crossing_view.py "$@" & VIEW=$!
    TOPICS=/camera/image_raw/compressed ;;
  *) echo "KIND: lane, light, sign or bar"; exit 1 ;;
esac
sleep 2
ros2 bag play "$BAG" --loop --topics $TOPICS > /tmp/color_replay_play.log 2>&1 & PLAY=$!
# Until the view window is closed or the playing stops.
wait -n $VIEW $PLAY
