#!/bin/bash
# [로봇] 차선 카메라(C920) 원본 화면만 띄운다. 바퀴·인식·제어 없음.
# 결과: PC 브라우저 http://<로봇IP>:8090 에 원본 카메라 1개. Ctrl+C 로 종료.
DIR="$(cd "$(dirname "$0")" && pwd)"
source "$DIR/_env.sh"
"$DIR/전체종료.sh" quiet
PARAMS="$WS/install/autorace_bringup/share/autorace_bringup/param/perception_real.yaml"
trap 'kill 0' INT TERM HUP EXIT
ros2 run autorace_perception usb_camera --ros-args --params-file "$PARAMS" \
  -r image_raw:=/camera_lane/image_raw -r camera_info:=/camera_lane/camera_info &
ros2 run autorace_perception web_view --ros-args --params-file "$PARAMS" \
  -p "topics:=['/camera_lane/image_raw']" -p "labels:=['차선 카메라 원본']" &
sleep 4
echo
echo "브라우저: http://$(hostname -I | awk '{print $1}'):8090    (종료: Ctrl+C)"
wait
