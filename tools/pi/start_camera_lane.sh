#!/bin/bash
# Lane camera alone (start_cameras.sh runs both): the Logitech C920 (11 cm up, 40 deg down), 432x240
# (16:9-ish, so the full 70 deg width is kept). Its own MJPEG frames go out untouched (passthrough).
source "$(dirname "$(readlink -f "$0")")/pi_env.sh"
exec ros2 run autorace_bringup csi_camera --ros-args -r __ns:=/camera_lane \
  -p v4l2_device:=/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920-video-index0 \
  -p width:=432 -p height:=240 -p fps:=${CAMERA_FPS:-30} -p passthrough:=true -p fov_deg:=70.4 \
  -p frame_id:=camera_lane_link
