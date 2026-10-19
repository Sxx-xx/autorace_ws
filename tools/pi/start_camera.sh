#!/bin/bash
# Forward camera alone (start_cameras.sh runs both): the CSI camera (imx219, 62 deg) through the
# legacy V4L2 driver (bcm2835-v4l2, start_x=1). Found by its V4L2 name, because the C920 takes
# /dev/video0 when it is plugged in.
source "$(dirname "$(readlink -f "$0")")/pi_env.sh"
exec ros2 run autorace_bringup csi_camera --ros-args -r __ns:=/camera -p v4l2_name:=camera0 \
  -p width:=320 -p height:=240 -p fps:=${CAMERA_FPS:-30} -p frame_id:=camera_link
