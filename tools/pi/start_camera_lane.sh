#!/bin/bash
# Lane camera: the Logitech C920 (11 cm up, 40 deg down), 432x240 (16:9-ish, so the full 70 deg
# width is kept). Its own MJPEG frames go out untouched (passthrough): the Pi does no image work.
source /opt/ros/jazzy/setup.bash
# The middleware and discovery the PC uses too (tools/dds.sh); ROS_DOMAIN_ID 30 by default.
DDS_SH="$(dirname "$(readlink -f "$0")")/../dds.sh"
[ -f "$DDS_SH" ] || DDS_SH=$HOME/dds.sh
if [ -f "$DDS_SH" ]; then source "$DDS_SH"; else export ROS_DOMAIN_ID=30;
  echo "dds.sh not found: Fast DDS; the PC must not use CycloneDDS" >&2; fi
# The camera node from this workspace (a copy in the home is older: no bitrate, reliable QoS).
CAM="$(dirname "$(readlink -f "$0")")/../../src/autorace_bringup/autorace_bringup/csi_camera.py"
[ -f "$CAM" ] || { CAM=$HOME/csi_camera.py; echo "using the older $CAM" >&2; }
exec python3 "$CAM" --ros-args -r __ns:=/camera_lane \
  -p v4l2_device:=/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920-video-index0 \
  -p width:=432 -p height:=240 -p fps:=30 -p passthrough:=true -p fov_deg:=70.4 \
  -p frame_id:=camera_lane_link
