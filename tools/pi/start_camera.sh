#!/bin/bash
# Forward camera for the signs, traffic light and level crossing: the CSI camera (imx219, 62 deg)
# through the legacy V4L2 driver (bcm2835-v4l2, start_x=1). Found by its V4L2 name, because the
# C920 takes /dev/video0 when it is plugged in.
source /opt/ros/jazzy/setup.bash
# The middleware and discovery the PC uses too (tools/dds.sh); ROS_DOMAIN_ID 30 by default.
DDS_SH="$(dirname "$(readlink -f "$0")")/../dds.sh"
[ -f "$DDS_SH" ] || DDS_SH=$HOME/dds.sh
if [ -f "$DDS_SH" ]; then source "$DDS_SH"; else export ROS_DOMAIN_ID=30;
  echo "dds.sh not found: Fast DDS; the PC must not use CycloneDDS" >&2; fi
# The camera node from this workspace (a copy in the home is older: no bitrate, reliable QoS).
CAM="$(dirname "$(readlink -f "$0")")/../../src/autorace_bringup/autorace_bringup/csi_camera.py"
[ -f "$CAM" ] || { CAM=$HOME/csi_camera.py; echo "using the older $CAM" >&2; }
# The camera's own MJPEG (the GPU's, rate controlled) goes out as it comes: the Pi no longer decodes
# and re-encodes every frame (at 640x480 that cost CPU and time). 5 Mbit/s: about the 586 KB/s the
# re-encoded stream took, ~40 KB a frame; 2 Mbit/s was visibly blocky.
exec python3 "$CAM" --ros-args -r __ns:=/camera -p v4l2_name:=camera0 \
  -p width:=640 -p height:=480 -p fps:=15 -p passthrough:=true -p bitrate:=5000000 \
  -p frame_id:=camera_link
