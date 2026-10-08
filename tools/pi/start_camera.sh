#!/bin/bash
# Forward camera for the signs, traffic light and level crossing: the CSI camera (imx219, 62 deg)
# through the legacy V4L2 driver (bcm2835-v4l2, start_x=1). Found by its V4L2 name, because the
# C920 takes /dev/video0 when it is plugged in.
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=30
exec python3 $HOME/csi_camera.py --ros-args -r __ns:=/camera -p v4l2_name:=camera0 \
  -p width:=320 -p height:=240 -p fps:=30 -p frame_id:=camera_link
