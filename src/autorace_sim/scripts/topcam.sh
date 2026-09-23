#!/bin/bash
# Spawn the overhead debug camera and bridge its image to ROS.
set -e
PKG=$(ros2 pkg prefix --share autorace_sim)
ros2 run ros_gz_sim create -world autorace \
  -file "${PKG}/models/autorace_topcam/model.sdf" -name autorace_topcam
ros2 run ros_gz_image image_bridge /topcam/image_raw
