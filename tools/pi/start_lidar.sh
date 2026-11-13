#!/bin/bash
source /opt/ros/jazzy/setup.bash
# The middleware and discovery the PC uses too (tools/dds.sh); ROS_DOMAIN_ID 30 by default.
DDS_SH="$(dirname "$(readlink -f "$0")")/../dds.sh"
[ -f "$DDS_SH" ] || DDS_SH=$HOME/dds.sh
if [ -f "$DDS_SH" ]; then source "$DDS_SH"; else export ROS_DOMAIN_ID=30;
  echo "dds.sh not found: Fast DDS; the PC must not use CycloneDDS" >&2; fi
# The lidar sits with its zero pointing backwards: what the camera saw ahead (the parking
# sign, the parked robot) the scan put behind, and only with 180 deg do the returns stay
# put in the world as the robot moves (bags of 2026-10-27).
exec python3 $HOME/ld_lidar.py --ros-args -p angle_offset_deg:=180.0
