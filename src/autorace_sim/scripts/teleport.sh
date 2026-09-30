#!/bin/bash
# Move the robot to a world pose: teleport.sh X Y [YAW_RAD] [MODEL]
set -e
X=${1:?usage: teleport.sh X Y [YAW] [MODEL]}
Y=${2:?usage: teleport.sh X Y [YAW] [MODEL]}
YAW=${3:-0.0}
MODEL=${4:-autorace_burger}
QZ=$(python3 -c "import math; print(math.sin($YAW/2))")
QW=$(python3 -c "import math; print(math.cos($YAW/2))")
ros2 service call /world/autorace/set_pose ros_gz_interfaces/srv/SetEntityPose \
  "{entity: {name: '${MODEL}', type: 2}, pose: {position: {x: ${X}, y: ${Y}, z: 0.02}, \
    orientation: {x: 0.0, y: 0.0, z: ${QZ}, w: ${QW}}}}"
