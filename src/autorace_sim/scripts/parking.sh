#!/bin/bash
# Park the house robot in the left or right bay: parking.sh left|right
set -e
BAY=${1:?usage: parking.sh left|right}
PARAMS=$(ros2 pkg prefix --share autorace_sim)/params/course.yaml
read X Y < <(python3 -c "
import yaml; b = yaml.safe_load(open('$PARAMS'))['course']['parking_bays']['$BAY']
print(b['x'], b['y'])")
ros2 service call /world/autorace/set_pose ros_gz_interfaces/srv/SetEntityPose \
  "{entity: {name: 'house_robot', type: 2}, pose: {position: {x: $X, y: $Y, z: 0.095}, \
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}" > /dev/null
echo "house robot: $BAY bay"
