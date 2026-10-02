#!/bin/bash
# Show the left or right direction sign at the intersection, or one picked at
# random as on race day: intersection.sh left|right|random
# The other one goes back below the floor.
set -e
SHOW=${1:?usage: intersection.sh left|right|random}
if [ "$SHOW" = random ]; then SHOW=$([ $((RANDOM % 2)) = 0 ] && echo left || echo right); fi
case $SHOW in left) HIDE=right ;; right) HIDE=left ;; *) echo "left, right or random" >&2; exit 1 ;; esac
PARAMS=$(ros2 pkg prefix --share autorace_sim)/params/course.yaml
read X Y YAW < <(python3 -c "
import yaml; s = yaml.safe_load(open('$PARAMS'))['course']['intersection_sign']
print(s['x'], s['y'], s['yaw'])")
move() {
  QZ=$(python3 -c "import math; print(math.sin($5/2))")
  QW=$(python3 -c "import math; print(math.cos($5/2))")
  ros2 service call /world/autorace/set_pose ros_gz_interfaces/srv/SetEntityPose \
    "{entity: {name: '$1', type: 2}, pose: {position: {x: $2, y: $3, z: $4}, \
      orientation: {x: 0.0, y: 0.0, z: $QZ, w: $QW}}}" > /dev/null
}
move autorace_sign_$HIDE 0.0 0.0 -5.0 0.0
move autorace_sign_$SHOW $X $Y 0.125 $YAW
echo "intersection sign: $SHOW"
