# Sourced by the start scripts: ROS, this workspace (found from where the scripts are, so ~/start_*.sh
# may be symlinks to them) and the same DDS settings as ~/.bashrc, which stops early over ssh.
PI_DIR=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")
WS=$(cd "$PI_DIR/../.." && pwd)
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-30} RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
[ -z "${CYCLONEDDS_URI:-}" ] && [ -f "$HOME/cyclonedds.xml" ] && export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
export TURTLEBOT3_MODEL=burger
# stop_pid NAME...: kill what ~/NAME.pid names (both cameras' scripts open /dev/video0: one at a time).
stop_pid() {
  local n
  for n in "$@"; do
    [ -f "$HOME/$n.pid" ] && kill $(cat "$HOME/$n.pid") 2>/dev/null
    rm -f "$HOME/$n.pid"
  done
}
