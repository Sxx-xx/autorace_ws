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
# start_all.sh starts each under setsid, so the pid is its process group: kill the whole group, or
# a node of a killed `ros2 launch` lives on and keeps its device.
stop_pid() {
  local n p
  for n in "$@"; do
    [ -f "$HOME/$n.pid" ] || continue
    p=$(cat "$HOME/$n.pid")
    kill -- -$p 2>/dev/null || kill $p 2>/dev/null
    for _ in 1 2 3 4 5; do pgrep -g $p >/dev/null || break; sleep 1; done
    pkill -KILL -g $p 2>/dev/null
    rm -f "$HOME/$n.pid"
  done
}
