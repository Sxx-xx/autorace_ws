#!/bin/bash
# check_sync.sh : does the Pi run what the repository says? Run ON THE PI from the workspace:
#   bash ~/<workspace>/tools/pi/check_sync.sh
# The start scripts use copies in the Pi's home (~/csi_camera.py, ~/ld_lidar.py,
# ~/tb3_burger_twist.yaml, ~/dds.sh, ~/start_*.sh), so a change in the repository does nothing on
# the robot until those are brought in line. This compares them and prints the command for each one
# that differs; it changes nothing itself. A link (ln -sf) into the workspace keeps a file in line
# from then on; a copy drifts again with the next change.
WS=$(cd "$(dirname "$(readlink -f "$0")")/../.." && pwd)
differ=0
same() {   # same HOME_FILE REPO_FILE
  local home=$1 repo=$2
  if [ ! -e "$home" ]; then
    echo "  MISSING  $home   (ln -sf $repo $home)"; differ=1
  elif [ -L "$home" ] && [ "$(readlink -f "$home")" = "$(readlink -f "$repo")" ]; then
    echo "  ok       $home -> repository"
  elif cmp -s "$home" "$repo"; then
    echo "  ok       $home (same as the repository)"
  else
    echo "  DIFFERS  $home   (see: diff $home $repo; keep any change worth keeping in the"
    echo "           repository, then link it: ln -sf $repo $home)"; differ=1
  fi
}
echo "== files the start scripts run from the home"
same ~/csi_camera.py "$WS/src/autorace_bringup/autorace_bringup/csi_camera.py"
same ~/ld_lidar.py "$WS/src/autorace_bringup/autorace_bringup/ld_lidar.py"
same ~/tb3_burger_twist.yaml "$WS/tools/pi/tb3_burger_twist.yaml"
same ~/dds.sh "$WS/tools/dds.sh"
for f in "$WS"/tools/pi/start_*.sh; do
  same ~/"$(basename "$f")" "$f"
done
echo "== middleware"
if [ -e /opt/ros/jazzy/lib/librmw_cyclonedds_cpp.so ]; then
  echo "  ok       CycloneDDS installed (the PC must have it too)"
else
  echo "  MISSING  CycloneDDS (sudo apt install ros-jazzy-rmw-cyclonedds-cpp, on the PC too)"; differ=1
fi
echo "== Wi-Fi"
ps=$(iw dev wlan0 get power_save 2>/dev/null)
echo "  ${ps:-wlan0 not found}   (start_all.sh turns it off; needs /etc/sudoers.d/iw-powersave)"
[ -e /etc/sudoers.d/iw-powersave ] || { echo "  MISSING  /etc/sudoers.d/iw-powersave"; differ=1; }
[ $differ = 0 ] && echo "== all in line" || echo "== some differ: see above"
