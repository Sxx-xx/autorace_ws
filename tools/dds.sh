#!/bin/bash
# dds.sh : the same middleware and discovery on the PC and on the robot. Source it, don't run it:
#   PC, every terminal:  export DDS_PEERS=turtlebot1.local; source tools/dds.sh
#   robot:               the tools/pi/start_*.sh scripts source it
# CycloneDDS when it is installed (sudo apt install ros-jazzy-rmw-cyclonedds-cpp) - on BOTH
# machines: a Cyclone PC and a Fast DDS robot never see each other. Without it, or with DDS=fastdds,
# both stay on Jazzy's default, Fast DDS. tools/check_robot.sh compares the two sides.
#   DDS_PEERS  hosts to look for the other side at by unicast besides multicast (the PC names the
#              robot; a phone hotspot or a laptop access point may drop multicast). Names that do
#              not resolve are left out with a warning, so a bad name cannot stop the nodes.
#   DDS_IFACE  the network interface to use when there are several (wlan0, eth0, ...).
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-30}
if [ "${DDS:-cyclone}" = cyclone ] && [ -e /opt/ros/jazzy/lib/librmw_cyclonedds_cpp.so ]; then
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  _peers=""
  for _h in ${DDS_PEERS:-}; do
    if getent hosts "$_h" > /dev/null 2>&1; then
      _peers="$_peers<Peer address=\"$_h\"/>"
    else
      echo "dds.sh: '$_h' does not resolve; left out of the peers" >&2
    fi
  done
  _iface=""
  [ -n "${DDS_IFACE:-}" ] && _iface="<Interfaces><NetworkInterface name=\"$DDS_IFACE\"/></Interfaces>"
  export CYCLONEDDS_URI="<CycloneDDS><Domain id=\"any\"><General>$_iface</General><Discovery><Peers>$_peers</Peers></Discovery></Domain></CycloneDDS>"
  unset _peers _h _iface
else
  unset RMW_IMPLEMENTATION CYCLONEDDS_URI
  echo "dds.sh: Fast DDS (CycloneDDS not installed, or DDS=${DDS:-}): the other side must use Fast DDS too" >&2
fi
