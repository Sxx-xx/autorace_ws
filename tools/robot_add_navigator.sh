#!/bin/bash
# robot_add_navigator.sh : teach the robot (Pi) the laptop's own Wi-Fi "Navigator" (robot-ap).
#
# The laptop's internal Wi-Fi makes "Navigator" (nmcli con up robot-ap); the robot then
# talks to the laptop directly instead of through a crowded phone hotspot. This adds
# Navigator to the Pi's netplan (backed up first), checks it and applies it. The Wi-Fi
# password is read from the laptop's robot-ap profile and never printed; the Pi asks
# for dd's sudo password once. PROFILE=robot-router teaches it the ipTIME router
# (iptime5G) instead; any laptop Wi-Fi profile name works. After the apply the Pi may move to Navigator at once
# and this ssh session drop: that is expected. Find it then on 10.42.0.x.
set -e
ROBOT=${ROBOT:-dd@turtlebot1.local}
PROFILE=${PROFILE:-robot-ap}
PSK=$(nmcli -s -g 802-11-wireless-security.psk con show "$PROFILE")
SSID=$(nmcli -g 802-11-wireless.ssid con show "$PROFILE")
[ -n "$PSK" ] && [ -n "$SSID" ] || { echo "$PROFILE profile not readable"; exit 1; }
B64=$(printf '%s' "$PSK" | base64)
ssh -t "$ROBOT" "sudo sh -c '
  set -e
  mkdir -p /root/netplan_backup && cp -a /etc/netplan/. /root/netplan_backup/\$(date +%m%d_%H%M%S)/ 2>/dev/null || cp -a /etc/netplan /root/netplan_backup/\$(date +%m%d_%H%M%S)
  P=\$(echo $B64 | base64 -d)
  netplan set \"wifis.wlan0.access-points.$SSID.password=\$P\"
  netplan generate
  echo \"$SSID added; Wi-Fi networks known now:\"
  netplan get wifis.wlan0.access-points | grep -E \"^[^ ].*:\$\"
  nohup netplan apply > /tmp/netplan_apply.log 2>&1 &
  echo \"applied (the robot may now move to $SSID; this session may drop)\"
'"
