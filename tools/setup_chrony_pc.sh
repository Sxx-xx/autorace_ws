#!/bin/bash
# The PC's clock follows the Pi's (tools/pi/setup_chrony.sh first): chrony instead of
# systemd-timesyncd, the Pi as the preferred server, polled every 1-16 s, and the clock stepped
# whenever it is off by more than 0.1 s (WSL2 jumps after the laptop sleeps). The internet pools
# stay as a fallback away from the robot.
# Run once on the PC:  sudo bash tools/setup_chrony_pc.sh [Pi address, default 172.20.10.4]
set -eu
[ "$(id -u)" = 0 ] || { echo "run with sudo"; exit 1; }
PI=${1:-172.20.10.4}
apt-get install -y chrony          # replaces systemd-timesyncd
grep -q '^sourcedir /etc/chrony/sources.d' /etc/chrony/chrony.conf \
  || echo 'sourcedir /etc/chrony/sources.d' >> /etc/chrony/chrony.conf
grep -q '^confdir /etc/chrony/conf.d' /etc/chrony/chrony.conf \
  || echo 'confdir /etc/chrony/conf.d' >> /etc/chrony/chrony.conf
cat > /etc/chrony/sources.d/autorace.sources <<EOF
# AutoRace (tools/setup_chrony_pc.sh): the robot's Pi
server $PI iburst minpoll 0 maxpoll 4 prefer
EOF
cat > /etc/chrony/conf.d/autorace.conf <<'EOF'
# AutoRace (tools/setup_chrony_pc.sh): step at once when off by more than 0.1 s, at any time.
makestep 0.1 -1
EOF
systemctl enable --now chrony
systemctl restart chrony
sleep 8
chronyc sources
chronyc tracking | sed -n '1,5p'
