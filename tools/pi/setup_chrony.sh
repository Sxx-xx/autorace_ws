#!/bin/bash
# The Pi's clock for the PC: chrony instead of systemd-timesyncd, synced to the internet when it can
# be and serving its time to the PC on the local network either way. The PC follows it
# (tools/setup_chrony_pc.sh), so the two agree to a few ms however bad the hotspot's internet is:
# detect_lane and the construction and parking missions match image and scan stamps to /odom.
# Run once on the Pi:  sudo bash ~/autorace_ws/tools/pi/setup_chrony.sh
set -eu
[ "$(id -u)" = 0 ] || { echo "run with sudo"; exit 1; }
apt-get install -y chrony          # replaces systemd-timesyncd
grep -q '^confdir /etc/chrony/conf.d' /etc/chrony/chrony.conf \
  || echo 'confdir /etc/chrony/conf.d' >> /etc/chrony/chrony.conf
cat > /etc/chrony/conf.d/autorace.conf <<'EOF'
# AutoRace (tools/pi/setup_chrony.sh): serve this clock to the PC on the local networks,
# and keep serving it with no internet (the PC needs agreement, not the true time).
allow 172.16.0.0/12
allow 192.168.0.0/16
allow 10.0.0.0/8
local stratum 10
# Step at once when off by more than 0.1 s, at any time (not only at boot).
makestep 0.1 -1
EOF
if ufw status | grep -q 'Status: active'; then
  for net in 172.16.0.0/12 192.168.0.0/16 10.0.0.0/8; do
    ufw allow from $net to any port 123 proto udp comment 'AutoRace NTP for the PC'
  done
fi
systemctl enable --now chrony
systemctl restart chrony
sleep 3
chronyc tracking | sed -n '1,5p'
echo "serving NTP on udp/123 to the local networks"
