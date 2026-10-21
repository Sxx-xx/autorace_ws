#!/bin/bash
# The Pi's clock follows the PC's: chrony instead of systemd-timesyncd, with the PC (Windows,
# serving NTP through tools/windows/time_server.ps1) as the preferred server, polled every 1-16 s,
# and the internet pools as a fallback. The two then agree to a few ms however bad the hotspot's
# internet is: detect_lane and the construction and parking missions match image and scan stamps
# to /odom. WSL2 takes Windows' clock through Hyper-V (tools/setup_chrony_pc.sh leaves it so).
# History (2026-10-22): the Pi as the server did not work: Windows polled it every 1024 s and kept
# a 47 ms offset, and chrony inside WSL was undone by Hyper-V within seconds.
# Run once on the Pi:  sudo bash ~/autorace_ws/tools/pi/setup_chrony.sh [PC address, default 172.20.10.2]
set -eu
[ "$(id -u)" = 0 ] || { echo "run with sudo"; exit 1; }
PC=${1:-172.20.10.2}
command -v chronyd >/dev/null || apt-get install -y chrony          # replaces systemd-timesyncd
for d in 'confdir /etc/chrony/conf.d' 'sourcedir /etc/chrony/sources.d'; do
  grep -q "^$d" /etc/chrony/chrony.conf || echo "$d" >> /etc/chrony/chrony.conf
done
cat > /etc/chrony/sources.d/autorace.sources <<EOF
# AutoRace (tools/pi/setup_chrony.sh): the PC
server $PC iburst minpoll 0 maxpoll 4 prefer trust
EOF
cat > /etc/chrony/conf.d/autorace.conf <<'EOF'
# AutoRace (tools/pi/setup_chrony.sh): step at once when off by more than 0.1 s, at any time
# (not only at boot).
makestep 0.1 -1
# Windows announces a root dispersion of several seconds (it syncs to the internet rarely), which
# the default 3 s maximum rejects; agreement with the PC is what counts here, not the true time.
maxdistance 16
EOF
# An earlier version served NTP from here; its firewall rules are no longer needed.
if ufw status | grep -q 'AutoRace NTP for the PC'; then
  for net in 172.16.0.0/12 192.168.0.0/16 10.0.0.0/8; do
    ufw delete allow from $net to any port 123 proto udp || true
  done
fi
systemctl enable --now chrony
systemctl restart chrony
sleep 8
chronyc sources | grep -E "^MS|^\^"
chronyc tracking | sed -n '1,5p'
