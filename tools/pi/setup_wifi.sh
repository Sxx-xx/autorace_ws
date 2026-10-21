#!/bin/bash
# The Pi's Wi-Fi for the phone's 5 GHz hotspot: power saving off, at every boot. With it on (the
# brcmfmac default) the radio dozes between beacons, and frames to and from the PC wait for it to
# wake. Measured 2026-10-22 before this: wlan0 at 5.745 GHz (channel 149), 433 Mb/s, power
# management on; regulatory domain KR (cmdline) already allows the 5 GHz channels.
# The band is not pinned in netplan: with Maximize Compatibility off the iPhone offers 5 GHz only,
# so a pin gains nothing and would keep the Pi off the hotspot if it were ever turned back on.
# Run once on the Pi:  sudo bash ~/autorace_ws/tools/pi/setup_wifi.sh      (undo: ... --undo)
set -eu
[ "$(id -u)" = 0 ] || { echo "run with sudo"; exit 1; }
UNIT=/etc/systemd/system/wifi-powersave-off.service
if [ "${1:-}" = --undo ]; then
  systemctl disable --now wifi-powersave-off.service 2>/dev/null || true
  rm -f "$UNIT"; systemctl daemon-reload
  iw dev wlan0 set power_save on || true
  iw dev wlan0 get power_save || true
  exit 0
fi
command -v iw >/dev/null || apt-get install -y iw
cat > "$UNIT" <<'EOF'
[Unit]
Description=AutoRace: Wi-Fi power saving off (tools/pi/setup_wifi.sh)
After=sys-subsystem-net-devices-wlan0.device network.target
Wants=sys-subsystem-net-devices-wlan0.device

[Service]
Type=oneshot
ExecStart=/usr/sbin/iw dev wlan0 set power_save off
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
systemctl daemon-reload
systemctl enable --now wifi-powersave-off.service
iw dev wlan0 get power_save
iw dev wlan0 link | grep -E 'SSID|freq|tx bitrate'
