#!/bin/bash
# The PC (WSL2) side of the clock: nothing of its own. WSL2's clock is set from Windows' by Hyper-V,
# Windows serves NTP to the Pi (tools/windows/time_server.ps1, admin), and the Pi follows it
# (tools/pi/setup_chrony.sh). A time daemon inside WSL only fights Hyper-V (measured 2026-10-22:
# chrony following the Pi was pulled back 43 ms within 30 s), and in mirrored networking it shares
# the PC's address and port 123 with Windows' server. So this stops chrony (and timesyncd) here.
# Run once on the PC:  sudo bash tools/setup_chrony_pc.sh
set -eu
[ "$(id -u)" = 0 ] || { echo "run with sudo"; exit 1; }
for s in chrony systemd-timesyncd; do
  systemctl disable --now $s 2>/dev/null || true
done
systemctl is-active chrony systemd-timesyncd 2>/dev/null | paste -sd' ' || true
echo "WSL's clock: Hyper-V from Windows; check the Pi with  ssh <pi> chronyc sources"
