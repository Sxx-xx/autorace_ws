#!/bin/bash
# Zero velocity when the PC's /cmd_vel stops coming (Wi-Fi drop, PC node gone): cmd_vel_watchdog.
source "$(dirname "$(readlink -f "$0")")/pi_env.sh"
exec ros2 run autorace_bringup cmd_vel_watchdog "$@"
