#!/bin/bash
source "$(dirname "$(readlink -f "$0")")/pi_env.sh"
exec ros2 run autorace_bringup ld_lidar
