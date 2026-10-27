#!/bin/bash
# Everything on the robot, detached: base (OpenCR), the cmd_vel watchdog, lidar, cameras (CSI
# forward, C920 lane if plugged in). Logs and pids in $HOME. Names restart only those: start_all.sh cameras. stop_all.sh stops.
source "$(dirname "$(readlink -f "$0")")/pi_env.sh"
cd $HOME
# turtlebot3_bringup also starts a lidar driver (LDS-01's hlds_laser_publisher, or LDS-02's
# ld08_driver with LDS_MODEL=LDS-02); it holds /dev/ttyUSB0 and cannot read this LDS-02
# (LDRobot protocol at 115200), so ld_lidar would get nothing.
kill_tb3_lidar() {
  for p in $(pgrep -f 'hls_lfcd_lds_driver/hlds_laser_publishe[r]|ld08_driver/ld08_drive[r]'); do kill $p; done
}
for n in ${*:-robot watchdog lidar cameras}; do
  case $n in
    camera|camera_lane|cameras) stop_pid camera camera_lane cameras ;;   # one opener of each device
    *) stop_pid $n ;;
  esac
  if [ $n = lidar ]; then
    kill_tb3_lidar
    sleep 1
  fi
  setsid nohup "$PI_DIR/start_$n.sh" > $n.log 2>&1 < /dev/null & echo $! > $n.pid
  sleep 3
  if [ $n = robot ]; then
    # The base on its own too (start_all.sh robot, ld_lidar already up): its lidar driver comes
    # up a moment after the launch and then starves ld_lidar (2026-10-27: no /scan for the tunnel).
    # Once it has had the port, ld_lidar stays up but sends nothing until it is restarted.
    for i in 1 2 3 4 5 6; do
      if pgrep -f 'hls_lfcd_lds_driver/hlds_laser_publishe[r]|ld08_driver/ld08_drive[r]' > /dev/null; then
        kill_tb3_lidar
        restart_lidar=1     # whichever order, an ld_lidar already up has been starved
        break
      fi
      sleep 1
    done
  fi
done
if [ -n "$restart_lidar" ] && [ -f lidar.pid ] && kill -0 $(cat lidar.pid) 2>/dev/null; then
  stop_pid lidar
  sleep 1
  setsid nohup "$PI_DIR/start_lidar.sh" > lidar.log 2>&1 < /dev/null & echo $! > lidar.pid
  echo "restarted lidar (the base's own lidar driver had the port; pid $(cat lidar.pid))"
fi
for n in ${*:-robot watchdog lidar cameras}; do echo "started $n (pid $(cat $n.pid), log ~/$n.log)"; done
