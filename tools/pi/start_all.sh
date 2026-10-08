#!/bin/bash
# Everything on the robot, detached: base (OpenCR), lidar, forward camera (CSI), lane camera (C920).
# Logs and pids in $HOME.
cd $HOME
for n in robot lidar camera_lane; do   # the Pi camera (start_camera.sh) is unused: the C920 reads signs too
  [ -f $n.pid ] && kill $(cat $n.pid) 2>/dev/null
  if [ $n = lidar ]; then
    # turtlebot3_bringup also starts the LDS-01 driver; it holds /dev/ttyUSB0 and cannot read
    # this lidar (LDRobot protocol at 115200), so ld_lidar would get nothing.
    for p in $(pgrep -f 'hls_lfcd_lds_driver/hlds_laser_publishe[r]'); do kill $p; done
    sleep 1
  fi
  setsid nohup ./start_$n.sh > $n.log 2>&1 < /dev/null & echo $! > $n.pid
  sleep 3
done
echo "started: robot lidar camera_lane (pids: $(cat robot.pid) $(cat lidar.pid) $(cat camera_lane.pid))"
