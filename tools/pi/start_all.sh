#!/bin/bash
# Everything on the robot, detached: base (OpenCR), lidar, forward camera (CSI), lane camera (C920).
# Logs and pids in $HOME.
cd $HOME
for n in robot lidar camera camera_lane; do   # camera: Pi CSI camera, forward (signs); camera_lane: C920 (lane)
  [ -f $n.pid ] && kill $(cat $n.pid) 2>/dev/null
  if [ $n = lidar ]; then
    # turtlebot3_bringup also starts a lidar driver (LDS-01's hlds_laser_publisher, or LDS-02's
    # ld08_driver with LDS_MODEL=LDS-02); it holds /dev/ttyUSB0 and cannot read this LDS-02
    # (LDRobot protocol at 115200), so ld_lidar would get nothing.
    for p in $(pgrep -f 'hls_lfcd_lds_driver/hlds_laser_publishe[r]|ld08_driver/ld08_drive[r]'); do kill $p; done
    sleep 1
  fi
  setsid nohup ./start_$n.sh > $n.log 2>&1 < /dev/null & echo $! > $n.pid
  sleep 3
done
echo "started: robot lidar camera camera_lane (pids: $(cat robot.pid) $(cat lidar.pid) $(cat camera.pid) $(cat camera_lane.pid))"
