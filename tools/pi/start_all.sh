#!/bin/bash
# Everything on the robot, detached: base (OpenCR), lidar, forward camera (CSI), lane camera (C920).
# Logs and pids in $HOME.
cd $HOME
# The Wi-Fi power save comes back on at every boot and stalls the link for seconds at a time
# (10/9, 10/27: laser, odom and both cameras stopping together). Needs the NOPASSWD rule in
# /etc/sudoers.d/iw-powersave; without it this only warns (sudo -n never asks).
if sudo -n /usr/sbin/iw dev wlan0 set power_save off 2>/dev/null; then
  echo "wifi $(iw dev wlan0 get power_save)"
else
  echo "WARNING: wifi power save still on (run: sudo iw dev wlan0 set power_save off)"
fi
for n in robot lidar camera camera_lane; do   # camera: Pi CSI camera, forward (signs); camera_lane: C920 (lane)
  [ -f $n.pid ] && kill $(cat $n.pid) 2>/dev/null
  if [ $n = lidar ]; then
    # turtlebot3_bringup also starts a lidar driver (LDS-01's hlds_laser_publisher, or LDS-02's
    # ld08_driver with LDS_MODEL=LDS-02); it holds /dev/ttyUSB0 and cannot read this LDS-02
    # (LDRobot protocol at 115200), so ld_lidar would get nothing.
    for p in $(pgrep -f 'hls_lfcd_lds_driver/hlds_laser_publishe[r]|ld08_driver/ld08_drive[r]'); do kill $p; done
    sleep 1
  fi
  if [ $n = camera_lane ]; then
    # Right after boot the C920 takes a while to enumerate; opened too early it fails
    # (10/27: start_all.sh 50 s after boot, "cannot open"). Wait for udev to name it.
    for i in $(seq 1 30); do
      [ -e /dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920-video-index0 ] && break
      [ $i = 1 ] && echo "waiting for the C920..."
      sleep 1
    done
  fi
  for try in 1 2; do
    setsid nohup ./start_$n.sh > $n.log 2>&1 < /dev/null & echo $! > $n.pid
    sleep 3
    kill -0 $(cat $n.pid) 2>/dev/null && break
    echo "$n died at once (try $try): $(tail -1 $n.log)"; sleep 3
  done
done
echo "started: robot lidar camera camera_lane (pids: $(cat robot.pid) $(cat lidar.pid) $(cat camera.pid) $(cat camera_lane.pid))"
