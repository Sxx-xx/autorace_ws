#!/bin/bash
# Why is the camera not coming up? Run on the Pi: ~/check_camera.sh
# Checks, in order: the CSI camera seen by the firmware, the V4L2 devices and their names (csi_camera
# looks for 'camera0'), permissions (video group), who holds the devices, frames through OpenCV, the
# last camera log, and the ROS topics (best effort, as csi_camera publishes them).
set -u
ok() { echo "  OK   $*"; }
bad() { echo "  FAIL $*"; }
echo "== user $(id -un): groups $(id -Gn)"
id -nG | grep -qw video && ok "in the video group" || bad "not in video: sudo usermod -aG video $(id -un), then log in again"
echo "== firmware camera (start_x=1 in /boot/firmware/config.txt for the legacy stack)"
grep -nE '^\s*(start_x|camera_auto_detect|dtoverlay=(imx|ov)|gpu_mem)' /boot/firmware/config.txt 2>/dev/null || echo "  (no camera lines in config.txt)"
command -v vcgencmd >/dev/null && vcgencmd get_camera || echo "  (no vcgencmd)"
lsmod | grep -qE '^bcm2835_v4l2' && ok "bcm2835_v4l2 loaded" || bad "bcm2835_v4l2 not loaded: sudo modprobe bcm2835-v4l2 (and start_x=1, reboot)"
echo "== V4L2 devices"
for d in /sys/class/video4linux/video*; do
  [ -e "$d" ] || { bad "no /dev/video*"; break; }
  n=/dev/$(basename $d); printf "  %-12s %-28s %s\n" "$n" "'$(cat $d/name)'" "$(stat -c '%U:%G %a' $n)"
done
grep -qx camera0 /sys/class/video4linux/video*/name 2>/dev/null && ok "CSI camera 'camera0' present" || bad "no V4L2 device named 'camera0' (start_camera.sh will exit)"
ls /dev/v4l/by-id/ 2>/dev/null | grep -q C920 && ok "C920 present" || bad "no C920 in /dev/v4l/by-id"
echo "== processes holding the cameras"
fuser -v /dev/video* 2>&1 | grep -v '^\s*$' || echo "  none"
pgrep -af 'csi_camera|usb_cam|camera_ros|libcamera|gst-launch' | sed 's/^/  /' || true
echo "== frames through OpenCV (stop the camera nodes first if they hold the device)"
python3 - <<'PY'
import glob, cv2
for d in sorted(glob.glob('/sys/class/video4linux/video*')):
    name = open(d + '/name').read().strip()
    if name != 'camera0' and 'C920' not in name:
        continue
    dev = '/dev/' + d.rsplit('/', 1)[1]
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    got = sum(cap.read()[0] for _ in range(30)) if cap.isOpened() else 0
    print(f'  {"OK  " if got else "FAIL"} {dev} {name!r}: {got}/30 frames' + ('' if cap.isOpened() else ' (cannot open)'))
    cap.release()
PY
echo "== last camera log"
for l in ~/camera.log ~/camera_lane.log; do [ -f $l ] && { echo "  -- $l"; tail -5 $l | sed 's/^/  /'; }; done
set +u   # setup.bash reads unset variables
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-30}   # as the start scripts
echo "== ROS topics (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)"
for t in /camera/image_raw/compressed /camera_lane/image_raw/compressed; do
  printf "  %-36s " $t
  timeout 6 ros2 topic hz $t 2>/dev/null | grep -m1 'average rate' || echo "no messages"
done
