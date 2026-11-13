#!/bin/bash
# Is the robot talking? Rates of the sensor topics, the clock offset between
# this PC and the robot's messages, and the laser's noise while standing
# still (for cmd_vel_mux's scan_change_threshold).
set -u
echo "== topics"
ros2 topic list 2>/dev/null | grep -E "^/(scan|odom|cmd_vel|imu|camera(_lane)?/(image_raw(/compressed)?|camera_info))$" || echo "  (no robot topics: ROS_DOMAIN_ID? same Wi-Fi?)"
for t in /scan /odom /camera/image_raw/compressed /camera_lane/image_raw/compressed; do
  printf "== %-36s " "$t"
  timeout 6 ros2 topic hz "$t" 2>/dev/null | grep -m1 "average rate" || echo "no messages"
done
echo "== middleware: the PC and the robot must match (tools/dds.sh on both)"
echo "  PC:    ${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp (Fast DDS, the default)}"
ROBOT=${ROBOT:-dd@turtlebot1.local}
printf "  robot: "
timeout 8 ssh -o ConnectTimeout=4 -o BatchMode=yes "$ROBOT" '
  for p in $(pgrep -f "[c]si_camera|[t]urtlebot3_ros|[l]d_lidar"); do
    v=$(tr "\0" "\n" < /proc/$p/environ 2>/dev/null | grep "^RMW_IMPLEMENTATION=" | cut -d= -f2)
    echo "${v:-rmw_fastrtps_cpp (Fast DDS, the default)}"; exit
  done; echo "no robot node running"' 2>/dev/null || echo "(ssh to $ROBOT failed: ROBOT=user@host)"
echo "== cameras: publisher QoS (best effort expected) and how many read them over the Wi-Fi"
for t in /camera/image_raw/compressed /camera_lane/image_raw/compressed; do
  # 12 s: right after the rate checks above the ros2 CLI can be slow to answer.
  info=$(timeout 12 ros2 topic info -v "$t" 2>/dev/null)
  if [ -z "$info" ]; then printf "  %-36s no answer (run again)\n" "$t"; continue; fi
  rel=$(echo "$info" | awk '/Endpoint type: PUBLISHER/{p=1} p && /Reliability/{print $2; exit}')
  subs=$(echo "$info" | grep -m1 "Subscription count" | awk '{print $3}')
  printf "  %-36s publisher %s, %s readers (each one is a copy over the Wi-Fi)\n" "$t" "${rel:-none}" "${subs:-0}"
done
echo "== clock offset (robot stamp vs PC now; keep under 0.05 s, chrony on both)"
timeout 10 python3 - <<'PY'
import rclpy, time
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
rclpy.init(); n = Node('check_offset'); got = []
def cb(m):
    got.append(time.time() - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9))
n.create_subscription(LaserScan, '/scan', cb, 10)
t0 = time.time()
while len(got) < 5 and time.time() - t0 < 8: rclpy.spin_once(n, timeout_sec=0.2)
print('  offset %.3f s (%d samples)' % (sum(got)/len(got), len(got)) if got else '  no /scan')
PY
echo "== laser noise while still (median |delta| between scans 3 s apart; threshold 0.05 must be well above)"
timeout 12 python3 - <<'PY'
import rclpy, time, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
rclpy.init(); n = Node('check_noise'); scans = []
n.create_subscription(LaserScan, '/scan', lambda m: scans.append((time.time(), np.array(m.ranges), m.range_min)), 10)
t0 = time.time()
while time.time() - t0 < 9: rclpy.spin_once(n, timeout_sec=0.2)
if len(scans) < 2: print('  no /scan'); raise SystemExit
(ta, a, rmin), (tb, b, _) = scans[0], scans[-1]
ok = (a > rmin) & (b > rmin) & np.isfinite(a) & np.isfinite(b)
d = np.abs(a[ok] - b[ok])
print('  %d scans in %.1f s (%.1f Hz), valid returns %d/%d, median |delta| %.4f m, share > 0.05 m: %.3f'
      % (len(scans), tb - ta, (len(scans)-1)/(tb-ta), ok.sum(), len(a), np.median(d), (d > 0.05).mean()))
PY
