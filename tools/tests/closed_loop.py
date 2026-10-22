"""Closed loop without wheels: the real DetectLane and LaneController (perception_real.yaml) drive a
simulated robot round a drawn course. Bird's eye views are rendered from the robot's pose, the
controller's Twist moves it. Reports how far the axle strays from the lane centre.
  python3 tools/tests/closed_loop.py src/autorace_bringup/param/perception_real.yaml out.png [true|false]
2026-10-23: the course in 18.8 s, axle off the centre mean 1.1 cm, max 3.1 cm.
"""
import math
import sys
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Bool, Header

from autorace_core.lane_controller import LaneController
from autorace_perception.detect_lane import DetectLane

yaml, out_png = sys.argv[1], sys.argv[2]
hough = sys.argv[3] if len(sys.argv) > 3 else 'true'
rclpy.init(args=['--ros-args', '--params-file', yaml, '-p', f'detect.hough.enable:={hough}'])
det = DetectLane()
ctl = LaneController()
W, H, PPM, NEAR = 800, 300, det.ppm, det.near
HALF = 0.33 / 2

# Course centre line: straight, left 90 deg (R 0.45), straight, right 90 deg (R 0.45), straight.
pts, x, y, th = [], 0.0, 0.0, 0.0
def go(length, curvature, step=0.005):
    global x, y, th
    for _ in range(int(length / step)):
        pts.append((x, y, th))
        th += curvature * step
        x += math.cos(th) * step
        y += math.sin(th) * step
go(0.8, 0.0); go(math.pi / 2 * 0.45, 1 / 0.45); go(0.6, 0.0); go(math.pi / 2 * 0.45, -1 / 0.45); go(1.0, 0.0)
centre = np.array(pts)

# Top-down world picture at 2 mm: grey floor, yellow line left of the centre, white right.
RES, X0, Y0 = 0.002, -0.6, -0.8
WW, WH = int(3.2 / RES), int(2.6 / RES)
world = np.clip(np.random.default_rng(0).normal(75, 8, (WH, WW, 3)), 0, 255).astype(np.uint8)
for side, colour in ((HALF, (0, 220, 255)), (-HALF, (235, 235, 235))):
    line = centre[:, :2] + side * np.stack([-np.sin(centre[:, 2]), np.cos(centre[:, 2])], axis=1)
    pix = np.round(np.stack([(line[:, 0] - X0) / RES, (line[:, 1] - Y0) / RES], axis=1)).astype(np.int32)
    cv2.polylines(world, [pix.reshape(-1, 1, 2)], False, colour, int(0.03 / RES))
gx, gy = np.meshgrid(np.arange(W, dtype=np.float32), np.arange(H, dtype=np.float32))
B_AHEAD = NEAR + (H - 1 - gy) / PPM
B_LEFT = (W / 2 - gx) / PPM


def render(px, py, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    wx = px + B_AHEAD * c - B_LEFT * s
    wy = py + B_AHEAD * s + B_LEFT * c
    return cv2.remap(world, ((wx - X0) / RES).astype(np.float32), ((wy - Y0) / RES).astype(np.float32),
                     cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(75, 75, 75))


def cross_track(px, py):
    d = np.hypot(centre[:, 0] - px, centre[:, 1] - py)
    i = int(np.argmin(d))
    nx, ny = -math.sin(centre[i, 2]), math.cos(centre[i, 2])
    return (px - centre[i, 0]) * nx + (py - centre[i, 1]) * ny, i


cmd = [0.0, 0.0]
ctl.pub_cmd_vel.publish = lambda m: cmd.__setitem__(slice(0, 2), [m.linear.x, m.angular.z])
targets = []
det.pub_target.publish = lambda m: (targets.append((m.point.x, m.point.y)), ctl.callback_target(m))
det.pub_state.publish = lambda m: ctl.callback_state(m)
ctl.callback_run_active(Bool(data=True))
ex = SingleThreadedExecutor(); ex.add_node(det); ex.add_node(ctl)

px, py, yaw = 0.0, 0.02, 0.0           # 2 cm off the centre at the start
log, trail = [], []
t0 = time.monotonic(); last_cam = last_ctl = -1.0; prev = t0
while True:
    now = time.monotonic()
    dt = now - prev; prev = now
    v, w = cmd
    px += v * math.cos(yaw) * dt; py += v * math.sin(yaw) * dt; yaw += w * dt
    stamp = det.get_clock().now().to_msg()
    odo = Odometry(); odo.header.stamp = stamp
    odo.pose.pose.position.x, odo.pose.pose.position.y = px, py
    odo.pose.pose.orientation.z, odo.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    odo.twist.twist.linear.x, odo.twist.twist.angular.z = v, w
    det.callback_odom(odo)
    if now - last_cam >= 1 / 30:
        last_cam = now
        det.process(render(px, py, yaw), Header(stamp=stamp))
    if now - last_ctl >= 1 / 50:
        last_ctl = now
        ctl.update()
    e, i = cross_track(px, py)
    log.append((now - t0, e, i, det.path_used, v, w))
    trail.append((px, py))
    if i >= len(centre) - 40 or now - t0 > 40 or abs(e) > 0.3:
        break
    time.sleep(max(0.0, 1 / 120 - (time.monotonic() - now)))

L = np.array([(a, b, c, d) for a, b, c, d, _, _ in log], float)
err = np.abs(L[:, 1]) * 100
seg = lambda lo, hi: err[(L[:, 2] >= lo) & (L[:, 2] < hi)]
n1 = int(0.8 / 0.005); n2 = n1 + int(math.pi / 2 * 0.45 / 0.005); n3 = n2 + int(0.6 / 0.005); n4 = n3 + int(math.pi / 2 * 0.45 / 0.005)
print(f'hough {hough}: {"reached the end" if L[-1, 2] >= len(centre) - 40 else "STOPPED/LEFT THE LANE"} in {L[-1, 0]:.1f} s; '
      f'axle off the centre: mean {err.mean():.1f} cm, max {err.max():.1f} cm')
for name, (lo, hi) in (('straight 1', (0, n1)), ('left bend', (n1, n2)), ('straight 2', (n2, n3)),
                       ('right bend', (n3, n4)), ('straight 3', (n4, len(centre)))):
    e = seg(lo, hi)
    if len(e):
        print(f'  {name:10} mean {e.mean():4.1f} cm  max {e.max():4.1f} cm')
print(f'  target from the remembered path {100 * L[:, 3].mean():.0f} % of the time, from the camera the rest')
# A picture of the run: the course and the axle's trail.
img = cv2.resize(world, None, fx=0.25, fy=0.25)
tr = np.array(trail)
tp = np.round(np.stack([(tr[:, 0] - X0) / RES / 4, (tr[:, 1] - Y0) / RES / 4], axis=1)).astype(np.int32)
cv2.polylines(img, [tp.reshape(-1, 1, 2)], False, (0, 0, 255), 2)
cv2.imwrite(out_png, cv2.flip(img, 0))
