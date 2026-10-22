"""Lane detection with the road region and the Hough support, alone and together, on noisy views.

  python3 tools/tests/noise_road.py src/autorace_bringup/param/perception_real.yaml
Bird's eye views of a known lane (random offset and bend) on two courses: the contest's black road
on a light floor, and a grey road like the practice floor (L 131) on a floor nearly as grey. Noise:
pieces of yellow and white tape and blobs off the road, specks on it, glare. 40 views each.
"""
import sys
import time

import cv2
import numpy as np
import rclpy
from std_msgs.msg import Header

from autorace_perception.detect_lane import DetectLane
from autorace_perception.detect_lane_sw import DetectLaneSlidingWindow

yaml = sys.argv[1]
W, H, PPM, NEAR = 800, 300, 1200.0, 0.18
HALF = 0.33 / 2
COURSES = {'black road': (35, 150), 'grey road': (110, 125)}   # road, floor beyond the lines


def scene(seed, course, noise):
    rng = np.random.default_rng(seed)
    road_v, floor_v = COURSES[course]
    off = rng.uniform(-0.04, 0.04)
    bend = rng.choice([0.0, rng.uniform(-2.5, 2.5)])
    centre = lambda a: off + bend * (a - NEAR) ** 2
    ys = np.arange(H)
    ahead = NEAR + (H - 1 - ys) / PPM
    xs = np.arange(W)
    left = (W / 2 - xs[None, :]) / PPM - centre(ahead)[:, None]       # from the lane centre
    img = np.where((np.abs(left) < HALF)[..., None], road_v, floor_v).astype(np.float32)
    img = np.repeat(img, 3, axis=2) + rng.normal(0, 7, (H, W, 3))
    img = np.clip(img, 0, 255).astype(np.uint8)
    for side, colour in ((HALF, (0, 220, 255)), (-HALF, (235, 235, 235))):
        for y in range(H):
            x = W / 2 - (centre(ahead[y]) + side) * PPM
            cv2.line(img, (int(x - 18), y), (int(x + 18), y), colour, 1)
    if noise:
        for _ in range(rng.integers(8, 20)):                         # tape pieces and blobs off the road
            c = (0, 220, 255) if rng.random() < 0.5 else (235, 235, 235)
            y = int(rng.integers(0, H)); a = ahead[y]
            sign = rng.choice([-1, 1])
            lft = centre(a) + sign * rng.uniform(HALF + 0.06, HALF + 0.30)
            x = int(W / 2 - lft * PPM)
            if rng.random() < 0.5:
                ang = rng.uniform(0, np.pi)
                d = rng.uniform(40, 120)
                cv2.line(img, (int(x - d * np.cos(ang)), int(y - d * np.sin(ang))),
                         (int(x + d * np.cos(ang)), int(y + d * np.sin(ang))), c, int(rng.integers(20, 40)))
            else:
                cv2.circle(img, (x, y), int(rng.integers(10, 40)), c, -1)
        for _ in range(rng.integers(3, 10)):                         # specks on the road
            c = (0, 220, 255) if rng.random() < 0.5 else (235, 235, 235)
            y = int(rng.integers(0, H)); x = int(W / 2 - (centre(ahead[y]) + rng.uniform(-0.08, 0.08)) * PPM)
            cv2.circle(img, (x, y), int(rng.integers(3, 8)), c, -1)
        glare = np.zeros((H, W), np.float32)                        # glare
        cv2.ellipse(glare, (int(rng.integers(0, W)), int(rng.integers(0, H))),
                    (int(rng.integers(60, 180)), int(rng.integers(30, 90))), float(rng.uniform(0, 180)),
                    0, 360, 1.0, -1)
        img = np.clip(img + cv2.GaussianBlur(glare, (61, 61), 0)[..., None] * 120, 0, 255).astype(np.uint8)
    return img, centre


def make(cls, hough, road):
    context = rclpy.Context()
    rclpy.init(args=['--ros-args', '--params-file', yaml, '-p', 'process_every_n:=1',
                     '-p', f'detect.hough.enable:={str(hough).lower()}',
                     '-p', f'detect.road.enable:={str(road).lower()}'], context=context)
    orig = rclpy.node.Node.__init__

    def init(self, name, **kw):
        orig(self, name, context=context, **kw)
    rclpy.node.Node.__init__ = init
    try:
        return cls(), context
    finally:
        rclpy.node.Node.__init__ = orig


def run(cls, hough, road, course, noise, n=40):
    node, context = make(cls, hough, road)
    got = {}
    node.pub_target.publish = lambda m: got.__setitem__('t', (m.point.x, m.point.y))
    node.pub_state.publish = lambda m: got.__setitem__('s', m.data)
    errors, both, times = [], 0, []
    for k in range(n):
        img, centre = scene(2000 + k, course, noise)
        got.clear()
        t0 = time.perf_counter()
        node.process(img, Header())
        times.append(time.perf_counter() - t0)
        both += got.get('s') == 2
        errors.append(abs(got['t'][1] - centre(got['t'][0])) * 100 if 't' in got else np.nan)
    node.destroy_node()
    rclpy.shutdown(context=context)
    e = np.array(errors)
    found = ~np.isnan(e)
    return (f'both {both:2d}/{n}, over 3 cm {(e[found] > 3).sum():2d}, no target {(~found).sum()}, '
            f'error median {np.nanmedian(e):4.1f} p90 {np.nanpercentile(e, 90):5.1f} cm, '
            f'{np.median(times) * 1000:4.1f} ms')


for cls in (DetectLane, DetectLaneSlidingWindow):
    for course in COURSES:
        for noise in (False, True):
            for hough, road in ((False, False), (True, False), (False, True), (True, True)):
                name = {(False, False): 'colour only', (True, False): '+hough',
                        (False, True): '+road', (True, True): '+hough +road'}[(hough, road)]
                print(f'{cls.__name__[:14]:14} {course:10} {"noisy" if noise else "clean"} {name:13}: '
                      f'{run(cls, hough, road, course, noise)}', flush=True)
