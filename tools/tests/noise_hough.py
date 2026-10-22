"""Lane detection with and without the Hough line support on noisy bird's eye views (known lanes).

  python3 tools/tests/noise_hough.py src/autorace_bringup/param/perception_real.yaml [sweep]
40 views per case (random offset and bend; noisy: specks and blobs of both colours, glare).
2026-10-23: detect_lane noisy, over 3 cm 34 -> 11 of 40 with the support; clean unchanged.
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


def scene(seed, noise):
    rng = np.random.default_rng(seed)
    img = np.clip(rng.normal(75, 10, (H, W, 3)), 0, 255).astype(np.uint8)
    off = rng.uniform(-0.05, 0.05)
    bend = rng.choice([0.0, rng.uniform(-2.5, 2.5)])
    centre = lambda a: off + bend * (a - NEAR) ** 2
    for side, colour in ((HALF, (0, 220, 255)), (-HALF, (235, 235, 235))):
        for y in range(H):
            a = NEAR + (H - 1 - y) / PPM
            x = W / 2 - (centre(a) + side) * PPM
            cv2.line(img, (int(x - 18), y), (int(x + 18), y), colour, 1)
    if noise:
        for _ in range(rng.integers(15, 40)):                      # specks and blobs of both colours
            c = (0, 220, 255) if rng.random() < 0.5 else (235, 235, 235)
            cv2.circle(img, (int(rng.integers(0, W)), int(rng.integers(0, H))),
                       int(rng.integers(6, 40)), c, -1)
        for _ in range(rng.integers(1, 3)):                        # glare: big soft bright patches
            glare = np.zeros((H, W), np.float32)
            cv2.ellipse(glare, (int(rng.integers(0, W)), int(rng.integers(0, H))),
                        (int(rng.integers(60, 200)), int(rng.integers(30, 100))),
                        float(rng.uniform(0, 180)), 0, 360, 1.0, -1)
            glare = cv2.GaussianBlur(glare, (61, 61), 0)[..., None]
            img = np.clip(img + glare * 170, 0, 255).astype(np.uint8)
    return img, centre


def run(cls, hough, noise, n=40, extra=()):
    args = ['--ros-args', '--params-file', yaml, '-p', 'process_every_n:=1',
            '-p', f'detect.hough.enable:={"true" if hough else "false"}'] + [x for e in extra for x in ('-p', e)]
    context = rclpy.Context(); rclpy.init(args=args, context=context)
    node = cls_with_context(cls, context)
    got = {}
    node.pub_target.publish = lambda m: got.__setitem__('t', (m.point.x, m.point.y))
    node.pub_state.publish = lambda m: got.__setitem__('s', m.data)
    errors, both, times = [], 0, []
    for k in range(n):
        img, centre = scene(1000 + k, noise)
        got.clear()
        t0 = time.perf_counter()
        node.process(img, Header())
        times.append(time.perf_counter() - t0)
        both += got.get('s') == 2
        if 't' in got:
            a, l = got['t']
            errors.append(abs(l - centre(a)) * 100)
        else:
            errors.append(np.nan)
    node.destroy_node(); rclpy.shutdown(context=context)
    e = np.array(errors)
    found = ~np.isnan(e)
    return (f'both lines {both}/{n}, target {found.sum()}/{n}, error cm median {np.nanmedian(e):.1f} '
            f'p90 {np.nanpercentile(e, 90):.1f} max {np.nanmax(e):.1f}, over 3 cm {(e[found] > 3).sum()}, '
            f'{np.median(times) * 1000:.1f} ms')


def cls_with_context(cls, context):
    # The detectors call super().__init__('detect_lane'); give them this context.
    orig = rclpy.node.Node.__init__

    def init(self, name, **kw):
        orig(self, name, context=context, **kw)
    rclpy.node.Node.__init__ = init
    try:
        return cls()
    finally:
        rclpy.node.Node.__init__ = orig


if len(sys.argv) > 2 and sys.argv[2] == 'sweep':
    for extra in ([], ['detect.hough.min_length_m:=0.08'], ['detect.hough.min_length_m:=0.10'],
                  ['detect.hough.support_width_m:=0.045'], ['detect.hough.threshold:=35'],
                  ['detect.hough.min_length_m:=0.08', 'detect.hough.support_width_m:=0.045'],
                  ['detect.hough.min_length_m:=0.08', 'detect.hough.threshold:=35']):
        print(f'{" ".join(extra) or "defaults":70} noisy: {run(DetectLane, True, True, extra=extra)}', flush=True)
        print(f'{"":70} clean: {run(DetectLane, True, False, extra=extra)}', flush=True)
    raise SystemExit
for cls in (DetectLane, DetectLaneSlidingWindow):
    for noise in (False, True):
        for hough in (False, True):
            print(f'{cls.__name__:24} {"noisy" if noise else "clean"}  hough {"on " if hough else "off"}: '
                  f'{run(cls, hough, noise)}', flush=True)
