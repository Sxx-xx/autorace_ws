"""detect_lane_sw on synthetic bird's eye views (perception_<profile>.yaml): known lane centre vs output.

  python3 tools/tests/sw_synthetic.py src/autorace_bringup/param/perception_real.yaml
Nine cases: straight, offset, bend, one line only (each side), askew, follow left, lost (held, then
dropped). Passing: every error within 0.1 cm.
"""
import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from std_msgs.msg import String

from autorace_perception.detect_lane_sw import DetectLaneSlidingWindow

yaml = sys.argv[1]
rclpy.init(args=['--ros-args', '--params-file', yaml, '-p', 'process_every_n:=1'] + sys.argv[2:])
node = DetectLaneSlidingWindow()
out = {}
node.pub_target.publish = lambda m: out.__setitem__('t', (m.point.x, m.point.y))
node.pub_state.publish = lambda m: out.__setitem__('s', m.data)
bridge = CvBridge()
PPM, NEAR, W, H, HALF = node.ppm, node.near, 800, 300, node.half_lane
rng = np.random.default_rng(1)


def view(yellow=None, white=None):
    img = np.clip(np.full((H, W, 3), 70.0) + rng.normal(0, 8, (H, W, 3)), 0, 255).astype(np.uint8)
    for f, colour in ((yellow, (0, 220, 255)), (white, (235, 235, 235))):
        if f is None:
            continue
        for y in range(H):
            ahead = NEAR + (H - 1 - y) / PPM
            x = W / 2 - f(ahead) * PPM
            cv2.line(img, (int(x - 15), y), (int(x + 15), y), colour, 1)
    return img


worst = 0.0


def run(name, img, expect):
    global worst
    out.clear()
    node.callback_image(bridge.cv2_to_imgmsg(img, 'bgr8'))
    t, s = out.get('t'), out.get('s')
    if t is None or expect is None:
        print(f'{name:30} state {s}  target {"none" if t is None else t}  expected {expect}')
        return
    want = expect(t[0]) if callable(expect) else expect
    err = abs(t[1] - want) * 100
    worst = max(worst, err)
    print(f'{name:30} state {s}  target ({t[0]:.2f}, {t[1]:+.3f})  expected left {want:+.3f}  error {err:.1f} cm')


A = node.target_ahead
bend = lambda base: (lambda a: base + 1.5 * (a - NEAR) ** 2)
run('straight, centred', view(lambda a: HALF, lambda a: -HALF), 0.0)
run('5 cm right of centre', view(lambda a: HALF + 0.05, lambda a: -HALF + 0.05), 0.05)
run('bend to the left', view(bend(HALF), bend(-HALF)), 1.5 * (A - NEAR) ** 2)
run('yellow only', view(lambda a: HALF + 0.02, None), 0.02)
run('white only', view(None, lambda a: -HALF - 0.03), -0.03)
t = np.tan(0.17)
run('askew 10 deg', view(lambda a: HALF + t * (a - NEAR), lambda a: -HALF + t * (a - NEAR)), t * (A - NEAR))
node.callback_follow(String(data='left'))
run('follow left (yellow 4 cm off)', view(lambda a: HALF + 0.04, lambda a: -HALF), 0.04)
node.follow = None
run('no lines (held 0.4 s)', view(None, None), 0.04)
node.last_target_time -= 1.0
run('no lines (after hold)', view(None, None), None)
print(f'worst error {worst:.1f} cm: {"PASS" if worst <= 0.1 else "FAIL"}')
