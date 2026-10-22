"""lane_pipeline without a robot: remap == warpPerspective, and JPEG camera pictures of a known lane
(a drawn bird's eye view projected back into the camera) give the right target.

  python3 tools/tests/pipeline_e2e.py src/autorace_bringup/param/perception_real.yaml detect_lane
  (or sliding_window). Passing: remap within 1 level, targets within 1 cm.
"""
import sys

import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, CompressedImage

from autorace_perception.detect_lane import DetectLane
from autorace_perception.detect_lane_sw import DetectLaneSlidingWindow
from autorace_perception.lane_pipeline import LanePipeline

yaml, detector_name = sys.argv[1], sys.argv[2]
rclpy.init(args=['--ros-args', '--params-file', yaml, '-p', 'process_every_n:=1'])
detector = {'detect_lane': DetectLane, 'sliding_window': DetectLaneSlidingWindow}[detector_name]()
pipe = LanePipeline(detector)
out = {}
detector.pub_target.publish = lambda m: out.__setitem__('t', (m.point.x, m.point.y))
detector.pub_state.publish = lambda m: out.__setitem__('s', m.data)

# The camera the yaml describes (C920 lane camera, 432x240, 70.4 deg) as camera_info.
W, H = 432, 240
fx = W / (2 * np.tan(np.radians(70.4) / 2))
pipe.callback_info(CameraInfo(width=W, height=H, k=[fx, 0.0, W / 2, 0.0, fx, H / 2, 0.0, 0.0, 1.0]))
pipe.build_homography()

rng = np.random.default_rng(0)
pic = rng.integers(0, 255, (H, W, 3), np.uint8)
ref = cv2.warpPerspective(cv2.GaussianBlur(pic, (pipe.blur, pipe.blur), 0), pipe.homography,
                          (pipe.out_w, pipe.out_h), flags=cv2.INTER_LINEAR,
                          borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))
diff = np.abs(ref.astype(int) - pipe.project_image(pic).astype(int))
print(f'remap vs warpPerspective: mean |diff| {diff.mean():.3f}, max {diff.max()} (8-bit levels)')
half = detector.get_parameter('lane.width_m').value / 2.0
worst = 0.0


def camera_picture(yellow, white):
    bev = np.full((pipe.out_h, pipe.out_w, 3), 70, np.uint8)
    for f, colour in ((yellow, (0, 220, 255)), (white, (235, 235, 235))):
        if f is None:
            continue
        for y in range(pipe.out_h):
            ahead = pipe.near + (pipe.out_h - 1 - y) / pipe.ppm
            x = pipe.out_w / 2 - f(ahead) * pipe.ppm
            cv2.line(bev, (int(x - 18), y), (int(x + 18), y), colour, 1)
    cam = cv2.warpPerspective(bev, pipe.homography, (W, H),
                              flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP,
                              borderMode=cv2.BORDER_CONSTANT, borderValue=(70, 70, 70))
    return cv2.imencode('.jpg', cam, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()


def run(name, yellow, white, expect):
    global worst
    out.clear()
    msg = CompressedImage(format='jpeg', data=camera_picture(yellow, white))
    msg.header.stamp = pipe.get_clock().now().to_msg()
    for _ in range(3):                  # detect_lane carries lines over a few pictures
        pipe.callback_compressed(msg)
    t = out.get('t')
    if t is None:
        print(f'{name:24} no target')
        worst = 99.0
        return
    want = expect(t[0]) if callable(expect) else expect
    err = abs(t[1] - want) * 100
    worst = max(worst, err)
    print(f'{name:24} state {out.get("s")}  target ({t[0]:.2f}, {t[1]:+.3f})  expected {want:+.3f}  '
          f'error {err:.1f} cm')


t = np.tan(0.14)
run('straight, centred', lambda a: half, lambda a: -half, 0.0)
run('4 cm right of centre', lambda a: half + 0.04, lambda a: -half + 0.04, 0.04)
run('askew 8 deg', lambda a: half + t * (a - pipe.near), lambda a: -half + t * (a - pipe.near),
    lambda a: t * (a - pipe.near))
run('yellow only', lambda a: half + 0.02, None, 0.02)
run('white only', None, lambda a: -half - 0.02, -0.02)
ok = diff.max() <= 1 and worst <= 1.0
print(f'worst error {worst:.1f} cm: {"PASS" if ok else "FAIL"}')
