#!/usr/bin/env python3
"""traffic_light_view.py : watch the traffic light detector decide, by eye.

Needs only the Pi's forward camera (~/start_all.sh on the Pi); no race launch.
It reads /camera/image_raw/compressed, runs the same checks as
detect_traffic_light.py with the thresholds from mission_real.yaml, and shows
two pictures side by side:

  left   the camera picture. Every bright, saturated patch of red, yellow or
         green gets a box: green box = a lamp (what the detector would report),
         red box = thrown out, with the reason beside it:
             area      smaller than min_area
             edge      touching the picture's edge
             round     not round (w/h or fill off)
             housing   not dark either side (median value beside it)
         Each box also shows the patch's median hue / saturation / value.
  right  the "lit" mask (saturation >= lamp.saturation_l and value >=
         lamp.value_l) coloured by hue class: red, yellow, green, grey = lit
         but no colour class.
  top    the detector's answer this frame and a running count of green frames
         in a row (the manager starts on green_streak of them, 5).

Click anywhere on the left picture: the H S V of that pixel and of the 5x5
around it are printed, to read off what a lit LED really looks like.
Keys: q quit, s save the picture to traffic_light_<time>.png.

  python3 tools/traffic_light_view.py                       (ROS_DOMAIN_ID=30)
  python3 tools/traffic_light_view.py --topic /camera/image_raw/compressed
  python3 tools/traffic_light_view.py --sat 60 --val 170     (try thresholds without editing the yaml)
Wayland: QT_QPA_PLATFORM=xcb.
"""
import argparse
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
import yaml

WINDOW = 'traffic light  [q quit | s save | click: HSV]'
RED, YELLOW, GREEN = 1, 2, 3
NAMES = {RED: 'red', YELLOW: 'yellow', GREEN: 'green'}
BGR = {RED: (0, 0, 255), YELLOW: (0, 220, 255), GREEN: (0, 200, 0)}


def load_params(path):
    cfg = yaml.safe_load(open(path))['detect_traffic_light']['ros__parameters']
    return {
        'sat': cfg['lamp']['saturation_l'], 'val': cfg['lamp']['value_l'],
        'red_margin': cfg['red']['hue_margin'],
        'yellow': (cfg['yellow']['hue_l'], cfg['yellow']['hue_h']),
        'green': (cfg['green']['hue_l'], cfg['green']['hue_h']),
        'min_area': cfg['min_area'], 'housing': cfg['housing']['value_h'],
    }


class View(Node):

    def __init__(self, topic, p):
        super().__init__('traffic_light_view')
        self.p = p
        self.frame = None
        self.hsv = None
        self.green_run = 0
        self.create_subscription(CompressedImage, topic, self.callback, qos_profile_sensor_data)
        cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(WINDOW, self.click)
        self.get_logger().info(f'{topic}; thresholds {p}')

    def click(self, event, x, y, flags, _):
        if event != cv2.EVENT_LBUTTONDOWN or self.hsv is None:
            return
        h, w = self.hsv.shape[:2]
        if x >= w or y >= h:
            return
        pix = self.hsv[y, x]
        patch = self.hsv[max(0, y - 2):y + 3, max(0, x - 2):x + 3].reshape(-1, 3)
        med = np.median(patch, axis=0).astype(int)
        print(f'({x},{y}) H {pix[0]} S {pix[1]} V {pix[2]}   5x5 median H {med[0]} S {med[1]} V {med[2]}')

    def callback(self, msg):
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is not None:
            self.frame = frame

    def classes(self, hue, lit):
        p = self.p
        out = {RED: lit & ((hue < p['red_margin']) | (hue > 180 - p['red_margin']))}
        for colour, key in ((YELLOW, 'yellow'), (GREEN, 'green')):
            lo, hi = p[key]
            out[colour] = lit & (hue >= lo) & (hue <= hi)
        return out

    def judge(self, hsv, masks):
        """Every candidate: (colour, box, reason or None, median hsv)."""
        p = self.p
        hue, sat, val = cv2.split(hsv)
        height, width = val.shape
        found = []
        for colour, mask in masks.items():
            contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                area = cv2.contourArea(c)
                if area < max(8, p['min_area'] / 4):
                    continue          # too small to bother drawing
                x, y, w, h = cv2.boundingRect(c)
                inside = hsv[y:y + h, x:x + w][mask[y:y + h, x:x + w]]
                med = np.median(inside, axis=0).astype(int) if len(inside) else (0, 0, 0)
                side = max(2, w // 3)
                reason = None
                if area < p['min_area']:
                    reason = f'area {int(area)}<{p["min_area"]}'
                elif x - side < 0 or y <= 0 or x + w + side > width or y + h >= height:
                    reason = 'edge'
                elif not 0.6 < w / h < 1.6 or not 0.7 < area / (np.pi / 4 * w * h) < 1.2:
                    reason = f'round w/h {w / h:.2f} fill {area / (np.pi / 4 * w * h):.2f}'
                else:
                    beside = np.concatenate([val[y:y + h, x - side:x].ravel(),
                                             val[y:y + h, x + w:x + w + side].ravel()])
                    if np.median(beside) > p['housing']:
                        reason = f'housing {int(np.median(beside))}>{p["housing"]}'
                found.append((area, colour, (x, y, w, h), reason, med))
        found.sort(key=lambda f: -f[0])
        return found

    def show(self):
        if self.frame is None:
            return True
        frame = self.frame.copy()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        self.hsv = hsv
        hue, sat, val = cv2.split(hsv)
        lit = (sat >= self.p['sat']) & (val >= self.p['val'])
        masks = self.classes(hue, lit)

        found = self.judge(hsv, masks)
        accepted = [f for f in found if f[3] is None]
        answer = accepted[0][1] if accepted else 0
        self.green_run = self.green_run + 1 if answer == GREEN else 0

        for area, colour, (x, y, w, h), reason, med in found:
            ok = reason is None
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 200, 0) if ok else (0, 0, 255), 2 if ok else 1)
            label = f'{NAMES[colour]} H{med[0]} S{med[1]} V{med[2]}' + ('' if ok else f'  x {reason}')
            cv2.putText(frame, label, (x, max(10, y - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.38,
                        (0, 200, 0) if ok else (0, 0, 255), 1)

        mask_view = np.zeros_like(frame)
        mask_view[lit] = (120, 120, 120)
        for colour, m in masks.items():
            mask_view[m] = BGR[colour]

        head = np.zeros((28, frame.shape[1] * 2, 3), np.uint8)
        text = (f'detect: {NAMES.get(answer, "none")}   green in a row: {self.green_run}'
                f'   lit = S>={self.p["sat"]} & V>={self.p["val"]}   min_area {self.p["min_area"]}'
                f'   housing V<={self.p["housing"]}')
        cv2.putText(head, text, (8, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    BGR.get(answer, (255, 255, 255)), 1)
        both = np.vstack([head, np.hstack([frame, mask_view])])
        cv2.imshow(WINDOW, both)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            return False
        if key == ord('s'):
            name = time.strftime('traffic_light_%H%M%S.png')
            cv2.imwrite(name, both)
            print(f'saved {name}')
        return True


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    default_yaml = os.path.join(here, '..', 'src', 'autorace_bringup', 'param', 'mission_real.yaml')
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--topic', default='/camera/image_raw/compressed')
    ap.add_argument('--yaml', default=default_yaml)
    ap.add_argument('--sat', type=int, help='lamp.saturation_l instead of the yaml')
    ap.add_argument('--val', type=int, help='lamp.value_l instead of the yaml')
    ap.add_argument('--min-area', type=int, help='min_area instead of the yaml')
    ap.add_argument('--housing', type=int, help='housing.value_h instead of the yaml')
    args = ap.parse_args()
    p = load_params(args.yaml)
    for key in ('sat', 'val', 'min_area', 'housing'):
        if getattr(args, key) is not None:
            p[key] = getattr(args, key)

    rclpy.init()
    node = View(args.topic, p)
    try:
        while rclpy.ok() and node.show():
            rclpy.spin_once(node, timeout_sec=0.03)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
