#!/usr/bin/python3
"""level_crossing_view.py : watch the level crossing (bar) detector decide, by eye.

Needs only the Pi's forward camera (~/start_camera.sh, or ~/start_all.sh); no
race launch. It reads /camera/image_raw/compressed and runs the detector's own
code (detect_level_crossing.py) with the parameters from mission_real.yaml,
so what it shows is what the robot would decide.

  left   the camera picture
           blue outline  the ROI, a trapezoid (roi_top / bottom, roi_left / right at
                         the bottom, roi_top_left / right at the top)
           red boxes     solid red stripes (the first way to see the bar)
           red circles   red fragments in a row (the second way, a dark bar)
           yellow line   the line through them, with its angle from level
           and how far across the ROI it reaches (closed_span)
  right  what is red to the detector: red = solid stripe red, orange = only
         the darker fragment red; outside the ROI dimmed
  top    the answer (no bar / open / closed), which way found it, why, and
         a count of closed answers in a row

Click the left picture: the H S V there is printed (to read the bar's red).
Keys: q quit, s save the picture to level_crossing_<time>.png.

  python3 tools/level_crossing_view.py                    (ROS_DOMAIN_ID=30)
  python3 tools/level_crossing_view.py -p red.value_l:=30 -p closed_span:=0.3
      (any detector parameter, without editing the yaml)
Wayland: QT_QPA_PLATFORM=xcb.
"""
import argparse
import math
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

from autorace_perception.detect_level_crossing import DetectLevelCrossing

WINDOW = 'level crossing  [q quit | s save | click: HSV]'
NAMES = ('no bar', 'open', 'closed')
COLOURS = ((200, 200, 200), (0, 200, 0), (0, 0, 255))


def red_mask(hsv, margin, floor):
    hue, sat, val = cv2.split(hsv)
    return (((hue < margin) | (hue > 180 - margin)) & (sat > floor[0]) & (val > floor[1]))


def line_of(points):
    """Direction (unit vector) of the line through the points, its angle from level."""
    offsets = points - points.mean(axis=0)
    _, vectors = np.linalg.eigh(offsets.T @ offsets)
    along = vectors[:, 1]
    from_upright = math.atan2(abs(along[0]), abs(along[1]))
    return along, math.pi / 2.0 - from_upright


class View(Node):

    def __init__(self, topic):
        super().__init__('level_crossing_view')
        self.frame = None
        self.hsv = None
        self.closed_run = 0
        self.create_subscription(CompressedImage, topic, self.callback, qos_profile_sensor_data)
        cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(WINDOW, self.click)

    def callback(self, msg):
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is not None:
            self.frame = frame

    def click(self, event, x, y, flags, _):
        if event != cv2.EVENT_LBUTTONDOWN or self.hsv is None:
            return
        y -= 28
        h, w = self.hsv.shape[:2]
        if not (0 <= x < w and 0 <= y < h):
            return
        pix = self.hsv[y, x]
        patch = self.hsv[max(0, y - 2):y + 3, max(0, x - 2):x + 3].reshape(-1, 3)
        med = np.median(patch, axis=0).astype(int)
        print(f'({x},{y}) H {pix[0]} S {pix[1]} V {pix[2]}   5x5 median H {med[0]} S {med[1]} V {med[2]}')

    def show(self, det):
        if self.frame is None:
            return True
        image = self.frame.copy()
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        self.hsv = hsv
        det.width = hsv.shape[1]
        top, bottom, left, right = det.roi(hsv.shape)

        stripes = det.stripes(hsv)
        state, alike = det.bar(stripes, hsv)
        how, points, fragments = '', None, []
        if state:
            how = 'stripes'
            points = np.array([s[:2] for s in alike], np.float64)
        elif det.fragments_enabled:
            state, fragments = det.fragment_bar(hsv)
            if state:
                how = 'fragments'
                points = np.asarray(fragments, np.float64)
        self.closed_run = self.closed_run + 1 if state == 2 else 0

        # Why: angle from level and the reach across the ROI.
        why = f'{len(stripes)} red stripes in the ROI'
        if points is not None and len(points) >= 2:
            along, from_level = line_of(points)
            if how == 'stripes':
                x_low = min(s[3][0] for s in alike)
                x_high = max(s[3][0] + s[3][2] for s in alike)
            else:
                x_low, x_high = points[:, 0].min(), points[:, 0].max()
            span = (x_high - x_low) / max(1, right - left)
            why = (f'{len(points)} {how}, {math.degrees(from_level):.0f} deg from level '
                   f'(closed <= {math.degrees(det.closed_angle):.0f}), '
                   f'across {span * 100:.0f}% of ROI (closed >= {det.closed_span * 100:.0f}%)')
            centre = points.mean(axis=0)
            reach = np.abs((points - centre) @ along).max() + 20
            a = (centre - along * reach).astype(int)
            b = (centre + along * reach).astype(int)
            cv2.line(image, tuple(a), tuple(b), (0, 255, 255), 2)

        polygon = det.roi_polygon(hsv.shape)
        cv2.polylines(image, [polygon], True, (255, 128, 0), 2)
        for x, y, thick, (bx, by, bw, bh) in stripes:
            used = any(s[3] == (bx, by, bw, bh) for s in alike)
            cv2.rectangle(image, (bx, by), (bx + bw, by + bh), (0, 0, 255), 2 if used else 1)
        for x, y in fragments:
            cv2.circle(image, (int(x), int(y)), 6, (0, 0, 255), 2)

        # Right: the red the detector sees.
        solid = red_mask(hsv, det.red_margin, det.red_floor)
        dark = red_mask(hsv, det.red_margin, det.fragment_floor) & ~solid
        mask_view = (image * 0.15).astype(np.uint8)
        mask_view[dark] = (0, 140, 255)
        mask_view[solid] = (0, 0, 255)
        inside = np.zeros(hsv.shape[:2], np.uint8)
        cv2.fillPoly(inside, [polygon], 1)
        outside = inside == 0
        mask_view[outside] = (mask_view[outside] * 0.35).astype(np.uint8)
        cv2.polylines(mask_view, [polygon], True, (255, 128, 0), 2)

        head = np.zeros((28, image.shape[1] * 2, 3), np.uint8)
        cv2.putText(head, f'{NAMES[state].upper()}   closed in a row: {self.closed_run}   {why}',
                    (8, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOURS[state], 1)
        both = np.vstack([head, np.hstack([image, mask_view])])
        cv2.imshow(WINDOW, both)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            return False
        if key == ord('s'):
            name = time.strftime('level_crossing_%H%M%S.png')
            cv2.imwrite(name, both)
            print(f'saved {name}')
        return True


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    default_yaml = os.path.join(here, '..', 'src', 'autorace_bringup', 'param', 'mission_real.yaml')
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--topic', default='/camera/image_raw/compressed')
    ap.add_argument('--yaml', default=default_yaml)
    ap.add_argument('-p', action='append', default=[], metavar='NAME:=VALUE',
                    help='detector parameter override, e.g. -p closed_span:=0.3')
    args = ap.parse_args()

    ros_args = ['--ros-args', '--params-file', args.yaml, '-p', 'publish_debug_image:=false']
    for p in args.p:
        ros_args += ['-p', p]
    rclpy.init(args=ros_args)
    det = DetectLevelCrossing()      # the detector's own code and parameters, not spun
    print(f'roi top {det.roi_top} bottom {det.roi_bottom}, at the top {det.roi_top_left}..'
          f'{det.roi_top_right}, at the bottom {det.roi_left}..{det.roi_right}, '
          f'closed within {math.degrees(det.closed_angle):.0f} deg of level and '
          f'{det.closed_span * 100:.0f}% across, red S>{det.red_floor[0]} V>{det.red_floor[1]}, '
          f'fragments S>{det.fragment_floor[0]} V>{det.fragment_floor[1]}')
    node = View(args.topic)
    try:
        while rclpy.ok() and node.show(det):
            rclpy.spin_once(node, timeout_sec=0.03)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
