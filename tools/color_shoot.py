#!/usr/bin/python3
"""color_shoot.py : photograph the colour scenes one by one, checked on the spot.

For the venue (체크리스트.md 1-0). Needs only the robot's ~/start_all.sh, no
launch: the robot does not move. It goes through a list of scenes (traffic
light red / yellow / green, lane straight / curve / glare, signs left / right /
parking / intersection, level crossing bar down / up). For each one the window shows the right camera live with what
the robot's own detectors (and the values in the yaml) make of it:

    SPACE  (or Enter) take the photo. With the keyboard in Korean input the
           letter keys may not reach the window: switch to English first. It is saved, and if the detector saw what the scene
           should show (OK) the next scene comes up; if not (NG), it stays on
           this scene: change the scene or the values and take it again
    n      next scene anyway (the photo is saved as NG)     b  back one scene
    p      pass: next scene without taking a photo
    a      (after a traffic light photo came out NG) use the suggested values
    q      quit; a summary is printed and saved

A traffic light photo that comes out NG is looked at again with loose limits:
the lit lamp in it is measured (H S V; the detector works in HSV, the RGB is
printed too) and the detector's values are worked out so that every light photo
of the session (red, yellow, green) reads right; they are printed ready for
mission_real.yaml. --check does the same over the photos of a directory.

A slow camera (a busy hotspot) does not matter: each photo is the last picture
that came in. Photos go to captures/<date_time>/ as NN_scene_OK.jpg / _NG.jpg
(the raw camera picture) and NN_scene_OK_view.jpg (with the detection drawn).

Afterwards, with the values changed in the yaml, check all the photos again
without the robot:

    python3 tools/color_shoot.py --check captures/<date_time>

Run in the workspace, ROS_DOMAIN_ID=30. Wayland: QT_QPA_PLATFORM=xcb.
"""
import argparse
import glob
import os
import re
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

from autorace_perception.bev_projector import BevProjector
from autorace_perception.detect_level_crossing import DetectLevelCrossing
from autorace_perception.detect_lane import DetectLane
from autorace_perception.detect_sign import DetectSign
from autorace_perception.detect_traffic_light import DetectTrafficLight

FORWARD = '/camera/image_raw/compressed'
LANE = '/camera_lane/image_raw/compressed'

# (name, camera, kind, expected, what to show the camera)
SCENES = [
    ('light_red', FORWARD, 'light', 'red', 'traffic light, RED lamp on, ~0.5-1 m ahead'),
    ('light_yellow', FORWARD, 'light', 'yellow', 'traffic light, YELLOW lamp on'),
    ('light_green', FORWARD, 'light', 'green', 'traffic light, GREEN lamp on'),
    ('lane_straight', LANE, 'lane', 'both', 'robot in the middle of a straight lane'),
    ('lane_curve', LANE, 'lane', 'both', 'robot in a bend, both lines in view'),
    ('lane_glare', LANE, 'lane', 'both', 'where the lights shine on the yellow tape'),
    ('sign_left', FORWARD, 'sign', 'left', 'LEFT turn sign ~0.5-0.8 m ahead'),
    ('sign_right', FORWARD, 'sign', 'right', 'RIGHT turn sign ~0.5-0.8 m ahead'),
    ('sign_parking', FORWARD, 'sign', 'parking', 'PARKING sign ~0.5-0.8 m ahead'),
    ('sign_intersection', FORWARD, 'sign', 'intersection', 'INTERSECTION warning sign'),
    ('bar_down', FORWARD, 'bar', 'closed', 'level crossing bar DOWN across the road, ~0.5 m ahead'),
    ('bar_up', FORWARD, 'bar', 'not closed', 'level crossing bar UP (raised), same place'),
]
LIGHT_NAMES = {0: 'none', 1: 'red', 2: 'yellow', 3: 'green'}
LANE_STATES = {0: 'none', 1: 'yellow only', 2: 'both', 3: 'white only'}
GREEN, RED, WHITE = (0, 200, 0), (0, 0, 255), (255, 255, 255)


class Detectors:
    """The robot's detectors, built from the yaml, called on one picture at a time."""

    def __init__(self):
        self.light = DetectTrafficLight()
        self.sign = DetectSign()
        self.bev = BevProjector()
        self.lane = DetectLane()
        self.bar = DetectLevelCrossing()
        if not self.bev.fx:
            # No camera_info is waited for: the C920's values (camera_lane.yaml).
            self.bev.fx, self.bev.fy, self.bev.cx, self.bev.cy = 306.2, 306.2, 216.0, 120.0
        self.bev.build_homography()

    def check(self, kind, image):
        """(what was seen, view image with it drawn)."""
        view = image.copy()
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        if kind == 'light':
            lamps = sorted(self.light.lamps(hsv), reverse=True)
            seen = LIGHT_NAMES[lamps[0][1]] if lamps else 'none'
            for _, colour, (x, y, w, h) in lamps:
                cv2.rectangle(view, (x, y), (x + w, y + h), WHITE, 2)
                cv2.putText(view, LIGHT_NAMES[colour], (x, max(12, y - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, WHITE, 1)
            return seen, view
        if kind == 'sign':
            found = {}
            for name, box in (list(self.sign.direction_signs(hsv))
                              + list(self.sign.warning_signs(hsv))
                              + list(self.sign.parking_signs(hsv))):
                if name not in found or box[2] * box[3] > found[name][2] * found[name][3]:
                    found[name] = box
            for name, (x, y, w, h) in found.items():
                cv2.rectangle(view, (x, y), (x + w, y + h), GREEN, 2)
                cv2.putText(view, name, (x, max(12, y - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, GREEN, 1)
            return (', '.join(sorted(found)) or 'none'), view
        if kind == 'bar':
            # Red and white stripes in a row, in the ROI: the detector's two ways.
            b = self.bar
            b.width = image.shape[1]
            stripes = b.stripes(hsv)
            state, alike = b.bar(stripes, hsv)
            fragments = []
            if state == 0 and b.fragments_enabled:
                state, fragments = b.fragment_bar(hsv)
            cv2.polylines(view, [b.roi_polygon(image.shape)], True, (255, 128, 0), 2)
            colour = RED if state == 2 else GREEN
            for _, _, _, (x, y, w, h) in alike:
                cv2.rectangle(view, (x, y), (x + w, y + h), colour, 2)
            for x, y in fragments:
                cv2.circle(view, (int(x), int(y)), 6, colour, 2)
            return ('none', 'open', 'closed')[state], view
        # lane: the camera picture to the ground (BEV), then the line masks.
        bev = cv2.warpPerspective(image, self.bev.homography, (self.bev.out_w, self.bev.out_h))
        if self.bev.blur > 1:
            bev = cv2.warpPerspective(cv2.GaussianBlur(image, (self.bev.blur, self.bev.blur), 0),
                                      self.bev.homography, (self.bev.out_w, self.bev.out_h))
        d = self.lane
        bhsv = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV)
        glare = d.glare_on_yellow(bhsv) if d.glare_kernel is not None else None
        white_only = d.thin_bright(bhsv) if d.white_contrast_min > 0 else None
        if glare is not None:
            not_glare = cv2.bitwise_not(glare)
            white_only = not_glare if white_only is None else cv2.bitwise_and(white_only, not_glare)
        y_pix, y_mask, _ = d.mask_line(bhsv, d.yellow, d.reliability_yellow, extra=glare)
        w_pix, w_mask, _ = d.mask_line(bhsv, d.white, d.reliability_white, only=white_only)
        have_y, have_w = y_pix > d.min_pixels, w_pix > d.min_pixels
        seen = 'both' if have_y and have_w else ('yellow only' if have_y else
                                                 ('white only' if have_w else 'none'))
        lines = (bev * 0.4).astype(np.uint8)
        lines[y_mask > 0] = (0, 220, 255)
        lines[w_mask > 0] = WHITE
        cv2.putText(lines, f'yellow {y_pix} px, white {w_pix} px (need > {d.min_pixels})',
                    (4, lines.shape[0] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, GREEN, 1)
        h = view.shape[0]
        lines = cv2.resize(lines, (int(lines.shape[1] * h / lines.shape[0]), h))
        return seen, np.hstack([view, lines])


# --- traffic light: values that make the photos read right ------------------

# Where each lamp colour may lie (OpenCV hue, 0-180). Wide: a green LED read
# H 30 and a yellow one H 23 on the practice course (2026-10-27).
FAMILY = {'red': lambda h: h <= 12 or h >= 165, 'yellow': lambda h: 8 <= h <= 40,
          'green': lambda h: 25 <= h <= 100}


def find_lamp(image, expected):
    """The lit lamp in a photo, looked for with loose limits: round, bright, of
    the expected colour, dark (the housing) to either side. Its median hue, the
    20 % lowest S and V of its pixels, area, brightness beside it, mean RGB."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hue, sat, val = cv2.split(hsv)
    height, width = val.shape
    lit = ((sat >= 50) & (val >= 130)).astype(np.uint8)
    contours, _ = cv2.findContours(lit, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 15:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        side = max(2, w // 3)
        if x - side < 0 or y <= 0 or x + w + side > width or y + h >= height:
            continue
        if not 0.5 < w / h < 2.0 or not 0.5 < area / (np.pi / 4 * w * h) < 1.3:
            continue
        beside = float(np.median(np.concatenate([val[y:y + h, x - side:x].ravel(),
                                                 val[y:y + h, x + w:x + w + side].ravel()])))
        if beside >= 160:
            continue                 # no dark housing: a window, a lamp in the hall
        inside = np.zeros_like(lit)
        cv2.drawContours(inside, [contour], -1, 1, -1)
        pix = hsv[(inside > 0) & (lit > 0)]
        if len(pix) < 10:
            continue
        hues = pix[:, 0].astype(float)
        if expected == 'red':      # red wraps round 180
            hues = np.where(hues > 90, hues - 180, hues)
        h_med = float(np.median(hues)) % 180
        if not FAMILY[expected](h_med):
            continue
        brightness = float(np.median(pix[:, 2]))
        score = brightness - beside  # bright lamp, dark housing
        if best is None or score > best['score']:
            rgb = image[inside > 0].mean(axis=0)[::-1]
            best = dict(score=score, h=h_med, s20=float(np.percentile(pix[:, 1], 20)),
                        v20=float(np.percentile(pix[:, 2], 20)), area=area, beside=beside,
                        box=(x, y, w, h), rgb=tuple(int(c) for c in rgb))
    return best


def light_values(light):
    return dict(sat=light.saturation_l, val=light.value_l, red=light.red_margin,
                y_l=light.hues[2][0], y_h=light.hues[2][1], g_l=light.hues[3][0],
                g_h=light.hues[3][1], area=light.min_area, housing=light.housing_value)


def set_light(light, v):
    light.saturation_l, light.value_l, light.red_margin = v['sat'], v['val'], v['red']
    light.hues = {2: (v['y_l'], v['y_h']), 3: (v['g_l'], v['g_h'])}
    light.min_area, light.housing_value = v['area'], v['housing']


def tune_light(det, photos):
    """photos: [(name, image, expected colour)]. Values (from the current ones,
    changed no more than needed) under which every photo reads its colour:
    (values, lines to print, all read right)."""
    now = light_values(det.light)
    v = dict(now)
    lines = []
    lamps = {}
    for name, image, expected in photos:
        lamp = find_lamp(image, expected)
        if lamp is None:
            lines.append(f'  {name}: no lit {expected} lamp found even with loose limits '
                         '(nearer, straight on, the lamp fully in the picture?)')
            continue
        lamps[expected] = lamp
        lines.append(f'  {name}: lamp H {lamp["h"]:.0f} S {lamp["s20"]:.0f}+ V {lamp["v20"]:.0f}+ '
                     f'area {lamp["area"]:.0f} beside V {lamp["beside"]:.0f}  (RGB {lamp["rgb"]})')
        v['sat'] = int(min(v['sat'], lamp['s20'] - 5))
        v['val'] = int(min(v['val'], lamp['v20'] - 5))
        v['area'] = int(min(v['area'], lamp['area'] * 0.6))
        v['housing'] = int(max(v['housing'], lamp['beside'] + 10))
        if expected == 'red':
            v['red'] = int(max(v['red'], min(lamp['h'], 180 - lamp['h']) + 4))
        elif expected == 'yellow':
            v['y_l'] = int(min(v['y_l'], lamp['h'] - 4))
            v['y_h'] = int(max(v['y_h'], lamp['h'] + 2))
        else:
            v['g_h'] = int(max(v['g_h'], lamp['h'] + 4))
            v['g_l'] = int(min(v['g_l'], lamp['h'] - 2))
    # Yellow and green must not overlap: the boundary halfway between the two lamps.
    if 'yellow' in lamps and 'green' in lamps:
        mid = int(round((lamps['yellow']['h'] + lamps['green']['h']) / 2.0))
        v['y_h'], v['g_l'] = mid, mid + 1
    elif v['y_h'] >= v['g_l']:
        if 'green' in lamps:
            v['y_h'] = v['g_l'] - 1
        else:
            v['g_l'] = v['y_h'] + 1
    v['red'] = min(v['red'], max(1, v['y_l'] - 1))      # red stops short of yellow
    v['sat'], v['val'] = max(20, v['sat']), max(60, v['val'])
    old = light_values(det.light)
    set_light(det.light, v)
    good = True
    for name, image, expected in photos:
        seen, _ = det.check('light', image)
        good &= seen == expected
        lines.append(f'  with these values {name}: seen {seen} ({"OK" if seen == expected else "NG"})')
    set_light(det.light, old)
    return v, lines, good


def light_yaml(v):
    return ('detect_traffic_light:\n  ros__parameters:\n'
            f'    lamp:\n      saturation_l: {v["sat"]}\n      value_l: {v["val"]}\n'
            f'    red:\n      hue_margin: {v["red"]}\n'
            f'    yellow:\n      hue_l: {v["y_l"]}\n      hue_h: {v["y_h"]}\n'
            f'    green:\n      hue_l: {v["g_l"]}\n      hue_h: {v["g_h"]}\n'
            f'    min_area: {v["area"]}\n    housing:\n      value_h: {v["housing"]}')


def report_tuning(det, photos):
    v, lines, good = tune_light(det, photos)
    now = light_values(det.light)
    print('\n== traffic light: values that read these photos')
    print('\n'.join(lines))
    changed = {k: (now[k], v[k]) for k in v if v[k] != now[k]}
    if not changed:
        print('  nothing to change' if good else '  no change found that reads them all')
        return v, good
    print('  change (now -> suggested): ' + ', '.join(f'{k} {a} -> {b}' for k, (a, b) in changed.items()))
    print('  ' + ('all read right with them' if good else 'still NOT all right: see the lines above'))
    print('  into src/autorace_bringup/param/mission_real.yaml:\n' + light_yaml(v))
    return v, good


def ok(kind, expected, seen):
    if expected == 'not closed':
        return seen != 'closed'
    if kind == 'sign':
        return expected in seen.split(', ')
    return seen == expected


class Shooter(Node):

    def __init__(self, out_dir):
        super().__init__('color_shoot')
        self.det = Detectors()
        self.out_dir = out_dir
        self.frames = {}
        for topic in (FORWARD, LANE):
            self.create_subscription(CompressedImage, topic,
                                     lambda m, t=topic: self.frames.__setitem__(t, (m, time.time())),
                                     qos_profile_sensor_data)
        self.index = 0
        self.results = {}
        self.light_photos = {}      # scene -> (image, expected): the latest light photos
        self.suggested = None
        self.window = 'color shoot  [SPACE/Enter photo | n next | p pass | b back | q quit]'
        cv2.namedWindow(self.window, cv2.WINDOW_AUTOSIZE)

    def announce(self):
        name, _, _, expected, what = SCENES[self.index]
        print(f'\n[{self.index + 1}/{len(SCENES)}] {name}: {what}  (want: {expected})')

    def save(self, image, view, verdict):
        name = SCENES[self.index][0]
        base = os.path.join(self.out_dir, f'{self.index + 1:02d}_{name}_{verdict}')
        # A retake never replaces an earlier photo: _2, _3, ...
        if os.path.exists(base + '.jpg'):
            n = 2
            while os.path.exists(f'{base}_{n}.jpg'):
                n += 1
            base = f'{base}_{n}'
        cv2.imwrite(base + '.jpg', image)
        cv2.imwrite(base + '_view.jpg', view)
        return base + '.jpg'

    def step(self):
        name, topic, kind, expected, what = SCENES[self.index]
        got = self.frames.get(topic)
        if got is None:
            canvas = np.zeros((300, 640, 3), np.uint8)
            cv2.putText(canvas, f'no picture yet on {topic}', (10, 150),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, RED, 1)
            image = view = None
            seen, good, age = 'none', False, 0.0
        else:
            msg, when = got
            image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            seen, view = self.det.check(kind, image)
            good = ok(kind, expected, seen)
            age = time.time() - when
            canvas = view
        head = np.zeros((48, canvas.shape[1], 3), np.uint8)
        cv2.putText(head, f'[{self.index + 1}/{len(SCENES)}] {name}: {what}', (6, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, WHITE, 1)
        cv2.putText(head, f'want {expected}   seen {seen}   {"OK" if good else "NG"}'
                    f'   (picture {age:.1f} s old)', (6, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, GREEN if good else RED, 1)
        cv2.imshow(self.window, np.vstack([head, canvas]))
        key = cv2.waitKey(30) & 0xFF
        if key == ord('q') or key == 27:
            return False
        if key == ord('p'):
            # Pass: on to the next scene, no photo.
            self.results.setdefault(name, ('-', 'passed', ''))
            print('  passed (no photo)')
            if self.index + 1 >= len(SCENES):
                print('\nall scenes done.')
                return False
            self.index += 1
            self.announce()
        elif key == ord('a') and self.suggested is not None:
            set_light(self.det.light, self.suggested)
            print('  suggested traffic light values in use for this session: '
                  + str(self.suggested))
            self.suggested = None
        elif key == ord('b'):
            self.index = max(0, self.index - 1)
            self.announce()
        elif key in (ord(' '), 13, 10, ord('n')) and image is not None:
            verdict = 'OK' if good else 'NG'
            path = self.save(image, view, verdict)
            if kind == 'light':
                self.light_photos[name] = (image, expected)
                if not good:
                    self.suggested, fits = report_tuning(
                        self.det, [(n, im, e) for n, (im, e) in self.light_photos.items()])
                    print('  a: try these values now (this session only; the yaml is not changed)')
            self.results[name] = (verdict, seen, path)
            print(f'  photo {path}: seen {seen} -> {verdict}')
            if good or key == ord('n'):
                if self.index + 1 >= len(SCENES):
                    print('\nall scenes done.')
                    return False
                self.index += 1
                self.announce()
            else:
                print('  NG: change the scene (distance, angle, light) or the values, SPACE again;'
                      ' n to move on')
        elif key == ord('n'):
            self.index = min(len(SCENES) - 1, self.index + 1)
            self.announce()
        return True

    def summary(self):
        lines = [f'{name:<18} {self.results[name][0] if name in self.results else "-":<3} '
                 f'seen {self.results[name][1] if name in self.results else "(no photo)"}'
                 for name, *_ in SCENES]
        text = '\n'.join(lines)
        print('\n== summary\n' + text)
        with open(os.path.join(self.out_dir, 'summary.txt'), 'w') as f:
            f.write(text + '\n')


def check_photos(directory):
    """Run the detectors with the current yaml on photos taken before."""
    det = Detectors()
    kinds = {name: (kind, expected) for name, _, kind, expected, _ in SCENES}
    rows = []
    for path in sorted(glob.glob(os.path.join(directory, '*.jpg'))):
        if path.endswith('_view.jpg'):
            continue
        m = re.match(r'\d+_(.+?)_(OK|NG)(?:_\d+)?\.jpg$', os.path.basename(path))
        if not m or m.group(1) not in kinds:
            continue
        kind, expected = kinds[m.group(1)]
        image = cv2.imread(path)
        # The lane detector's lightness floor follows the pictures: let it
        # settle on this one, as it does on the robot over a few frames.
        for _ in range(15 if kind == 'lane' else 1):
            seen, view = det.check(kind, image)
        verdict = 'OK' if ok(kind, expected, seen) else 'NG'
        cv2.imwrite(path.replace('.jpg', '_recheck.jpg'), view)
        rows.append(f'{os.path.basename(path):<34} want {expected:<12} seen {seen:<20} {verdict}'
                    f'   (when taken: {m.group(2)})')
    print('\n'.join(rows) if rows else f'no photos in {directory}')
    photos = []
    for path in sorted(glob.glob(os.path.join(directory, '*_light_*.jpg'))):
        m = re.match(r'\d+_(light_\w+?)_(OK|NG)(?:_\d+)?\.jpg$', os.path.basename(path))
        if m and m.group(1) in kinds:
            photos.append((m.group(1), cv2.imread(path), kinds[m.group(1)][1]))
    if photos:
        report_tuning(det, photos)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--check', metavar='DIR', help='re-check the photos in DIR with the current yaml')
    ap.add_argument('--start', type=int, default=1, help='begin at scene number N')
    args = ap.parse_args()
    here = os.path.dirname(os.path.abspath(__file__))
    param = os.path.join(here, '..', 'src', 'autorace_bringup', 'param')
    rclpy.init(args=['--ros-args',
                     '--params-file', os.path.join(param, 'perception_real.yaml'),
                     '--params-file', os.path.join(param, 'mission_real.yaml'),
                     '-p', 'publish_debug_image:=false'])
    try:
        if args.check:
            check_photos(args.check)
            return
        out_dir = os.path.join('captures', time.strftime('%Y%m%d_%H%M'))
        os.makedirs(out_dir, exist_ok=True)
        print(f'photos to {out_dir}/')
        node = Shooter(out_dir)
        node.index = max(0, min(len(SCENES) - 1, args.start - 1))
        node.announce()
        try:
            while rclpy.ok() and node.step():
                rclpy.spin_once(node, timeout_sec=0.0)
        except KeyboardInterrupt:
            pass
        node.summary()
        cv2.destroyAllWindows()
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
