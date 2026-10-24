#!/usr/bin/env python3
"""picam_calibrate.py : the lane camera's pitch, yaw and height from one picture of a straight lane.

Put the robot on a straight stretch, between the yellow line (left) and the white one (right),
pointing along it. Then, with the camera publishing:
  python3 tools/picam_calibrate.py --camera /camera --write tools/drive_test_picam.yaml

How: the two lines are straight and parallel on the floor, so in the picture they meet at the
vanishing point of the lane's direction. Its row gives the pitch (how far the horizon is above
the image centre), its column the camera's yaw against the lane. The height follows from the
lines' known spacing (lane.width_m between their centres): projected onto the floor with a camera
1 m up they lie width/h apart, so h = width / that. The intrinsics come from camera_info.

Writes bev_projector camera.height / pitch / yaw_check and a bev.near that the bottom image row
reaches, into the given yaml (other keys kept). Prints a picture with the fitted lines to check.
"""
import argparse
import math
import time

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage


def line_fit(mask):
    """(vx, vy, x0, y0) of the largest piece of a mask, and its pixel count; None if too small."""
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if n <= 1:
        return None, 0
    i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    ys, xs = np.nonzero(labels == i)
    if len(xs) < 200:
        return None, len(xs)
    vx, vy, x0, y0 = cv2.fitLine(np.stack([xs, ys], 1).astype(np.float32), cv2.DIST_HUBER, 0, 0.01, 0.01).ravel()
    return (vx, vy, x0, y0), len(xs)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--camera', default='/camera')
    p.add_argument('--width', type=float, default=0.33, help='lane width between line centres, m')
    p.add_argument('--forward-offset', type=float, default=0.045, help='lens ahead of the axle, m')
    p.add_argument('--white-only', action='store_true',
                   help='only the white line in view: robot centred and straight (yaw 0, line width/2 right)')
    p.add_argument('--write', help='yaml to put the result in')
    p.add_argument('--picture', default='/tmp/picam_calibrate.jpg')
    a = p.parse_args()

    rclpy.init()
    node = rclpy.create_node('picam_calibrate')
    got = {}
    node.create_subscription(CompressedImage, a.camera + '/image_raw/compressed',
                             lambda m: got.__setitem__('j', m), qos_profile_sensor_data)
    node.create_subscription(CameraInfo, a.camera + '/camera_info',
                             lambda m: got.__setitem__('i', m), qos_profile_sensor_data)
    end = time.time() + 10
    while time.time() < end and not ('j' in got and 'i' in got):
        rclpy.spin_once(node, timeout_sec=0.05)
    if 'j' not in got or 'i' not in got:
        raise SystemExit('no picture or camera_info from ' + a.camera)
    img = cv2.imdecode(np.frombuffer(got['j'].data, np.uint8), cv2.IMREAD_COLOR)
    fx, fy, cx, cy = got['i'].k[0], got['i'].k[4], got['i'].k[2], got['i'].k[5]
    hsv = cv2.cvtColor(cv2.GaussianBlur(img, (5, 5), 0), cv2.COLOR_BGR2HSV)
    yellow = cv2.inRange(hsv, (20, 110, 120), (38, 255, 255))   # not the brown of a wooden floor
    white = cv2.inRange(hsv, (0, 0, 170), (179, 60, 255))
    k = np.ones((5, 5), np.uint8)
    yellow, white = (cv2.morphologyEx(m, cv2.MORPH_OPEN, k) for m in (yellow, white))
    ly, ny = line_fit(yellow)
    lw, nw = line_fit(white)
    out = img.copy()
    for line, colour in ((ly, (255, 0, 0)), (lw, (255, 0, 255))):
        if line is not None:
            vx, vy, x0, y0 = line
            cv2.line(out, (int(x0 - 1000 * vx), int(y0 - 1000 * vy)), (int(x0 + 1000 * vx), int(y0 + 1000 * vy)), colour, 2)
    cv2.imwrite(a.picture, out)
    print(f'picture {img.shape[1]}x{img.shape[0]}, fx {fx:.1f}; yellow {ny} px, white {nw} px; lines drawn in {a.picture}')
    if a.white_only:
        if lw is None:
            raise SystemExit('no white line in view')
        # Yaw 0: the lane's vanishing point is on the centre column; the white line meets it there.
        v2x, v2y, x2, y2 = lw
        u, v = cx, y2 + (cx - x2) / v2x * v2y
    else:
        if ly is None or lw is None:
            raise SystemExit('both lines must be in view (yellow left, white right): move or turn the '
                             'robot, or --white-only')
        # Vanishing point: where the two image lines cross.
        (v1x, v1y, x1, y1), (v2x, v2y, x2, y2) = ly, lw
        A = np.array([[v1x, -v2x], [v1y, -v2y]], float)
        t = np.linalg.solve(A, np.array([x2 - x1, y2 - y1], float))
        u, v = x1 + t[0] * v1x, y1 + t[0] * v1y
    pitch = math.atan2(cy - v, fy)                 # positive: looking down
    yaw = math.atan2(u - cx, fx)                   # camera turned right of the lane (if positive)
    # Floor positions of the lines with the camera 1 m up: rays through the bottom-half points.
    def lateral(line, row):
        vx, vy, x0, y0 = line
        x = x0 + (row - y0) / vy * vx
        # camera frame: forward along the axis, down by the pitch
        dx = (x - cx) / fx
        dy = (row - cy) / fy
        cp, sp = math.cos(pitch), math.sin(pitch)
        down = dy * cp + sp                        # ray's downward component (per unit forward)
        ahead = cp - dy * sp
        return -dx / down * 1.0, ahead / down     # (left of the camera, ahead) at height 1
    row = int(min(img.shape[0] - 5, 0.85 * img.shape[0]))
    left_w, _ = lateral(lw, row)
    if a.white_only:
        height = (a.width / 2.0) / abs(left_w)
        if ly is not None:
            # Check: the bit of yellow in view should land half a lane to the left.
            ys, xs = np.nonzero(yellow)
            dx, dy = (xs - cx) / fx, (ys - cy) / fy
            cp, sp = math.cos(pitch), math.sin(pitch)
            left = -dx / (dy * cp + sp) * height
            print(f'check: yellow pixels lie {np.median(left):+.3f} m left of the camera '
                  f'(expected about {+a.width / 2:+.3f})')
    else:
        left_y, _ = lateral(ly, row)
        height = a.width / abs(left_y - left_w) * math.cos(yaw)
    near_row = img.shape[0] - 1
    dy = (near_row - cy) / fy
    near = height * (math.cos(pitch) - dy * math.sin(pitch)) / (dy * math.cos(pitch) + math.sin(pitch)) + a.forward_offset
    print(f'vanishing point ({u:.0f}, {v:.0f}): pitch {math.degrees(pitch):.1f} deg ({pitch:.3f} rad), '
          f'yaw {math.degrees(yaw):+.1f} deg; height {height:.3f} m; bottom row reaches {near:.3f} m ahead of the axle')
    if abs(math.degrees(yaw)) > 5:
        print('note: the camera looks more than 5 deg off the lane; set the robot straight and run again')
    if a.write:
        try:
            doc = yaml.safe_load(open(a.write)) or {}
        except FileNotFoundError:
            doc = {}
        bev = doc.setdefault('bev_projector', {}).setdefault('ros__parameters', {})
        bev.setdefault('camera', {}).update({'height': round(height, 4), 'pitch': round(pitch, 4),
                                             'forward_offset': a.forward_offset})
        bev.setdefault('bev', {})['near'] = round(near + 0.01, 3)
        doc.setdefault('detect_lane', {}).setdefault('ros__parameters', {}).setdefault('bev', {})['near'] = \
            round(near + 0.01, 3)
        with open(a.write, 'w') as f:
            f.write(f'# picam_calibrate.py {time.strftime("%Y-%m-%d %H:%M")}: pitch {math.degrees(pitch):.1f} deg, '
                    f'yaw {math.degrees(yaw):+.1f} deg, height {height:.3f} m\n')
            yaml.safe_dump(doc, f, sort_keys=False)
        print('written to', a.write)


if __name__ == '__main__':
    main()
