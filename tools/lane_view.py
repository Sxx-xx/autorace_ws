#!/usr/bin/env python3
"""lane_view.py : the lane pipeline's pictures, live, in one OpenCV window (not rqt).

By default only the final picture: the camera picture with the lines found,
the remembered path, the target and the arc the controller drives drawn on
(/detect/image_overlay, from lane_pipeline), enlarged, with its rate. --map
shows the detector's own top-down picture (/detect/image_lane) instead.
Nothing else is subscribed, so lane_pipeline publishes no bird's eye view or
decoded picture for it and keeps its whole time for the lane. --all shows the
six steps instead (they cost the pipeline those extra pictures):

  1 camera        the decoded lane camera (<camera>/image_decoded)
  2 BEV           bev_projector's bird's eye view (/camera/image_projected)
  3 detector      the lane detector's own picture (/detect/image_lane): the
                  sliding windows and fitted lines, or detect_lane's map
  4 yellow mask   HSV thresholds on the BEV, as the detector has them
  5 white mask    (detect_lane section of perception_<profile>.yaml)
  6 Sobel x       edges across the BEV (what sliding.sobel_min keeps white near)

Run on the PC with the lane pipeline up (race.launch.py or lane_drive.launch.py):
  python3 tools/lane_view.py                          # C920: /camera_lane
  python3 tools/lane_view.py --camera /camera         # the forward camera
q or Esc closes it. --save FILE writes the mosaic to FILE after --after
seconds instead of showing a window (and exits), for comparing settings.
"""
import argparse
import os
import time

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

TILE = (400, 150)       # width, height of each picture in the window


def thresholds(profile):
    """The detector's HSV bounds from perception_<profile>.yaml (detect_lane's defaults otherwise)."""
    path = os.path.join(get_package_share_directory('autorace_bringup'), 'param',
                        f'perception_{profile}.yaml')
    params = yaml.safe_load(open(path)).get('detect_lane', {}).get('ros__parameters', {})
    lane = params.get('detect', {}).get('lane', {})
    defaults = {'white': (0, 179, 0, 70, 105, 255), 'yellow': (10, 127, 70, 255, 95, 255)}
    bounds = {}
    for colour, d in defaults.items():
        c = lane.get(colour, {})
        get = lambda k, i: c.get(k, d[i])
        bounds[colour] = (np.array([get('hue_l', 0), get('saturation_l', 2), get('lightness_l', 4)],
                                   np.uint8),
                          np.array([get('hue_h', 1), get('saturation_h', 3), get('lightness_h', 5)],
                                   np.uint8))
    return bounds


def tile(image, label):
    """Fit a picture into a tile, letterboxed, with its label."""
    out = np.zeros((TILE[1], TILE[0], 3), np.uint8)
    if image is not None:
        if image.ndim == 2:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        scale = min(TILE[0] / image.shape[1], TILE[1] / image.shape[0])
        small = cv2.resize(image, (int(image.shape[1] * scale), int(image.shape[0] * scale)))
        y, x = (TILE[1] - small.shape[0]) // 2, (TILE[0] - small.shape[1]) // 2
        out[y:y + small.shape[0], x:x + small.shape[1]] = small
    else:
        cv2.putText(out, 'waiting...', (140, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (120, 120, 120), 1)
    cv2.rectangle(out, (0, 0), (TILE[0] - 1, TILE[1] - 1), (80, 80, 80), 1)
    cv2.putText(out, label, (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera', default='/camera_lane', help='camera namespace')
    parser.add_argument('--profile', default='real')
    parser.add_argument('--all', action='store_true',
                        help='all six steps instead of the final picture only')
    parser.add_argument('--map', action='store_true',
                        help="the detector's top-down picture instead of the camera overlay")
    parser.add_argument('--scale', type=float, default=2.0, help='final picture enlarged by')
    parser.add_argument('--save', help='write the picture here and exit, no window')
    parser.add_argument('--after', type=float, default=8.0, help='seconds before --save')
    args = parser.parse_args()
    bounds = thresholds(args.profile)

    rclpy.init()
    node = rclpy.create_node('lane_view')
    bridge = CvBridge()
    latest = {}
    counts = {}
    fresh = [False]         # a new picture since the window was last drawn

    def keep(name):
        def callback(msg):
            latest[name] = bridge.imgmsg_to_cv2(msg, 'bgr8')
            counts[name] = counts.get(name, 0) + 1
            fresh[0] = True
        return callback

    streams = [('detector', '/detect/image_lane' if args.map or args.all
                else '/detect/image_overlay')]
    if args.all:
        streams += [('camera', args.camera + '/image_decoded'), ('bev', '/camera/image_projected')]
    for name, topic in streams:
        node.create_subscription(Image, topic, keep(name), qos_profile_sensor_data)

    title = ('AutoRace lane preprocessing (q: quit)' if args.all
             else 'AutoRace lane: final picture (q: quit)')
    reason = 'ROS shut down (SIGINT or SIGTERM)'
    if not args.save:
        cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    rates, since, began = {}, time.time(), time.time()
    while rclpy.ok():
        # Take everything waiting (three streams at up to 90 Hz), then draw once.
        rclpy.spin_once(node, timeout_sec=0.01)
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.0)
        if time.time() - since >= 1.0:
            rates = {k: v / (time.time() - since) for k, v in counts.items()}
            counts.clear()
            since = time.time()
        if not fresh[0] and not args.save:
            if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
                reason = 'q or Esc'
                break
            continue
        fresh[0] = False
        if not args.all:
            final = latest.get('detector')
            if final is None:
                continue
            final = cv2.resize(final, None, fx=args.scale, fy=args.scale,
                               interpolation=cv2.INTER_NEAREST)
            label = f'shown {rates.get("detector", 0):.0f} Hz'
            cv2.putText(final, label, (final.shape[1] - 170, 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            if args.save:
                if time.time() - began >= args.after:
                    cv2.imwrite(args.save, final)
                    break
                continue
            cv2.imshow(title, final)
            if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
                reason = 'q or Esc'
                break
            continue
        bev = latest.get('bev')
        yellow = white = edges = None
        if bev is not None:
            hsv = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV)
            yellow = cv2.inRange(hsv, *bounds['yellow'])
            white = cv2.inRange(hsv, *bounds['white'])
            gray = cv2.cvtColor(bev, cv2.COLOR_BGR2GRAY)
            edges = cv2.convertScaleAbs(cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3))
        rate = lambda k: f' {rates.get(k, 0):.0f} Hz'
        top = np.hstack([tile(latest.get('camera'), '1 camera ' + args.camera + rate('camera')),
                         tile(bev, '2 BEV (warpPerspective)' + rate('bev')),
                         tile(latest.get('detector'), '3 detector /detect/image_lane' + rate('detector'))])
        bottom = np.hstack([tile(yellow, '4 yellow HSV mask'), tile(white, '5 white HSV mask'),
                            tile(edges, '6 Sobel x')])
        if args.save:
            if time.time() - began >= args.after:
                cv2.imwrite(args.save, np.vstack([top, bottom]))
                break
            continue
        # A window closed with its X comes back: only q or Esc end the viewer.
        cv2.imshow(title, np.vstack([top, bottom]))
        if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
            reason = 'q or Esc'
            break
    else:
        reason = 'ROS shut down (SIGINT or SIGTERM)'
    if args.save:
        reason = f'saved {args.save}'
    print(f'lane_view closed: {reason}', flush=True)
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
