#!/usr/bin/env python3
"""rate_probe.py : how a compressed camera stream arrives on the PC, measured in the node.

Not `ros2 topic hz` (slow on big messages, nothing on gaps): every second one
line with the rate, frame size, Mbps, the largest gap between frames, how
many gaps were over 200 ms, the frame's age on arrival (arrival time minus
header.stamp: includes the PC-Pi clock offset, so compare runs, not absolutes,
until the clocks are synced) and the mean brightness (the JPEG decoded at 1/8
size). A summary over the whole run at the end.

  python3 tools/rate_probe.py /camera/image_raw/compressed --seconds 60
"""
import argparse
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('topic', nargs='?', default='/camera/image_raw/compressed')
    parser.add_argument('--seconds', type=float, default=60.0)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node('rate_probe')
    clock = node.get_clock()
    arrivals, sizes, ages, levels = [], [], [], []

    def callback(msg):
        now = clock.now()
        arrivals.append(time.monotonic())
        sizes.append(len(msg.data))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        ages.append(now.nanoseconds / 1e9 - stamp)
        small = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_REDUCED_GRAYSCALE_8)
        levels.append(float(small.mean()) if small is not None else float('nan'))

    node.create_subscription(CompressedImage, args.topic, callback, qos_profile_sensor_data)
    print(f'{args.topic} for {args.seconds:.0f} s')
    start = time.monotonic()
    second_start, first = start, 0
    per_second = []
    while time.monotonic() - start < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.01)
        if time.monotonic() - second_start >= 1.0:
            span = time.monotonic() - second_start
            now_index = len(arrivals)
            a = np.array(arrivals[max(0, first - 1):now_index])
            gaps = np.diff(a) * 1000 if len(a) > 1 else np.array([0.0])
            n = now_index - first
            s = np.array(sizes[first:now_index]) if n else np.array([0])
            age = np.array(ages[first:now_index]) * 1000 if n else np.array([np.nan])
            lvl = np.nanmean(levels[first:now_index]) if n else float('nan')
            per_second.append(n / span)
            print(f'{n / span:5.1f} Hz  {s.mean() / 1024:5.1f} KB  {s.sum() * 8 / span / 1e6:5.1f} Mbps  '
                  f'max gap {gaps.max():6.1f} ms  gaps>200ms {int((gaps > 200).sum())}  '
                  f'age p50 {np.nanmedian(age):6.1f} ms  brightness {lvl:5.1f}', flush=True)
            first, second_start = now_index, time.monotonic()
    span = time.monotonic() - start
    gaps = np.diff(np.array(arrivals)) * 1000 if len(arrivals) > 1 else np.array([0.0])
    age = np.array(ages) * 1000
    ps = np.array(per_second)
    print(f'\nsummary {span:.0f} s: {len(arrivals) / span:.1f} Hz mean, per-second min {ps.min():.1f} '
          f'p5 {np.percentile(ps, 5):.1f} median {np.median(ps):.1f}; '
          f'{np.mean(sizes) / 1024 if sizes else 0:.1f} KB/frame, '
          f'{np.sum(sizes) * 8 / span / 1e6:.1f} Mbps; gaps ms p50 {np.median(gaps):.1f} '
          f'p99 {np.percentile(gaps, 99):.1f} max {gaps.max():.1f}, '
          f'>200 ms: {int((gaps > 200).sum())} ({(gaps > 200).sum() / span * 60:.1f}/min); '
          f'age ms p50 {np.median(age):.1f} p95 {np.percentile(age, 95):.1f} '
          f'max {age.max():.1f} (with the clock offset); brightness {np.nanmean(levels):.1f}')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
