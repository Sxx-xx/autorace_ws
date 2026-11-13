#!/usr/bin/python3
"""parking_timing.py : how long each step of the parking took, from a bag.

Reads the parking mission's log lines (/rosout) and the odometry from a bag
recorded by parking_test.launch.py, and prints each step with the time and
the distance driven since the mission started. The first line to look at is
the entrance: how long, and how far, from the start of the mission (right
after the construction zone) until the gap in the left yellow line was found.

  python3 tools/parking_timing.py bags/<bag>              (a finished bag)
  python3 tools/parking_timing.py bags/<bag>/<bag>_0.mcap (one still recording)
"""
import math
import sys

from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions

STEPS = (
    ('started', '시작'),
    ('gap in the left yellow line', '입구 인식'),
    ('at the entrance', '입구에서 회전'),
    ('turned in', '진입로'),
    ('the lot, as', '주차장 도착'),
    ('into the', '칸 선택'),
    ('in the bay', '칸 안'),
    ('backing round', '후진 호'),
    ('mouth of the lane', '진입로 끝'),
    ('back on the road', '도로 복귀'),
    ('no entrance', '입구 못 찾음'),
)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return
    reader = SequentialReader()
    reader.open(StorageOptions(uri=sys.argv[1], storage_id='mcap'), ConverterOptions('cdr', 'cdr'))
    reader.set_filter(StorageFilter(topics=['/rosout', '/odom']))
    t0, travelled, last = None, 0.0, None
    runs = []                       # each parking: list of (time, distance, label, message)
    for _ in iter(int, 1):
        if not reader.has_next():
            break
        topic, data, stamp = reader.read_next()
        t0 = t0 or stamp
        now = (stamp - t0) / 1e9
        if topic == '/odom':
            p = deserialize_message(data, Odometry).pose.pose.position
            if last is not None:
                travelled += math.hypot(p.x - last[0], p.y - last[1])
            last = (p.x, p.y)
            continue
        msg = deserialize_message(data, Log)
        if msg.name != 'parking_mission':
            continue
        for key, label in STEPS:
            if key in msg.msg:
                if key == 'started':
                    runs.append([])
                if runs:
                    runs[-1].append((now, travelled, label, msg.msg))
                break
    if not runs:
        print('no parking in this bag')
    for i, run in enumerate(runs, 1):
        t_start, d_start = run[0][0], run[0][1]
        print(f'== parking {i}: started at {t_start:.1f} s in the bag')
        print(f'   {"step":<10} {"s":>6} {"m":>6}   log')
        for now, travelled, label, text in run:
            print(f'   {label:<10} {now - t_start:6.1f} {travelled - d_start:6.2f}   {text[:90]}')
        entrance = [r for r in run if r[2] == '입구 인식']
        if entrance:
            print(f'   -> 입구 인식까지 {entrance[0][0] - t_start:.1f} s, {entrance[0][1] - d_start:.2f} m')
        else:
            print('   -> 입구를 인식하지 못함')


if __name__ == '__main__':
    main()
