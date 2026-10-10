#!/usr/bin/env python3
"""mission_marker.py : the mission state and the construction takeover strip as RViz markers.

/viz/mission (MarkerArray, frame base_link):
  - text above the robot: the mission manager's state (/autorace/mission_state), red while a
    mission other than lane following runs, with the last trigger and sign seen;
  - the strip the construction mission watches (0..takeover_distance ahead of the axle,
    +/- takeover_half_width): grey, orange with 3 or more laser returns in it (what makes the
    construction mission ask to take over after two scans in a row).
Read with tools/scan_view.rviz.
"""
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

try:
    from autorace_msgs.msg import MissionState, SignDetection
except ImportError:
    MissionState = SignDetection = None

LIDAR_OFFSET = -0.032       # LDS-02 behind the axle
REACH, HALF_WIDTH = 0.45, 0.13


class MissionMarker(Node):

    def __init__(self):
        super().__init__('mission_marker')
        self.state = 'waiting for the mission manager'
        self.state_code = None
        self.trigger = ('', 0.0)
        self.sign = ('', 0.0)
        self.in_strip = 0
        self.pub = self.create_publisher(MarkerArray, '/viz/mission', 1)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 5)
        self.create_subscription(String, '/autorace/trigger',
                                 lambda m: setattr(self, 'trigger', (m.data, time.time())), 5)
        if MissionState is not None:
            self.create_subscription(MissionState, '/autorace/mission_state', self.on_state, 5)
            self.create_subscription(SignDetection, '/autorace/sign',
                                     lambda m: setattr(self, 'sign', (m.name, time.time())), 5)
        self.create_timer(0.1, self.publish)

    def on_state(self, m):
        self.state = m.detail
        self.state_code = m.state

    def on_scan(self, m):
        r = np.asarray(m.ranges, dtype=np.float32)
        a = m.angle_min + np.arange(len(r)) * m.angle_increment
        ok = np.isfinite(r) & (r > m.range_min + 1e-3) & (r < 1.0)
        x = LIDAR_OFFSET + r[ok] * np.cos(a[ok])
        y = r[ok] * np.sin(a[ok])
        self.in_strip = int(np.count_nonzero((x > 0.0) & (x < REACH) & (np.abs(y) < HALF_WIDTH)))

    def marker(self, mid, kind):
        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'mission'
        m.id = mid
        m.type = kind
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def publish(self):
        now = time.time()
        busy = self.state_code not in (None, 0, 1) and not self.state.startswith('lane following')
        construction = self.state.startswith('construction')

        text = self.marker(0, Marker.TEXT_VIEW_FACING)
        text.pose.position.x, text.pose.position.z = 0.15, 0.35
        text.scale.z = 0.07
        lines = [('CONSTRUCTION: ' if construction else 'MISSION: ') + self.state]
        if now - self.trigger[1] < 3.0:
            lines.append(f'trigger: {self.trigger[0]}')
        if now - self.sign[1] < 1.0:
            lines.append(f'sign: {self.sign[0]}')
        lines.append(f'returns in the strip: {self.in_strip}')
        text.text = '\n'.join(lines)
        text.color.r, text.color.g, text.color.b, text.color.a = (
            (1.0, 0.2, 0.2, 1.0) if busy else (0.3, 1.0, 0.3, 1.0))

        strip = self.marker(1, Marker.CUBE)
        strip.pose.position.x = REACH / 2.0
        strip.pose.position.z = 0.005
        strip.scale.x, strip.scale.y, strip.scale.z = REACH, 2 * HALF_WIDTH, 0.01
        hot = self.in_strip >= 3
        strip.color.r, strip.color.g, strip.color.b, strip.color.a = (
            (1.0, 0.55, 0.0, 0.55) if hot else (0.6, 0.6, 0.6, 0.25))

        self.pub.publish(MarkerArray(markers=[text, strip]))


def main():
    rclpy.init()
    node = MissionMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
