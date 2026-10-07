#!/usr/bin/env python3
"""LDRobot LD-series lidar (LD06/LD08/LD14/LD19 protocol) to sensor_msgs/LaserScan.

The robot's lidar turned out to speak the LDRobot 47-byte packet protocol at
115200 baud (header 0x54 0x2C, 12 points a packet, distances in mm), which
neither hls_lfcd_lds_driver (LDS-01) nor ld08_driver (230400) reads. Each
revolution is binned into 360 one-degree ranges like the LDS-01 driver's
scans, which is what the missions were written against.

Parameters:
  port (/dev/ttyUSB0), baud (115200), frame_id (base_scan),
  angle_offset_deg (0.0): lidar angle that points along the robot's +x,
  clockwise (true): the lidar's angles grow clockwise seen from above, so
    they are mirrored into ROS's counter-clockwise convention,
  range_min (0.05), range_max (8.0) in metres.
"""
import math
import struct
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial


class LdLidar(Node):

    def __init__(self):
        super().__init__('ld_lidar')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frame_id', 'base_scan')
        self.declare_parameter('angle_offset_deg', 0.0)
        self.declare_parameter('clockwise', True)
        self.declare_parameter('range_min', 0.05)
        self.declare_parameter('range_max', 8.0)
        p = self.get_parameter
        self.frame_id = p('frame_id').value
        self.offset = p('angle_offset_deg').value
        self.clockwise = p('clockwise').value
        self.range_min = p('range_min').value
        self.range_max = p('range_max').value
        self.pub = self.create_publisher(LaserScan, 'scan', 10)
        self.ser = serial.Serial(p('port').value, p('baud').value, timeout=0.2)
        self.ranges = [float('inf')] * 360
        self.intensities = [0.0] * 360
        self.last_angle = None
        self.scan_start = None
        self.buf = bytearray()
        self.get_logger().info(f"LD lidar on {p('port').value} at {p('baud').value}")

    def spin(self):
        while rclpy.ok():
            self.buf += self.ser.read(470)
            while True:
                i = self.buf.find(b'\x54\x2c')
                if i < 0 or len(self.buf) - i < 47:
                    if i > 0:
                        del self.buf[:i]
                    break
                self.packet(bytes(self.buf[i:i + 47]))
                del self.buf[:i + 47]

    def packet(self, p):
        start = struct.unpack_from('<H', p, 4)[0] / 100.0
        end = struct.unpack_from('<H', p, 42)[0] / 100.0
        span = (end - start) % 360.0
        if self.scan_start is None:
            self.scan_start = self.get_clock().now()
        for k in range(12):
            dist, intensity = struct.unpack_from('<HB', p, 6 + 3 * k)
            angle = (start + span * k / 11.0) % 360.0
            ros_deg = (self.offset - angle) if self.clockwise else (angle - self.offset)
            index = int(round(ros_deg)) % 360
            r = dist / 1000.0
            self.ranges[index] = r if self.range_min <= r <= self.range_max else float('inf')
            self.intensities[index] = float(intensity)
        # A revolution is complete when the angle wraps past 360.
        if self.last_angle is not None and end < self.last_angle - 180.0:
            self.publish()
        self.last_angle = end

    def publish(self):
        msg = LaserScan()
        now = self.get_clock().now()
        msg.header.stamp = self.scan_start.to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi - math.radians(1.0)
        msg.angle_increment = math.radians(1.0)
        msg.scan_time = (now - self.scan_start).nanoseconds * 1e-9
        msg.time_increment = msg.scan_time / 360.0
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.ranges = list(self.ranges)
        msg.intensities = list(self.intensities)
        self.pub.publish(msg)
        self.ranges = [float('inf')] * 360
        self.intensities = [0.0] * 360
        self.scan_start = now


def main(args=None):
    rclpy.init(args=args)
    node = LdLidar()
    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
