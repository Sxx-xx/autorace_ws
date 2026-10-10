#!/usr/bin/env python3
"""estop.py : a terminal that stops the robot at a key press.

Enter (or any line) stops it:
  1. the cmd_vel mux and the lane / mission nodes are killed at once, so nothing commands
     the wheels any more;
  2. zero velocity goes out on /cmd_vel for 2 s from a publisher that is already up;
  3. the race launch, perception, bag recorders and the video recorder are shut down.
The window stays open for the next run.  Ctrl+C closes it.
Run:  python3 tools/estop.py   (ROS_DOMAIN_ID=30)
"""
import subprocess
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

KILL_NOW = ['autorace_core/cmd_vel_mux', 'autorace_core/lane_controller',
            'autorace_core/mission_manager', 'autorace_core/.*_mission',
            'image_lane_follower.py']
SHUT_DOWN = ['ros2 launch autorace_bringup', 'autorace_perception/',
             'image_transport.*republish', 'ros2 bag record', 'tools/record_run.py']


def pkill(sig, pattern):
    subprocess.run(['pkill', sig, '-f', pattern], stdout=subprocess.DEVNULL,
                   stderr=subprocess.DEVNULL)


def main():
    rclpy.init()
    node = Node('estop')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    print('\n  ================ E-STOP ================')
    print('  Press Enter to stop the robot.  Ctrl+C closes this window.\n')
    try:
        while True:
            sys.stdin.readline()
            t0 = time.time()
            for p in KILL_NOW:
                pkill('-KILL', p)
            zero = Twist()
            end = time.time() + 2.0
            while time.time() < end:
                pub.publish(zero)
                time.sleep(0.05)
            print(f'  STOPPED at {time.strftime("%H:%M:%S")}'
                  f' (wheels cut {time.time() - t0 - 2.0:.2f} s after the key)')
            for p in SHUT_DOWN:
                pkill('-INT', p)
            time.sleep(3)
            for p in SHUT_DOWN:
                pkill('-TERM', p)
            time.sleep(2)
            for p in SHUT_DOWN:
                pkill('-KILL', p)
            for _ in range(10):
                pub.publish(zero)
                time.sleep(0.1)
            print('  race, bag and video recorders shut down.  Enter stops again.\n')
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
