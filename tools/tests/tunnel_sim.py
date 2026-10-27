"""Tunnel test without the robot: tunnel_test_mission drives a simulated Burger through a drawn tunnel.

The tunnel as tunnel_mission has it, sized by the node's goal, in the start's frame (x into the
tunnel, y to the left, the entrance in the entry wall x = 0): the entrance 0.18 m from the right
wall, the exit centred on the goal in the left wall, 0.18 m from the far wall, both 0.34 m wide
(goal 1.56, 1.56: the 1.74 m tunnel; 1.8, 1.8: 1.98 m). Square obstacles (side 0.15 m) stand
where a scenario puts them. The laser is cast against the walls and the obstacles (360 beams, 8 Hz,
1 cm noise); the node's Twist moves the robot. Reports whether it reached the goal, how long it took,
how far it drove against the straight line, and how close the axle came to anything.
  python3 tools/tests/tunnel_sim.py [scenario ...] [--png DIR]      scenarios: see SCENARIOS
Scenarios named enter_* start outside, askew, with start_on: tunnel: a stand-in for the lane
follower drives the robot (0.12 m/s, back to the lane's middle) up to the tunnel's front wall,
which hands it to the node; the camera goes dark once it is under the roof (5 cm ahead of the
axle). enter_askew: the lane runs 6 cm left of the entrance's middle and 7 deg off square.
"""
import math
import os
import sys
import time

import cv2
import numpy as np
from nav_msgs.msg import Odometry
import rclpy
from sensor_msgs.msg import LaserScan
from rclpy.parameter import Parameter
from std_msgs.msg import Bool

from autorace_core.tunnel_test_mission import TunnelTestMission

RIGHT, OPEN, BOX = 0.18, 0.34, 0.15
GOAL = (1.8, 1.8)                  # set from the node in main()
START = (0.0, 0.0, 0.0)           # the axle in the doorway, pointing in (the goal is from here)
OUTSIDE = (-0.90, 0.03, 0.06)     # enter_*: 90 cm short of the doorway, 3 cm left, 3.4 deg askew
LANES = {'enter_askew': (0.06, 0.12)}   # the lane: y at the doorway, angle off square (rad)
SCENARIOS = {
    'empty': [],
    # Shares of the way along the diagonal (scaled to the goal in main()).
    'middle': [(0.50, 0.50)],                       # on the diagonal, half way
    'offset': [(0.48, 0.43)],                       # just off the diagonal, to its right
    'two': [(0.35, 0.35), (0.70, 0.70)],            # two on the diagonal
    'wall': [(0.50, 0.50), (0.50, 0.70), (0.50, 0.90)],  # a row towards the left wall: round the right
    'near_goal': [(0.80, 0.80)],
    'enter_empty': [],
    'enter_middle': [(0.50, 0.50)],
    'enter_askew': [],
}


def walls():
    """Wall segments ((x0, y0), (x1, y1)) with the two openings left out."""
    left, far = GOAL[1], GOAL[0] + RIGHT
    exit_x = GOAL[0]
    h = OPEN / 2
    return [((0.0, -RIGHT), (0.0, -h)), ((0.0, h), (0.0, left)),            # entry wall
            ((0.0, -RIGHT), (far, -RIGHT)),                                 # right wall
            ((far, -RIGHT), (far, left)),                                   # far wall
            ((0.0, left), (exit_x - h, left)), ((exit_x + h, left), (far, left))]  # left wall


def segments(boxes):
    out = list(walls())
    for sx, sy in boxes:
        cx, cy = sx * GOAL[0], sy * GOAL[1]
        h = BOX / 2
        c = [(cx - h, cy - h), (cx + h, cy - h), (cx + h, cy + h), (cx - h, cy + h)]
        out += [(c[i], c[(i + 1) % 4]) for i in range(4)]
    return np.array(out, float)


def cast(segs, x, y, yaw, n=360, max_range=8.0):
    """Ranges from the laser (3.2 cm behind the axle) at n angles, like LaserScan.ranges."""
    lx, ly = x - 0.032 * math.cos(yaw), y - 0.032 * math.sin(yaw)
    angles = np.arange(n) * 2 * math.pi / n
    d = np.stack([np.cos(yaw + angles), np.sin(yaw + angles)], 1)            # n x 2
    a, b = segs[:, 0], segs[:, 1]                                             # m x 2
    e = b - a
    denom = d[:, None, 0] * e[None, :, 1] - d[:, None, 1] * e[None, :, 0]     # n x m
    w = a[None] - np.array([lx, ly])
    with np.errstate(divide='ignore', invalid='ignore'):
        t = (w[..., 0] * e[None, :, 1] - w[..., 1] * e[None, :, 0]) / denom
        u = (w[..., 0] * d[:, None, 1] - w[..., 1] * d[:, None, 0]) / denom
    t = np.where((denom != 0) & (t > 0) & (u >= 0) & (u <= 1), t, np.inf)
    r = t.min(axis=1)
    r = np.where(r < max_range, r + np.random.default_rng().normal(0, 0.01, n), np.inf)
    return r.astype(np.float32)


def distance_to(segs, x, y):
    p = np.array([x, y])
    a, b = segs[:, 0], segs[:, 1]
    t = np.clip(np.einsum('ij,ij->i', p - a, b - a) / np.einsum('ij,ij->i', b - a, b - a), 0, 1)
    return float(np.min(np.linalg.norm(a + (b - a) * t[:, None] - p, axis=1)))


def run(node, name, boxes, png_dir=None):
    segs = segments(boxes)
    lane_y, lane_a = LANES.get(name, (0.0, 0.0))
    obstacles = segments(boxes)[len(walls()):] if boxes else None
    cmd = [0.0, 0.0]
    node.pub_cmd_vel.publish = lambda m: cmd.__setitem__(slice(0, 2), [m.linear.x, m.angular.z])
    for pub in (node.pub_path, node.pub_route, node.pub_goal, node.pub_grid):
        pub.publish = lambda m: None
    x, y, yaw = OUTSIDE if name.startswith('enter') else START

    def odom():
        o = Odometry()
        o.pose.pose.position.x, o.pose.pose.position.y = x, y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        node.callback_odom(o)

    def camera():
        # Dark under the roof: the camera (5 cm ahead of the axle) inside the walls.
        cx, cy = x + 0.05 * math.cos(yaw), y + 0.05 * math.sin(yaw)
        node.brightness = 15.0 if 0.0 < cx < GOAL[0] + RIGHT and -RIGHT < cy < GOAL[1] else 150.0

    def scan():
        s = LaserScan()
        s.angle_min, s.angle_increment, s.range_min, s.range_max = 0.0, 2 * math.pi / 360, 0.1, 8.0
        s.ranges = cast(segs, x, y, yaw).tolist()
        node.callback_scan(s)

    odom()
    node.callback_run_active(Bool(data=True))
    scan()
    t0 = time.monotonic(); prev = t0; last_scan = last_ctl = t0
    trail, closest_wall, closest_box = [], math.inf, math.inf
    aligned = None                 # the heading when leave began
    while node.state != 'done' and time.monotonic() - t0 < 60:
        now = time.monotonic()
        dt = now - prev; prev = now
        v, w = cmd
        if node.state == 'armed':
            # The lane follower's stand-in: on along the lane (y = 0), back to its middle.
            off = (y - lane_y - math.tan(lane_a) * x) * math.cos(lane_a)
            v, w = 0.12, -3.0 * off - 1.5 * math.atan2(math.sin(yaw - lane_a), math.cos(yaw - lane_a))
        x += v * math.cos(yaw) * dt; y += v * math.sin(yaw) * dt; yaw += w * dt
        odom()
        if now - last_scan >= 1 / 8:
            last_scan = now
            camera()
            scan()
        if now - last_ctl >= 0.05:
            last_ctl = now
            node.update()
        trail.append((x, y))
        if aligned is None and node.state == 'leave':
            aligned = yaw
        closest_wall = min(closest_wall, distance_to(segments([]), x, y))
        if obstacles is not None:
            closest_box = min(closest_box, distance_to(obstacles, x, y))
        time.sleep(max(0.0, 1 / 100 - (time.monotonic() - now)))
    tr = np.array(trail)
    length = float(np.sum(np.linalg.norm(np.diff(tr, axis=0), axis=1)))
    straight = float(np.hypot(node.goal[0] - trail[0][0], node.goal[1] - trail[0][1]))
    ok = node.state == 'done'
    print(f'{name:10} {"REACHED" if ok else "NOT REACHED (" + node.state + ")"} in {time.monotonic() - t0:4.1f} s;'
          f' drove {length:.2f} m (straight {straight:.2f});'
          f' axle to obstacles min {closest_box * 100:4.1f} cm, to walls {closest_wall * 100:4.1f} cm;'
          f' exit: aligned {"-" if aligned is None else f"{math.degrees(aligned - math.pi / 2):+.1f}"} deg off parallel,'
          f' out {(y - GOAL[1]) * 100:.0f} cm past the left wall at {math.degrees(yaw - math.pi / 2):+.1f} deg')
    if png_dir:
        k = 200
        size = max(GOAL) + 0.6
        img = np.full((int(size * k), int(size * k), 3), 60, np.uint8)
        to = lambda p: (int((p[0] + 0.3) * k), int((size - 0.3 - p[1]) * k))
        for a, b in segs:
            cv2.line(img, to(a), to(b), (230, 230, 230), 2)
        cv2.line(img, to(START), to(node.goal), (0, 200, 255), 1)
        cv2.polylines(img, [np.array([to(p) for p in tr], np.int32)], False, (0, 0, 255), 2)
        cv2.imwrite(f'{png_dir}/tunnel_{name}.png', img)
    return ok


def main():
    args = sys.argv[1:]
    png_dir = None
    if '--png' in args:
        i = args.index('--png'); png_dir = args[i + 1]; del args[i:i + 2]
    names = args or list(SCENARIOS)
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=false'])
    global GOAL
    probe = TunnelTestMission(); GOAL = tuple(probe.goal); probe.destroy_node()
    # TUNNEL=1.56 draws the tunnel for that exit instead of the node's goal (a goal that does not
    # fit the tunnel: 1.8 m against the 1.74 m tunnel of mission_real.yaml).
    if os.environ.get('TUNNEL'):
        GOAL = (float(os.environ['TUNNEL']),) * 2
    results = []
    for name in names:
        overrides = [Parameter('start_on', value='tunnel')]
        node = TunnelTestMission(parameter_overrides=overrides if name.startswith('enter') else [])
        results.append(run(node, name, SCENARIOS[name], png_dir))
        node.destroy_node()
    rclpy.shutdown()
    print(f'{sum(results)}/{len(results)} reached the goal')


if __name__ == '__main__':
    main()
