#!/usr/bin/env python3
"""render_assembly.py OUT.png : Burger + C920 + E1 camera mount, side view and 3/4 view.

Robot meshes from turtlebot3_description (placed as in the sim model), the mount from
export/E1_camera_mount_robot.stl (robot frame, made by gen_camera_mount.py), the C920 as
simple blocks at the designed lens pose. The floor shows the lane lines (yellow left,
white right, 0.25 m apart) and the patch of floor the camera sees.
"""
import math
import os
import struct
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MESH = '/opt/ros/jazzy/share/turtlebot3_description/meshes/'


def load_stl(path):
    data = open(path, 'rb').read()
    if data[:5] == b'solid' and b'facet' in data[:300]:
        tris = [list(map(float, l.split()[1:])) for l in data.decode(errors='ignore').splitlines()
                if l.strip().startswith('vertex')]
        return np.array(tris, np.float64).reshape(-1, 3, 3)
    n = struct.unpack('<I', data[80:84])[0]
    a = np.frombuffer(data[84:84 + n * 50], dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    return a['v'].astype(np.float64).reshape(-1, 3, 3)


def decimate(tris, keep):
    if len(tris) <= keep:
        return tris
    area = np.linalg.norm(np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]), axis=1)
    return tris[np.argsort(-area)[:keep]]


def box(lo, hi):
    x0, y0, z0 = lo
    x1, y1, z1 = hi
    c = np.array([[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
                  [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]])
    f = [(0, 1, 2), (0, 2, 3), (4, 6, 5), (4, 7, 6), (0, 4, 5), (0, 5, 1),
         (1, 5, 6), (1, 6, 2), (2, 6, 7), (2, 7, 3), (3, 7, 4), (3, 4, 0)]
    return c[np.array(f)]


def geometry():
    g = {}
    for line in open(os.path.join(HERE, 'export', 'geometry.txt')):
        if line.startswith('lens at'):
            p = line.split()
            g['x'], g['z'], g['pitch'] = float(p[3]), float(p[9]), float(p[12])
    return g


def c920(lens_x, lens_z, pitch_deg, cam_up=42.0, cam_fwd=8.0):
    """C920 as blocks in its own frame (f forward, y left, u up from the tripod socket)."""
    parts = [box((-15, -12, 0), (15, 12, 7)),          # clip foot
             box((-9, -9, 7), (3, 9, 28)),             # clip / hinge
             box((-6, -47, 28), (18, 47, 56)),         # head
             box((18, -6, 36), (21, 6, 48))]           # lens bezel
    th = math.radians(pitch_deg)
    f = np.array([math.cos(th), 0, -math.sin(th)])
    u = np.array([math.sin(th), 0, math.cos(th)])
    y = np.array([0, 1.0, 0])
    socket = np.array([lens_x, 0, lens_z]) - cam_fwd * f - cam_up * u
    out = []
    for p in parts:
        out.append(socket + p[..., 0:1] * f + p[..., 1:2] * y + p[..., 2:3] * u)
    return np.concatenate(out)


def footprint(lens_x, lens_z, pitch_deg, hfov=70.4, vfov=43.3):
    th = math.radians(pitch_deg)
    pts = []
    for sv, sh in ((-1, -1), (-1, 1), (1, 1), (1, -1)):
        a = th + sv * math.radians(vfov / 2)                 # below horizontal
        ahead = lens_z / math.tan(a)
        slant = math.hypot(ahead, lens_z)
        off = math.radians(vfov / 2) * sv
        lat = slant * math.tan(math.radians(hfov / 2)) / math.cos(off) * sh
        pts.append((lens_x + ahead, lat, 0.5))
    return np.array(pts)


def shade(tris, rgb, light=np.array([0.4, -0.6, 0.7])):
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-9
    k = 0.45 + 0.55 * np.abs(n @ (light / np.linalg.norm(light)))
    return np.clip(np.array(rgb)[None, :] * k[:, None], 0, 1)


def scene():
    g = geometry()
    items = []
    base = load_stl(MESH + 'bases/burger_base.stl') + np.array([-32, 0, 10])
    items.append((decimate(base, 25000), (0.30, 0.30, 0.33), 'TurtleBot3 Burger'))
    lds = load_stl(MESH + 'sensors/lds.stl') + np.array([-32, 0, 181])
    items.append((decimate(lds, 6000), (0.15, 0.15, 0.17), 'lidar'))
    for side, name in ((80, 'left_tire'), (-80, 'right_tire')):
        t = load_stl(MESH + f'wheels/{name}.stl') + np.array([0, side, 33])
        items.append((decimate(t, 4000), (0.1, 0.1, 0.1), None))
    mount = load_stl(os.path.join(HERE, 'export', 'E1_camera_mount_robot.stl'))
    mount = mount - np.array([0, 11.0, 0])      # the part is 22 wide from y = 0: centre it
    items.append((mount, (0.95, 0.45, 0.10), 'E1 mount (print)'))
    items.append((c920(g['x'], g['z'], g['pitch']), (0.12, 0.12, 0.12), 'C920'))
    return g, items


def draw(ax, g, items, elev, azim, title):
    floor = [[(-160, -260, 0), (480, -260, 0), (480, 260, 0), (-160, 260, 0)]]
    ax.add_collection3d(Poly3DCollection(floor, facecolors=(0.55, 0.55, 0.57), alpha=0.35))
    for y, colour in ((125, (1.0, 0.82, 0.1)), (-125, (1.0, 1.0, 1.0))):
        ax.add_collection3d(Poly3DCollection([[(-160, y - 20, 0.3), (480, y - 20, 0.3),
                                               (480, y + 20, 0.3), (-160, y + 20, 0.3)]],
                                             facecolors=colour, edgecolors=(0.4, 0.4, 0.4), linewidths=0.3))
    fp = footprint(g['x'], g['z'], g['pitch'])
    ax.add_collection3d(Poly3DCollection([fp], facecolors=(0.1, 0.8, 0.3), alpha=0.35,
                                         edgecolors=(0.0, 0.5, 0.1)))
    lens = np.array([g['x'], 0, g['z']])
    for p in fp:
        ax.plot(*zip(lens, p), color=(0.0, 0.55, 0.15), lw=0.6, alpha=0.7)
    for tris, rgb, _ in items:
        ax.add_collection3d(Poly3DCollection(tris, facecolors=shade(tris, rgb), edgecolors='none'))
    ax.set_xlim(-150, 480)
    ax.set_ylim(-300, 300)
    ax.set_zlim(0, 300)
    ax.set_box_aspect((630, 600, 300))
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()
    ax.set_title(title, fontsize=11)


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, 'export', 'assembly.png')
    g, items = scene()
    fig = plt.figure(figsize=(16, 7.5), dpi=110)
    a1 = fig.add_subplot(1, 2, 1, projection='3d')
    draw(a1, g, items, 0, -90, 'side (front to the right)')
    a1.set_box_aspect((630, 600, 300), zoom=1.35)
    a2 = fig.add_subplot(1, 2, 2, projection='3d')
    draw(a2, g, items, 24, -125, '3/4 view from the rear left')
    a2.set_box_aspect((630, 600, 300), zoom=1.25)
    fig.text(0.5, 0.03,
             f"lens {g['x']:+.0f} mm ahead of the axle, {g['z']:.0f} mm up, {g['pitch']:.0f} deg down.  "
             "green: floor in view.  yellow = left line, white = right line (0.25 m apart).  "
             "orange: printed mount.  lidar mesh is the sim LDS-01; the real one is smaller.",
             ha='center', fontsize=10)
    plt.subplots_adjust(left=0, right=1, top=0.95, bottom=0.06, wspace=0)
    fig.savefig(out)
    print('wrote', out)


if __name__ == '__main__':
    main()
