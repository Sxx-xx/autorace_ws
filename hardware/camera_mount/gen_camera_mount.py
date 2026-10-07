#!/usr/bin/env freecadcmd
# -*- coding: utf-8 -*-
"""C920 차선 카메라 거치대 — 라이다 뒤 기둥 + 라이다 위로 뻗은 팔.

실행:  cd hardware/camera_mount && freecadcmd gen_camera_mount.py
결과:  export/E1_camera_mount.stl (출력 자세: 옆면이 바닥, 서포트 없음),
       export/E1_camera_mount.step, export/geometry.txt (렌즈 위치·각도·시야 점검)

왜 이 모양인가: 카메라를 로봇 뒤에 낮게 달면 윗판 앞 모서리가 가까운 바닥을
가린다 (0.10 m 앞 바닥을 보려면 렌즈가 0.48 m 높이여야 한다). 그래서 기둥만
라이다 뒤에 세우고, 팔을 라이다 위로 뻗어 렌즈를 바퀴 축 바로 앞 높은 곳에 둔다.
라이다는 자기 높이의 수평면만 재므로 그 위를 지나는 팔은 가리지 않는다. 기둥은
라이다 바로 뒤 몇 도만 가리는데, 미션은 라이다의 앞과 옆만 쓴다 (ld_lidar 의
range_min 을 0.08 로 올리면 기둥이 아예 안 잡힌다).

좌표: mm, 로봇 기준. x 는 바퀴 축에서 앞으로, z 는 바닥에서 위로.
MEASURE 표시는 실제 로봇에서 재서 고칠 값이다 (지금 값은 Burger 추정치).
"""
import math
import os

import FreeCAD as App
import Mesh
import Part

V = App.Vector
HERE = os.environ.get('MOUNT_DIR') or os.getcwd()
OUT = os.path.join(HERE, 'export')

# ---------------------------------------------------------------- 로봇 (MEASURE)
PLATE_Z = 145.0        # 맨 윗판 윗면 높이
PLATE_REAR_X = -100.0  # 윗판 뒤 모서리 (축 기준, 뒤는 음수)
PLATE_FRONT_X = 40.0   # 윗판 앞 모서리 (시야를 가리는지 점검용)
LIDAR_X = -32.0        # 라이다 중심
LIDAR_R = 25.0         # 라이다 반지름 + 여유
LIDAR_TOP_Z = 190.0    # 라이다 맨 윗면 높이

# ---------------------------------------------------------------- 카메라 (C920 추정)
CAM_UP = 42.0          # 받침(삼각대 나사) 바닥면에서 렌즈 중심까지, 카메라 위쪽 방향
CAM_FWD = 8.0          # 나사 구멍에서 렌즈까지, 카메라 앞쪽 방향
CAM_VFOV = 43.3        # 16:9 모드 세로 화각 (도)
CAM_HFOV = 70.4

# ---------------------------------------------------------------- 정할 값
LENS_X = 20.0          # 렌즈를 둘 앞뒤 위치 (축 앞 2 cm)
PITCH = 51.0           # 카메라를 숙이는 각 (도): 바닥 0.10~0.43 m 가 화면에 들어온다

# ---------------------------------------------------------------- 부품 치수
W = 22.0               # 부품 폭 (로봇 좌우 방향 = 출력할 때 높이)
WALL = 6.0
FOOT_GAP = 3.0         # 발 앞 끝과 라이다 사이 틈
FOOT_T = 5.0           # 발 두께
POST_T = 14.0          # 기둥 두께 (앞뒤)
ARM_T = 12.0           # 팔 두께 (위아래)
ARM_CLEAR = 12.0       # 라이다 윗면과 팔 사이 틈
GUSSET = 20.0
ACCESS_W = 7.0         # 팔을 뚫는 드라이버 구멍 폭 (발의 M3 볼트를 위에서 조인다)
PAD_L = 34.0           # 카메라 받침면 길이
SLOT_W = 3.4           # M3 통과
BOLT_D = 6.6           # 1/4-20 통과
HEAD_D = 13.0          # 볼트 머리 자리
SEAT = 8.0             # 카메라 아래 남는 살 두께 → 1/4-20 x 1/2" (13 mm) 볼트


def prism_xz(points):
    """XZ 평면 다각형을 y 로 W 만큼 밀어낸 입체."""
    pts = [V(x, 0, z) for x, z in points] + [V(points[0][0], 0, points[0][1])]
    face = Part.Face(Part.makePolygon(pts))
    return face.extrude(V(0, W, 0))


def cyl_along(d, length, start_xz, dir_xz):
    dx, dz = dir_xz
    return Part.makeCylinder(d / 2.0, length, V(start_xz[0], W / 2.0, start_xz[1]), V(dx, 0, dz))


def main():
    os.makedirs(OUT, exist_ok=True)
    th = math.radians(PITCH)
    fwd = (math.cos(th), -math.sin(th))     # 카메라 앞
    up = (math.sin(th), math.cos(th))       # 카메라 위 = 받침면의 법선

    x0 = PLATE_REAR_X + 2.0                 # 발 뒤 끝
    arm_bot = LIDAR_TOP_Z + ARM_CLEAR
    arm_top = arm_bot + ARM_T

    # 받침면 위 나사 위치 S: 렌즈가 LENS_X 에 오도록, 받침 앞 끝이 팔 윗면과 같은 높이.
    half = PAD_L / 2.0
    s_x = LENS_X - CAM_FWD * fwd[0] - CAM_UP * up[0]
    s_z = arm_top + half * math.sin(th)
    p1 = (s_x - half * fwd[0], s_z - half * fwd[1])   # 받침 뒤 끝 (높은 쪽)
    p2 = (s_x + half * fwd[0], s_z + half * fwd[1])   # 받침 앞 끝 (낮은 쪽)
    lens = (s_x + CAM_FWD * fwd[0] + CAM_UP * up[0], s_z + CAM_FWD * fwd[1] + CAM_UP * up[1])

    post_front = x0 + POST_T
    foot_end = LIDAR_X - LIDAR_R - FOOT_GAP
    foot = prism_xz([(x0, PLATE_Z), (foot_end, PLATE_Z),
                     (foot_end, PLATE_Z + FOOT_T), (x0, PLATE_Z + FOOT_T)])
    post = prism_xz([(x0, PLATE_Z), (post_front, PLATE_Z), (post_front, arm_top), (x0, arm_top)])
    arm = prism_xz([(x0, arm_bot), (p2[0], arm_bot), (p2[0], arm_top), (x0, arm_top)])
    gusset_high = prism_xz([(post_front, arm_bot), (post_front + GUSSET, arm_bot),
                            (post_front, arm_bot - GUSSET)])
    pad = prism_xz([(p1[0], arm_bot), (p2[0], arm_bot), p2, p1])
    body = foot.fuse([post, arm, gusset_high, pad]).removeSplitter()

    # 발의 M3 홈 (위아래로 관통): 홈 길이 방향은 앞뒤. 기둥 앞에서 발 끝까지.
    slot_x0 = post_front + 2.0
    SLOT_L = foot_end - 2.5 - slot_x0
    slot = Part.makeBox(SLOT_L - SLOT_W, SLOT_W, FOOT_T + 2,
                        V(slot_x0 + SLOT_W / 2, W / 2 - SLOT_W / 2, PLATE_Z - 1))
    slot = slot.fuse([Part.makeCylinder(SLOT_W / 2, FOOT_T + 2, V(slot_x0 + SLOT_W / 2, W / 2, PLATE_Z - 1)),
                      Part.makeCylinder(SLOT_W / 2, FOOT_T + 2, V(slot_x0 + SLOT_L - SLOT_W / 2, W / 2, PLATE_Z - 1))])
    # 1/4-20 볼트: 받침면에서 아래로 (−up), 그 아래는 머리 자리.
    down = (-up[0], -up[1])
    bolt = cyl_along(BOLT_D, 80.0, (s_x + 1.0 * up[0], s_z + 1.0 * up[1]), down)
    head = cyl_along(HEAD_D, 80.0, (s_x + SEAT * down[0], s_z + SEAT * down[1]), down)
    # 홈 바로 위로 팔(과 버팀)을 뚫어 드라이버가 들어가게 한다.
    access = Part.makeBox(SLOT_L, ACCESS_W, arm_top - PLATE_Z + 10,
                          V(slot_x0, W / 2 - ACCESS_W / 2, PLATE_Z + FOOT_T))
    body = body.cut(slot.fuse([bolt, head, access]))

    # 점검: 시야, 라이다, 윗판 모서리
    lines = []
    lines.append(f'lens at x {lens[0]:+.1f} mm (from the axle), z {lens[1]:.1f} mm, pitch {PITCH:.1f} deg')
    def ground(angle_deg):
        a = math.radians(angle_deg)
        return lens[0] + lens[1] / math.tan(a) if a > 0 else float('inf')
    near, far = ground(PITCH + CAM_VFOV / 2), ground(PITCH - CAM_VFOV / 2)
    lines.append(f'floor in view: {near / 1000:.3f} .. {far / 1000:.3f} m from the axle')
    def half_width(x):
        dist = x - lens[0]
        slant = math.hypot(dist, lens[1])
        off = math.atan2(lens[1], dist) - th
        return slant * math.tan(math.radians(CAM_HFOV / 2)) / math.cos(off)
    for x in (100.0, 150.0, 200.0):
        lines.append(f'half width at {x / 1000:.2f} m: {half_width(x) / 1000:.3f} m (both lines need 0.145)')
    # 0.10 m 앞 바닥으로 가는 광선이 윗판 앞 모서리 위를 지나는가
    target = 100.0
    if PLATE_FRONT_X > lens[0]:
        z_at_edge = lens[1] * (target - PLATE_FRONT_X) / (target - lens[0])
        lines.append(f'ray to 0.10 m passes the plate front edge at z {z_at_edge:.0f} mm '
                     f'(edge {PLATE_Z:.0f}): {"clear" if z_at_edge > PLATE_Z + 3 else "BLOCKED"}')
    lidar_front = LIDAR_X + LIDAR_R
    lines.append(f'lidar front edge x {lidar_front:+.0f}, lens x {lens[0]:+.0f}: '
                 f'{"lidar behind the lens, out of the picture" if lens[0] > lidar_front else "LIDAR MAY BE IN VIEW"}')
    lines.append(f'arm bottom z {arm_bot:.0f} over lidar top {LIDAR_TOP_Z:.0f}; '
                 f'post {x0:.0f}..{post_front:.0f} vs lidar rear edge {LIDAR_X - LIDAR_R:.0f}: '
                 f'{"clear" if post_front + GUSSET < LIDAR_X - LIDAR_R else "GUSSET TOUCHES LIDAR"}')
    lines.append(f'foot slot: x {slot_x0:.0f} .. {slot_x0 + SLOT_L:.0f} ({SLOT_L:.0f} mm long, M3), '
                 f'screwdriver hole through the arm above it')
    lines.append(f'part size: {body.BoundBox.XLength:.0f} x {body.BoundBox.YLength:.0f} x '
                 f'{body.BoundBox.ZLength:.0f} mm, volume {body.Volume / 1000:.1f} cm3 '
                 f'(~{body.Volume * 1.24e-3:.0f} g PLA solid)')
    lines.append('camera_lane yaml: height %.3f, pitch %.2f rad, forward_offset %.3f'
                 % (lens[1] / 1000, th, lens[0] / 1000))
    text = '\n'.join(lines)
    print(text)
    open(os.path.join(OUT, 'geometry.txt'), 'w').write(text + '\n')

    # 로봇 좌표 그대로 STEP, 출력 자세(옆면이 바닥)로 STL
    body.exportStep(os.path.join(OUT, 'E1_camera_mount.step'))
    # 로봇 좌표 그대로의 메쉬: render_assembly.py 가 조립 그림에 쓴다 (출력용 아님)
    Mesh.Mesh(body.tessellate(0.1)).write(os.path.join(OUT, 'E1_camera_mount_robot.stl'))
    printed = body.copy()
    printed.rotate(V(0, 0, 0), V(1, 0, 0), 90)
    bb = printed.BoundBox
    printed.translate(V(-bb.XMin, -bb.YMin, -bb.ZMin))
    mesh = Mesh.Mesh(printed.tessellate(0.05))
    mesh.write(os.path.join(OUT, 'E1_camera_mount.stl'))
    print('wrote', os.path.join(OUT, 'E1_camera_mount.stl'))


main()
