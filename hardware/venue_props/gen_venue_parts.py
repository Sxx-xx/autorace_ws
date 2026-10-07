#!/usr/bin/env freecadcmd
# -*- coding: utf-8 -*-
"""오토레이스 연습 경기장 소품 — 3D 프린팅용 부품 생성기.

실행:  cd hardware/venue_props && freecadcmd gen_venue_parts.py
결과:  export/stl/*.stl, export/step/*.step, export/venue_props.FCStd, export/parts.md

치수는 모두 mm. 시뮬레이션(src/autorace_sim)의 모델 치수를 따르고, 아두이노가
붙는 신호등·차단바는 배선 경로(기둥 속 통로, 발판 밑 케이블 홈, 뒷판 케이블
구멍, 컨트롤러 박스)를 넣어 두었다. 각 부품은 출력 자세 그대로 내보낸다
(바닥에 닿는 면이 z = 0).

부품 번호는 README.md의 표와 같다.
"""

import math
import os
import sys

import FreeCAD as App
import Mesh
import MeshPart
import Part

V = App.Vector

HERE = os.environ.get('VENUE_DIR') or os.getcwd()
OUT = os.path.join(HERE, 'export')

# ---------------------------------------------------------------- 공통 치수
CLEAR = 0.3          # 끼워 맞춤 한쪽 틈새
PIN_D = 2.9          # 3 mm 필라멘트 토막/M3 셀프탭 핀 구멍
M3_FREE = 3.4        # M3 통과 구멍
M3_TAP = 2.6         # M3 셀프탭
M2_TAP = 1.8         # M2 셀프탭 (서보 탭, 혼)
DENSITY = 1.24e-3    # PLA g/mm^3 (100 % 채움 기준)

PARTS = []           # (번호, 이름, 수량, 설명, shape)


# ---------------------------------------------------------------- 도형 도우미
def box(lx, ly, lz, x=0.0, y=0.0, z=0.0, cx=False, cy=False):
    if cx:
        x -= lx / 2.0
    if cy:
        y -= ly / 2.0
    return Part.makeBox(lx, ly, lz, V(x, y, z))


AXES = {'x': V(1, 0, 0), 'y': V(0, 1, 0), 'z': V(0, 0, 1)}


def cyl(r, h, x=0.0, y=0.0, z=0.0, axis='z'):
    return Part.makeCylinder(r, h, V(x, y, z), AXES[axis])


def fuse(first, *rest):
    s = first
    for r in rest:
        s = s.fuse(r)
    return s.removeSplitter()


def cut(base, *tools):
    s = base
    for t in tools:
        s = s.cut(t)
    return s.removeSplitter()


def prism(points, h, z=0.0):
    """xy 다각형을 z 방향으로 h만큼 뽑는다."""
    pts = [V(p[0], p[1], z) for p in points] + [V(points[0][0], points[0][1], z)]
    return Part.Face(Part.makePolygon(pts)).extrude(V(0, 0, h))


def octagon(across_flats, h, z=0.0):
    r = across_flats / 2.0 / math.cos(math.pi / 8)
    pts = [(r * math.cos(math.pi / 8 + i * math.pi / 4),
            r * math.sin(math.pi / 8 + i * math.pi / 4)) for i in range(8)]
    return prism(pts, h, z)


def rect_tube(lx, ly, lz, wall, x=0.0, y=0.0, z=0.0, cx=True, cy=True,
              floor=0.0, roof=0.0):
    """직사각 튜브. floor/roof > 0 이면 그 두께로 바닥/천장을 막는다."""
    outer = box(lx, ly, lz, x, y, z, cx, cy)
    ix = x if cx else x + lx / 2.0
    iy = y if cy else y + ly / 2.0
    inner = box(lx - 2 * wall, ly - 2 * wall, lz - floor - roof,
                ix, iy, z + floor, True, True)
    return outer.cut(inner)


def add(number, name, qty, desc, shape):
    shape = shape.removeSplitter()
    if not shape.isValid():
        shape.fix(0.01, 0.01, 0.01)
    PARTS.append((number, name, qty, desc, shape))
    return shape


# ================================================================ A. 신호등
# 시뮬: 하우징 90 × 30 × 260, 램프 소켓 지름 52, 간격 75, 램프면이 -y.
# 실물은 램프 하나짜리 모듈 3개를 핀으로 쌓는다 (어느 프린터에나 들어가고,
# 모듈 사이로 배선이 지난다). 모듈 좌표: x ±45, y 0(앞면)…40(뒷면 개방), z 0…75.
TL_W, TL_D, TL_H = 90.0, 40.0, 75.0
TL_WALL = 2.4
TL_LAMP_D = 52.0
TL_BOSS = 6.0
TL_BOSS_XY = [(sx * (TL_W / 2 - TL_WALL - TL_BOSS / 2), y)
              for sx in (-1, 1)
              for y in (TL_WALL + TL_BOSS / 2, TL_D - TL_WALL - TL_BOSS / 2)]
TL_COVER_Z = (12.0, TL_H - 12.0)
TL_HOOD_R = 30.0


def tl_bosses(h, z=0.0):
    return fuse(*[box(TL_BOSS, TL_BOSS, h, bx, by, z, True, True)
                  for bx, by in TL_BOSS_XY])


def tl_pin_holes(z_bottom, z_top, depth=12.0):
    holes = []
    for bx, by in TL_BOSS_XY:
        holes.append(cyl(PIN_D / 2, depth, bx, by, z_bottom))
        holes.append(cyl(PIN_D / 2, depth, bx, by, z_top - depth))
    return holes


def tl_module():
    """램프 모듈. 앞면을 바닥에 두고 출력한다 (뒷면 개방, 서포트 불필요).
    차양은 별도 부품(tl_hood)을 앞면의 얕은 홈에 맞춰 붙인다."""
    body = box(TL_W, TL_D, TL_H, 0, 0, 0, cx=True)
    cavity = box(TL_W - 2 * TL_WALL, TL_D - TL_WALL + 1, TL_H - 2 * TL_WALL,
                 0, TL_WALL, TL_WALL, cx=True)          # 뒷면(y=40) 개방
    body = body.cut(cavity)
    # 위아래 벽의 배선 통로
    for z in (-1, TL_H - TL_WALL - 1):
        body = body.cut(box(50, 26, TL_WALL + 2, 0, 7, z, cx=True))
    body = body.fuse(tl_bosses(TL_H))
    lamp_z = TL_H / 2
    tools = [cyl(TL_LAMP_D / 2, TL_WALL + 2, 0, -1, lamp_z, axis='y')]
    # 차양 자리 홈: 반원 고리, 깊이 0.6
    groove = cyl(TL_HOOD_R + 0.3, 1.6, 0, -1, lamp_z, axis='y').cut(
        cyl(TL_HOOD_R - 1.6 - 0.3, 2, 0, -1.2, lamp_z, axis='y'))
    groove = groove.cut(box(70, 10, 40, 0, -5, lamp_z - 40, cx=True))
    tools.append(groove)
    tools += tl_pin_holes(0.0, TL_H)
    for bx, by in TL_BOSS_XY:
        if by > TL_D / 2:
            for z in TL_COVER_Z:
                tools.append(cyl(M3_TAP / 2, 9.0, bx, TL_D + 0.5, z, axis='y')
                             .rotate(V(bx, TL_D + 0.5, z), V(0, 0, 1), 180))
    body = cut(body, *tools)
    # 출력 자세: 앞면(y=0)을 바닥으로
    body.rotate(V(0, 0, 0), V(1, 0, 0), 90)
    bb = body.BoundBox
    body.translate(V(0, -bb.YMin, -bb.ZMin))
    return body


def tl_hood():
    """차양: 램프 위 반원 덮개, 앞으로 18 mm. 서서 출력(C자 벽)."""
    hood = cyl(TL_HOOD_R, 18.0).cut(cyl(TL_HOOD_R - 1.6, 20, 0, 0, -1))
    hood = hood.cut(box(70, 40, 20, 0, -40, -1, cx=True))
    # 홈에 들어가는 0.6 mm 턱
    lip = cyl(TL_HOOD_R, 0.6, 0, 0, -0.6).cut(cyl(TL_HOOD_R - 1.6, 1, 0, 0, -1))
    lip = lip.cut(box(70, 40, 2, 0, -40, -1, cx=True))
    hood = fuse(hood, lip)
    hood.translate(V(0, 0, 0.6))
    return hood


def tl_module_cover():
    plate = box(TL_W, TL_H, 2.0, 0, 0, 0, cx=True)
    tools = []
    for bx, by in TL_BOSS_XY:
        if by > TL_D / 2:
            for z in TL_COVER_Z:
                tools.append(cyl(M3_FREE / 2, 4, bx, z, -1))
    tools.append(cyl(4.0, 4, 0, 9.0, -1))              # 케이블 구멍 Ø8
    return cut(plate, *tools)


def tl_top_cap():
    h = 14.0
    cap = rect_tube(TL_W, TL_D, h, TL_WALL, 0, TL_D / 2, 0, roof=TL_WALL)
    cap = cap.fuse(tl_bosses(h - TL_WALL))
    cap = cut(cap, *[cyl(PIN_D / 2, 10.0, bx, by, -1) for bx, by in TL_BOSS_XY])
    # 출력: 천장을 바닥으로
    cap.rotate(V(0, 0, 0), V(1, 0, 0), 180)
    bb = cap.BoundBox
    cap.translate(V(0, -bb.YMin, -bb.ZMin))
    return cap


def tl_foot():
    L, W, T = 130.0, 110.0, 6.0
    plate = box(L, W, T, 0, 0, 0, cx=True, cy=True)
    oy = -TL_D / 2                                   # 모듈 y=0 → 발판 y=-20
    tools = [cyl(PIN_D / 2, 5.0, bx, by + oy, T - 5.0) for bx, by in TL_BOSS_XY]
    tools.append(cyl(5.0, T + 2, 0, TL_D / 2 + oy, -1))            # 배선 구멍
    tools.append(box(8.0, W / 2 + 1, 3.0, 0, 0, -0.5, cx=True))    # 밑면 케이블 홈 → +y 가장자리
    for sx in (-1, 1):
        for sy in (-1, 1):
            tools.append(cyl(M3_FREE / 2, T + 2, sx * (L / 2 - 7), sy * (W / 2 - 7), -1))
    return cut(plate, *tools)


def tl_lamp_cup():
    r_out = TL_LAMP_D / 2 - 0.2
    lens = cyl(r_out, 1.2)
    barrel = cyl(r_out, 25.0 - 1.2, 0, 0, 1.2).cut(cyl(r_out - 1.5, 30, 0, 0, 1.2))
    flange = cyl(29.0, 2.0, 0, 0, TL_WALL).cut(cyl(r_out - 1.5, 3, 0, 0, TL_WALL - 0.5))
    return fuse(lens, barrel, flange)


def tl_led_plate():
    r = TL_LAMP_D / 2 - 0.2 - 1.5 - CLEAR
    plate = cyl(r, 3.0)
    tools = [cyl(5.1, 5, 0, 0, -1)]                                # 10 mm LED
    for k in range(3):
        a = math.radians(90 + 120 * k)
        tools.append(cyl(2.6, 5, 13 * math.cos(a), 13 * math.sin(a), -1))  # 5 mm LED ×3
    return cut(plate, *tools)


# ================================================================ B. 컨트롤러 박스 (Arduino Mega 2560)
# 보드 101.6 × 53.3. 구멍 6개 (Uno 와 같은 4개 + 오른쪽 2개), USB-B·DC 잭은 Uno 와 같은 자리.
CB_IN = (113.0, 62.0, 30.0)
CB_WALL, CB_FLOOR = 2.0, 2.0
MEGA_HOLES = [(14.0, 2.5), (15.3, 50.7), (66.1, 7.6), (66.1, 35.5), (90.2, 50.7), (96.5, 2.5)]
MEGA_AT = (5.0, 4.0)     # 보드 왼쪽 아래 모서리의 박스 내부 좌표


def ctrl_box():
    lx, ly, lz = (CB_IN[0] + 2 * CB_WALL, CB_IN[1] + 2 * CB_WALL, CB_IN[2] + CB_FLOOR)
    body = rect_tube(lx, ly, lz, CB_WALL, 0, 0, 0, cx=False, cy=False, floor=CB_FLOOR)
    ox, oy = CB_WALL + MEGA_AT[0], CB_WALL + MEGA_AT[1]
    for hx, hy in MEGA_HOLES:
        body = body.fuse(cyl(3.0, 5.0, ox + hx, oy + hy, CB_FLOOR))
    tools = [cyl(M3_TAP / 2, 8.0, ox + hx, oy + hy, CB_FLOOR - 1) for hx, hy in MEGA_HOLES]
    z0 = CB_FLOOR + 5.0 + 1.6 - 1.0                                   # 보드 윗면 바로 아래
    tools.append(box(CB_WALL + 2, 14.0, 13.0, -1, oy + 38.1, z0, cy=True))  # USB-B
    tools.append(box(CB_WALL + 2, 12.0, 13.0, -1, oy + 7.6, z0, cy=True))   # DC 잭
    for x in (24.0, 58.0, 92.0):                                       # 윗가장자리 케이블 홈 (뒷벽)
        tools.append(box(8.0, CB_WALL + 2, 9.0, x, ly - CB_WALL - 1, lz - 8.0, cx=True))
    return cut(body, *tools)


def ctrl_box_lid():
    lx, ly = CB_IN[0] + 2 * CB_WALL, CB_IN[1] + 2 * CB_WALL
    lid = box(lx, ly, 2.0)
    lip = rect_tube(CB_IN[0] - 2 * CLEAR, CB_IN[1] - 2 * CLEAR, 3.0, 1.6,
                    lx / 2, ly / 2, 2.0)
    return fuse(lid, lip)


# ================================================================ C. 차단바
# 시뮬: 기둥 30 × 30 × 220, 힌지 높이 200, 바 300 × 20 × 50. 로봇 라이다 171 mm.
# 실물: 바가 내려오면 155…205 mm를 가린다 (허브의 소켓이 힌지보다 20 mm 아래).
# 기둥 212 는 150 mm 베드(큐비콘 스타일 플러스-A15)에 안 들어가 두 토막으로 나눈다.
LC_POST = (30.0, 26.0, 212.0)
LC_WALL = 2.4
HINGE_Z = 200.0
SG90 = dict(body_x=12.6, body_z=23.2, shaft_from_top=5.9, tab_pitch=27.8)
BAR_H, BAR_T, BAR_WALL = 50.0, 12.0, 1.6
TONGUE_L = 12.0
SOCK_IN = (BAR_H - 2 * BAR_WALL, BAR_T - 2 * BAR_WALL)          # 46.8 × 8.8
TONGUE = (SOCK_IN[0] - 2 * CLEAR, SOCK_IN[1] - 2 * CLEAR)      # 46.2 × 8.2


LC_SPLIT = 100.0     # 기둥을 두 토막으로 나누는 높이 (150 mm 베드: 아래 100+플러그 20, 위 112)
LC_PLUG_H = 20.0


def lc_post_top():
    """기둥 윗토막 z = LC_SPLIT…212 (서보 포켓). 내보내기는 바닥 z = 0."""
    lx, ly, lz = LC_POST
    post = rect_tube(lx, ly, lz, LC_WALL, 0, 0, 0, roof=LC_WALL)
    # 서보 포켓: +y 벽, 몸통 세로, 축이 +y 를 향한다. 축 중심 z = HINGE_Z
    z_top = HINGE_Z + SG90['shaft_from_top']
    pocket = box(SG90['body_x'], LC_WALL + 2, SG90['body_z'],
                 0, ly / 2 - LC_WALL - 1, z_top - SG90['body_z'], cx=True)
    z_mid = z_top - 22.5 / 2
    tabs = [cyl(M2_TAP / 2, LC_WALL + 2, 0, ly / 2 - LC_WALL - 1, z_mid + s * SG90['tab_pitch'] / 2, axis='y')
            for s in (-1, 1)]
    pin = cyl(PIN_D / 2, ly + 2, 0, -ly / 2 - 1, LC_SPLIT + LC_PLUG_H / 2, axis='y')  # 이음 핀
    top = cut(post, pocket, pin, *tabs)
    top = top.cut(box(lx + 2, ly + 2, LC_SPLIT + 1, 0, 0, -1, cx=True, cy=True))
    top.translate(App.Vector(0, 0, -LC_SPLIT))
    return top


def lc_post_base():
    """기둥 아랫토막 z = 0…LC_SPLIT, 위에 윗토막 속으로 들어가는 플러그."""
    lx, ly, _ = LC_POST
    post = rect_tube(lx, ly, LC_SPLIT, LC_WALL, 0, 0, 0)
    plug = rect_tube(lx - 2 * LC_WALL - 2 * CLEAR, ly - 2 * LC_WALL - 2 * CLEAR,
                     LC_PLUG_H, LC_WALL, 0, 0, LC_SPLIT)
    post = post.fuse(plug)
    # 바닥 옆 배선 슬롯 (-y 벽), 발판을 안 쓸 때
    slot = box(8.0, LC_WALL + 2, 10.0, 0, -ly / 2 - 1, -1, cx=True)
    pin = cyl(PIN_D / 2, ly + 2, 0, -ly / 2 - 1, LC_SPLIT + LC_PLUG_H / 2, axis='y')
    return cut(post, slot, pin)


def lc_foot():
    T = 6.0
    plate = box(148.0, 100.0, T, -50.0, -50.0, 0)             # 기둥 중심이 (0,0), +x 가 도로 쪽 (148: 150 베드)
    rim = rect_tube(LC_POST[0] + 2 * CLEAR + 2 * 2.4, LC_POST[1] + 2 * CLEAR + 2 * 2.4,
                    12.0, 2.4, 0, 0, T)
    plate = plate.fuse(rim)
    tools = [cyl(5.0, T + 2, 0, 0, -1),
             box(50.0 + 1, 8.0, 3.0, -51.0, 0, -0.5, cy=True)]  # 밑면 홈 → -x 가장자리(박스 쪽)
    for x in (-43, 91):
        for y in (-43, 43):
            tools.append(cyl(M3_FREE / 2, T + 2, x, y, -1))
    return cut(plate, *tools)


def lc_hub():
    """서보 혼에 붙는 허브. 출력 자세: 서보 축 = z, 바 = +x, 꼬리 = -x.
    y 가 실제 수직: 바는 힌지 아래 45, 위 5 (바 폭 50). 혼은 아랫면 홈에 앉고
    45° 방향 4개 구멍으로 M2 나사 고정, 가운데 구멍으로 혼 나사를 조인다."""
    disc = cyl(14.0, 6.0)
    y0 = -45.0
    sock = box(60.0, BAR_H, BAR_T, -30.0, y0, 0)
    hub = fuse(disc, sock)
    tools = []
    for s in (-1, 1):
        x0 = 9.0 if s > 0 else -30.0
        tools.append(box(21.0, SOCK_IN[0], SOCK_IN[1], x0, y0 + BAR_WALL, BAR_WALL))
    tools.append(cyl(10.6, 3.0, 0, 0, -1))                # 둥근 혼 자리 (Ø21, 깊이 2)
    tools.append(cyl(3.25, 20, 0, 0, -1))                 # 혼 나사 드라이버 통로
    for a in (45, 135, 225, 315):
        x, y = 7.5 * math.cos(math.radians(a)), 7.5 * math.sin(math.radians(a))
        tools.append(cyl(M2_TAP / 2, 20, x, y, -1))
        tools.append(cyl(2.0, 20, x, y, 4.5))             # 나사머리 카운터보어
    return cut(hub, *tools)


def lc_bar_segment(length=50.0, closed_end=False):
    seg = rect_tube(BAR_H, BAR_T, length, BAR_WALL, 0, 0, 0,
                    floor=BAR_WALL if closed_end else 0.0)
    tongue = rect_tube(TONGUE[0], TONGUE[1], TONGUE_L + 1, 1.2, 0, 0, length - 1)
    return fuse(seg, tongue)


def lc_sensor_post(kind):
    base = box(60.0, 60.0, 4.0, 0, 0, 0, cx=True, cy=True)
    col = box(26.0, 26.0, 58.0, 0, 0, 4.0, cx=True, cy=True)
    post = fuse(base, col)
    tools = [box(6.0, 31.0, 2.0, 0, 0, -0.5, cx=True)]       # 밑면 케이블 홈 → +y
    z = 40.0
    if kind == 'e18':
        tools.append(cyl(18.4 / 2, 30, -15, 0, z, axis='x'))     # E18-D80NK M18 몸통
    else:
        tools.append(cyl(6.4 / 2, 30, -15, 0, z, axis='x'))      # KY-008 레이저 헤드 / 5 mm 수광부
        tools.append(box(23.6 + 1, 19.0, 16.5, -13.0 - 1, 0, z - 16.5 / 2, cy=True))  # PCB 포켓(뒤에서)
    return cut(post, *tools)


# ================================================================ D. 표지판
# 시뮬: 12 × 2.5 × 25 cm 상자에 그린 텍스처. 판 지름 120, 기둥 아래 끝까지 250.
SIGN_D = 120.0
SIGN_T = 3.0
TENON = 7.6


def sign_stand():
    base = box(70.0, 30.0, 8.0, 0, 0, 0, cx=True, cy=True)
    col = box(12.0, 12.0, 122.0, 0, 0, 8.0, cx=True, cy=True)
    tenon = box(TENON, TENON, 12.0, 0, 0, 130.0, cx=True, cy=True)
    return fuse(base, col, tenon)


def sign_outline(shape, size, h, z=0.0):
    """판 윤곽. size 는 가로 폭(원 지름 / 팔각 평행면 / 삼각 한 변 / 정사각 한 변).
    (도형, 아래 가장자리 y) 를 돌려준다. 중심은 (0, 0)."""
    if shape == 'round':
        return cyl(size / 2, h, 0, 0, z), -size / 2
    if shape == 'octagon':
        return octagon(size, h, z), -size / 2
    if shape == 'square':
        return box(size, size, h, 0, 0, z, cx=True, cy=True), -size / 2
    if shape == 'triangle':
        r = size / math.sqrt(3)                       # 외접원 반지름, 무게중심이 원점
        pts = [(0, r), (-size / 2, -r / 2), (size / 2, -r / 2)]
        return prism(pts, h, z), -r / 2
    raise ValueError(shape)


def sign_plate(shape='round'):
    """표지판 판. 출력면(z=0)이 앞면, 앞면에 0.8 mm 스티커 홈, 아래 가장자리 뒤에
    받침 기둥의 장부가 들어가는 보스."""
    plate, y_bottom = sign_outline(shape, SIGN_D, SIGN_T)
    face, _ = sign_outline(shape, SIGN_D - 4, 0.8, -0.1)
    # 삼각형은 안쪽 윤곽을 같은 아래 가장자리에 맞추지 않아도 된다 (중심 기준 축소).
    boss = box(12.0, 25.0, 12.0, 0, y_bottom - 5.0, 0, cx=True)
    plate = fuse(plate, boss)
    mortise = box(TENON + 2 * CLEAR, 13.0, TENON + 2 * CLEAR,
                  0, y_bottom - 6.0, 6.0 - (TENON + 2 * CLEAR) / 2, cx=True)
    return cut(plate, face, mortise)


# ================================================================ E. 공사 구간 기둥
# 시뮬: 90 × 234, 높이 400, 흑백 띠 4개. 띠 하나 = 반쪽 트레이 2개 + 이음 슬리브.
CP_L, CP_W, CP_BAND = 234.0, 90.0, 100.0
CP_WALL, CP_FLOOR = 1.2, 1.6


def cp_band_half():
    half = CP_L / 2
    body = box(half, CP_W, CP_BAND)
    cavity = box(half - CP_WALL + 1, CP_W - 2 * CP_WALL, CP_BAND, CP_WALL, CP_WALL, CP_FLOOR)
    body = body.cut(cavity)
    # 아래로 6 mm 플러그 (U자, 열린 끝은 없음)
    ins = CP_WALL + CLEAR
    plug = box(half - ins, CP_W - 2 * ins, 6.0, ins, ins, -6.0)
    plug = plug.cut(box(half - ins - 1.2 + 2, CP_W - 2 * ins - 2.4, 8.0, ins + 1.2, ins + 1.2, -7.0))
    body = body.fuse(plug)
    bb = body.BoundBox
    body.translate(V(0, 0, -bb.ZMin))
    return body.removeSplitter()


def cp_cap_half():
    half = CP_L / 2
    cap = box(half, CP_W, CP_FLOOR)
    ins = CP_WALL + CLEAR
    plug = box(half - ins, CP_W - 2 * ins, 6.0, ins, ins, -6.0)
    plug = plug.cut(box(half - ins - 1.2 + 2, CP_W - 2 * ins - 2.4, 8.0, ins + 1.2, ins + 1.2, -7.0))
    cap = cap.fuse(plug)
    bb = cap.BoundBox
    cap.translate(V(0, 0, -bb.ZMin))
    return cap.removeSplitter()


def cp_splice():
    # 두 반쪽의 이음매에 걸치는 슬리브: 안쪽 폭에 맞춘 사각 튜브, x 방향 60.
    w = CP_W - 2 * CP_WALL - 2 * CLEAR
    h = 80.0
    return rect_tube(60.0, w, h, 1.6, 0, 0, 0, cx=False, cy=False)


# ================================================================ F. 터널 장애물 (속 빈 껍데기, 뒤집어 출력)
def shell_box(lx, ly, lz, wall=1.2):
    return rect_tube(lx, ly, lz, wall, 0, 0, 0, floor=wall)


def shell_cyl(r, h, wall=1.2):
    return cyl(r, h).cut(cyl(r - wall, h, 0, 0, wall))


# ================================================================ 조립 및 내보내기
def build():
    add('A1', 'tl_module', 3, '신호등 램프 모듈 90×40×75, 뒷면 개방, 앞면을 바닥에 두고 출력', tl_module())
    add('A7', 'tl_hood', 3, '램프 차양 Ø60 반원, 앞면 홈에 접착', tl_hood())
    add('A2', 'tl_module_cover', 3, '모듈 뒷판 90×75×2, M3 ×4, 케이블 Ø8', tl_module_cover())
    add('A3', 'tl_top_cap', 1, '신호등 위 덮개', tl_top_cap())
    add('A4', 'tl_foot', 1, '신호등 발판 130×110×6, 밑면 케이블 홈', tl_foot())
    add('A5', 'tl_lamp_cup', 3, '램프 컵+확산 렌즈(투명/백색 출력), 안쪽에서 끼움', tl_lamp_cup())
    add('A6', 'tl_led_plate', 3, 'LED 판: 10 mm ×1 또는 5 mm ×3', tl_led_plate())
    add('B1', 'ctrl_box', 2, 'Arduino Mega 2560 박스 117×66×32, USB/DC 창, 케이블 홈 3', ctrl_box())
    add('B2', 'ctrl_box_lid', 2, '박스 뚜껑 (압입)', ctrl_box_lid())
    add('C1', 'lc_post_top', 1, '차단바 기둥 윗토막 30×26×112, SG90/MG90S 포켓, 속 빈 배선 통로', lc_post_top())
    add('C9', 'lc_post_base', 1, '차단바 기둥 아랫토막 30×26×100 + 플러그 20, 핀으로 윗토막과 잇는다', lc_post_base())
    add('C2', 'lc_foot', 1, '차단바 발판 148×100×6, 기둥 소켓, 밑면 케이블 홈', lc_foot())
    add('C3', 'lc_hub', 1, '서보 혼 허브, 양쪽 소켓(바/꼬리)', lc_hub())
    add('C4', 'lc_bar_segment', 5, '바 토막 50×12×50 + 장부, 빨강/흰색 번갈아 출력', lc_bar_segment())
    add('C5', 'lc_bar_end', 1, '바 끝 토막 (막힌 끝)', lc_bar_segment(closed_end=True))
    add('C6', 'lc_tail', 1, '균형추 꼬리 60 mm (동전/너트 넣고 끼움)', lc_bar_segment(60.0, closed_end=True))
    add('C7', 'lc_sensor_post_e18', 2, '센서 기둥: E18-D80NK(M18) 축 높이 40', lc_sensor_post('e18'))
    add('C8', 'lc_sensor_post_laser', 2, '센서 기둥: KY-008 레이저 / 5 mm 수광부 + PCB 포켓', lc_sensor_post('laser'))
    add('D1', 'sign_stand', 7, '표지판 받침+기둥 70×30, 높이 142, 장부', sign_stand())
    add('D2', 'sign_plate_round', 2, '둥근 판 Ø120×3 (좌/우회전; 여분 진입금지·pl_left 도 이 판)', sign_plate('round'))
    add('D3', 'sign_plate_octagon', 1, 'STOP 팔각 판 120×3', sign_plate('octagon'))
    add('D4', 'sign_plate_triangle', 3, '삼각 판 한 변 120×3 (갈림길·공사·터널)', sign_plate('triangle'))
    add('D5', 'sign_plate_square', 1, '정사각 판 120×3 (주차)', sign_plate('square'))
    add('E1', 'cp_band_half', 8, '공사 기둥 띠 반쪽 117×90×100 (트레이), 2개 = 띠 1', cp_band_half())
    add('E2', 'cp_splice', 4, '띠 반쪽 이음 슬리브', cp_splice())
    add('E3', 'cp_cap_half', 2, '공사 기둥 윗덮개 반쪽', cp_cap_half())
    add('F1', 'to_box_200x150', 1, '터널 장애물 상자 200×150×200 껍데기', shell_box(200, 150, 200))
    add('F2', 'to_box_120x150', 2, '터널 장애물 상자 120×150×200 ×2 = 120×300', shell_box(120, 150, 200))
    add('F3', 'to_cyl_d160', 1, '터널 장애물 원기둥 Ø160×200 껍데기', shell_cyl(80, 200))


def export():
    os.makedirs(os.path.join(OUT, 'stl'), exist_ok=True)
    os.makedirs(os.path.join(OUT, 'step'), exist_ok=True)
    doc = App.newDocument('venue_props')
    rows = []
    for number, name, qty, desc, shape in PARTS:
        obj = doc.addObject('Part::Feature', f'{number}_{name}')
        obj.Shape = shape
        mesh = MeshPart.meshFromShape(Shape=shape, LinearDeflection=0.05,
                                      AngularDeflection=0.1, Relative=False)
        mesh.write(os.path.join(OUT, 'stl', f'{number}_{name}.stl'))
        shape.exportStep(os.path.join(OUT, 'step', f'{number}_{name}.step'))
        bb = shape.BoundBox
        grams = shape.Volume * DENSITY
        rows.append((number, name, qty, desc,
                     f'{bb.XLength:.0f} × {bb.YLength:.0f} × {bb.ZLength:.0f}',
                     grams, shape.isValid(), mesh.CountFacets))
    doc.recompute()
    doc.saveAs(os.path.join(OUT, 'venue_props.FCStd'))
    with open(os.path.join(OUT, 'parts.md'), 'w', encoding='utf-8') as f:
        f.write('<!-- gen_venue_parts.py 가 만든 표. 손으로 고치지 말 것. -->\n')
        f.write('| 번호 | 파일 | 수량 | 설명 | 출력 크기 (mm, x × y × 높이) | 개당 g (100 % 채움) |\n')
        f.write('|---|---|---|---|---|---|\n')
        total = 0.0
        for number, name, qty, desc, size, grams, ok, _ in rows:
            total += grams * qty
            f.write(f'| {number} | `{number}_{name}.stl` | {qty} | {desc} | {size} | {grams:.0f} |\n')
        f.write(f'\n전체 {total / 1000:.2f} kg (100 % 채움 기준; 껍데기 두께가 곧 벽이라 채움률 영향이 작다).\n')
    for r in rows:
        print('%-4s %-22s x%d  %-18s %6.0f g  valid=%s  facets=%d' % (r[0], r[1], r[2], r[4], r[5], r[6], r[7]))
    print('total %.2f kg' % (sum(r[5] * r[2] for r in rows) / 1000))


build()
export()
