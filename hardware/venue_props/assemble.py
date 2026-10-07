#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""조립도·분해도 렌더링: export/stl 의 부품을 실제 위치에 놓고 그린다.

실행:  python3 assemble.py            → export/preview/assembly_*.png
부품의 출력 자세(gen_venue_parts.py 의 내보내기 프레임)를 조립 프레임으로 옮기는
변환이 여기 들어 있다. 치수를 바꾸면 여기 숫자도 같이 봐야 한다.
"""

import os
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from render import load_stl
from raster import rot

HERE = os.path.dirname(os.path.abspath(__file__))
STL = os.path.join(HERE, 'export', 'stl')
OUT = os.path.join(HERE, 'export', 'preview')

BLACK, WHITE, GREY, RED, YEL, GRN, BLUE, CLEAR = (
    (70, 70, 75), (235, 235, 230), (170, 175, 185), (220, 60, 50), (235, 190, 40),
    (60, 190, 90), (70, 110, 200), (200, 225, 240))

_cache = {}


def part(name):
    if name not in _cache:
        _cache[name] = load_stl(os.path.join(STL, name + '.stl'))
    return _cache[name].copy()


def place(tris, m, t):
    """tris (n,3,3) → M @ p + t. m 은 3×3 행렬 또는 '열 재배열' 문자열."""
    return tris @ np.asarray(m, float).T + np.asarray(t, float)


def axes(ex, ey, ez):
    """내보내기 축 (x, y, z) 가 조립 프레임의 어느 벡터로 가는지로 행렬을 만든다."""
    return np.array([ex, ey, ez], float).T


X, Y, Z = (1, 0, 0), (0, 1, 0), (0, 0, 1)
NX, NY, NZ = (-1, 0, 0), (0, -1, 0), (0, 0, -1)


def box_mesh(lx, ly, lz, at):
    x0, y0, z0 = at
    v = np.array([[x, y, z] for z in (z0, z0 + lz) for y in (y0, y0 + ly) for x in (x0, x0 + lx)])
    f = [(0, 2, 1), (1, 2, 3), (4, 5, 6), (5, 7, 6), (0, 1, 4), (1, 5, 4),
         (2, 6, 3), (3, 6, 7), (0, 4, 2), (2, 4, 6), (1, 3, 5), (3, 7, 5)]
    return v[np.array(f)]


def render(items, size=900, el=-65, az=-35, labels=(), scale=None, center=None):
    R = rot(el, az)
    allv = np.concatenate([t.reshape(-1, 3) for t, _ in items]) @ R.T
    lo, hi = allv.min(0), allv.max(0)
    c = (lo + hi) / 2 if center is None else np.asarray(center) @ R.T
    s = scale or (size * 0.9) / max(hi[0] - lo[0], hi[1] - lo[1])
    zbuf = np.full((size, size), -np.inf)
    img = np.full((size, size, 3), 255, np.uint8)
    light = np.array([-0.4, 0.5, 0.75]); light /= np.linalg.norm(light)
    for tris, color in items:
        v = (tris.reshape(-1, 3) @ R.T).reshape(-1, 3, 3)
        p = (v - c) * s
        px = p[:, :, 0] + size / 2; py = size / 2 - p[:, :, 1]; pz = p[:, :, 2]
        n = np.cross(v[:, 1] - v[:, 0], v[:, 2] - v[:, 0])
        nn = np.linalg.norm(n, axis=1); keep = nn > 1e-9
        n = n[keep] / nn[keep, None]; px, py, pz = px[keep], py[keep], pz[keep]
        shade = 0.3 + 0.7 * np.abs(n @ light)
        col = np.array(color, float)
        for i in range(len(px)):
            x, y, z = px[i], py[i], pz[i]
            x0, x1 = int(max(0, np.floor(x.min()))), int(min(size - 1, np.ceil(x.max())))
            y0, y1 = int(max(0, np.floor(y.min()))), int(min(size - 1, np.ceil(y.max())))
            if x1 < x0 or y1 < y0:
                continue
            Xg, Yg = np.meshgrid(np.arange(x0, x1 + 1) + 0.5, np.arange(y0, y1 + 1) + 0.5)
            d = (x[1] - x[0]) * (y[2] - y[0]) - (x[2] - x[0]) * (y[1] - y[0])
            if abs(d) < 1e-9:
                continue
            w1 = ((Xg - x[0]) * (y[2] - y[0]) - (x[2] - x[0]) * (Yg - y[0])) / d
            w2 = ((x[1] - x[0]) * (Yg - y[0]) - (Xg - x[0]) * (y[1] - y[0])) / d
            w0 = 1 - w1 - w2
            inside = (w0 >= -1e-3) & (w1 >= -1e-3) & (w2 >= -1e-3)
            if not inside.any():
                continue
            Zg = w0 * z[0] + w1 * z[1] + w2 * z[2]
            sub = zbuf[y0:y1 + 1, x0:x1 + 1]
            upd = inside & (Zg > sub)
            sub[upd] = Zg[upd]
            img[y0:y1 + 1, x0:x1 + 1][upd] = np.clip(col * shade[i], 0, 255).astype(np.uint8)
    im = Image.fromarray(img)
    d = ImageDraw.Draw(im)
    font = ImageFont.load_default(size=15) if hasattr(ImageFont, 'load_default') else None
    for text, pt in labels:
        q = (np.asarray(pt, float) @ R.T - c) * s
        d.text((q[0] + size / 2 + 4, size / 2 - q[1] - 8), text, fill=(20, 20, 20), font=font)
    return im


def side_by_side(images, titles, out):
    font = ImageFont.load_default(size=18) if hasattr(ImageFont, 'load_default') else None
    W = sum(i.width for i in images); H = max(i.height for i in images) + 30
    sheet = Image.new('RGB', (W, H), 'white'); d = ImageDraw.Draw(sheet); x = 0
    for im, t in zip(images, titles):
        sheet.paste(im, (x, 30)); d.text((x + 10, 6), t, fill=(0, 0, 0), font=font); x += im.width
    sheet.save(out); print('wrote', out)


# ---------------------------------------------------------------- 신호등
def traffic_light(explode=0.0):
    """조립 프레임: 발판 중심이 원점, 램프면이 -y, 위가 +z."""
    items, labels = [], []
    e = explode
    items.append((part('A4_tl_foot'), GREY))
    labels.append(('A4 foot', (70, 0, 0)))
    lamp_col = [RED, YEL, GRN]
    for k in range(3):
        z0 = 6 + k * 75 + e * (k + 1) * 60
        # 모듈: 내보내기 (x, y, z) = (x, 75 - 모듈z, 모듈y). 앞면 y=0 → 조립 y=-20.
        m = place(part('A1_tl_module'), axes(X, NZ, Y), (0, -20, z0 + 75))
        items.append((m, BLACK)); labels.append((f'A1 module {k + 1}', (50, 20, z0 + 37)))
        # 뒷판: 내보내기 y = 모듈 z, z = 두께 → 조립 y = 20 + z
        cov = place(part('A2_tl_module_cover'), axes(X, Z, Y), (0, 20 + e * 40, z0))
        items.append((cov, GREY)); labels.append((f'A2 cover', (-50, 22 + e * 40, z0 + 60)))
        # 램프 컵: 축 z → 조립 -y (렌즈가 앞면 y=-20). 분해 시 앞으로 뺀다
        cup = place(part('A5_tl_lamp_cup'), axes(X, Z, NY), (0, -20 - e * 60, z0 + 37.5))
        items.append((cup, CLEAR))
        led = place(part('A6_tl_led_plate'), axes(X, Z, NY), (0, -20 - e * 60 + 22, z0 + 37.5))
        items.append((led, lamp_col[k]))
        if k == 0:
            labels.append(('A5 cup + A6 LED plate', (30, -22 - e * 60, z0 + 15)))
        # 차양: 축 z → 조립 -y, 열린 쪽(-y) 이 아래 → 내보내기 y → 조립 z
        hood = place(part('A7_tl_hood'), axes(X, Z, NY), (0, -20 - e * 100 + 0.6, z0 + 37.5))
        items.append((hood, BLACK))
        if k == 2:
            labels.append(('A7 hood', (0, -40 - e * 100, z0 + 70)))
    z_top = 6 + 225 + e * 4 * 60
    cap = place(part('A3_tl_top_cap'), axes(X, NY, NZ), (0, 20, z_top + 14))
    items.append((cap, BLACK)); labels.append(('A3 top cap', (50, 0, z_top + 10)))
    # 핀 (분해도에만): 보스 위치의 3 mm 필라멘트
    if e:
        for bx in (-39.6, 39.6):
            for by in (5.4, 34.6):
                for k in range(4):
                    zp = 6 + k * 75 + e * (k + 0.5) * 60 - 10
                    items.append((box_mesh(3, 3, 20, (bx - 1.5, by - 20 - 1.5, zp)), RED))
        labels.append(('pins Ø3 x 20 (x4 per joint)', (45, -20, 6 + e * 30)))
    return items, labels


# ---------------------------------------------------------------- 차단바
def level_crossing(explode=0.0):
    """조립 프레임: 기둥 중심이 원점, 도로가 +x, 서보 축이 +y, 위가 +z."""
    items, labels = [], []
    e = explode
    items.append((part('C2_lc_foot'), GREY)); labels.append(('C2 foot', (60, -45, 6)))
    items.append((place(part('C1_lc_post'), np.eye(3), (0, 0, 6 + e * 40)), BLACK))
    labels.append(('C1 post (servo pocket on +y face)', (16, 0, 120 + e * 40)))
    # 서보 (구매품) : 몸통 12.6 × 22.5(z) 가 +y 벽을 관통, 탭이 바깥, 축이 +y
    ys = 13 + e * 50
    items.append((box_mesh(12.4, 22.0, 22.5, (-6.2, ys - 16, 183.4 + e * 40)), BLUE))
    items.append((box_mesh(32.0, 2.5, 4.0, (-16, ys, 192.6 + e * 40)), BLUE))  # 탭(가로로 그림)
    items.append((box_mesh(5, 4, 5, (-2.5, ys + 6, 197.5 + e * 40)), BLUE))        # 축
    labels.append(('MG90S servo', (-20, ys, 215 + e * 40)))
    # 허브: 내보내기 z(축) → +y, x → +x(도로), y → +z
    yh = 23 + e * 80
    items.append((place(part('C3_lc_hub'), axes(X, Z, Y), (0, yh, 200 + e * 40)), GREY))
    labels.append(('C3 hub (horn in recess, screw through center hole)', (-30, yh, 222 + e * 40)))
    # 바 토막: 내보내기 z(길이, 장부가 끝) → -x, x(50) → z, y(12) → y
    yb = yh + 6
    for i in range(6):
        name = 'C5_lc_bar_end' if i == 5 else 'C4_lc_bar_segment'
        x0 = 30 + 50 * i + e * 30 * (i + 1)
        seg = place(part(name), axes(Z, Y, NX), (x0 + 50, yb, 180 + e * 40))
        items.append((seg, WHITE if i % 2 == 0 else BLACK))
    labels.append(('C4 x5 + C5 end: 50 mm each, tongue toward hub, alternate colours',
                   (60, yb + 8, 150 + e * 40)))
    tail = place(part('C6_lc_tail'), axes(Z, Y, X), (-90 - e * 30, yb, 180 + e * 40))
    items.append((tail, GREY)); labels.append(('C6 tail (weights inside)', (-120 - e * 30, yb, 215 + e * 40)))
    # 컨트롤러 박스: 발판 홈이 나오는 -x 쪽
    items.append((place(part('B1_ctrl_box'), np.eye(3), (-190, -33, 0)), GREY))
    labels.append(('B1 ctrl box (cable via foot channel)', (-190, 40, 0)))
    # 2번 센서 기둥: 도로 건너편, 바에서 60 mm 앞(-y). 구멍이 x 축 → 도로를 바라본다
    items.append((place(part('C7_lc_sensor_post_e18'), np.eye(3), (360, -60 - 30, 0)), GREY))
    labels.append(('C7 sensor 2 (60 mm before bar, h=40)', (360, -140, 70)))
    return items, labels


# ---------------------------------------------------------------- 표지판 / 공사 기둥
def sign(explode=0.0):
    e = explode
    items = [(part('D1_sign_stand'), GREY)]
    # 판: 내보내기 z(앞면=0) → -y 방향 두께, y → z. 장부 보스 바닥(ey=-65) 이 기둥 위(130)
    for i, (name, col, dx) in enumerate([('D2_sign_plate_round', BLUE, 0), ('D4_sign_plate_triangle', RED, 160),
                                         ('D3_sign_plate_octagon', RED, 320), ('D5_sign_plate_square', BLUE, 480)]):
        items.append((place(part('D1_sign_stand'), np.eye(3), (dx, 0, 0)), GREY))
        items.append((place(part(name), axes(X, Z, NY), (dx, 6 + 0 * e, 195 + e * 60)), col))
    labels = [('D1 stand x4', (0, 0, 0)), ('plates: mortise over 7.6 mm tenon, sticker side forward (-y)', (0, -10, 270 + e * 60))]
    return items[1:], labels


def construction_post(explode=0.0):
    e = explode
    items, labels = [], []
    for k in range(4):
        z = k * 100 + e * (k + 1) * 50
        col = WHITE if k % 2 == 0 else BLACK
        items.append((place(part('E1_cp_band_half'), np.eye(3), (0, 0, z - 6)), col))
        items.append((place(part('E1_cp_band_half'), axes(NX, Y, Z), (234 + e * 60, 0, z - 6)), col))
        items.append((place(part('E2_cp_splice'), np.eye(3), (87 + e * 30, 1.5, z + 2 + e * 25)), GREY))
    z = 400 + e * 5 * 50
    items.append((place(part('E3_cp_cap_half'), np.eye(3), (0, 0, z - 6)), GREY))
    items.append((place(part('E3_cp_cap_half'), axes(NX, Y, Z), (234 + e * 60, 0, z - 6)), GREY))
    labels = [('E1 half-bands x2 per band, plug drops into band below', (240 + e * 60, 0, 50)),
              ('E2 splice bridges the seam', (90 + e * 30, 95, 100 + e * 125)),
              ('E3 cap halves', (120, 0, z + 5))]
    return items, labels


def main():
    os.makedirs(OUT, exist_ok=True)
    for name, fn, el, az in [('traffic_light', traffic_light, -62, -40), ('level_crossing', level_crossing, -62, -30),
                             ('signs', sign, -65, -35), ('construction_post', construction_post, -62, -35)]:
        a, la = fn(0.0)
        b, lb = fn(1.0)
        side_by_side([render(a, 900, el, az, la), render(b, 900, el, az, lb)],
                     [f'{name}: assembled', f'{name}: exploded'],
                     os.path.join(OUT, f'assembly_{name}.png'))


if __name__ == '__main__':
    main()
