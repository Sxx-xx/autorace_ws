#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""표지판 스티커 원고: 시뮬 텍스처의 판 부분을 잘라 실물 크기로 만든다.

실행:  python3 make_sign_faces.py
결과:  signs/<이름>_116mm.png (300 dpi), signs/sign_sheet_A4.pdf (A4, 장당 2개)

판 지름은 sign_plate_round 의 스티커 홈(Ø116)에 맞춘다. 텍스처는 272 px 폭의
저해상도라 확대하면 가장자리가 조금 부드러워진다; 라벨지에 출력해 붙인다.
"""

import glob
import os

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = os.path.join(HERE, '..', '..', 'src', 'autorace_sim', 'models')
OUT = os.path.join(HERE, 'signs')
DPI = 300
FACE_MM = 116.0
PX = int(round(FACE_MM / 25.4 * DPI))
A4 = (int(round(210 / 25.4 * DPI)), int(round(297 / 25.4 * DPI)))

# 코스에 서는 7개를 먼저, 여분 2개를 뒤에.
SHAPES = {'intersection': 'triangle', 'construction': 'triangle', 'tunnel': 'triangle',
          'parking': 'square', 'stop': 'octagon'}
ORDER = ['intersection', 'left', 'right', 'construction', 'parking', 'tunnel', 'stop',
         'noentry', 'pl_left']


def texture(name):
    pattern = os.path.join(MODELS, f'autorace_sign_{name}', 'materials', 'textures', '*.png')
    files = glob.glob(pattern)
    if not files:
        raise SystemExit(f'텍스처 없음: {pattern}')
    return Image.open(files[0]).convert('RGBA')


def plate_box(im):
    """판(위쪽 덩어리)의 정사각 영역. 기둥은 판보다 훨씬 좁다."""
    a = np.asarray(im)
    opaque = a[:, :, 3] > 128
    dark = (a[:, :, :3].min(axis=2) < 235) & opaque
    rows = np.where(dark.any(axis=1))[0]
    top = rows[0]
    widths = dark.sum(axis=1)
    cols = np.where(dark[top:top + int(im.height * 0.55)].any(axis=0))[0]
    x0, x1 = cols[0], cols[-1] + 1
    width = x1 - x0
    # 판 아래쪽: 가장 넓은 줄 아래에서 가로 폭이 판 폭의 1/3 밑으로 떨어지는 첫 줄
    # (삼각형·원형 판은 꼭대기가 좁으므로 위에서부터 재면 안 된다)
    widest = top + int(np.argmax(widths[top:top + int(im.height * 0.55)]))
    bottom = im.height
    for r in range(widest, im.height):
        if widths[r] < width / 3:
            bottom = r
            break
    size = max(width, bottom - top)
    cy = (top + bottom) / 2.0
    cx = (x0 + x1) / 2.0
    half = size / 2.0
    return (int(round(cx - half)), int(round(cy - half)), int(round(cx + half)), int(round(cy + half))), bottom


def face(name):
    im = texture(name)
    box, bottom = plate_box(im)
    crop = Image.new('RGBA', (box[2] - box[0], box[3] - box[1]), (255, 255, 255, 0))
    crop.paste(im, (-box[0], -box[1]))
    # 판 아래로 이어지는 기둥은 지운다
    crop.paste((255, 255, 255, 0), (0, bottom - box[1], crop.width, crop.height))
    crop = crop.resize((PX, PX), Image.LANCZOS)
    white = Image.new('RGBA', crop.size, (255, 255, 255, 255))
    return Image.alpha_composite(white, crop).convert('RGB')


def main():
    os.makedirs(OUT, exist_ok=True)
    faces = []
    for name in ORDER:
        f = face(name)
        f.save(os.path.join(OUT, f'{name}_116mm.png'), dpi=(DPI, DPI))
        faces.append((name, f))
        print(f'{name:14s} {f.size[0]} px = {f.size[0] / DPI * 25.4:.0f} mm')
    pages = []
    margin = int(15 / 25.4 * DPI)
    gap = int(10 / 25.4 * DPI)
    for i in range(0, len(faces), 2):
        page = Image.new('RGB', A4, 'white')
        for k, (name, f) in enumerate(faces[i:i + 2]):
            x = (A4[0] - PX) // 2
            y = margin + k * (PX + gap)
            page.paste(f, (x, y))
            # 재단선: 판 지름 120 mm 원
            from PIL import ImageDraw
            d = ImageDraw.Draw(page)
            r = int(120 / 25.4 * DPI / 2)
            cx, cy = x + PX // 2, y + PX // 2
            d.ellipse((cx - r, cy - r, cx + r, cy + r), outline=(160, 160, 160), width=2)
            d.text((x, y + PX + 8), f'{name}  face 116 mm / plate 120 mm ({SHAPES.get(name, "round")})', fill=(90, 90, 90))
        pages.append(page)
    pages[0].save(os.path.join(OUT, 'sign_sheet_A4.pdf'), save_all=True,
                  append_images=pages[1:], resolution=DPI)
    print(f'{len(pages)} pages -> signs/sign_sheet_A4.pdf')


if __name__ == '__main__':
    main()
