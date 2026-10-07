#!/usr/bin/env python3
"""Render the course floor texture from the competition map drawing.

map.png (the "TB3 Auto Race Map" drawing at the workspace root) is only
325 x 309 px for the 4 m x 4 m board, so it is classified into floor, road,
yellow line and white line, the icons drawn on top of it are painted out, and
the classes are upsampled smoothly to a square texture. The red stop line at
the start is put back afterwards: it is paint on the floor, not an icon.

Image axes: up = world +X, left = world +Y, board centre = world origin.
"""
from pathlib import Path

import cv2
import numpy as np

WS = Path(__file__).resolve().parents[3]
MAP = WS / 'map.png'
OUT = (WS / 'src/autorace_sim/models/autorace_course_map/materials/textures'
       / 'course_map.png')

# The board inside map.png, in map pixels (inclusive). The drawing is not
# square, so each axis gets its own scale.
BOARD_X = (319, 643)
BOARD_Y = (53, 361)
SIZE = 2000  # texture pixels for 4 m: 2 mm per pixel

FLOOR, ROAD, YELLOW, WHITE, STOP = range(5)
# The drawing has a light grey floor round the road; the venue (see
# wkrsus.png, 2022) is black all over, so the floor is painted like the road.
COLOURS = {  # BGR
    FLOOR: (20, 20, 20),
    ROAD: (20, 20, 20),
    YELLOW: (0, 215, 255),
    WHITE: (250, 250, 250),
    STOP: (30, 30, 230),
}

# The white lines are one map pixel wide and come out some 12 mm; the tape
# on the course is wider. Grown by this many texture pixels each side.
WHITE_GROWTH = 4
# Texture pixels: the 3 cm of tape the lane detector assumes (lane.line_width_m).
LINE_WIDTH = 15

# The stop line across the top road, where the robot waits for the traffic
# light: map pixels (inclusive), as drawn.
STOP_LINE_X = (407, 410)
STOP_LINE_Y = (63, 79)


def classify(image):
    b, g, r = (image[..., i].astype(int) for i in range(3))
    labels = np.full(image.shape[:2], -1, np.int8)
    labels[(abs(r - 204) < 12) & (abs(g - 204) < 12) & (abs(b - 204) < 12)] = FLOOR
    # Tile grid lines on the floor.
    labels[(abs(r - g) < 8) & (abs(g - b) < 8) & (r > 150) & (r < 230)] = FLOOR
    labels[(r < 110) & (g < 110) & (b < 110) & (abs(r - b) < 15)] = ROAD
    labels[(r > 180) & (g > 160) & (b < 120)] = YELLOW
    # The white edge lines are one pixel wide and anti-aliased into the floor.
    labels[(r > 222) & (g > 222) & (b > 222) & (abs(r - b) < 12)] = WHITE
    return labels


def icon_boxes(image):
    """Boxes around the signs, arrows and the stop bar drawn on the board."""
    b, g, r = (image[..., i].astype(int) for i in range(3))
    red = (r > 180) & (g < 90) & (b < 90)
    blue = (b > 150) & (r < 90) & (g < 150)
    cyan = (b > 180) & (g > 180) & (r < 120)
    mask = np.zeros(image.shape[:2], np.uint8)
    for colour, pad in ((red, 6), (blue, 5), (cyan, 1)):
        count, components = cv2.connectedComponents(colour.astype(np.uint8))
        for i in range(1, count):
            if np.count_nonzero(components == i) >= 3:
                mask[cv2.dilate((components == i).astype(np.uint8),
                                np.ones((2 * pad + 1, 2 * pad + 1), np.uint8)) > 0] = 1
    # Merge overlapping pads, then take each group's bounding box.
    count, _, stats, _ = cv2.connectedComponentsWithStats(mask)
    return [tuple(s[:4]) for s in stats[1:]]


def paint_out(labels, box):
    """Continue the road and lines through an icon from the box's borders.

    Whatever crosses the box (a road edge, a line) runs either up-down or
    left-right, so copy the border pixels across in whichever direction the
    two opposite borders agree most.
    """
    h_max, w_max = labels.shape
    x, y, w, h = box
    x0, y0 = max(x - 1, 0), max(y - 1, 0)
    x1, y1 = min(x + w, w_max - 1), min(y + h, h_max - 1)
    top, bottom = labels[y0, x0:x1 + 1], labels[y1, x0:x1 + 1]
    left, right = labels[y0:y1 + 1, x0], labels[y0:y1 + 1, x1]
    inner = labels[y0 + 1:y1, x0 + 1:x1]
    rows, cols = np.mgrid[y0 + 1:y1, x0 + 1:x1]
    # Two floor borders agree trivially; what matters is the road and lines
    # that carry on through the box.
    def score(a, b):
        return np.mean(a == b) + np.mean((a == b) & (a != FLOOR))

    if score(top, bottom) >= score(left, right):
        nearer_top = (rows - y0) <= (y1 - rows)
        inner[:] = np.where(nearer_top, top[cols - x0], bottom[cols - x0])
    else:
        nearer_left = (cols - x0) <= (x1 - cols)
        inner[:] = np.where(nearer_left, left[rows - y0], right[rows - y0])


def draw_intersection_island(texture):
    """Redraw the island inside the intersection loop.

    The direction signs drawn on it cover almost all of it in the map. What
    shows is white round the top half and yellow round the bottom half, on
    both sides, changing at map row 149. That gives each branch of the loop
    the usual pair of lines: the island's white on the right of the upper
    branch, its yellow on the left of the lower one. Texture pixels.
    """
    left, right, top, bottom = 440, 570, 424, 820
    split = 622
    radius = (right - left) // 2
    centre_x = (left + right) // 2
    line = 14
    floor = COLOURS[FLOOR]
    cv2.rectangle(texture, (left, top + radius), (right, bottom - radius), floor, -1)
    cv2.circle(texture, (centre_x, top + radius), radius, floor, -1)
    cv2.circle(texture, (centre_x, bottom - radius), radius, floor, -1)
    white, yellow = COLOURS[WHITE], COLOURS[YELLOW]
    inset = line // 2
    cv2.ellipse(texture, (centre_x, top + radius), (radius - inset,) * 2,
                0, 180, 360, white, line)
    for x in (left + inset, right - inset):
        cv2.line(texture, (x, top + radius), (x, split), white, line)
        cv2.line(texture, (x, split), (x, bottom - radius), yellow, line)
    cv2.ellipse(texture, (centre_x, bottom - radius), (radius - inset,) * 2,
                0, 0, 180, yellow, line)


def main():
    image = cv2.imread(str(MAP))
    x0, x1 = BOARD_X
    y0, y1 = BOARD_Y
    board = image[y0:y1 + 1, x0:x1 + 1]

    labels = classify(board)
    for box in icon_boxes(board):
        labels[box[1]:box[1] + box[3], box[0]:box[0] + box[2]] = -1

    # White not touching the road is the background of an icon.
    road = cv2.dilate((labels == ROAD).astype(np.uint8), np.ones((5, 5), np.uint8))
    count, components = cv2.connectedComponents((labels == WHITE).astype(np.uint8))
    for i in range(1, count):
        blob = components == i
        if not road[blob].any():
            labels[blob] = -1

    for box in icon_boxes(board):
        paint_out(labels, box)

    # Anything still unknown (anti-aliasing) takes the nearest known class.
    unknown = (labels < 0).astype(np.uint8)
    _, nearest = cv2.distanceTransformWithLabels(
        unknown, cv2.DIST_L2, 5, labelType=cv2.DIST_LABEL_PIXEL)
    # Known pixels carry their own index in `nearest`.
    known = labels >= 0
    lookup = np.zeros(nearest.max() + 1, np.int8)
    lookup[nearest[known]] = labels[known]
    labels = lookup[nearest]

    # Leftover bits of drawing on the floor: one-pixel strokes and small
    # blobs such as the stop bar's housing.
    road = (labels == ROAD).astype(np.uint8)
    thin = road & ~cv2.morphologyEx(road, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    labels[thin > 0] = FLOOR
    count, components, stats, _ = cv2.connectedComponentsWithStats(
        (labels == ROAD).astype(np.uint8))
    for i in range(1, count):
        if stats[i, cv2.CC_STAT_AREA] < 200:
            labels[components == i] = FLOOR
    # Edge lines run along the road; white further out is an icon's border.
    off_road = cv2.distanceTransform((labels != ROAD).astype(np.uint8), cv2.DIST_L2, 3)
    labels[(labels == WHITE) & (off_road > 3)] = FLOOR

    # The stop line went with the icons; it covers the road between the lines.
    stop = labels[STOP_LINE_Y[0] - y0:STOP_LINE_Y[1] - y0 + 1,
                  STOP_LINE_X[0] - x0:STOP_LINE_X[1] - x0 + 1]
    stop[stop == ROAD] = STOP

    # Smooth upsampling: blur each class indicator and take the strongest.
    scores = []
    for k in range(len(COLOURS)):
        indicator = (labels == k).astype(np.float32)
        up = cv2.resize(indicator, (SIZE, SIZE), interpolation=cv2.INTER_LINEAR)
        scores.append(cv2.GaussianBlur(up, (0, 0), 2.0))
    # Lines are one or two map pixels wide; favour them so they survive.
    scores[YELLOW] *= 1.35
    scores[WHITE] *= 1.35
    scores[STOP] *= 1.35
    classes = np.argmax(np.stack(scores), axis=0)
    if WHITE_GROWTH:
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (2 * WHITE_GROWTH + 1, 2 * WHITE_GROWTH + 1))
        grown = cv2.dilate((classes == WHITE).astype(np.uint8), kernel) > 0
        classes[grown & ((classes == FLOOR) | (classes == ROAD))] = WHITE

    if LINE_WIDTH:
        # Favoured and grown so that they survive the upsampling, the lines
        # come out 4-5 cm wide. Tape is 3 cm: redraw each line that wide
        # along its centre line. Otherwise the zigzag lane, which the map
        # draws a little narrower than the rest, is narrower than the robot.
        disk = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (LINE_WIDTH, LINE_WIDTH))
        for k in (YELLOW, WHITE):
            mask = (classes == k).astype(np.uint8) * 255
            axis = cv2.ximgproc.thinning(mask)
            line = cv2.dilate(axis, disk) > 0
            classes[(classes == k) & ~line] = ROAD
    texture = np.zeros((SIZE, SIZE, 3), np.uint8)
    for k, colour in COLOURS.items():
        texture[classes == k] = colour
    draw_intersection_island(texture)
    texture = cv2.GaussianBlur(texture, (0, 0), 0.8)

    OUT.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(OUT), texture)
    print('wrote', OUT)


if __name__ == '__main__':
    main()
