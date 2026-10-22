#!/usr/bin/env python3
#
# Copyright 2026 AutoRace Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Where the bird's eye view has a painted line: grayscale -> blur -> Canny -> Hough.

The colour masks of the lane detectors take anything of the line's colour:
glare, a light floor tile, a sticker, specks. A painted line is also a long,
straight-sided strip, which none of those is. Its two sides are edges (Canny)
lying along straight segments (probabilistic Hough) at least `min_length_m`
long; drawn back `support_width_m` thick they cover the strip between them.
The detectors keep the colour only where it lies on such a segment.

A bend is followed by its chords: at the course's tightest radius (about
0.2 m) a 5 cm chord strays 1.6 mm (2 px) from the arc, which a Hough
distance step `rho_px` of 2 takes as one line; a blob's edge (radius of a few
cm) bends ten times as much and stays out. With rho 1 such bends were cut and
the target missed by 3-6 cm (synthetic views, 2026-10-23).

Each detector has its own defaults (declare(node, **defaults)), found on
synthetic noisy views: detect_lane rho 2, threshold 25; detect_lane_sw rho 2,
threshold 20. A shorter min_length_m keeps bends too but lets blob edges in.

Parameters, under detect.hough (perception_*.yaml), with enable false to go
back to colour alone:
  enable, blur (kernel px), canny_low, canny_high, threshold (votes),
  min_length_m, max_gap_m, support_width_m
"""

import cv2
import numpy as np

DEFAULTS = {
    'enable': True,
    'blur': 5,
    'canny_low': 40,
    'canny_high': 120,
    'threshold': 20,
    'rho_px': 1,
    'min_length_m': 0.05,
    'max_gap_m': 0.02,
    'support_width_m': 0.06,
}


def declare(node, **defaults):
    """Declare detect.hough.* on a detector node (its own defaults over DEFAULTS); return them."""
    for name, value in {**DEFAULTS, **defaults}.items():
        node.declare_parameter(f'detect.hough.{name}', value)
    return {name: node.get_parameter(f'detect.hough.{name}').value for name in DEFAULTS}


def line_support(image, ppm, params):
    """Mask (0/255) of the bird's eye view near long straight edges, or None when disabled.

    `image` is the BGR bird's eye view, `ppm` its pixels per metre.
    """
    if not params['enable']:
        return None
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    k = int(params['blur']) | 1
    if k > 1:
        gray = cv2.GaussianBlur(gray, (k, k), 0)
    edges = cv2.Canny(gray, params['canny_low'], params['canny_high'])
    segments = cv2.HoughLinesP(edges, float(params['rho_px']), np.pi / 180.0,
                               int(params['threshold']),
                               minLineLength=max(3, int(params['min_length_m'] * ppm)),
                               maxLineGap=max(1, int(params['max_gap_m'] * ppm)))
    support = np.zeros(gray.shape, np.uint8)
    if segments is not None:
        width = max(1, int(params['support_width_m'] * ppm))
        for x1, y1, x2, y2 in segments[:, 0]:
            cv2.line(support, (int(x1), int(y1)), (int(x2), int(y2)), 255, width)
    return support
