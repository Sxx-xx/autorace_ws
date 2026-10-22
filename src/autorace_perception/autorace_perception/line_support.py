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

A bend is followed by its chords: at the course's tightest radius a chord of
5 cm strays from the arc by well under a millimetre, so short segments with
small gaps (`max_gap_m`) trace it.

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
    'min_length_m': 0.05,
    'max_gap_m': 0.02,
    'support_width_m': 0.06,
}


def declare(node):
    """Declare detect.hough.* on a detector node and return them as a dict."""
    for name, value in DEFAULTS.items():
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
    segments = cv2.HoughLinesP(edges, 1, np.pi / 180.0, int(params['threshold']),
                               minLineLength=max(3, int(params['min_length_m'] * ppm)),
                               maxLineGap=max(1, int(params['max_gap_m'] * ppm)))
    support = np.zeros(gray.shape, np.uint8)
    if segments is not None:
        width = max(1, int(params['support_width_m'] * ppm))
        for x1, y1, x2, y2 in segments[:, 0]:
            cv2.line(support, (int(x1), int(y1)), (int(x2), int(y2)), 255, width)
    return support
