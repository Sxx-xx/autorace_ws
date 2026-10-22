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

"""The road the robot stands on, segmented in the bird's eye view; lines are what bounds it.

The robot drives on the dark surface between its two lines. That surface is
one even colour, and the robot is on it: the patch just ahead of the axle
(the seed) says what it looks like in this picture, whatever the lights and
whether the course is the contest's black mat or the practice floor (L 131).
The road is the pixels like the seed, dark-ish and grey (within `value_tol` of
its brightness, saturation below `saturation_max`), joined to the seed. A
dark thing standing apart (a shadow, a box beside the course) is not joined
to it and stays out.

The lane lines are what bounds that region, so a pixel of a line's colour
counts only within `line_margin_m` of the road. Glare, tape and specks of the
line colours off the road fall away, as the colour masks cannot tell them.

Classical segmentation, not a trained network: the course has one kind of
road, and a network needs training data, a model and more time than the
30 Hz the lane follower runs at.

If no road is found next to the seed (the robot over a line, a crossing in
front) the region is not used for that picture, so the lines are not lost.

Parameters, under detect.road (perception_*.yaml), enable false to leave it out:
  enable, seed_ahead_m (depth of the seed patch from the bottom of the view),
  seed_half_width_m, value_tol, saturation_max, line_margin_m,
  min_area_m2 (less road than this is not trusted)
"""

import cv2
import numpy as np

DEFAULTS = {
    'enable': True,
    'seed_ahead_m': 0.04,
    'seed_half_width_m': 0.06,
    'value_tol': 35,
    'saturation_max': 90,
    'line_margin_m': 0.06,
    'min_area_m2': 0.01,
}


def declare(node, **defaults):
    """Declare detect.road.* on a detector node (its own defaults over DEFAULTS); return them."""
    for name, value in {**DEFAULTS, **defaults}.items():
        node.declare_parameter(f'detect.road.{name}', value)
    return {name: node.get_parameter(f'detect.road.{name}').value for name in DEFAULTS}


def road_region(hsv, ppm, params):
    """(road mask, band near it) of a bird's eye view (HSV), 0/255 each; (None, None) if unused."""
    if not params['enable']:
        return None, None
    h, w = hsv.shape[:2]
    value = hsv[..., 2]
    outside = cv2.inRange(hsv, (0, 0, 0), (0, 0, 0))       # beyond the camera's view
    seed_h = max(2, int(params['seed_ahead_m'] * ppm))
    seed_w = max(2, int(params['seed_half_width_m'] * ppm))
    seed = np.zeros((h, w), np.uint8)
    seed[h - seed_h:, w // 2 - seed_w:w // 2 + seed_w] = 255
    seed &= ~outside
    seed_values = value[seed > 0]
    if seed_values.size < 20:
        return None, None
    level = float(np.median(seed_values))
    tol = params['value_tol']
    like = cv2.inRange(hsv, (0, 0, max(1, level - tol)),
                       (179, params['saturation_max'], min(255, level + tol)))
    like &= ~outside
    like = cv2.morphologyEx(like, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, labels, stats, _ = cv2.connectedComponentsWithStats(like, connectivity=4)
    if n <= 1:
        return None, None
    # The region the seed lies in: the label most of the seed's pixels carry.
    seed_labels = labels[seed > 0]
    seed_labels = seed_labels[seed_labels > 0]
    if seed_labels.size == 0:
        return None, None
    label = int(np.bincount(seed_labels).argmax())
    if stats[label, cv2.CC_STAT_AREA] < params['min_area_m2'] * ppm * ppm:
        return None, None
    road = np.where(labels == label, 255, 0).astype(np.uint8)
    # Close the gaps a line's own shadow or a stop line's crossing leaves along the edge.
    road = cv2.morphologyEx(road, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    # Out from the road's edge as far as a line's width and some: 2 * margin + 1 across.
    margin = max(1, int(params['line_margin_m'] * ppm))
    band = cv2.dilate(road, cv2.getStructuringElement(cv2.MORPH_RECT, (2 * margin + 1,) * 2))
    return road, band
