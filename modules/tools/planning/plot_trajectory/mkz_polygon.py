#!/usr/bin/env python3

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

import math


def get(position, heading):

    front_edge_to_center = 3.89
    back_edge_to_center = 1.043
    left_edge_to_center = 1.055
    right_edge_to_center = 1.055

    cos_h = math.cos(heading)
    sin_h = math.sin(heading)
    #  (p3)  -------- (p0)
    #        | o     |
    #   (p2) -------- (p1)
    p0_x, p0_y = front_edge_to_center, left_edge_to_center
    p1_x, p1_y = front_edge_to_center, -right_edge_to_center
    p2_x, p2_y = -back_edge_to_center, -left_edge_to_center
    p3_x, p3_y = -back_edge_to_center, right_edge_to_center

    p0_x, p0_y = p0_x * cos_h - p0_y * sin_h, p0_x * sin_h + p0_y * cos_h
    p1_x, p1_y = p1_x * cos_h - p1_y * sin_h, p1_x * sin_h + p1_y * cos_h
    p2_x, p2_y = p2_x * cos_h - p2_y * sin_h, p2_x * sin_h + p2_y * cos_h
    p3_x, p3_y = p3_x * cos_h - p3_y * sin_h, p3_x * sin_h + p3_y * cos_h

    [x, y, z] = position
    polygon = []
    polygon.append([p0_x + x, p0_y + y, 0])
    polygon.append([p1_x + x, p1_y + y, 0])
    polygon.append([p2_x + x, p2_y + y, 0])
    polygon.append([p3_x + x, p3_y + y, 0])
    return polygon


def plot(position, quaternion, ax):
    polygon = get(position, quaternion)
    px = []
    py = []
    for point in polygon:
        px.append(point[0])
        py.append(point[1])
    point = polygon[0]
    px.append(point[0])
    py.append(point[1])
    ax.plot(px, py, "g-")
    ax.plot([position[0]], [position[1]], 'go')
