#!/usr/bin/env python

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

import numpy
import math

_NEXT_AXIS = [1, 2, 0, 1]
_EPS = numpy.finfo(float).eps * 4.0


def _quaternion_matrix(quaternion):
    q = numpy.array(quaternion, dtype=numpy.float64, copy=True)
    n = numpy.dot(q, q)
    if n < _EPS:
        return numpy.identity(4)
    q *= math.sqrt(2.0 / n)
    q = numpy.outer(q, q)
    return numpy.array(
        [[1.0 - q[2, 2] - q[3, 3], q[1, 2] - q[3, 0], q[1, 3] + q[2, 0], 0.0],
         [q[1, 2] + q[3, 0], 1.0 - q[1, 1] - q[3, 3], q[2, 3] - q[1, 0], 0.0],
         [q[1, 3] - q[2, 0], q[2, 3] + q[1, 0], 1.0 - q[1, 1] - q[2, 2],
          0.0], [0.0, 0.0, 0.0, 1.0]])


def _euler_from_matrix(matrix):
    firstaxis = 0
    parity = 0
    repetition = 0
    frame = 0

    i = firstaxis
    j = _NEXT_AXIS[i + parity]
    k = _NEXT_AXIS[i - parity + 1]

    M = numpy.array(matrix, dtype=numpy.float64, copy=False)[:3, 0:3]
    if repetition:
        sy = math.sqrt(M[i, j] * M[i, j] + M[i, k] * M[i, k])
        if sy > _EPS:
            ax = math.atan2(M[i, j], M[i, k])
            ay = math.atan2(sy, M[i, i])
            az = math.atan2(M[j, i], -M[k, i])
        else:
            ax = math.atan2(-M[j, k], M[j, j])
            ay = math.atan2(sy, M[i, i])
            az = 0.0
    else:
        cy = math.sqrt(M[i, i] * M[i, i] + M[j, i] * M[j, i])
        if cy > _EPS:
            ax = math.atan2(M[k, j], M[k, k])
            ay = math.atan2(-M[k, i], cy)
            az = math.atan2(M[j, i], M[i, i])
        else:
            ax = math.atan2(-M[j, k], M[j, j])
            ay = math.atan2(-M[k, i], cy)
            az = 0.0

    if parity:
        ax, ay, az = -ax, -ay, -az
    if frame:
        ax, az = az, ax
    return ax, ay, az


def get(position, quaternion):
    """
    :param position: [x, y, z]
    :param quaternion: [x, y, z, w]
    :return:
    """
    front_edge_to_center = 3.89
    back_edge_to_center = 1.043
    left_edge_to_center = 1.055
    right_edge_to_center = 1.055

    x, y, z = _euler_from_matrix(_quaternion_matrix(quaternion))
    heading = x + math.pi / 2.0

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
