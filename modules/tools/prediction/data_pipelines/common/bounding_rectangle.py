#!/usr/bin/env python3

###############################################################################
# Copyright 2018 The Apollo Authors. All Rights Reserved.
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


from .vector2d import Vector2
from .rotation2d import *
from .util import segment_overlap


class BoundingRectangle:
    def __init__(self, x, y, theta, length, width):

        self.vertices = [None] * 4
        dx = 0.5 * length
        dy = 0.5 * width

        cos_theta = cos(theta)
        sin_theta = sin(theta)

        self.vertices[0] = rotate_fast(Vector2(dx, -dy), cos_theta, sin_theta)
        self.vertices[1] = rotate_fast(Vector2(dx, dy), cos_theta, sin_theta)
        self.vertices[2] = rotate_fast(Vector2(-dx, dy), cos_theta, sin_theta)
        self.vertices[3] = rotate_fast(Vector2(-dx, -dy), cos_theta, sin_theta)

        for i in range(4):
            self.vertices[i].x += x
            self.vertices[i].y += y

    def overlap(self, rect):
        for i in range(4):
            v0 = self.vertices[i]
            v1 = self.vertices[(i + 1) % 4]

            range_self = self.project(v0, v1)
            range_other = rect.project(v0, v1)

            if segment_overlap(range_self[0], range_self[1], range_other[0], range_other[1]) == False:
                return False

        for i in range(4):
            v0 = rect.vertices[i]
            v1 = rect.vertices[(i + 1) % 4]

            range_self = self.project(v0, v1)
            range_other = rect.project(v0, v1)

            if segment_overlap(range_self[0], range_self[1], range_other[0], range_other[1]) == False:
                return False

        return True

    def project(self, p0, p1):
        v = p1.subtract(p0)
        n = v.norm()

        rmin = float("inf")
        rmax = float("-inf")

        for i in range(4):
            t = self.vertices[i].subtract(p0)
            r = t.dot(v) / n

            if r < rmin:
                rmin = r

            if r > rmax:
                rmax = r

        return [rmin, rmax]

    def print_vertices(self):
        for i in range(4):
            print(str(self.vertices[i].x) + "\t" +
                  str(self.vertices[i].y) + "\n")
