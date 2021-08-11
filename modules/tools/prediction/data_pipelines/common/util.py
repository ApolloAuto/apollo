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


def segment_overlap(a, b, x, y):
    if b < x or a > y:
        return False

    return True


def vector_projection_overlap(p0, p1, p2, p3):
    v = p1.subtract(p0)
    n_square = v.norm_square()

    v0 = p2.subtract(p0)
    v1 = p3.subtract(p0)

    t0 = v0.dot(v)
    t1 = v1.dot(v)

    if t0 > t1:
        t = t0
        t0 = t1
        t1 = t

    return segment_overlap(t0, t1, 0.0, n_square)
