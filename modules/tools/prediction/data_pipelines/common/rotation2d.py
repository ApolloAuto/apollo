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


from math import cos, sin
from .vector2d import Vector2


def rotate(v, theta):
    cos_theta = cos(theta)
    sin_theta = sin(theta)

    return rotate_fast(v, cos_theta, sin_theta)


def rotate_fast(v, cos_theta, sin_theta):
    x = cos_theta * v.x - sin_theta * v.y
    y = sin_theta * v.x + cos_theta * v.y

    return Vector2(x, y)
