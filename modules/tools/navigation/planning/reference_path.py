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
from numpy.polynomial.polynomial import polyval


class ReferencePath:
    def __init__(self):
        self.MINIMUM_PATH_LENGTH = 5
        self.MAX_LAT_CHANGE = 0.1
        self.init_y_last = None

    def get_path_length(self, speed_mps):
        path_length = self.MINIMUM_PATH_LENGTH
        current_speed = speed_mps
        if current_speed is not None:
            if path_length < current_speed * 2:
                path_length = math.ceil(current_speed * 2)
        return path_length

    def get_ref_path_init_y(self, init_y_perception):
        if self.init_y_last is None:
            return 0
        if abs(init_y_perception - self.init_y_last) < self.MAX_LAT_CHANGE:
            return init_y_perception
        else:
            if init_y_perception > self.init_y_last:
                return self.init_y_last + self.MAX_LAT_CHANGE
            else:
                return self.init_y_last - self.MAX_LAT_CHANGE

    def get_ref_path_by_lm(self, perception, chassis):
        path_length = self.get_path_length(chassis.get_speed_mps())
        init_y_perception = (perception.right_lm_coef[0] +
                             perception.left_lm_coef[0]) / -2.0
        init_y = self.get_ref_path_init_y(init_y_perception)
        self.init_y_last = init_y
        path_x, path_y = self._get_perception_ref_path(
            perception, path_length, init_y)
        return path_x, path_y, path_length

    def _get_perception_ref_path(self, perception, path_length, init_y):
        path_coef = [0, 0, 0, 0]

        path_coef[0] = -1 * init_y
        quality = perception.right_lm_quality + perception.left_lm_quality
        if quality > 0:
            for i in range(1, 4):
                path_coef[i] = (perception.right_lm_coef[i] *
                                perception.right_lm_quality +
                                perception.left_lm_coef[i] *
                                perception.left_lm_quality) / quality
        path_x = []
        path_y = []
        for x in range(int(path_length)):
            y = -1 * polyval(x, path_coef)
            path_x.append(x)
            path_y.append(y)
        return path_x, path_y

    def get_ref_path_by_lmr(self, perception, routing, adv):

        path_length = self.get_path_length(adv.speed_mps)

        rpath_x, rpath_y = routing.get_local_segment_spline(adv.x,
                                                            adv.y,
                                                            adv.heading)
        init_y_perception = (perception.right_lm_coef[0] +
                             perception.left_lm_coef[0]) / -2.0
        quality = perception.right_lm_quality + perception.left_lm_quality
        quality = quality / 2.0

        if len(rpath_x) >= path_length and routing.human and rpath_y[0] <= 3:
            init_y_routing = rpath_y[0]
            init_y = self.get_ref_path_init_y(init_y_routing)
            if quality > 0.1:
                quality = 0.1
            self.init_y_last = init_y
        else:
            init_y = self.get_ref_path_init_y(init_y_perception)
            self.init_y_last = init_y

        lmpath_x, lmpath_y = self._get_perception_ref_path(
            perception, path_length, init_y)

        if len(rpath_x) < path_length:
            return lmpath_x, lmpath_y, path_length

        routing_shift = rpath_y[0] - init_y
        path_x = []
        path_y = []
        for i in range(int(path_length)):
            # TODO(yifei): more accurate shift is needed.
            y = (lmpath_y[i] * quality + rpath_y[i] - routing_shift) / (
                1 + quality)
            path_x.append(i)
            path_y.append(y)

        return path_x, path_y, path_length

    def shift_point(self, p, p2, distance):
        delta_y = p2.y - p.y
        delta_x = p2.x - p.x
        angle = 0
        if distance >= 0:
            angle = math.atan2(delta_y, delta_x) + math.pi / 2.0
        else:
            angle = math.atan2(delta_y, delta_x) - math.pi / 2.0
        p1n = []
        p1n.append(p.x + (math.cos(angle) * distance))
        p1n.append(p.y + (math.sin(angle) * distance))

        p2n = []
        p2n.append(p2.x + (math.cos(angle) * distance))
        p2n.append(p2.y + (math.sin(angle) * distance))
        return p1n, p2n
