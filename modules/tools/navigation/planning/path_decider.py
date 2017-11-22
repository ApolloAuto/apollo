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
import math


class PathDecider:
    def __init__(self):
        self.MINIMUM_PATH_LENGTH = 5
        self.MAX_LAT_CHANGE = 0.1
        self.last_init_lat = None

    def get_path_length(self, speed_mps):
        path_length = self.MINIMUM_PATH_LENGTH
        current_speed = speed_mps
        if current_speed is not None:
            if path_length < current_speed * 2:
                path_length = math.ceil(current_speed * 2)
        return path_length

    def get_reference_line_offset(self, current_init_lat):
        if self.last_init_lat is None:
            return 0
        if abs(current_init_lat - self.last_init_lat) < self.MAX_LAT_CHANGE:
            return 0
        else:
            if current_init_lat > self.last_init_lat:
                return - (abs(current_init_lat - self.last_init_lat) -
                          self.MAX_LAT_CHANGE)
            else:
                return abs(
                    current_init_lat - self.last_init_lat) - self.MAX_LAT_CHANGE

    def get_path(self, left_marker_coef, right_marker_coef, speed_mps):
        path_length = self.get_path_length(speed_mps)
        offset = self.get_reference_line_offset(
            (right_marker_coef[0] +
             left_marker_coef[0]) / 2.0)
        path_coef = [0, 0, 0, 0]

        path_coef[0] = ((right_marker_coef[0] +
                         left_marker_coef[0]) / 2.0) + offset
        for i in range(1, 4):
            path_coef[i] = (right_marker_coef[i] +
                            left_marker_coef[i]) / 2.0

        self.last_init_lat = path_coef[0]
        return path_coef, path_length
