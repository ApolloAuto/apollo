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
from numpy.polynomial.polynomial import polyval


class PathDecider:
    def __init__(self):
        self.MINIMUM_PATH_LENGTH = 5
        self.MAX_LAT_CHANGE = 0.1
        self.mobileye_pb = None
        self.chassis_pb = None
        self.left_lane_marker_coef = []
        self.right_lane_marker_coef = []
        self.last_init_lat = None

    def update_mobileye_pb(self, mobileye_pb):
        self.mobileye_pb = mobileye_pb

        rc0 = mobileye_pb.lka_768.position
        rc1 = mobileye_pb.lka_769.heading_angle
        rc2 = mobileye_pb.lka_768.curvature
        rc3 = mobileye_pb.lka_768.curvature_derivative
        # rrangex = mobileye_pb.lka_769.view_range
        self.right_lane_marker_coef = [rc0, rc1, rc2, rc3]

        lc0 = mobileye_pb.lka_766.position
        lc1 = mobileye_pb.lka_767.heading_angle
        lc2 = mobileye_pb.lka_766.curvature
        lc3 = mobileye_pb.lka_766.curvature_derivative
        # lrangex = mobileye_pb.lka_767.view_range
        self.left_lane_marker_coef = [lc0, lc1, lc2, lc3]

    def update_chassis_pb(self, chassis_pb):
        self.chassis_pb = chassis_pb

    def get_path_length(self):
        path_length = self.MINIMUM_PATH_LENGTH
        if self.chassis_pb is not None:
            speed = self.chassis_pb.speed_mps
            if path_length < speed * 2:
                path_length = math.ceil(speed * 2)
        return path_length

    def get_reference_line_offset(self, current_init_lat):
        if self.last_init_lat is None:
            return 0
        if abs(current_init_lat - self.last_init_lat) < self.MAX_LAT_CHANGE:
            return 0
        else:
            if current_init_lat > self.last_init_lat:
                return - (
                    abs(
                        current_init_lat - self.last_init_lat) - self.MAX_LAT_CHANGE)
            else:
                return abs(
                    current_init_lat - self.last_init_lat) - self.MAX_LAT_CHANGE

    def get_path(self):
        path_length = self.get_path_length()
        offset = self.get_reference_line_offset(
            (self.right_lane_marker_coef[0] +
             self.left_lane_marker_coef[0]) / 2.0)
        ref_lane_x = []
        ref_lane_y = []
        for y in range(int(path_length)):
            rx = polyval(y, self.right_lane_marker_coef)
            lx = polyval(y, self.left_lane_marker_coef)
            ref_lane_x.append((rx + lx) / 2.0 + offset)
            ref_lane_y.append(y)

        self.last_init_lat = ref_lane_x[0]
        return ref_lane_x, ref_lane_y
