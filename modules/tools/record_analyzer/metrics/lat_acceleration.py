#!/usr/bin/env python3

###############################################################################
# Copyright 2019 The Apollo Authors. All Rights Reserved.
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

import numpy as np


class LatAcceleration:
    def __init__(self):
        self.centripetal_accel_list = []
        self.centripetal_jerk_list = []

    def put(self, adc_trajectory):
        init_point = adc_trajectory.debug.planning_data.init_point
        # centripetal_jerk
        centripetal_jerk = 2 * init_point.v * init_point.a \
            * init_point.path_point.kappa + init_point.v \
            * init_point.v * init_point.path_point.dkappa
        if not math.isnan(centripetal_jerk):
            self.centripetal_jerk_list.append(centripetal_jerk)

        # centripetal_accel
        centripetal_accel = init_point.v * init_point.v \
            * init_point.path_point.kappa

        if not math.isnan(centripetal_accel):
            self.centripetal_accel_list.append(centripetal_accel)

    def get_acceleration(self):
        # [1, 2) [-2, -1)
        LAT_ACCEL_M_LB_P = 1
        LAT_ACCEL_M_UB_P = 2
        LAT_ACCEL_M_LB_N = -2
        LAT_ACCEL_M_UB_N = -1
        lat_accel_medium_cnt = 0

        # [2, inf)  [-inf,-2)
        LAT_ACCEL_H_LB_P = 2
        LAT_ACCEL_H_UB_N = -2
        lat_accel_high_cnt = 0

        for centripetal_accel in self.centripetal_accel_list:
            if LAT_ACCEL_M_LB_P <= centripetal_accel < LAT_ACCEL_M_UB_P \
                    or LAT_ACCEL_M_LB_N < centripetal_accel <= LAT_ACCEL_M_UB_N:
                lat_accel_medium_cnt += 1
            if centripetal_accel >= LAT_ACCEL_H_LB_P \
                    or centripetal_accel <= LAT_ACCEL_H_UB_N:
                lat_accel_high_cnt += 1

        # centripetal_accel
        lat_accel = {}
        if len(self.centripetal_accel_list) > 0:
            lat_accel["max"] = abs(max(self.centripetal_accel_list, key=abs))
            accel_avg = np.average(np.absolute(self.centripetal_accel_list))
            lat_accel["avg"] = accel_avg
        else:
            lat_accel["max"] = 0
            lat_accel["avg"] = 0
        lat_accel["medium_cnt"] = lat_accel_medium_cnt
        lat_accel["high_cnt"] = lat_accel_high_cnt

        return lat_accel

    def get_jerk(self):
        # [0.5,1) [-1, -0.5)
        LAT_JERK_M_LB_P = 0.5
        LAT_JERK_M_UB_P = 1
        LAT_JERK_M_LB_N = -1
        LAT_JERK_M_UB_N = -0.5
        lat_jerk_medium_cnt = 0

        # [1, inf)  [-inf,-1)
        LAT_JERK_H_LB_P = 1
        LAT_JERK_H_UB_N = -1
        lat_jerk_high_cnt = 0

        for centripetal_jerk in self.centripetal_jerk_list:
            if LAT_JERK_M_LB_P <= centripetal_jerk < LAT_JERK_M_UB_P \
                    or LAT_JERK_M_LB_N < centripetal_jerk <= LAT_JERK_M_UB_N:
                lat_jerk_medium_cnt += 1
            if centripetal_jerk >= LAT_JERK_H_LB_P \
                    or centripetal_jerk <= LAT_JERK_H_UB_N:
                lat_jerk_high_cnt += 1

        # centripetal_jerk
        lat_jerk = {}
        if len(self.centripetal_jerk_list) > 0:
            lat_jerk["max"] = abs(max(self.centripetal_jerk_list, key=abs))
            jerk_avg = np.average(np.absolute(self.centripetal_jerk_list))
            lat_jerk["avg"] = jerk_avg
        else:
            lat_jerk["max"] = 0
            lat_jerk["avg"] = 0
        lat_jerk["medium_cnt"] = lat_jerk_medium_cnt
        lat_jerk["high_cnt"] = lat_jerk_high_cnt

        return lat_jerk
