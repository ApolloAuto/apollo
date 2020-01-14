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


class LonAcceleration:
    def __init__(self):
        self.last_velocity = None
        self.last_velocity_timestamp = None
        self.last_acceleration = None
        self.last_acceleration_timestamp = None
        self.acceleration_list = []
        self.deceleration_list = []
        self.acc_jerk_list = []
        self.dec_jerk_list = []

    def put(self, adc_trajectory):
        init_point = adc_trajectory.debug.planning_data.init_point
        current_velocity_timestamp = adc_trajectory.header.timestamp_sec + \
            init_point.relative_time
        current_velocity = init_point.v

        if self.last_velocity_timestamp is not None and self.last_velocity is not None:
            # acceleration
            duration = current_velocity_timestamp - self.last_velocity_timestamp
            if duration > 0.03:
                current_acceleration = (
                    current_velocity - self.last_velocity) / duration
                if current_acceleration > 0 and not math.isnan(current_acceleration):
                    self.acceleration_list.append(current_acceleration)
                elif current_acceleration < 0 and not math.isnan(current_acceleration):
                    self.deceleration_list.append(current_acceleration)

                if self.last_acceleration is not None and self.last_acceleration_timestamp is not None:
                    # jerk
                    acc_duration = current_velocity_timestamp - self.last_acceleration_timestamp
                    if acc_duration > 0.03:
                        current_jerk = (current_acceleration -
                                        self.last_acceleration) / acc_duration
                        if current_acceleration > 0 and not math.isnan(current_jerk):
                            self.acc_jerk_list.append(current_jerk)
                        elif current_acceleration < 0 and not math.isnan(current_jerk):
                            self.dec_jerk_list.append(current_jerk)

                self.last_acceleration = current_acceleration
                self.last_acceleration_timestamp = current_velocity_timestamp

        self.last_velocity_timestamp = current_velocity_timestamp
        self.last_velocity = current_velocity

    def get_acceleration(self):
        # [2, 4) unit m/s^2
        ACCEL_M_LB = 2
        ACCEL_M_UB = 4
        accel_medium_cnt = 0

        # [4, ) unit m/s^2
        ACCEL_H_LB = 4
        accel_high_cnt = 0

        lon_acceleration = {}
        for accel in self.acceleration_list:
            if ACCEL_M_LB <= accel < ACCEL_M_UB:
                accel_medium_cnt += 1
            if ACCEL_H_LB <= accel:
                accel_high_cnt += 1

        if len(self.acceleration_list) > 0:
            lon_acceleration["max"] = max(self.acceleration_list)
            lon_acceleration["avg"] = np.average(self.acceleration_list)
        else:
            lon_acceleration["max"] = 0.0
            lon_acceleration["avg"] = 0.0
        lon_acceleration["medium_cnt"] = accel_medium_cnt
        lon_acceleration["high_cnt"] = accel_high_cnt

        return lon_acceleration

    def get_deceleration(self):
        # [-4, -2)
        DECEL_M_LB = -4
        DECEL_M_UB = -2
        decel_medium_cnt = 0

        # [-4, )
        DECEL_H_UB = -4
        decel_high_cnt = 0

        for accel in self.deceleration_list:
            if DECEL_M_LB < accel <= DECEL_M_UB:
                decel_medium_cnt += 1
            if accel <= DECEL_H_UB:
                decel_high_cnt += 1

        lon_deceleration = {}
        if len(self.deceleration_list) > 0:
            lon_deceleration["max"] = abs(max(self.deceleration_list, key=abs))
            lon_deceleration["avg"] = np.average(
                np.absolute(self.deceleration_list))
        else:
            lon_deceleration["max"] = 0.0
            lon_deceleration["avg"] = 0.0
        lon_deceleration["medium_cnt"] = decel_medium_cnt
        lon_deceleration["high_cnt"] = decel_high_cnt

        return lon_deceleration

    def get_acc_jerk(self):
        # [1,2) (-2, -1]
        JERK_M_LB_P = 1
        JERK_M_UB_P = 2
        JERK_M_LB_N = -2
        JERK_M_UB_N = -1
        jerk_medium_cnt = 0

        # [2, inf) (-inf, -2]
        JERK_H_LB_P = 2
        JERK_H_UB_N = -2
        jerk_high_cnt = 0

        for jerk in self.acc_jerk_list:
            if JERK_M_LB_P <= jerk < JERK_M_UB_P or \
                    JERK_M_LB_N < jerk <= JERK_M_UB_N:
                jerk_medium_cnt += 1
            if jerk >= JERK_H_LB_P or jerk <= JERK_H_UB_N:
                jerk_high_cnt += 1

        lon_acc_jerk = {}
        if len(self.acc_jerk_list) > 0:
            lon_acc_jerk["max"] = abs(max(self.acc_jerk_list, key=abs))
            jerk_avg = np.average(np.absolute(self.acc_jerk_list))
            lon_acc_jerk["avg"] = jerk_avg
        else:
            lon_acc_jerk["max"] = 0
            lon_acc_jerk["avg"] = 0
        lon_acc_jerk["medium_cnt"] = jerk_medium_cnt
        lon_acc_jerk["high_cnt"] = jerk_high_cnt

        return lon_acc_jerk

    def get_dec_jerk(self):
        # [1,2) (-2, -1]
        JERK_M_LB_P = 1
        JERK_M_UB_P = 2
        JERK_M_LB_N = -2
        JERK_M_UB_N = -1
        jerk_medium_cnt = 0

        # [2, inf) (-inf, -2]
        JERK_H_LB_P = 2
        JERK_H_UB_N = -2
        jerk_high_cnt = 0

        for jerk in self.dec_jerk_list:
            if JERK_M_LB_P <= jerk < JERK_M_UB_P or \
                    JERK_M_LB_N < jerk <= JERK_M_UB_N:
                jerk_medium_cnt += 1
            if jerk >= JERK_H_LB_P or jerk <= JERK_H_UB_N:
                jerk_high_cnt += 1

        lon_dec_jerk = {}
        if len(self.dec_jerk_list) > 0:
            lon_dec_jerk["max"] = abs(max(self.dec_jerk_list, key=abs))
            jerk_avg = np.average(np.absolute(self.dec_jerk_list))
            lon_dec_jerk["avg"] = jerk_avg
        else:
            lon_dec_jerk["max"] = 0
            lon_dec_jerk["avg"] = 0
        lon_dec_jerk["medium_cnt"] = jerk_medium_cnt
        lon_dec_jerk["high_cnt"] = jerk_high_cnt

        return lon_dec_jerk
