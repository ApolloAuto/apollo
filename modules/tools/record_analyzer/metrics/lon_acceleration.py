#!/usr/bin/env python

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

class LonAcceleration:
    def __init__(self):
        self.last_velocity = None
        self.last_timestamp = None
        self.last_acceleration = None
        self.acceleration = []
        self.deceleration = []
        self.acc_jerk = []
        self.dec_jerk = []

    def put(self, adc_trajectory):
        init_point = adc_trajectory.debug.planning_data.init_point
        current_timestamp = adc_trajectory.header.timestamp_sec + init_point.relative_time
        current_velocity = init_point.v

        if self.last_timestamp is not None and self.last_velocity is not None:
            duration = current_timestamp - self.last_timestamp
            if duration > 0.03:
                acc = (current_velocity - self.last_velocity) / duration
                if acc > 0:
                    self.acceleration.append(acc)
                elif acc < 0:
                    self.deceleration.append(acc)

        self.last_timestamp = current_timestamp
        self.last_velocity = current_velocity
