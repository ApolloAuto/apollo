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


class SpeedDecider:
    def __init__(self, max_cruise_speed, enable_follow):
        self.CRUISE_SPEED = max_cruise_speed  # m/s
        self.enable_follow = enable_follow

    def get_target_speed_and_path_length(self, mobileye_provider,
                                         chassis_provider, path_length):
        obstacle_closest_lon = 999
        obstacle_speed = None
        obstacles = mobileye_provider.obstacles
        for obs in obstacles:
            if obs.lane == 1:
                if (obs.x - obs.length / 2.0) < obstacle_closest_lon:
                    obstacle_closest_lon = obs.x - obs.length / 2.0
                    obstacle_speed = obs.rel_speed + \
                        chassis_provider.get_speed_mps()

        new_path_length = path_length
        if obstacle_closest_lon < new_path_length:
            new_path_length = obstacle_closest_lon
        if obstacle_speed is None or obstacle_speed > self.CRUISE_SPEED:
            return self.CRUISE_SPEED, new_path_length
        else:
            return obstacle_speed, new_path_length

    def get(self, mobileye_provider, chassis_provider, path_length):
        if self.enable_follow:
            return self.get_target_speed_and_path_length(mobileye_provider,
                                                         chassis_provider,
                                                         path_length)
        else:
            return self.CRUISE_SPEED, path_length
