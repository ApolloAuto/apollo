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

from reference_path import ReferencePath


class PathDecider:
    def __init__(self, enable_routing_aid, enable_nudge, enable_change_lane):
        self.MINIMUM_PATH_LENGTH = 5
        self.MAX_LAT_CHANGE = 0.1
        self.last_init_lat = None
        self.ref = ReferencePath()
        self.enable_routing_aid = enable_routing_aid
        self.enable_nudge = enable_nudge
        self.enable_change_lane = enable_change_lane

    def get_path_by_lm(self, mobileye, chassis):
        return self.ref.get_ref_path_by_lm(mobileye, chassis)

    def get_path_by_lmr(self, perception, routing,
                        localization, chassis):
        path_x, path_y, path_len = self.ref.get_ref_path_by_lmr(perception,
                                                                routing,
                                                                localization,
                                                                chassis)
        if self.enable_nudge:
            path_x, path_y, path_len = self.nudge_process(path_x, path_y,
                                                          path_len)
        return path_x, path_y, path_len

    def nudge_process(self, path_x, path_y, path_len):
        return path_x, path_y, path_len

    def get(self, perception, routing, localization, chassis):
        if self.enable_routing_aid:
            return self.get_path_by_lmr(perception, routing, localization,
                                        chassis)
        else:
            return self.get_path_by_lm(perception, chassis)
