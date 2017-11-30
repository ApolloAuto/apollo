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
    def __init__(self):
        self.MINIMUM_PATH_LENGTH = 5
        self.MAX_LAT_CHANGE = 0.1
        self.last_init_lat = None
        self.ref = ReferencePath()

    def get_path_by_lm(self, mobileye, chassis):
        return self.ref.get_ref_path_by_lm(mobileye, chassis)

    def get_path_by_lmr(self, perception, routing, localization, chassis):
        return self.ref.get_ref_path_by_lmr(perception,
                                            routing,
                                            localization,
                                            chassis)
