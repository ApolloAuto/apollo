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

import sys

import numpy as np


class ReferenceLine:

    def __init__(self):
        self.rl_is_offroad_cnt = 0
        self.rl_minimum_boundary = sys.float_info.max
        self.rl_kappa_rms_list = []
        self.rl_dkappa_rms_list = []
        self.rl_kappa_max_abs_list = []
        self.rl_dkappa_max_abs_list = []

    def put(self, adc_trajectory):
        for ref_line_debug in adc_trajectory.debug.planning_data.reference_line:
            if ref_line_debug.HasField("is_offroad") and ref_line_debug.is_offroad:
                self.rl_is_offroad_cnt += 1
            if ref_line_debug.HasField("minimum_boundary") and \
                    ref_line_debug.minimum_boundary < self.rl_minimum_boundary:
                self.rl_minimum_boundary = ref_line_debug.minimum_boundary
            if ref_line_debug.HasField("kappa_rms"):
                self.rl_kappa_rms_list.append(ref_line_debug.kappa_rms)
            if ref_line_debug.HasField("dkappa_rms"):
                self.rl_dkappa_rms_list.append(ref_line_debug.dkappa_rms)
            if ref_line_debug.HasField("kappa_max_abs"):
                self.rl_kappa_max_abs_list.append(ref_line_debug.kappa_max_abs)
            if ref_line_debug.HasField("dkappa_max_abs"):
                self.rl_dkappa_max_abs_list.append(ref_line_debug.dkappa_max_abs)

    def get(self):
        kappa_rms = 0
        if len(self.rl_kappa_rms_list) > 0:
            kappa_rms = np.average(self.rl_kappa_rms_list)

        dkappa_rms = 0
        if len(self.rl_dkappa_rms_list) > 0:
            dkappa_rms = np.average(self.rl_dkappa_rms_list)

        if self.rl_minimum_boundary > 999:
            self.rl_minimum_boundary = 0

        kappa_max_abs = 0
        if len(self.rl_kappa_max_abs_list) > 0:
            kappa_max_abs = max(self.rl_kappa_max_abs_list)

        dkappa_max_abs = 0
        if len(self.rl_dkappa_max_abs_list) > 0:
            dkappa_max_abs = max(self.rl_dkappa_max_abs_list)

        reference_line = {
            "is_offroad": self.rl_is_offroad_cnt,
            "minimum_boundary": self.rl_minimum_boundary,
            "kappa_rms": kappa_rms,
            "dkappa_rms": dkappa_rms,
            "kappa_max_abs": kappa_max_abs,
            "dkappa_max_abs": dkappa_max_abs
        }
        return reference_line
