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

import numpy as np


class Curvature:
    def __init__(self):
        self.curvature_list = []
        self.curvature_derivative_list = []

    def put(self, adc_trajectory):
        init_point = adc_trajectory.debug.planning_data.init_point
        self.curvature_list.append(abs(init_point.path_point.kappa))
        self.curvature_derivative_list.append(abs(init_point.path_point.dkappa))

    def get_curvature(self):
        curvature = {}
        if len(self.curvature_list) == 0:
            curvature["max"] = 0
            curvature["avg"] = 0
            return curvature

        curvature["max"] = max(self.curvature_list, key=abs)
        curvature["avg"] = np.average(np.absolute(self.curvature_list))

        return curvature

    def get_curvature_derivative(self):
        curvature_derivative = {}
        if len(self.curvature_derivative_list) == 0:
            curvature_derivative["max"] = 0
            curvature_derivative["avg"] = 0

        curvature_derivative["max"] = max(self.curvature_derivative_list, key=abs)
        curvature_derivative["avg"] = np.average(np.absolute(self.curvature_derivative_list))

        return curvature_derivative
