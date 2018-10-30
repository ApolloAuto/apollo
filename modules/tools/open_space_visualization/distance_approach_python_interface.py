#!/usr/bin/env python

###############################################################################
# Copyright 2018 The Apollo Authors. All Rights Reserved.
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

import ctypes
from ctypes import cdll, c_ushort, c_int
from ctypes import c_double
from ctypes import POINTER
import math

lib = cdll.LoadLibrary(
    '/apollo/bazel-bin/modules/planning/open_space/distance_approach_problem_wrapper_lib.so')


class DistancePlanner(object):
    def __init__(self):
        self.warm_start_planner = lib.CreateHybridAPtr()
        self.obstacles = lib.DistanceCreateObstaclesPtr()
        self.result = lib.DistanceCreateResultPtr()

    def AddWarmStartObstacle(self, x, y, heading, length, width, identity):
        lib.AddWarmStartObstacle(self.obstacles, c_double(x),
                                 c_double(y), c_double(heading), c_double(length), c_double(width), c_int(identity))

    def AddDistanceApproachObstacle(self, ROI_distance_approach_parking_boundary):
        lib.AddDistanceApproachObstacle(self.obstacles, POINTER(
            c_double)(ROI_distance_approach_parking_boundary))

    def DistancePlan(self, sx, sy, sphi, ex, ey, ephi, XYbounds):
        return lib.DistancePlan(self.warm_start_planner, self.obstacles, self.result, c_double(sx),
                        c_double(sy), c_double(sphi), c_double(ex), c_double(ey), c_double(ephi), POINTER(c_double)(XYbounds))

    def DistanceGetResult(self, x, y, phi, v, a, steer, opt_x, opt_y, opt_phi, opt_v, opt_a, opt_steer, opt_time, output_size):
        lib.DistanceGetResult(self.result, POINTER(c_double)(x), POINTER(c_double)(y),
                      POINTER(c_double)(phi), POINTER(c_double)(v), POINTER(c_double)(a), POINTER(
                          c_double)(steer), POINTER(c_double)(opt_x), POINTER(c_double)(opt_y),
                      POINTER(c_double)(opt_phi), POINTER(c_double)(opt_v), POINTER(c_double)(opt_a), POINTER(c_double)(opt_steer), POINTER(c_double)(opt_time), POINTER(c_ushort)(output_size))
