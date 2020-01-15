#!/usr/bin/env python3

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

import math

from ctypes import c_bool
from ctypes import c_double
from ctypes import c_ushort
from ctypes import c_void_p


lib = cdll.LoadLibrary(
    '/apollo/bazel-bin/modules/planning/open_space/tools/distance_approach_problem_wrapper_lib.so')

lib.CreateHybridAPtr.argtypes = []
lib.CreateHybridAPtr.restype = c_void_p
lib.DistanceCreateResultPtr.argtypes = []
lib.DistanceCreateResultPtr.restype = c_void_p
lib.DistanceCreateObstaclesPtr.argtypes = []
lib.DistanceCreateObstaclesPtr.restype = c_void_p
lib.AddObstacle.argtypes = [c_void_p, POINTER(c_double)]
lib.DistancePlan.restype = c_bool
lib.DistancePlan.argtypes = [c_void_p, c_void_p, c_void_p, c_double, c_double, c_double, c_double,
                             c_double, c_double, POINTER(c_double)]
lib.DistanceGetResult.argtypes = [c_void_p, c_void_p, POINTER(c_double), POINTER(c_double), POINTER(c_double),
                                  POINTER(c_double), POINTER(c_double), POINTER(
                                      c_double), POINTER(c_double),
                                  POINTER(c_double), POINTER(c_double), POINTER(
                                      c_double), POINTER(c_double),
                                  POINTER(c_double), POINTER(c_double), POINTER(
                                      c_double), POINTER(c_double),
                                  POINTER(c_ushort), POINTER(c_double), POINTER(c_double), POINTER(c_double)]


class DistancePlanner(object):
    def __init__(self):
        self.warm_start_planner = lib.CreateHybridAPtr()
        self.obstacles = lib.DistanceCreateObstaclesPtr()
        self.result = lib.DistanceCreateResultPtr()

    def AddObstacle(self, ROI_distance_approach_parking_boundary):
        lib.AddObstacle(self.obstacles, POINTER(
            c_double)(ROI_distance_approach_parking_boundary))

    def DistancePlan(self, sx, sy, sphi, ex, ey, ephi, XYbounds):
        return lib.DistancePlan(self.warm_start_planner, self.obstacles, self.result, c_double(sx),
                                c_double(sy), c_double(sphi), c_double(ex), c_double(ey), c_double(ephi), POINTER(c_double)(XYbounds))

    def DistanceGetResult(self, x, y, phi, v, a, steer, opt_x, opt_y, opt_phi, opt_v, opt_a, opt_steer, opt_time,
                          opt_dual_l, opt_dual_n, output_size, hybrid_time, dual_time, ipopt_time):
        lib.DistanceGetResult(self.result, self.obstacles, POINTER(c_double)(x), POINTER(c_double)(y),
                              POINTER(c_double)(phi), POINTER(c_double)(v), POINTER(c_double)(a), POINTER(
            c_double)(steer), POINTER(c_double)(opt_x), POINTER(c_double)(opt_y),
            POINTER(c_double)(opt_phi), POINTER(c_double)(opt_v), POINTER(c_double)(opt_a),
            POINTER(c_double)(opt_steer), POINTER(c_double)(
                opt_time), POINTER(c_double)(opt_dual_l),
            POINTER(c_double)(opt_dual_n), POINTER(c_ushort)(output_size),
            POINTER(c_double)(hybrid_time), POINTER(c_double)(dual_time), POINTER(c_double)(ipopt_time))
