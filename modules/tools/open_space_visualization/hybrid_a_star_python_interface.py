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
    '/apollo/bazel-bin/modules/tools/open_space_visualization/hybrid_a_star_wrapper_lib.so')


class HybridAStarPlanner(object):
    def __init__(self):
        self.planner = lib.CreatePlannerPtr()
        self.obstacles = lib.CreateObstaclesPtr()
        self.result = lib.CreateResultPtr()

    def AddVirtualObstacle(self, obstacle_x, obstacle_y, vertice_num):
        lib.AddVirtualObstacle(self.obstacles, POINTER(c_double)(obstacle_x),
                               POINTER(c_double)(obstacle_y), (c_int)(vertice_num))

    def Plan(self, sx, sy, sphi, ex, ey, ephi, XYbounds):
        return lib.Plan(self.planner, self.obstacles, self.result, c_double(sx),
                        c_double(sy), c_double(sphi), c_double(ex), c_double(ey), c_double(ephi), POINTER(c_double)(XYbounds))

    def GetResult(self, x, y, phi, v, a, steer, output_size):
        lib.GetResult(self.result, POINTER(c_double)(x), POINTER(c_double)(y),
                      POINTER(c_double)(phi), POINTER(c_double)(v), POINTER(c_double)(a), POINTER(c_double)(steer), POINTER(c_ushort)(output_size))
