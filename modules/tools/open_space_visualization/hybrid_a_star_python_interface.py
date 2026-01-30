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

import os
import math
import ctypes
from  ctypes import  POINTER
from ctypes import c_bool
from ctypes import c_double
from ctypes import c_int
from ctypes import c_ushort
from ctypes import c_void_p
from ctypes import cdll


APOLLO_DISTRIBUTION_HOME = os.environ.get(
    'APOLLO_DISTRIBUTION_HOME', '/opt/apollo/neo')
if APOLLO_DISTRIBUTION_HOME.startswith('/opt/apollo/neo'):
    lib_path = f"{APOLLO_DISTRIBUTION_HOME}/lib/modules/planning/planning_open_space/hybrid_a_star_wrapper_lib.so"
else:
    lib_path = f"{APOLLO_DISTRIBUTION_HOME}/bazel-bin/modules/planning/planning_open_space/hybrid_a_star_wrapper_lib.so"  

lib = cdll.LoadLibrary(lib_path)

lib.CreatePlannerPtr.argtypes = []
lib.CreatePlannerPtr.restype = c_void_p
lib.CreateResultPtr.argtypes = []
lib.CreateResultPtr.restype = c_void_p
lib.CreateObstaclesPtr.argtypes = []
lib.CreateObstaclesPtr.restype = c_void_p
lib.AddVirtualObstacle.argtypes = [c_void_p, POINTER(c_double), POINTER(c_double), c_int]
lib.Plan.restype = c_bool
lib.Plan.argtypes = [c_void_p, c_void_p, c_void_p, c_double, c_double, c_double, c_double,
                     c_double, c_double, POINTER(c_double)]
lib.GetResult.argtypes = [c_void_p, POINTER(c_double), POINTER(c_double), POINTER(c_double),
                          POINTER(c_double), POINTER(c_double), POINTER(c_double), POINTER(c_ushort)]


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
                        c_double(sy), c_double(sphi), c_double(ex), c_double(ey),
                        c_double(ephi), POINTER(c_double)(XYbounds))

    def GetResult(self, x, y, phi, v, a, steer, output_size):
        lib.GetResult(self.result, POINTER(c_double)(x), POINTER(c_double)(y),
                      POINTER(c_double)(phi), POINTER(c_double)(v), POINTER(c_double)(a),
                      POINTER(c_double)(steer), POINTER(c_ushort)(output_size))
