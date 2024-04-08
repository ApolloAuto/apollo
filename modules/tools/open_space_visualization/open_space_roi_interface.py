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

import ctypes
import math
import os
from ctypes import cdll, c_ushort, c_int, c_char_p, c_double, POINTER

APOLLO_DISTRIBUTION_HOME = os.environ.get(
    'APOLLO_DISTRIBUTION_HOME', '/opt/apollo/neo')
if APOLLO_DISTRIBUTION_HOME.startswith('/opt/apollo/neo'):
    lib_path = f"{APOLLO_DISTRIBUTION_HOME}/lib/modules/planning/planning_base/open_space_roi_wrapper_lib.so"
else:
    lib_path = f"{APOLLO_DISTRIBUTION_HOME}/bazel-bin/modules/planning/planning_base/open_space_roi_wrapper_lib.so"  

lib = cdll.LoadLibrary(lib_path)


class open_space_roi(object):
    def __init__(self):
        self.open_space_roi_test = lib.CreateROITestPtr()

    def ROITest(self, lane_id, parking_id,
                unrotated_roi_boundary_x, unrotated_roi_boundary_y, roi_boundary_x, roi_boundary_y,
                parking_spot_x, parking_spot_y, end_pose,
                xy_boundary, origin_pose):
        return lib.ROITest(self.open_space_roi_test, (c_char_p)(lane_id.encode('utf-8')),
                           (c_char_p)(parking_id.encode('utf-8')),
                           POINTER(c_double)(unrotated_roi_boundary_x), POINTER(
                               c_double)(unrotated_roi_boundary_y),
                           POINTER(c_double)(roi_boundary_x), POINTER(
                               c_double)(roi_boundary_y), POINTER(c_double)(
            parking_spot_x), POINTER(c_double)(
            parking_spot_y), POINTER(c_double)(end_pose),
            POINTER(c_double)(xy_boundary), POINTER(c_double)(origin_pose))
