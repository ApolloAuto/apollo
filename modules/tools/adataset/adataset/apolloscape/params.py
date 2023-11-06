#!/usr/bin/env python3

###############################################################################
# Copyright 2023 The Apollo Authors. All Rights Reserved.
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


scape2apollo_lidar = np.array([[1.0000000, 0.0000000, 0.0000000, 0.0000000],
                               [0.0000000, 1.0000000, 0.0000000, 0.0000000],
                               [0.0000000, 0.0000000, 1.0000000, 0.0000000],
                               [0.0000000, 0.0000000, 0.0000000, 1.0000000]])

scape2apollo_imu = np.array([[0.0000000, -1.0000000, 0.0000000, 0.0000000],
                             [1.0000000, 0.0000000, 0.0000000, 0.5000000],
                             [0.0000000, 0.0000000, 1.0000000, 1.5000000],
                             [0.0000000, 0.0000000, 0.0000000, 1.0000000]])

apollo2scape_lidar = np.linalg.inv(scape2apollo_lidar)
apollo2scape_imu = np.linalg.inv(scape2apollo_imu)
