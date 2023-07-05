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

"""
function to parse lidar data from *.record files, created using Apollo-Auto

parsed data is saved to *.txt file, for each scan

current implementation for:
* Velodyne VLS-128 lidar

"""

import os
import sys

from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import record
from modules.common_msgs.sensor_msgs.pointcloud_pb2 import PointCloud


def parse_data(channelname, msg, out_folder):
    """
    """
    msg_lidar = PointCloud()
    msg_lidar.ParseFromString(msg)
    nPts = len(msg_lidar.point)

    pcd = []
    for j in range(nPts):
        p = msg_lidar.point[j]
        pcd.append([p.x, p.y, p.z, p.intensity])

    tstamp = msg_lidar.measurement_time
    temp_time = str(tstamp).split('.')

    if len(temp_time[1]) == 1:
        temp_time1_adj = temp_time[1] + '0'
    else:
        temp_time1_adj = temp_time[1]

    pcd_time = temp_time[0] + '_' + temp_time1_adj
    pcd_filename = "pcd_" + pcd_time + ".txt"

    with open(out_folder + pcd_filename, 'w') as outfile:
        for item in pcd:
            data = str(item)[1:-1]
            outfile.write("%s\n" % data)

    return tstamp
