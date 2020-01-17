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
function to parse camera images from *.record files, created using Apollo-Auto

parsed data is saved to *.jpeg file, for each capture

"""

import os
import sys

from cyber_py3 import cyber
from cyber_py3 import record
from modules.drivers.proto.sensor_image_pb2 import CompressedImage


def parse_data(channelname, msg, out_folder):
    """
    parser images from Apollo record file
    """
    msg_camera = CompressedImage()
    msg_camera.ParseFromString(str(msg))

    tstamp = msg_camera.measurement_time

    temp_time = str(tstamp).split('.')
    if len(temp_time[1]) == 1:
        temp_time1_adj = temp_time[1] + '0'
    else:
        temp_time1_adj = temp_time[1]
    image_time = temp_time[0] + '_' + temp_time1_adj

    image_filename = "image_" + image_time + ".jpeg"
    f = open(out_folder + image_filename, 'w+b')
    f.write(msg_camera.data)
    f.close()

    return tstamp

