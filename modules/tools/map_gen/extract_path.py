#!/usr/bin/env python3

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
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
Extract localization message from data record file,
and save them into specified  file

Usage:
    extract_path.py save_fileName  bag1 bag2

See the gflags for more optional args.
"""

import sys
sys.path.append("/apollo/")
sys.path.append("/apollo/bazel-bin/")
import math
from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3.record import RecordReader
from modules.common_msgs.localization_msgs import localization_pb2

if len(sys.argv) < 3:
    print("Usage: %s <filename> <fbags>" % sys.argv[0])
    sys.exit(0)

filename = sys.argv[1]
fbags = sys.argv[2:]

first_flag = True
lastx = 0
lasty = 0
s = 0.0

with open(filename, 'w') as f:
    for fbag in fbags:
        reader = RecordReader(fbag)
        for msg in reader.read_messages():
            if msg.topic == "/apollo/localization/pose":
                localization = localization_pb2.LocalizationEstimate()
                localization.ParseFromString(msg.message)
                x = localization.pose.position.x
                y = localization.pose.position.y
                z = localization.pose.position.z
                
                if first_flag:
                    first_flag = False
                    lastx = x
                    lasty = y
                vx = localization.pose.linear_velocity.x
                vy = localization.pose.linear_velocity.y
                vz = localization.pose.linear_velocity.z
                v = math.sqrt(vx**2 + vy**2)
                ax = localization.pose.linear_acceleration.x
                ay = localization.pose.linear_acceleration.y
                az = localization.pose.linear_acceleration.z
                a = math.sqrt(ax**2 + ay**2)
                heading = localization.pose.heading # + 90.0 / 180.0 * 3.14159265358979323846
                time = localization.measurement_time
                
                s += math.sqrt((x - lastx)**2 + (y - lasty)**2)
                
                lastx = x
                lasty = y
                
                print(x, y)
                f.write(str(x) + ", "  + str(y) + ", " + str(z) + ", " + str(v) + ", " + str(a) + ", " + str(0.0) + ", " + str(0.0) + ", " + str(time) + ", " + str(heading) + ", " + str(1) + ", " + str(s) + ", " + str(0.0) + ", " + str(0.0) + ", " + str(0.0) + "\n")
print("File written to: %s" % filename)
