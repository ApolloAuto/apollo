#!/usr/bin/env python

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
from cyber_py import cyber
from cyber_py.record import RecordReader
from modules.localization.proto import localization_pb2

if len(sys.argv) < 3:
    print("Usage: %s <filename> <fbags>" % sys.argv[0])
    sys.exit(0)

filename = sys.argv[1]
fbags = sys.argv[2:]

with open(filename, 'w') as f:
    for fbag in fbags:
        reader = RecordReader(fbag)
        for msg in reader.read_messages():
            if msg.topic == "/apollo/localization/pose":
                localization = localization_pb2.LocalizationEstimate()
                localization.ParseFromString(msg.message)
                x = localization.pose.position.x
                y = localization.pose.position.y
                f.write(str(x) + "," + str(y) + "\n")
print("File written to: %s" % filename)
