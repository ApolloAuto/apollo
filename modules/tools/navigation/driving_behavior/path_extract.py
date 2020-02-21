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
extract localization message from  bag files
Usage:
    python path_extract.py file1 file2 ...
"""
import sys
import datetime
from cyber_py3.record import RecordReader
from modules.localization.proto import localization_pb2

kLocalizationTopic = '/apollo/localization/pose'

if __name__ == '__main__':
    bag_files = sys.argv[1:]

    bag_file = bag_files[0]
    now = datetime.datetime.now().strftime("%Y-%m-%d_%H.%M.%S")
    f = open("path_" + bag_file.split('/')[-1] + ".txt", 'w')

    for bag_file in bag_files:
        print("begin to extract path from file :", bag_file)
        reader = RecordReader(bag_file)
        localization = localization_pb2.LocalizationEstimate()
        for msg in reader.read_messages():
            if msg.topic == kLocalizationTopic:
                localization.ParseFromString(msg.message)
                x = localization.pose.position.x
                y = localization.pose.position.y
                f.write(str(x) + "," + str(y) + "\n")
        print("Finished extracting path from file :", bag_file)
    f.close()
