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

import sys
from datetime import datetime
from cyber_py3.record import RecordReader
from modules.localization.proto import localization_pb2

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("usage: python record_extractor.py record_file1 record_file2 ...")

    frecords = sys.argv[1:]
    now = datetime.now().strftime("%Y-%m-%d_%H.%M.%S")

    with open("path_" + frecords[0].split('/')[-1] + ".txt", 'w') as f:
        for frecord in frecords:
            print("processing " + frecord)
            reader = RecordReader(frecord)
            for msg in reader.read_messages():
                if msg.topic == "/apollo/localization/pose":
                    localization = localization_pb2.LocalizationEstimate()
                    localization.ParseFromString(msg.message)
                    x = localization.pose.position.x
                    y = localization.pose.position.y
                    f.write(str(x) + "," + str(y) + "\n")
