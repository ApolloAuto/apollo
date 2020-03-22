#!/usr/bin/env python3

###############################################################################
# Copyright 2019 The Apollo Authors. All Rights Reserved.
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

import math
from record_reader import RecordItemReader


class ChassisSpeed:

    def __init__(self):
        self.timestamp_list = []
        self.speed_list = []

    def add(self, chassis):
        timestamp_sec = chassis.header.timestamp_sec
        speed_mps = chassis.speed_mps

        self.timestamp_list.append(timestamp_sec)
        self.speed_list.append(speed_mps)

    def get_speed_list(self):
        return self.speed_list

    def get_timestamp_list(self):
        return self.timestamp_list

    def get_lastest_speed(self):
        if len(self.speed_list) > 0:
            return self.speed_list[-1]
        else:
            return None

    def get_lastest_timestamp(self):
        if len(self.timestamp_list) > 0:
            return self.timestamp_list[-1]
        else:
            return None


if __name__ == "__main__":
    import sys
    import matplotlib.pyplot as plt
    from os import listdir
    from os.path import isfile, join

    folders = sys.argv[1:]
    fig, ax = plt.subplots()
    colors = ["g", "b", "r", "m", "y"]
    markers = ["o", "o", "o", "o"]
    for i in range(len(folders)):
        folder = folders[i]
        color = colors[i % len(colors)]
        marker = markers[i % len(markers)]
        fns = [f for f in listdir(folder) if isfile(join(folder, f))]
        for fn in fns:
            reader = RecordItemReader(folder+"/"+fn)
            processor = ChassisSpeed()
            last_chassis_data = None
            topics = ["/apollo/canbus/chassis"]
            for data in reader.read(topics):
                if "chassis" in data:
                    last_chassis_data = data["chassis"]
                    processor.add(last_chassis_data)
                    last_pose_data = None
                    last_chassis_data = None

            data_x = processor.get_timestamp_list()
            data_y = processor.get_speed_list()

            ax.scatter(data_x, data_y, c=color, marker=marker, alpha=0.4)

    plt.show()
