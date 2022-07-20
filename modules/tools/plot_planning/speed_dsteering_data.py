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

import sys
from modules.tools.plot_planning.record_reader import RecordItemReader
import matplotlib.pyplot as plt
from cyber.python.cyber_py3.record import RecordReader
from modules.common_msgs.chassis_msgs import chassis_pb2


class SpeedDsteeringData:
    def __init__(self):
        self.last_steering_percentage = None
        self.last_speed_mps = None
        self.last_timestamp_sec = None
        self.speed_data = []
        self.d_steering_data = []

    def add(self, chassis):
        steering_percentage = chassis.steering_percentage
        speed_mps = chassis.speed_mps
        timestamp_sec = chassis.header.timestamp_sec

        if self.last_timestamp_sec is None:
            self.last_steering_percentage = steering_percentage
            self.last_speed_mps = speed_mps
            self.last_timestamp_sec = timestamp_sec
            return

        if (timestamp_sec - self.last_timestamp_sec) > 0.02:
            d_steering = (steering_percentage - self.last_steering_percentage) \
                / (timestamp_sec - self.last_timestamp_sec)
            self.speed_data.append(speed_mps)
            self.d_steering_data.append(d_steering)

            self.last_steering_percentage = steering_percentage
            self.last_speed_mps = speed_mps
            self.last_timestamp_sec = timestamp_sec

    def get_speed_dsteering(self):
        return self.speed_data, self.d_steering_data


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
            processor = SpeedDsteeringData()
            last_pose_data = None
            last_chassis_data = None
            topics = ["/apollo/localization/pose", "/apollo/canbus/chassis"]
            for data in reader.read(topics):
                if "chassis" in data:
                    last_chassis_data = data["chassis"]
                if last_chassis_data is not None:
                    processor.add(last_chassis_data)
                    #last_pose_data = None
                    #last_chassis_data = None

            data_x, data_y = processor.get_speed_dsteering()
            ax.scatter(data_x, data_y, c=color, marker=marker, alpha=0.2)

    plt.show()
