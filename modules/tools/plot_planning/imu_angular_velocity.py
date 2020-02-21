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


class ImuAngularVelocity:
    def __init__(self):
        self.timestamp_list = []
        self.angular_velocity_list = []
        self.corrected_angular_velocity_list = []

        self.last_corrected_angular_velocity = None
        self.last_timestamp = None

    def add(self, location_est):
        timestamp_sec = location_est.header.timestamp_sec
        angular_velocity = location_est.pose.angular_velocity.z

        if self.last_corrected_angular_velocity is not None:
            corrected = self.correct_angular_velocity(
                angular_velocity, timestamp_sec)
        else:
            corrected = angular_velocity

        self.timestamp_list.append(timestamp_sec)
        self.angular_velocity_list.append(angular_velocity)
        self.corrected_angular_velocity_list.append(corrected)

        self.last_corrected_angular_velocity = corrected
        self.last_timestamp = timestamp_sec

    def correct_angular_velocity(self, angular_velocity, timestamp_sec):
        if self.last_corrected_angular_velocity is None:
            return angular_velocity
        delta = abs(angular_velocity - self.last_corrected_angular_velocity)\
            / abs(self.last_corrected_angular_velocity)

        if delta > 0.25:
            corrected = angular_velocity / 2.0
            return corrected
        else:
            return angular_velocity

    def get_anglular_velocity_list(self):
        return self.angular_velocity_list

    def get_corrected_anglular_velocity_list(self):
        return self.corrected_angular_velocity_list

    def get_timestamp_list(self):
        return self.timestamp_list

    def get_latest_angular_velocity(self):
        if len(self.angular_velocity_list) == 0:
            return None
        else:
            return self.angular_velocity_list[-1]

    def get_latest_corrected_angular_velocity(self):
        if len(self.corrected_angular_velocity_list) == 0:
            return None
        else:
            return self.corrected_angular_velocity_list[-1]

    def get_latest_timestamp(self):
        if len(self.timestamp_list) == 0:
            return None
        else:
            return self.timestamp_list[-1]


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
        fns.sort()
        for fn in fns:
            reader = RecordItemReader(folder+"/"+fn)
            processor = ImuAngularVelocity()
            for data in reader.read(["/apollo/localization/pose"]):
                processor.add(data["pose"])

            data_x = processor.get_timestamp_list()
            data_y = processor.get_corrected_anglular_velocity_list()
            ax.scatter(data_x, data_y, c=color, marker=marker, alpha=0.4)

            data_y = processor.get_anglular_velocity_list()
            ax.scatter(data_x, data_y, c='k', marker="+", alpha=0.8)

    plt.show()
