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
from modules.tools.plot_planning.record_reader import RecordItemReader


class ImuAcc:

    def __init__(self):
        self.timestamp_list = []
        self.corrected_acc_list = []
        self.acc_list = []

        self.last_corrected_acc = None
        self.last_timestamp = None

    def add(self, location_est):
        timestamp = location_est.measurement_time
        acc = location_est.pose.linear_acceleration.x * \
            math.cos(location_est.pose.heading) + \
            location_est.pose.linear_acceleration.y * \
            math.sin(location_est.pose.heading)

        if self.last_corrected_acc is not None:
            corrected_acc = self._correct_acc(acc, self.last_corrected_acc)
        else:
            corrected_acc = acc

        self.acc_list.append(acc)
        self.corrected_acc_list.append(corrected_acc)
        self.timestamp_list.append(timestamp)

        self.last_timestamp = timestamp
        self.last_corrected_acc = corrected_acc

    def get_acc_list(self):
        return self.acc_list

    def get_corrected_acc_list(self):
        return self.corrected_acc_list

    def get_timestamp_list(self):
        return self.timestamp_list

    def get_lastest_corrected_acc(self):
        if len(self.corrected_acc_list) > 0:
            return self.corrected_acc_list[-1]
        else:
            return None

    def get_lastest_acc(self):
        if len(self.acc_list) > 0:
            return self.acc_list[-1]
        else:
            return None

    def get_lastest_timestamp(self):
        if len(self.timestamp_list) > 0:
            return self.timestamp_list[-1]
        else:
            return None

    def _correct_acc(self, acc, last_acc):
        if last_acc is None:
            return last_acc
        delta = abs(acc - last_acc) / abs(last_acc)
        if delta > 0.4:
            corrected = acc / 2.0
            return corrected
        else:
            return acc


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
            processor = ImuAcc()
            last_pose_data = None
            last_chassis_data = None
            topics = ["/apollo/localization/pose"]
            for data in reader.read(topics):
                if "pose" in data:
                    last_pose_data = data["pose"]
                    processor.add(last_pose_data)
                    last_pose_data = None
                    last_chassis_data = None

            data_x = processor.get_timestamp_list()
            data_y = processor.get_corrected_acc_list()
            ax.scatter(data_x, data_y, c=color, marker=marker, alpha=0.4)
            data_y = processor.get_acc_list()
            ax.scatter(data_x, data_y, c="k", marker="+", alpha=0.4)

    plt.show()
