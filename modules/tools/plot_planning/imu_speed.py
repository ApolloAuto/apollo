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


class ImuSpeed:

    def __init__(self, is_lateral=False):
        self.timestamp_list = []
        self.speed_list = []

        self.last_speed_mps = None
        self.last_imu_speed = None

        self.is_lateral = is_lateral

    def add(self, location_est):
        timestamp_sec = location_est.measurement_time
        self.timestamp_list.append(timestamp_sec)
        if self.is_lateral:
            speed = -1 * location_est.pose.linear_velocity.x \
                * math.sin(location_est.pose.heading) + \
                location_est.pose.linear_velocity.y * \
                math.cos(location_est.pose.heading)
        else:
            speed = location_est.pose.linear_velocity.x \
                * math.cos(location_est.pose.heading) + \
                location_est.pose.linear_velocity.y * \
                math.sin(location_est.pose.heading)
        self.speed_list.append(speed)

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
    fig, ax = plt.subplots(2, 1)
    colors = ["g", "b", "r", "m", "y"]
    markers = ["o", "o", "o", "o"]
    for i in range(len(folders)):
        folder = folders[i]
        color = colors[i % len(colors)]
        marker = markers[i % len(markers)]
        fns = [f for f in listdir(folder) if isfile(join(folder, f))]
        for fn in fns:
            reader = RecordItemReader(folder+"/"+fn)
            lat_speed_processor = ImuSpeed(True)
            lon_speed_processor = ImuSpeed(False)

            last_pose_data = None
            last_chassis_data = None
            topics = ["/apollo/localization/pose"]
            for data in reader.read(topics):
                if "pose" in data:
                    last_pose_data = data["pose"]
                    lat_speed_processor.add(last_pose_data)
                    lon_speed_processor.add(last_pose_data)

            data_x = lon_speed_processor.get_timestamp_list()
            data_y = lon_speed_processor.get_speed_list()
            ax[0].scatter(data_x, data_y, c=color, marker=marker, alpha=0.4)

            data_x = lat_speed_processor.get_timestamp_list()
            data_y = lat_speed_processor.get_speed_list()
            ax[1].scatter(data_x, data_y, c=color, marker=marker, alpha=0.4)

    ax[0].set_xlabel('Timestamp')
    ax[0].set_ylabel('Lon Acc')
    ax[1].set_xlabel('Timestamp')
    ax[1].set_ylabel('Lat Acc')

    plt.show()
