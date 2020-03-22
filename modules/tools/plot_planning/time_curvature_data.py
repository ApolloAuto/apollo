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

from record_reader import RecordItemReader
from time_angular_velocity_data import TimeAngularVelocityData
from time_speed_data import TimeSpeedData
import math


class TimeCurvatureData:
    def __init__(self):
        self.timestamp_list = []
        self.curvature_list = []
        self.speed_list = []

        self.corrected_timestamp_list = []
        self.corrected_velocity_list = []

        self.last_angular_velocity_z = None

        self.angular_velocity_data = TimeAngularVelocityData()
        self.speed_data = TimeSpeedData()

    def add(self, location_est, chassis):
        timestamp_sec = location_est.header.timestamp_sec

        self.angular_velocity_data.add_location_estimation(location_est)
        self.speed_data.add(location_est, chassis)

        angular_velocity_z = self.angular_velocity_data.get_latest()
        speed_mps = self.speed_data.get_imu_based_lastest_speed()

        if speed_mps > 0.5:
            kappa = angular_velocity_z / speed_mps
            if kappa > 0.05:
                self.timestamp_list.append(timestamp_sec)
                self.curvature_list.append(kappa)
                self.speed_list.append(speed_mps)

        self.last_angular_velocity_z = angular_velocity_z

    def get_time_curvature(self):
        return self.timestamp_list, self.curvature_list

    def get_speed_curvature(self):
        return self.speed_list, self.curvature_list

    def get_fixed_ca_speed_curvature(self):
        speed_list = list(range(1, 31))
        curvature_list = []
        for speed in speed_list:
            curvature = 2.0 / (speed * speed)
            curvature_list.append(curvature)
        return speed_list, curvature_list


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
            print(fn)
            reader = RecordItemReader(folder+"/"+fn)
            processor = TimeCurvatureData()
            last_pose_data = None
            last_chassis_data = None
            for data in reader.read(["/apollo/localization/pose",
                                     "/apollo/canbus/chassis"]):
                if "pose" in data:
                    last_pose_data = data["pose"]
                if "chassis" in data:
                    last_chassis_data = data["chassis"]
                if last_chassis_data is not None and last_pose_data is not None:
                    processor.add(last_pose_data, last_chassis_data)

            data_x, data_y = processor.get_speed_curvature()
            data_y = [abs(i) for i in data_y]
            ax.scatter(data_x, data_y, c=color, marker=marker, alpha=0.4)
            #data_x, data_y = processor.speed_data.get_time_speed()
            #ax.scatter(data_x, data_y, c=color, marker="+", alpha=0.4)
    processor = TimeCurvatureData()
    x, y = processor.get_fixed_ca_speed_curvature()
    ax.plot(x, y)
    plt.show()
