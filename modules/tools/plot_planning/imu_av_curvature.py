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
from imu_angular_velocity import ImuAngularVelocity
from imu_speed import ImuSpeed


class ImuAvCurvature:
    def __init__(self):
        self.timestamp_list = []
        self.curvature_list = []

        self.last_angular_velocity_z = None

        self.imu_angular_velocity = ImuAngularVelocity()
        self.imu_speed = ImuSpeed()

    def add(self, location_est):
        timestamp_sec = location_est.header.timestamp_sec

        self.imu_angular_velocity.add(location_est)
        self.imu_speed.add(location_est)

        angular_velocity_z \
            = self.imu_angular_velocity.get_latest_corrected_angular_velocity()
        speed_mps = self.imu_speed.get_lastest_speed()
        if speed_mps > 0.03:
            kappa = angular_velocity_z / speed_mps
        else:
            kappa = 0

        self.timestamp_list.append(timestamp_sec)
        self.curvature_list.append(kappa)

        self.last_angular_velocity_z = angular_velocity_z

    def get_timestamp_list(self):
        return self.timestamp_list

    def get_curvature_list(self):
        return self.curvature_list

    def get_last_timestamp(self):
        if len(self.timestamp_list) > 0:
            return self.timestamp_list[-1]
        return None

    def get_last_curvature(self):
        if len(self.curvature_list) > 0:
            return self.curvature_list[-1]
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
        fns.sort()
        for fn in fns:
            print(fn)
            reader = RecordItemReader(folder+"/"+fn)
            curvature_processor = ImuAvCurvature()
            speed_processor = ImuSpeed()
            av_processor = ImuAngularVelocity()
            last_pose_data = None
            last_chassis_data = None
            for data in reader.read(["/apollo/localization/pose"]):
                if "pose" in data:
                    last_pose_data = data["pose"]
                    curvature_processor.add(last_pose_data)
                    speed_processor.add(last_pose_data)
                    av_processor.add(last_pose_data)

            data_x = curvature_processor.get_timestamp_list()
            data_y = curvature_processor.get_curvature_list()
            ax.scatter(data_x, data_y, c=color, marker=marker, alpha=0.4)

            data_x = speed_processor.get_timestamp_list()
            data_y = speed_processor.get_speed_list()
            ax.scatter(data_x, data_y, c='r', marker=marker, alpha=0.4)

            data_x = av_processor.get_timestamp_list()
            data_y = av_processor.get_corrected_anglular_velocity_list()
            ax.scatter(data_x, data_y, c='b', marker=marker, alpha=0.4)

    plt.show()
