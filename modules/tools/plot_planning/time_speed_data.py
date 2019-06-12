#!/usr/bin/env python

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
import math

class TimeSpeedData:
    
    def __init__(self):
        self.timestamp_list = []
        self.speed_list = []
        self.imu_timestamp_list = []
        self.imu_speed_list = []

        self.last_speed_mps = None
        self.last_imu_speed = None

    def add(self, location_est, chassis):
        timestamp_sec = chassis.header.timestamp_sec
        speed_mps = chassis.speed_mps

        imu_based_timestamp = location_est.measurement_time
        self.imu_timestamp_list.append(imu_based_timestamp)

        imu_based_speed = location_est.pose.linear_velocity.x \
            * math.cos(location_est.pose.heading) + \
                location_est.pose.linear_velocity.y * \
                    math.sin(location_est.pose.heading)
        self.imu_speed_list.append(imu_based_speed)
            
        self.timestamp_list.append(timestamp_sec)
        self.speed_list.append(speed_mps)

        self.last_speed_mps = speed_mps

    def get_imu_based_time_speed(self):
        return self.imu_timestamp_list, self.imu_speed_list

    def get_imu_based_lastest_speed(self):
        if len(self.imu_speed_list) > 0:
            return self.imu_speed_list[-1]
        else:
            return None 

    def get_time_speed(self):
        return self.timestamp_list, self.speed_list

    def get_lastest(self):
        if len(self.speed_list) > 0:
            return self.speed_list[-1]
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
            processor = TimeSpeedData()
            last_pose_data = None
            last_chassis_data = None
            topics = ["/apollo/localization/pose", "/apollo/canbus/chassis"]
            for data in reader.read(topics):
                if "pose" in data:
                    last_pose_data = data["pose"]
                if "chassis" in data:
                    last_chassis_data = data["chassis"]
                if last_chassis_data is not None and last_pose_data is not None:
                    processor.add(last_pose_data, last_chassis_data)
                    last_pose_data = None
                    last_chassis_data = None

            data_x, data_y = processor.get_time_speed()
            ax.scatter(data_x, data_y, c=color, marker=marker, alpha=0.4)
            data_x, data_y = processor.get_imu_based_time_speed()
            ax.scatter(data_x, data_y, c="r", marker="+", alpha=0.4)

    plt.show() 
