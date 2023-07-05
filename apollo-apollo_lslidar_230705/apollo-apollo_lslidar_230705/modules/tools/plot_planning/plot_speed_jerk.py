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

from os import listdir
from os.path import isfile, join
import math
import sys

import matplotlib.pyplot as plt
import numpy as np

from modules.tools.plot_planning.imu_speed import ImuSpeed
from modules.tools.plot_planning.imu_speed_jerk import ImuSpeedJerk
from modules.tools.plot_planning.record_reader import RecordItemReader


def grid(data_list, shift):
    data_grid = []
    for data in data_list:
        data_grid.append(round(data) + shift / 10.0)
    return data_grid


def generate_speed_jerk_dict(speed_jerk_dict, speed_list, jerk_list):
    for i in range(len(speed_list)):
        speed = int(speed_list[i])
        jerk = int(jerk_list[i])
        if speed in speed_jerk_dict:
            if jerk not in speed_jerk_dict[speed]:
                speed_jerk_dict[speed].append(jerk)
        else:
            speed_jerk_dict[speed] = [jerk]
    return speed_jerk_dict


if __name__ == "__main__":

    folders = sys.argv[1:]
    fig, ax = plt.subplots(1, 1)
    colors = ["g", "b", "r", "m", "y"]
    markers = [".", ".", ".", "."]
    speed_jerk_dict = {}

    for i in range(len(folders)):
        x = []
        y = []
        folder = folders[i]
        color = colors[i % len(colors)]
        marker = markers[i % len(markers)]
        fns = [f for f in listdir(folder) if isfile(join(folder, f))]
        fns.sort()
        for fn in fns:
            reader = RecordItemReader(folder+"/"+fn)
            jerk_processor = ImuSpeedJerk(True)
            speed_processor = ImuSpeed(True)

            topics = ["/apollo/localization/pose"]
            for data in reader.read(topics):
                if "pose" in data:
                    pose_data = data["pose"]
                    speed_processor.add(pose_data)
                    jerk_processor.add(pose_data)

            data_x = grid(speed_processor.get_speed_list(), i + 1)
            data_y = grid(jerk_processor.get_jerk_list(), i + 1)
            data_x = data_x[-1 * len(data_y):]
            x.extend(data_x)
            y.extend(data_y)
            speed_jerk_dict = generate_speed_jerk_dict(speed_jerk_dict, x, y)

        if len(x) <= 0:
            continue
        ax.scatter(x, y, c=color, marker=marker, alpha=0.4)
        #ax.plot(x, y, c=color, alpha=0.4)

        ax.set_xlabel('Speed')
        ax.set_ylabel('Jerk')
    print(speed_jerk_dict)
    plt.show()
