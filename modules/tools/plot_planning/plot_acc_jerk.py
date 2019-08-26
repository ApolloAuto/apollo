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

import math
from record_reader import RecordItemReader
from imu_speed_jerk import ImuSpeedJerk
from imu_speed_acc import ImuSpeedAcc


if __name__ == "__main__":
    import sys
    import matplotlib.pyplot as plt
    import numpy as np
    from os import listdir
    from os.path import isfile, join


    folders = sys.argv[1:]
    fig, ax = plt.subplots(1, 1)
    colors = ["g", "b", "r", "m", "y"]
    markers = [".", ".", ".", "."]
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
            jerk_processor = ImuSpeedJerk()
            acc_processor = ImuSpeedAcc()

            topics = ["/apollo/localization/pose"]
            for data in reader.read(topics):
                if "pose" in data:
                    pose_data = data["pose"]
                    acc_processor.add(pose_data)
                    jerk_processor.add(pose_data)

            data_x = acc_processor.get_acc_list()
            data_y = jerk_processor.get_jerk_list()
            data_x = data_x[-1* len(data_y):]
            x.extend(data_x)
            y.extend(data_y)

        if len(x) <= 0:
            continue
        ax.scatter(x, y, c=color, marker=marker, alpha=0.4)
        #ax.plot(x, y, c=color, alpha=0.4)

        ax.set_xlabel('Acc')
        ax.set_ylabel('Jerk')

    plt.show()
