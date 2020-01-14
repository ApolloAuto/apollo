#!/usr/bin/env python3

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
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


class SpeedSubplot:
    def __init__(self, ax):
        self.ax = ax
        self.speed_lines = []
        self.speed_lines_size = 3
        colors = ['b', 'g', 'r', 'k']
        for i in range(self.speed_lines_size):
            line, = ax.plot(
                [0], [0],
                colors[i % len(colors)] + ".",
                lw=3 + i * 3,
                alpha=0.4)
            self.speed_lines.append(line)

        ax.set_xlabel("t (second)")
        ax.set_xlim([-2, 10])
        ax.set_ylim([-1, 40])
        ax.set_ylabel("speed (m/s)")
        ax.set_title("PLANNING SPEED")
        self.set_visible(False)

    def set_visible(self, visible):
        for line in self.speed_lines:
            line.set_visible(visible)

    def show(self, planning):
        cnt = 0
        planning.speed_data_lock.acquire()
        for name in planning.speed_data_time.keys():
            if cnt >= self.speed_lines_size:
                print("WARNING: number of path lines is more than "
                      + str(self.speed_lines_size))
                continue
            if len(planning.speed_data_time[name]) <= 1:
                continue
            speed_line = self.speed_lines[cnt]
            speed_line.set_visible(True)
            speed_line.set_xdata(planning.speed_data_time[name])
            speed_line.set_ydata(planning.speed_data_val[name])
            speed_line.set_label(name[0:5])
            cnt += 1

        self.ax.legend(loc="upper left", borderaxespad=0., ncol=5)
        # self.ax.axis('equal')
        planning.speed_data_lock.release()
