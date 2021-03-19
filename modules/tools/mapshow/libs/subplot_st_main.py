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


class StMainSubplot:
    def __init__(self, ax, st_name):
        self.st_curve_line, = ax.plot([0], [0], "k.", lw=3, alpha=0.5)
        self.kernel_cruise_line, = ax.plot([0], [0], "g.", lw=3, alpha=0.5)
        self.kernel_follow_line, = ax.plot([0], [0], "y.", lw=3, alpha=0.5)
        self.obstacle_boundary_lines = []
        self.obstacle_annotations = []
        self.obstacle_boundary_size = 10
        for i in range(self.obstacle_boundary_size):
            self.obstacle_boundary_lines.append(
                ax.plot([0], [0], "r-", lw=1, alpha=1)[0])
            self.obstacle_annotations.append(ax.text(0, 0, ""))

        # self.st_name = planning_config_pb2.TaskType.Name(
        #    planning_config_pb2.QP_SPLINE_ST_SPEED_OPTIMIZER)
        self.st_name = st_name
        ax.set_xlim(-3, 9)
        ax.set_ylim(-10, 220)
        ax.set_xlabel("t (second)")
        ax.set_ylabel("s (m)")
        ax.set_title(st_name)

        self.set_visible(False)

    def set_visible(self, visible):
        self.st_curve_line.set_visible(visible)
        self.kernel_cruise_line.set_visible(visible)
        self.kernel_follow_line.set_visible(visible)
        for line in self.obstacle_boundary_lines:
            line.set_visible(visible)
        for text in self.obstacle_annotations:
            text.set_visible(visible)

    def show(self, planning):
        self.set_visible(False)
        planning.st_data_lock.acquire()
        if self.st_name not in planning.st_data_boundary_s:
            planning.st_data_lock.release()
            return
        obstacles_boundary_s = planning.st_data_boundary_s[self.st_name]
        obstacles_boundary_t = planning.st_data_boundary_t[self.st_name]
        obstacles_type = planning.st_data_boundary_type[self.st_name]
        cnt = 1
        for boundary_name in obstacles_boundary_s.keys():
            if cnt >= self.obstacle_boundary_size:
                print("WARNING: number of path lines is more than "
                      + self.obstacle_boundary_size)
                continue
            boundary = self.obstacle_boundary_lines[cnt]
            boundary.set_visible(True)

            boundary.set_xdata(obstacles_boundary_t[boundary_name])
            boundary.set_ydata(obstacles_boundary_s[boundary_name])
            center_t = 0
            center_s = 0
            for i in range(len(obstacles_boundary_t[boundary_name]) - 1):
                center_s += obstacles_boundary_s[boundary_name][i]
                center_t += obstacles_boundary_t[boundary_name][i]
            center_s /= float(len(obstacles_boundary_s[boundary_name]) - 1)
            center_t /= float(len(obstacles_boundary_t[boundary_name]) - 1)

            annotation = self.obstacle_annotations[cnt]
            annotation.set_visible(True)
            annotation.set_text(boundary_name + "_"
                                + obstacles_type[boundary_name]
                                .replace("ST_BOUNDARY_TYPE_", ""))
            annotation.set_x(center_t)
            annotation.set_y(center_s)

            cnt += 1

        self.st_curve_line.set_visible(True)
        self.st_curve_line.set_xdata(planning.st_curve_t[self.st_name])
        self.st_curve_line.set_ydata(planning.st_curve_s[self.st_name])
        self.st_curve_line.set_label(self.st_name[0:5])

        self.kernel_cruise_line.set_visible(True)
        self.kernel_cruise_line.set_xdata(
            planning.kernel_cruise_t[self.st_name])
        self.kernel_cruise_line.set_ydata(
            planning.kernel_cruise_s[self.st_name])
        self.kernel_follow_line.set_visible(True)
        self.kernel_follow_line.set_xdata(
            planning.kernel_follow_t[self.st_name])
        self.kernel_follow_line.set_ydata(
            planning.kernel_follow_s[self.st_name])

        planning.st_data_lock.release()
